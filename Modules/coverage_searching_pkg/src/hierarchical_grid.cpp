#include "coverage_search.h"
#include <limits>

// ============================================================
// HierarchicalGrid 实现 - 与原版相同
// ============================================================
void HierarchicalGrid::init(ros::NodeHandle &nh, CoverageMap *map) {
    map_ = map;
    nh.param("hgrid/cell_size", cell_size_, 5.0);
    nh.param("hgrid/min_unknown_num", min_unknown_num_, 10);
    nh.param("hgrid/unknown_penalty", unknown_penalty_, 2.0);

    num_cells_x_ = ceil(map_->map_size_3d_(0) / cell_size_);
    num_cells_y_ = ceil(map_->map_size_3d_(1) / cell_size_);
    grid_origin_ = map_->origin_;

    int total_cells = num_cells_x_ * num_cells_y_;
    grid_cells_.resize(total_cells);

    for (int i = 0; i < num_cells_x_; i++) {
        for (int j = 0; j < num_cells_y_; j++) {
            int id = i * num_cells_y_ + j;
            grid_cells_[id].id_ = id;
            grid_cells_[id].center_ = grid_origin_ + Eigen::Vector3d(
                (i + 0.5) * cell_size_, (j + 0.5) * cell_size_, map_->fly_height_);
            grid_cells_[id].bbox_min_ = grid_origin_ + Eigen::Vector3d(
                i * cell_size_, j * cell_size_, map_->min_range_(2));
            grid_cells_[id].bbox_max_ = grid_origin_ + Eigen::Vector3d(
                (i + 1) * cell_size_, (j + 1) * cell_size_, map_->max_range_(2));
            grid_cells_[id].unknown_num_ = 0;
            grid_cells_[id].free_num_ = 0;
            grid_cells_[id].frontier_num_ = 0;
            grid_cells_[id].is_active_ = false;
        }
    }

    string uav_name; int uav_id;
    nh.param("uav_id", uav_id, 1);
    uav_name = "/uav" + std::to_string(uav_id);
    grid_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        uav_name + "/prometheus/coverage_search/grid_vis", 1);

    cout << GREEN << "[HierarchicalGrid] init. Cell size: " << cell_size_
         << ", Grid: " << num_cells_x_ << "x" << num_cells_y_
         << " = " << total_cells << " cells" << TAIL << endl;
}

void HierarchicalGrid::updateFromMap() {
    // ★ 注意：不清除frontier信息，因为inputFrontiers可能在updateFromMap之后调用
    // 只重新统计unknown_num和free_num
    active_cell_ids_.clear();
    for (auto &cell : grid_cells_) {
        cell.unknown_num_ = 0; cell.free_num_ = 0;
        // ★ 不重置frontier_num_和frontier_ids_等，保留inputFrontiers的结果

        Eigen::Vector3i min_idx, max_idx;
        map_->posToIndex(cell.bbox_min_, min_idx);
        map_->posToIndex(cell.bbox_max_, max_idx);
        for (int i = 0; i < 3; i++) {
            min_idx(i) = max(min_idx(i), 0);
            max_idx(i) = min(max_idx(i), map_->grid_size_(i) - 1);
        }
        for (int x = min_idx(0); x <= max_idx(0); x++) {
            for (int y = min_idx(1); y <= max_idx(1); y++) {
                if (map_->isUnknown2D(x, y)) cell.unknown_num_++;
                else if (map_->isFree2D(x, y)) cell.free_num_++;
            }
        }
        if (cell.unknown_num_ >= min_unknown_num_ || cell.frontier_num_ > 0) {
            cell.is_active_ = true;
            active_cell_ids_.push_back(cell.id_);
        }
    }
}

void HierarchicalGrid::inputFrontiers(const std::vector<Eigen::Vector3d> &frontier_averages) {
    for (auto &cell : grid_cells_) {
        cell.frontier_ids_.clear(); cell.frontier_vps_.clear(); cell.frontier_num_ = 0;
    }
    for (int i = 0; i < (int)frontier_averages.size(); i++) {
        Eigen::Vector3d pos = frontier_averages[i];
        int cx = floor((pos(0) - grid_origin_(0)) / cell_size_);
        int cy = floor((pos(1) - grid_origin_(1)) / cell_size_);
        if (cx < 0 || cx >= num_cells_x_ || cy < 0 || cy >= num_cells_y_) continue;
        int id = cx * num_cells_y_ + cy;
        grid_cells_[id].frontier_ids_.push_back(i);
        grid_cells_[id].frontier_vps_.push_back(pos);
        grid_cells_[id].frontier_num_++;
        grid_cells_[id].is_active_ = true;
        if (std::find(active_cell_ids_.begin(), active_cell_ids_.end(), id) == active_cell_ids_.end()) {
            active_cell_ids_.push_back(id);
        }
    }
}

void HierarchicalGrid::calculateCostMatrix(const Eigen::Vector3d &cur_pos,
                                             Eigen::MatrixXd &cost_matrix,
                                             std::vector<Eigen::Vector3d> &cell_centers) {
    cell_centers.clear();
    cell_centers.push_back(cur_pos);
    for (int id : active_cell_ids_) { cell_centers.push_back(grid_cells_[id].center_); }

    int n = cell_centers.size();
    cost_matrix.resize(n, n);
    cost_matrix.setZero();

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            double dist = (cell_centers[i] - cell_centers[j]).norm();
            if (j > 0 && grid_cells_[active_cell_ids_[j-1]].frontier_num_ > 0) dist *= 0.8;
            if (j > 0 && grid_cells_[active_cell_ids_[j-1]].unknown_num_ < min_unknown_num_) dist *= unknown_penalty_;
            cost_matrix(i, j) = dist;
        }
    }
}

void HierarchicalGrid::solveCoveragePathGreedy(const Eigen::Vector3d &cur_pos,
                                                  std::vector<Eigen::Vector3d> &coverage_path) {
    coverage_path.clear();
    if (active_cell_ids_.empty()) return;

    std::vector<bool> visited(active_cell_ids_.size(), false);
    Eigen::Vector3d current_pos = cur_pos;

    for (int step = 0; step < (int)active_cell_ids_.size(); step++) {
        int best_idx = -1;
        double best_cost = std::numeric_limits<double>::max();
        for (int i = 0; i < (int)active_cell_ids_.size(); i++) {
            if (visited[i]) continue;
            Eigen::Vector3d center = grid_cells_[active_cell_ids_[i]].center_;
            double cost = (current_pos - center).norm();
            if (grid_cells_[active_cell_ids_[i]].frontier_num_ > 0) cost *= 0.7;
            if (cost < best_cost) { best_cost = cost; best_idx = i; }
        }
        if (best_idx < 0) break;
        visited[best_idx] = true;
        // ★ 允许unknown区域的GridCell加入路径（需要去探索）
        // 但如果目标点是occupied，跳过
        Eigen::Vector3d center = grid_cells_[active_cell_ids_[best_idx]].center_;
        Eigen::Vector3i idx;
        map_->posToIndex(center, idx);
        if (map_->isFree2D(idx(0), idx(1))) {
            coverage_path.push_back(center);
            current_pos = center;
        } else {
            // ★ occupied中心：尝试找该单元中最近的free点作为替代航点
            Eigen::Vector3d best_alt = center;
            double best_alt_dist = 1e6;
            for (int ddx = -2; ddx <= 2; ddx++) {
                for (int ddy = -2; ddy <= 2; ddy++) {
                    if (ddx == 0 && ddy == 0) continue;
                    Eigen::Vector3d alt = center + Eigen::Vector3d(ddx * map_->resolution_, ddy * map_->resolution_, 0);
                    Eigen::Vector3i alt_idx;
                    map_->posToIndex(alt, alt_idx);
                    if (map_->isInMap2D(alt_idx(0), alt_idx(1)) && map_->isFree2D(alt_idx(0), alt_idx(1))) {
                        double d = (alt - current_pos).norm();
                        if (d < best_alt_dist) {
                            best_alt_dist = d;
                            best_alt = alt;
                        }
                    }
                }
            }
            if (best_alt_dist < 1e6) {
                best_alt(2) = map_->fly_height_;
                coverage_path.push_back(best_alt);
                current_pos = best_alt;
            }
            // 如果连替代点都找不到，跳过该单元
        }
    }
}

void HierarchicalGrid::getActiveCellIds(std::vector<int> &ids) { ids = active_cell_ids_; }
Eigen::Vector3d HierarchicalGrid::getCellCenter(int id) { return grid_cells_[id].center_; }

void HierarchicalGrid::publishGrid() {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.header.frame_id = "world"; clear.ns = "hgrid";
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (int id : active_cell_ids_) {
        visualization_msgs::Marker line;
        line.header.frame_id = "world"; line.header.stamp = ros::Time::now();
        line.ns = "hgrid"; line.id = id;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.scale.x = 0.05;
        line.color.r = 0.0; line.color.g = 0.5; line.color.b = 1.0; line.color.a = 0.5;
        line.lifetime = ros::Duration(2.0);
        auto &bmin = grid_cells_[id].bbox_min_;
        auto &bmax = grid_cells_[id].bbox_max_;
        double z = map_->fly_height_;
        geometry_msgs::Point p; p.z = z;
        p.x = bmin(0); p.y = bmin(1); line.points.push_back(p);
        p.x = bmax(0); p.y = bmin(1); line.points.push_back(p);
        p.x = bmax(0); p.y = bmax(1); line.points.push_back(p);
        p.x = bmin(0); p.y = bmax(1); line.points.push_back(p);
        p.x = bmin(0); p.y = bmin(1); line.points.push_back(p);
        markers.markers.push_back(line);
    }
    grid_vis_pub_.publish(markers);
}
