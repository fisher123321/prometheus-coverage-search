#include "two_uav_coverage_search.h"

// ============================================================
// FrontierFinder 实现 - 与原版相同
// ============================================================
void FrontierFinder::init(ros::NodeHandle &nh, CoverageMap *map) {
    map_ = map;
    nh.param("frontier_finder/cluster_min", cluster_min_, 5);
    nh.param("frontier_finder/cluster_size_xy", cluster_size_xy_, 3.0);
    nh.param("frontier_finder/candidate_radius", candidate_radius_, 1.4);
    nh.param("frontier_finder/candidate_clearance", candidate_clearance_, 0.45);
    nh.param("frontier_finder/down_sample", down_sample_, 2);
    nh.param("frontier_finder/min_frontier_height", min_frontier_height_, 0.25);
    nh.param("frontier_finder/max_frontier_height", max_frontier_height_, 2.5);
    nh.param("frontier_finder/max_frontier_above_flight", max_frontier_above_flight_, 0.35);
    nh.param("frontier_finder/frontier_z_half_layers", frontier_z_half_layers_, 4);
    nh.param("frontier_finder/split_min_cells", split_min_cells_, 120);
    nh.param("frontier_finder/split_min_aspect_ratio", split_min_aspect_ratio_, 2.2);
    nh.param("frontier_finder/split_max_aspect_ratio", split_max_aspect_ratio_, 6.0);
    nh.param("frontier_finder/min_frontier_z_layers", min_frontier_z_layers_, 3);
    nh.param("frontier_finder/min_frontier_z_extent", min_frontier_z_extent_, 0.35);
    nh.param("frontier_finder/min_frontier_xy_cells", min_frontier_xy_cells_, 8);
    nh.param("frontier_finder/min_frontier_neighbor_ratio", min_frontier_neighbor_ratio_, 0.55);
    nh.param("frontier_finder/vis_down_sample", vis_down_sample_, 2);
    nh.param("frontier_finder/vis_max_points", vis_max_points_, 3000);
    nh.param("frontier_finder/vis_max_total_points", vis_max_total_points_, 12000);
    nh.param("coverage_search/sensing_range", sensing_range_, 5.0);
    nh.param("coverage_search/sensing_fov_h", sensing_fov_h_, 1.5708);
    nh.param("coverage_search/sensing_fov_v", sensing_fov_v_, 1.287);
    nh.param("coverage_search/camera_pitch", sensing_pitch_, 0.35);

    min_frontier_height_ = std::max(min_frontier_height_, map_->origin_(2) + map_->resolution_);
    max_frontier_height_ = std::min(max_frontier_height_,
                                    map_->fly_height_ + max_frontier_above_flight_);
    max_frontier_height_ = std::min(max_frontier_height_, map_->max_range_(2) - map_->resolution_);
    if (max_frontier_height_ <= min_frontier_height_) {
        min_frontier_height_ = std::max(map_->origin_(2) + map_->resolution_,
                                        map_->fly_height_ - 0.8);
        max_frontier_height_ = std::min(map_->max_range_(2) - map_->resolution_,
                                        map_->fly_height_ + 0.8);
    }
    if (map_->resolution_ > 0.15) {
        cluster_min_ = std::max(3, cluster_min_ / 2);
        cout << YELLOW << "[FrontierFinder] Low resolution map: cluster_min="
             << cluster_min_ << TAIL << endl;
    }

    int buf_size = map_->grid_size_(0) * map_->grid_size_(1) * map_->grid_size_(2);
    frontier_flag_.resize(buf_size, false);

    string uav_name;
    int uav_id;
    nh.param("uav_id", uav_id, 1);
    uav_name = "/uav" + std::to_string(uav_id);

    frontier_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        uav_name + "/prometheus/coverage_search/frontier_vis", 1);

    cout << GREEN << "[FrontierFinder] init. cluster_min: " << cluster_min_
         << ", sensing_range: " << sensing_range_
         << ", frontier_z=[" << min_frontier_height_ << ","
         << max_frontier_height_ << "], half_layers="
         << frontier_z_half_layers_ << TAIL << endl;
}

void FrontierFinder::searchFrontiersFull(const Eigen::Vector3d &cur_pos, bool relaxed) {
    frontiers_.clear();
    fill(frontier_flag_.begin(), frontier_flag_.end(), false);

    const int z_min = std::max(0, map_->fly_z_idx_ - std::max(1, frontier_z_half_layers_));
    const int z_max = std::min(map_->grid_size_(2) - 1,
                               map_->fly_z_idx_ + std::max(1, frontier_z_half_layers_));
    for (int x = 0; x < map_->grid_size_(0); x++) {
        for (int y = 0; y < map_->grid_size_(1); y++) {
            for (int z = z_min; z <= z_max; z++) {
                Eigen::Vector3i seed(x, y, z);
                int adr = map_->toAddress(seed);
                if (frontier_flag_[adr]) continue;
                if (!isFrontierCell(seed)) continue;

                std::vector<Eigen::Vector3d> cluster_cells;
                expandFrontier(seed, cluster_cells);
                const int min_cluster = relaxed ? 2 : cluster_min_;
                if ((int)cluster_cells.size() >= min_cluster &&
                    isValidFrontierCluster(cluster_cells, relaxed)) {
                    FrontierCluster ftr;
                    ftr.cells = cluster_cells;
                    ftr.changed_generation = update_generation_;
                    computeFrontierInfo(ftr);
                    frontiers_.push_back(ftr);
                }
            }
        }
    }

    splitLargeFrontiers();
    computeViewpoints(cur_pos);
}

void FrontierFinder::searchResidualFrontiers(const Eigen::Vector3d &cur_pos) {
    ++update_generation_;
    searchFrontiersFull(cur_pos, true);
    ROS_WARN("[CoverageSearch] Relaxed residual-frontier full scan found %zu clusters.",
             frontiers_.size());
}

void FrontierFinder::searchFrontiers(const Eigen::Vector3d &cur_pos) {
    last_aabb_search_ms_ = 0.0;
    last_viewpoint_ms_ = 0.0;
    last_local_update_box_count_ = 0;
    last_remote_update_box_count_ = 0;
    last_merged_box_count_ = 0;
    const auto search_start = std::chrono::steady_clock::now();
    Eigen::Vector3d update_min, update_max;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> update_boxes;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> local_update_boxes;
    if (map_->getUpdatedBox(update_min, update_max, true)) {
        update_boxes.emplace_back(update_min, update_max);
        local_update_boxes.emplace_back(update_min, update_max);
    }
    last_local_update_box_count_ = static_cast<int>(local_update_boxes.size());
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> remote_boxes;
    map_->getRemoteUpdatedBoxes(remote_boxes, true);
    last_remote_update_box_count_ = static_cast<int>(remote_boxes.size());
    update_boxes.insert(update_boxes.end(), remote_boxes.begin(), remote_boxes.end());
    if (update_boxes.empty()) return;

    const double margin = std::max(2.0 * map_->resolution_,
                                   map_->esdf_safe_distance_ + map_->resolution_);
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> merged_boxes;
    for (const auto &box : update_boxes) {
        Eigen::Vector3d merged_min = box.first;
        Eigen::Vector3d merged_max = box.second;
        bool merged = true;
        while (merged) {
            merged = false;
            for (auto it = merged_boxes.begin(); it != merged_boxes.end(); ++it) {
                if (!haveOverlap(merged_min - Eigen::Vector3d::Constant(margin),
                                 merged_max + Eigen::Vector3d::Constant(margin),
                                 it->first, it->second)) continue;
                merged_min = merged_min.cwiseMin(it->first);
                merged_max = merged_max.cwiseMax(it->second);
                merged_boxes.erase(it);
                merged = true;
                break;
            }
        }
        merged_boxes.emplace_back(merged_min, merged_max);
    }
    last_merged_box_count_ = static_cast<int>(merged_boxes.size());

    ++update_generation_;

    std::vector<FrontierCluster> kept;
    kept.reserve(frontiers_.size());
    for (auto &ftr : frontiers_) {
        bool affected = false;
        for (const auto &box : merged_boxes) {
            if (haveOverlap(ftr.box_min_, ftr.box_max_,
                            box.first - Eigen::Vector3d::Constant(margin),
                            box.second + Eigen::Vector3d::Constant(margin))) {
                affected = true;
                break;
            }
        }
        if (affected && isFrontierChanged(ftr)) {
            resetFrontierFlag(ftr);
        } else {
            kept.push_back(ftr);
        }
    }
    frontiers_.swap(kept);

    int new_count = 0;
    for (const auto &box : merged_boxes) {
        Eigen::Vector3d search_min = box.first - Eigen::Vector3d::Constant(margin);
        Eigen::Vector3d search_max = box.second + Eigen::Vector3d::Constant(margin);
        for (int k = 0; k < 3; ++k) {
            search_min(k) = std::max(search_min(k), map_->min_range_(k));
            search_max(k) = std::min(search_max(k), map_->max_range_(k));
        }

        Eigen::Vector3i min_id, max_id;
        map_->posToIndex(search_min, min_id);
        map_->posToIndex(search_max, max_id);
        for (int k = 0; k < 3; ++k) {
            min_id(k) = std::max(0, min_id(k));
            max_id(k) = std::min(map_->grid_size_(k) - 1, max_id(k));
        }
        const int z_min = std::max(min_id(2), map_->fly_z_idx_ - std::max(1, frontier_z_half_layers_));
        const int z_max = std::min(max_id(2), map_->fly_z_idx_ + std::max(1, frontier_z_half_layers_));
        for (int x = min_id(0); x <= max_id(0); ++x) {
            for (int y = min_id(1); y <= max_id(1); ++y) {
                for (int z = z_min; z <= z_max; ++z) {
                    Eigen::Vector3i seed(x, y, z);
                    if (frontier_flag_[map_->toAddress(seed)] || !isFrontierCell(seed)) continue;
                    std::vector<Eigen::Vector3d> cluster_cells;
                    expandFrontier(seed, cluster_cells);
                    if (static_cast<int>(cluster_cells.size()) >= cluster_min_ &&
                        isValidFrontierCluster(cluster_cells, false)) {
                        FrontierCluster ftr;
                        ftr.cells = cluster_cells;
                        ftr.changed_generation = update_generation_;
                        computeFrontierInfo(ftr);
                        for (const auto &local_box : local_update_boxes) {
                            if (haveOverlap(ftr.box_min_, ftr.box_max_,
                                            local_box.first - Eigen::Vector3d::Constant(margin),
                                            local_box.second + Eigen::Vector3d::Constant(margin))) {
                                ftr.local_discovery = true;
                                break;
                            }
                        }
                        frontiers_.push_back(ftr);
                        ++new_count;
                    }
                }
            }
        }
    }
    if (new_count > 0) splitLargeFrontiers();
    const auto search_end = std::chrono::steady_clock::now();
    computeViewpoints(cur_pos);
    const auto viewpoints_end = std::chrono::steady_clock::now();
    last_aabb_search_ms_ = std::chrono::duration<double, std::milli>(search_end - search_start).count();
    last_viewpoint_ms_ = std::chrono::duration<double, std::milli>(viewpoints_end - search_end).count();
}

bool FrontierFinder::haveOverlap(const Eigen::Vector3d &min1, const Eigen::Vector3d &max1,
                                 const Eigen::Vector3d &min2, const Eigen::Vector3d &max2) {
    for (int k = 0; k < 3; ++k) {
        if (max1(k) < min2(k) || max2(k) < min1(k)) return false;
    }
    return true;
}

bool FrontierFinder::isFrontierChanged(const FrontierCluster &ftr) {
    if (ftr.cells.empty()) return true;
    for (const auto &cell : ftr.cells) {
        Eigen::Vector3i idx;
        map_->posToIndex(cell, idx);
        if (!isFrontierCell(idx)) return true;
    }
    return false;
}

bool FrontierFinder::isFrontierCellsCovered(const std::vector<Eigen::Vector3d> &cells,
                                            double changed_fraction) {
    if (cells.empty()) return false;
    const int threshold = std::max(1, static_cast<int>(std::ceil(
        changed_fraction * static_cast<double>(cells.size()))));
    int changed = 0;
    for (const auto &cell : cells) {
        Eigen::Vector3i idx;
        map_->posToIndex(cell, idx);
        if (!isFrontierCell(idx) && ++changed >= threshold) return true;
    }
    return false;
}

void FrontierFinder::resetFrontierFlag(const FrontierCluster &ftr) {
    for (const auto &cell : ftr.cells) {
        Eigen::Vector3i idx;
        map_->posToIndex(cell, idx);
        if (!map_->isInMapIndex(idx)) continue;
        frontier_flag_[map_->toAddress(idx)] = false;
    }
}

void FrontierFinder::expandFrontier(const Eigen::Vector3i &seed,
                                     std::vector<Eigen::Vector3d> &cluster_cells) {
    std::queue<Eigen::Vector3i> cell_queue;
    cell_queue.push(seed);
    frontier_flag_[map_->toAddress(seed)] = true;

    Eigen::Vector3d pos;
    map_->indexToPos(seed, pos);
    cluster_cells.push_back(pos);

    while (!cell_queue.empty()) {
        Eigen::Vector3i cur = cell_queue.front();
        cell_queue.pop();

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    Eigen::Vector3i nbr = cur + Eigen::Vector3i(dx, dy, dz);
                    if (!map_->isInMapIndex(nbr)) continue;

                    int adr = map_->toAddress(nbr);
                    if (frontier_flag_[adr]) continue;
                    if (!isFrontierCell(nbr)) continue;

                    frontier_flag_[adr] = true;
                    Eigen::Vector3d n_pos;
                    map_->indexToPos(nbr, n_pos);
                    cluster_cells.push_back(n_pos);
                    cell_queue.push(nbr);
                }
            }
        }
    }
}

bool FrontierFinder::isFrontierCell(const Eigen::Vector3i &idx) {
    if (!map_->isFreeIndex(idx)) return false;

    Eigen::Vector3d pos;
    map_->indexToPos(idx, pos);
    if (map_->isInDynamicPeerVolume(pos)) return false;
    if (pos(2) < min_frontier_height_ || pos(2) > max_frontier_height_) return false;

    // 只在无人机可飞行高度带附近找前沿，避免地面/顶部未观测层把整片已探索 free 区域染成前沿。
    if (std::abs(idx(2) - map_->fly_z_idx_) > std::max(1, frontier_z_half_layers_)) return false;

    // FALCON style: known free voxel with a direct 6-neighbor unknown voxel.
    // Height filters above keep floor/ceiling remnants from driving exploration.
    static const int dirs[6][3] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };
    for (const auto &d : dirs) {
        Eigen::Vector3i nbr = idx + Eigen::Vector3i(d[0], d[1], d[2]);
        if (map_->isUnknownIndex(nbr)) return true;
    }
    return false;
}

bool FrontierFinder::isValidFrontierCluster(const std::vector<Eigen::Vector3d> &cells,
                                            bool relaxed) {
    if ((int)cells.size() < (relaxed ? 2 : cluster_min_)) return false;

    std::set<int> z_layers;
    std::set<int> xy_cells;
    std::set<int> addr_set;
    std::vector<Eigen::Vector3i> indices;
    indices.reserve(cells.size());
    double min_z = 1e9;
    double max_z = -1e9;

    for (const auto &cell : cells) {
        Eigen::Vector3i idx;
        map_->posToIndex(cell, idx);
        if (!map_->isInMapIndex(idx)) continue;
        indices.push_back(idx);
        z_layers.insert(idx(2));
        xy_cells.insert(idx(0) * map_->grid_size_(1) + idx(1));
        addr_set.insert(map_->toAddress(idx));
        min_z = std::min(min_z, cell(2));
        max_z = std::max(max_z, cell(2));
    }

    if (max_z > max_frontier_height_ + 1e-3) return false;
    if (relaxed) return (int)xy_cells.size() >= 2;
    if ((int)z_layers.size() < min_frontier_z_layers_) return false;
    if (max_z - min_z + map_->resolution_ < min_frontier_z_extent_) return false;
    if ((int)xy_cells.size() < min_frontier_xy_cells_) return false;

    static const int dirs[6][3] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };
    int supported = 0;
    for (const auto &idx : indices) {
        int nbr_count = 0;
        for (const auto &d : dirs) {
            Eigen::Vector3i nbr = idx + Eigen::Vector3i(d[0], d[1], d[2]);
            if (!map_->isInMapIndex(nbr)) continue;
            if (addr_set.find(map_->toAddress(nbr)) != addr_set.end()) nbr_count++;
        }
        if (nbr_count >= 1) supported++;
    }

    double support_ratio = indices.empty() ? 0.0 : (double)supported / (double)indices.size();
    return support_ratio >= min_frontier_neighbor_ratio_;
}

void FrontierFinder::splitLargeFrontiers() {
    std::vector<FrontierCluster> pending = frontiers_;
    std::vector<FrontierCluster> result;

    while (!pending.empty()) {
        FrontierCluster ftr = pending.back();
        pending.pop_back();

        const auto &split_cells = ftr.filtered_cells.empty() ? ftr.cells : ftr.filtered_cells;
        bool need_split = false;
        for (auto &cell : split_cells) {
            if ((cell.head<2>() - ftr.average.head<2>()).norm() > cluster_size_xy_) {
                need_split = true; break;
            }
        }
        if (!need_split) {
            result.push_back(ftr);
            continue;
        }

        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (auto &cell : split_cells) {
            Eigen::Vector2d diff = cell.head<2>() - ftr.average.head<2>();
            cov += diff * diff.transpose();
        }
        cov /= std::max(1, (int)split_cells.size());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
        double lambda_small = std::max(1e-6, es.eigenvalues()(0));
        double lambda_large = std::max(1e-6, es.eigenvalues()(1));
        double aspect_ratio = std::sqrt(lambda_large / lambda_small);
        if ((int)ftr.cells.size() < split_min_cells_ ||
            aspect_ratio < split_min_aspect_ratio_ ||
            aspect_ratio > split_max_aspect_ratio_) {
            result.push_back(ftr);
            continue;
        }
        Eigen::Vector2d pc = es.eigenvectors().col(1);

        FrontierCluster f1, f2;
        f1.changed_generation = ftr.changed_generation;
        f2.changed_generation = ftr.changed_generation;
        f1.local_discovery = ftr.local_discovery;
        f2.local_discovery = ftr.local_discovery;
        for (auto &cell : ftr.cells) {
            if ((cell.head<2>() - ftr.average.head<2>()).dot(pc) >= 0)
                f1.cells.push_back(cell);
            else
                f2.cells.push_back(cell);
        }
        if ((int)f1.cells.size() < std::max(cluster_min_ * 2, 20) ||
            (int)f2.cells.size() < std::max(cluster_min_ * 2, 20)) {
            result.push_back(ftr);
            continue;
        }
        computeFrontierInfo(f1);
        computeFrontierInfo(f2);
        pending.push_back(f1);
        pending.push_back(f2);
        ROS_DEBUG_THROTTLE(2.0,
            "[FrontierFinder] PCA split frontier: cells=%zu -> %zu + %zu, cluster_size_xy=%.2f",
            ftr.cells.size(), f1.cells.size(), f2.cells.size(), cluster_size_xy_);
    }
    frontiers_ = result;
}

void FrontierFinder::computeFrontierInfo(FrontierCluster &ftr) {
    ftr.viewpoints.clear();
    ftr.viewpoint_yaws.clear();
    ftr.viewpoint_visib_nums.clear();
    ftr.visib_num = 0;
    ftr.average.setZero();
    ftr.box_min_ = ftr.cells.front();
    ftr.box_max_ = ftr.cells.front();
    for (auto &cell : ftr.cells) {
        ftr.average += cell;
        for (int i = 0; i < 3; i++) {
            ftr.box_min_(i) = min(ftr.box_min_(i), cell(i));
            ftr.box_max_(i) = max(ftr.box_max_(i), cell(i));
        }
    }
    ftr.average /= (double)ftr.cells.size();
    downsample(ftr.cells, ftr.filtered_cells);
}

void FrontierFinder::computeViewpoints(const Eigen::Vector3d &cur_pos) {
    for (auto &ftr : frontiers_) {
        if (ftr.viewpoints.empty()) sampleViewpoints(ftr, cur_pos);
    }
}

bool FrontierFinder::isViewpointVisible(const FrontierCluster &ftr,
                                        const Eigen::Vector3d &pos, double yaw) {
    Eigen::Vector3d target;
    if (!getRepresentativeFrontierCell(ftr, target)) return false;
    const double expected_yaw = std::atan2(target(1) - pos(1), target(0) - pos(0));
    const double yaw_error = std::atan2(std::sin(yaw - expected_yaw),
                                        std::cos(yaw - expected_yaw));
    return std::fabs(yaw_error) < 0.05 && !map_->isRayOccluded(pos, target);
}

void FrontierFinder::refreshViewpoints(FrontierCluster &ftr,
                                       const Eigen::Vector3d &cur_pos) {
    ftr.viewpoints.clear();
    ftr.viewpoint_yaws.clear();
    ftr.viewpoint_visib_nums.clear();
    sampleViewpoints(ftr, cur_pos);
}

void FrontierFinder::downsample(const std::vector<Eigen::Vector3d> &cluster_in,
                                std::vector<Eigen::Vector3d> &cluster_out) {
    cluster_out.clear();
    if (cluster_in.empty()) return;
    if (down_sample_ <= 1) {
        cluster_out = cluster_in;
        return;
    }

    std::set<int> used;
    const double coarse_res = map_->resolution_ * down_sample_;
    for (auto &cell : cluster_in) {
        Eigen::Vector3i coarse;
        for (int i = 0; i < 3; ++i) {
            coarse(i) = floor((cell(i) - map_->origin_(i)) / coarse_res);
        }
        int key = coarse(0) * 73856093 ^ coarse(1) * 19349663 ^ coarse(2) * 83492791;
        if (used.insert(key).second) cluster_out.push_back(cell);
    }
    if (cluster_out.empty()) cluster_out = cluster_in;
}

void FrontierFinder::sampleViewpoints(FrontierCluster &ftr, const Eigen::Vector3d &cur_pos) {
    ftr.viewpoints.clear();
    ftr.viewpoint_yaws.clear();
    ftr.viewpoint_visib_nums.clear();
    ftr.visib_num = 0;

    Eigen::Vector3d target;
    if (!getRepresentativeFrontierCell(ftr, target)) {
        ROS_DEBUG_THROTTLE(5.0,
            "[FrontierFinder] No free frontier representative at flight height; cells=%zu.",
            ftr.cells.size());
        return;
    }

    struct ViewpointCandidate {
        Eigen::Vector3d position;
        double yaw = 0.0;
        double clearance = 0.0;
        int information_gain = 0;
    };
    std::vector<ViewpointCandidate> safe_candidates;
    safe_candidates.reserve(36);
    int reject_not_free = 0;
    int reject_occupied = 0;
    int reject_occluded = 0;

    // The full cluster average is stable; the representative target uses
    // downsampled frontier cells, with a complete-cluster fallback.
    Eigen::Vector3d center = ftr.average;
    center(2) = map_->fly_height_;
    std::set<int> sampled_cells;
    for (int sample_id = 0; sample_id < 36; ++sample_id) {
        const double phi = -M_PI + sample_id * (M_PI / 18.0);
        Eigen::Vector3d pos = center + candidate_radius_ *
            Eigen::Vector3d(std::cos(phi), std::sin(phi), 0.0);
        pos(2) = map_->fly_height_;
        if (!map_->isInMap(pos)) {
            ++reject_not_free;
            continue;
        }

        Eigen::Vector3i idx;
        map_->posToIndex(pos, idx);
        Eigen::Vector3d voxel_center;
        map_->indexToPos(idx, voxel_center);
        pos(0) = voxel_center(0);
        pos(1) = voxel_center(1);
        if (!sampled_cells.insert(map_->toAddress(idx)).second) continue;

        if (!map_->isFree(pos)) {
            ++reject_not_free;
            continue;
        }

        const double clearance = map_->getDistance(pos);
        if (clearance + 1e-3 < candidate_clearance_) {
            ++reject_occupied;
            continue;
        }

        ViewpointCandidate candidate;
        candidate.position = pos;
        candidate.yaw = std::atan2(target(1) - pos(1), target(0) - pos(0));
        candidate.clearance = clearance;
        safe_candidates.push_back(candidate);
    }

    std::vector<ViewpointCandidate> visible_candidates;
    visible_candidates.reserve(safe_candidates.size());
    for (const auto &candidate : safe_candidates) {
        // CoverageMap only treats occupied voxels as ray blockers; unknown is observable.
        if (map_->isRayOccluded(candidate.position, target)) {
            ++reject_occluded;
            continue;
        }
        visible_candidates.push_back(candidate);
    }

    std::sort(visible_candidates.begin(), visible_candidates.end(),
              [&](const ViewpointCandidate &a, const ViewpointCandidate &b) {
                  if (std::fabs(a.clearance - b.clearance) > 1e-3)
                      return a.clearance > b.clearance;
                  return (a.position - cur_pos).squaredNorm() <
                         (b.position - cur_pos).squaredNorm();
              });
    if (visible_candidates.size() > 10) visible_candidates.resize(10);

    for (auto &candidate : visible_candidates) {
        candidate.information_gain = countVisibleUnknown(candidate.position, candidate.yaw);
        ftr.visib_num = std::max(ftr.visib_num, candidate.information_gain);
    }

    std::sort(visible_candidates.begin(), visible_candidates.end(),
              [&](const ViewpointCandidate &a, const ViewpointCandidate &b) {
                  if (a.information_gain != b.information_gain)
                      return a.information_gain > b.information_gain;
                  if (std::fabs(a.clearance - b.clearance) > 1e-3)
                      return a.clearance > b.clearance;
                  return (a.position - cur_pos).squaredNorm() <
                         (b.position - cur_pos).squaredNorm();
              });

    const size_t stored_num = std::min<size_t>(2, visible_candidates.size());
    for (size_t i = 0; i < stored_num; ++i) {
        const auto &candidate = visible_candidates[i];
        ROS_ASSERT_MSG(map_->isFree(candidate.position) &&
                           !map_->isRayOccluded(candidate.position, target),
                       "Stored viewpoint must stay free with an unobstructed center ray");
        ftr.viewpoints.push_back(candidate.position);
        ftr.viewpoint_yaws.push_back(candidate.yaw);
        ftr.viewpoint_visib_nums.push_back(std::max(1, candidate.information_gain));
    }

    if (ftr.viewpoints.empty()) {
        ROS_DEBUG_THROTTLE(5.0,
            "[FrontierFinder] No fixed-ring viewpoint: cells=%zu center=(%.2f, %.2f), "
            "free_rejected=%d occupied_rejected=%d center_ray_rejected=%d.",
            ftr.cells.size(), center(0), center(1),
            reject_not_free, reject_occupied, reject_occluded);
    }
}

bool FrontierFinder::getRepresentativeFrontierCell(const FrontierCluster &ftr,
                                                    Eigen::Vector3d &target) {
    if (ftr.cells.empty()) return false;

    const Eigen::Vector2d center = ftr.average.head<2>();
    double best_dist2 = std::numeric_limits<double>::infinity();
    auto find_target = [&](const std::vector<Eigen::Vector3d> &cells) {
        bool found = false;
        for (const auto &cell : cells) {
            Eigen::Vector3i idx;
            map_->posToIndex(cell, idx);
            if (idx(2) != map_->fly_z_idx_) continue;
            Eigen::Vector3d projected = cell;
            projected(2) = map_->fly_height_;
            if (!map_->isFree(projected)) continue;
            const double dist2 = (projected.head<2>() - center).squaredNorm();
            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                target = projected;
                found = true;
            }
        }
        return found;
    };

    const auto &downsampled = ftr.filtered_cells.empty() ? ftr.cells : ftr.filtered_cells;
    if (find_target(downsampled)) return true;
    return downsampled.data() == ftr.cells.data() ? false : find_target(ftr.cells);
}

int FrontierFinder::countVisibleUnknown(const Eigen::Vector3d &pos, double yaw) {
    std::set<int> unknown_addrs;
    const double half_h = sensing_fov_h_ * 0.5;
    const double half_v = sensing_fov_v_ * 0.5;
    const int yaw_samples = 5;
    const int pitch_samples = 3;

    for (int iy = 0; iy < yaw_samples; ++iy) {
        double ay = -half_h + 2.0 * half_h * iy / std::max(1, yaw_samples - 1);
        for (int ip = 0; ip < pitch_samples; ++ip) {
            double ap = -half_v + 2.0 * half_v * ip / std::max(1, pitch_samples - 1);
            double elev = ap - sensing_pitch_;
            Eigen::Vector3d dir(cos(yaw + ay) * cos(elev),
                                sin(yaw + ay) * cos(elev),
                                sin(elev));
            map_->collectVisibleUnknown(pos, pos + sensing_range_ * dir,
                                        unknown_addrs);
        }
    }
    return (int)unknown_addrs.size();
}

void FrontierFinder::getFrontierAverages(std::vector<Eigen::Vector3d> &averages) {
    averages.clear();
    for (auto &ftr : frontiers_) averages.push_back(ftr.average);
}

void FrontierFinder::getFrontierClusters(std::vector<std::vector<Eigen::Vector3d>> &clusters) {
    clusters.clear();
    for (auto &ftr : frontiers_) clusters.push_back(ftr.cells);
}

int FrontierFinder::getFrontierCount() { return frontiers_.size(); }

void FrontierFinder::getTopViewpoints(const Eigen::Vector3d &cur_pos,
                                        double cur_yaw,
                                        std::vector<Eigen::Vector3d> &points,
                                        std::vector<double> &yaws) {
    points.clear(); yaws.clear();
    // 深度相机下每簇选"可见前沿/未知收益高 + 距离近"的代表视点。
    // 最终目标由上层使用测地路径、当前速度和偏航时间统一排序。
    for (auto &ftr : frontiers_) {
        if (ftr.viewpoints.empty()) continue;
        int best_idx = 0;
        double best_score = -1e9;
        for (int i = 0; i < (int)ftr.viewpoints.size(); i++) {
            double visib = (double)ftr.viewpoint_visib_nums[i];
            double dist = (cur_pos - ftr.viewpoints[i]).norm();
            double score = visib - 0.2 * dist;
            if (score > best_score) {
                best_score = score; best_idx = i;
            }
        }
        points.push_back(ftr.viewpoints[best_idx]);
        yaws.push_back(ftr.viewpoint_yaws[best_idx]);
    }
}

void FrontierFinder::publishFrontiers() {
    if (frontier_vis_pub_.getNumSubscribers() < 1 && last_frontier_marker_count_ == 0) return;

    visualization_msgs::MarkerArray markers;

    auto hsvToRgb = [](double h, double s, double v) {
        h = h - std::floor(h);
        double c = v * s;
        double hp = h * 6.0;
        double x = c * (1.0 - std::fabs(std::fmod(hp, 2.0) - 1.0));
        double r = 0.0, g = 0.0, b = 0.0;
        if (hp < 1.0) { r = c; g = x; }
        else if (hp < 2.0) { r = x; g = c; }
        else if (hp < 3.0) { g = c; b = x; }
        else if (hp < 4.0) { g = x; b = c; }
        else if (hp < 5.0) { r = x; b = c; }
        else { r = c; b = x; }
        double m = v - c;
        return std::tuple<double, double, double>(r + m, g + m, b + m);
    };

    int total_added = 0;
    const int total_cap = std::max(vis_max_points_, vis_max_total_points_);
    for (int i = 0; i < (int)frontiers_.size(); i++) {
        auto c = hsvToRgb(std::fmod(0.02 + i * 0.61803398875, 1.0), 0.92, 0.95);
        // RViz shows every frontier voxel; filtered_cells remains view-generation only.
        const auto &vis_cells = frontiers_[i].cells;

        visualization_msgs::Marker cell_marker;
        cell_marker.header.frame_id = "world";
        cell_marker.header.stamp = ros::Time::now();
        cell_marker.ns = "frontier_cells";
        cell_marker.id = i;
        cell_marker.type = visualization_msgs::Marker::CUBE_LIST;
        cell_marker.action = visualization_msgs::Marker::ADD;
        cell_marker.scale.x = map_->resolution_ * 0.85;
        cell_marker.scale.y = map_->resolution_ * 0.85;
        cell_marker.scale.z = map_->resolution_ * 0.85;
        cell_marker.color.r = std::get<0>(c); cell_marker.color.g = std::get<1>(c);
        cell_marker.color.b = std::get<2>(c); cell_marker.color.a = 0.85;
        cell_marker.lifetime = ros::Duration(0.0);

        int checked = 0;
        int added = 0;
        const int stride = std::max(1, vis_down_sample_);
        for (auto &cell : vis_cells) {
            Eigen::Vector3i idx;
            map_->posToIndex(cell, idx);
            if (!isFrontierCell(idx)) continue;
            if ((checked++ % stride) != 0) continue;
            if (added >= vis_max_points_) break;
            if (total_added >= total_cap) break;
            geometry_msgs::Point p;
            p.x = cell(0); p.y = cell(1); p.z = cell(2);
            cell_marker.points.push_back(p);
            added++;
            total_added++;
        }

        if (!cell_marker.points.empty()) {
            markers.markers.push_back(cell_marker);
        } else {
            cell_marker.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(cell_marker);
        }

        if (!frontiers_[i].viewpoints.empty()) {
            int best_idx = 0;
            if (!frontiers_[i].viewpoint_visib_nums.empty()) {
                best_idx = (int)(std::max_element(frontiers_[i].viewpoint_visib_nums.begin(),
                                                   frontiers_[i].viewpoint_visib_nums.end()) -
                                  frontiers_[i].viewpoint_visib_nums.begin());
            }
            best_idx = std::max(0, std::min(best_idx, (int)frontiers_[i].viewpoints.size() - 1));
            const Eigen::Vector3d &vp = frontiers_[i].viewpoints[best_idx];
            double yaw = 0.0;
            if (best_idx < (int)frontiers_[i].viewpoint_yaws.size()) {
                yaw = frontiers_[i].viewpoint_yaws[best_idx];
            } else {
                Eigen::Vector3d target;
                if (getRepresentativeFrontierCell(frontiers_[i], target))
                    yaw = std::atan2(target(1) - vp(1), target(0) - vp(0));
            }

            visualization_msgs::Marker vp_marker;
            vp_marker.header.frame_id = "world";
            vp_marker.header.stamp = ros::Time::now();
            vp_marker.ns = "frontier_best_viewpoint";
            vp_marker.id = i;
            vp_marker.type = visualization_msgs::Marker::SPHERE;
            vp_marker.action = visualization_msgs::Marker::ADD;
            vp_marker.pose.position.x = vp(0);
            vp_marker.pose.position.y = vp(1);
            vp_marker.pose.position.z = vp(2);
            vp_marker.pose.orientation.w = 1.0;
            vp_marker.scale.x = 0.14;
            vp_marker.scale.y = 0.14;
            vp_marker.scale.z = 0.14;
            vp_marker.color.r = std::get<0>(c);
            vp_marker.color.g = std::get<1>(c);
            vp_marker.color.b = std::get<2>(c);
            vp_marker.color.a = 1.0;
            vp_marker.lifetime = ros::Duration(0.0);
            markers.markers.push_back(vp_marker);

            visualization_msgs::Marker yaw_marker;
            yaw_marker.header.frame_id = "world";
            yaw_marker.header.stamp = ros::Time::now();
            yaw_marker.ns = "frontier_best_view_yaw";
            yaw_marker.id = i;
            yaw_marker.type = visualization_msgs::Marker::ARROW;
            yaw_marker.action = visualization_msgs::Marker::ADD;
            yaw_marker.scale.x = 0.035;
            yaw_marker.scale.y = 0.09;
            yaw_marker.scale.z = 0.12;
            yaw_marker.color.r = std::get<0>(c);
            yaw_marker.color.g = std::get<1>(c);
            yaw_marker.color.b = std::get<2>(c);
            yaw_marker.color.a = 1.0;
            yaw_marker.lifetime = ros::Duration(0.0);
            geometry_msgs::Point p0, p1;
            p0.x = vp(0); p0.y = vp(1); p0.z = vp(2);
            p1.x = vp(0) + 0.45 * cos(yaw);
            p1.y = vp(1) + 0.45 * sin(yaw);
            p1.z = vp(2);
            yaw_marker.points.push_back(p0);
            yaw_marker.points.push_back(p1);
            markers.markers.push_back(yaw_marker);
        } else {
            visualization_msgs::Marker del_vp;
            del_vp.header.frame_id = "world";
            del_vp.header.stamp = ros::Time::now();
            del_vp.ns = "frontier_best_viewpoint";
            del_vp.id = i;
            del_vp.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(del_vp);

            visualization_msgs::Marker del_yaw = del_vp;
            del_yaw.ns = "frontier_best_view_yaw";
            markers.markers.push_back(del_yaw);
        }

    }

    for (int i = (int)frontiers_.size(); i < last_frontier_marker_count_; i++) {
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = "world";
        delete_marker.header.stamp = ros::Time::now();
        delete_marker.ns = "frontier_cells";
        delete_marker.id = i;
        delete_marker.action = visualization_msgs::Marker::DELETE;
        markers.markers.push_back(delete_marker);
    }
    for (int i = (int)frontiers_.size(); i < last_viewpoint_marker_count_; i++) {
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = "world";
        delete_marker.header.stamp = ros::Time::now();
        delete_marker.ns = "frontier_best_viewpoint";
        delete_marker.id = i;
        delete_marker.action = visualization_msgs::Marker::DELETE;
        markers.markers.push_back(delete_marker);

        visualization_msgs::Marker delete_yaw = delete_marker;
        delete_yaw.ns = "frontier_best_view_yaw";
        markers.markers.push_back(delete_yaw);
    }
    last_frontier_marker_count_ = (int)frontiers_.size();
    last_viewpoint_marker_count_ = (int)frontiers_.size();
    frontier_vis_pub_.publish(markers);
}
