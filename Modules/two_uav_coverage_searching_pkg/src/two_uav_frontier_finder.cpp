#include "two_uav_coverage_search.h"

// ============================================================
// FrontierFinder 实现 - 与原版相同
// ============================================================
void FrontierFinder::init(ros::NodeHandle &nh, CoverageMap *map) {
    map_ = map;
    nh.param("frontier_finder/cluster_min", cluster_min_, 5);
    nh.param("frontier_finder/cluster_size_xy", cluster_size_xy_, 3.0);
    nh.param("frontier_finder/candidate_rmin", candidate_rmin_, 1.2);
    nh.param("frontier_finder/candidate_rmax", candidate_rmax_, 1.6);
    nh.param("frontier_finder/candidate_dphi", candidate_dphi_, 0.524);
    nh.param("frontier_finder/candidate_rnum", candidate_rnum_, 4);
    nh.param("frontier_finder/down_sample", down_sample_, 1);
    nh.param("frontier_finder/min_candidate_dist", min_candidate_dist_, 1.0);
    nh.param("frontier_finder/min_visib_num", min_visib_num_, 3);
    nh.param("frontier_finder/min_candidate_clearance", min_candidate_clearance_, 0.2);
    nh.param("frontier_finder/min_candidate_occupied_clearance", min_candidate_occupied_clearance_, 0.55);
    nh.param("frontier_finder/min_viewpoint_frontier_dist", min_viewpoint_frontier_dist_, 0.55);
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
        min_visib_num_ = std::max(1, (int)floor(min_visib_num_ * 0.8));
        cout << YELLOW << "[FrontierFinder] Low resolution map: cluster_min="
             << cluster_min_ << ", min_visib_num=" << min_visib_num_ << TAIL << endl;
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

void FrontierFinder::refreshFrontiersFull(const Eigen::Vector3d &cur_pos) {
    ++update_generation_;
    searchFrontiersFull(cur_pos);
}

void FrontierFinder::searchFrontiers(const Eigen::Vector3d &cur_pos) {
    Eigen::Vector3d update_min, update_max;
    bool has_update = map_->getUpdatedBox(update_min, update_max, true);
    if (!has_update) return;
    ++update_generation_;

    const double margin = std::max(2.0 * map_->resolution_,
                                   map_->esdf_safe_distance_ + map_->resolution_);
    Eigen::Vector3d search_min = update_min - Eigen::Vector3d(margin, margin, margin);
    Eigen::Vector3d search_max = update_max + Eigen::Vector3d(margin, margin, margin);
    for (int k = 0; k < 3; ++k) {
        search_min(k) = std::max(search_min(k), map_->min_range_(k));
        search_max(k) = std::min(search_max(k), map_->max_range_(k));
    }

    std::vector<FrontierCluster> kept;
    kept.reserve(frontiers_.size());
    for (auto &ftr : frontiers_) {
        if (haveOverlap(ftr.box_min_, ftr.box_max_, search_min, search_max) &&
            isFrontierChanged(ftr)) {
            // Re-search the entire changed cluster, not merely the newest ray AABB.
            search_min = search_min.cwiseMin(ftr.box_min_ - Eigen::Vector3d(margin, margin, margin));
            search_max = search_max.cwiseMax(ftr.box_max_ + Eigen::Vector3d(margin, margin, margin));
            resetFrontierFlag(ftr);
        } else {
            kept.push_back(ftr);
        }
    }
    frontiers_.swap(kept);
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
    int new_count = 0;
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
                    frontiers_.push_back(ftr);
                    ++new_count;
                }
            }
        }
    }
    if (new_count > 0) splitLargeFrontiers();
    computeViewpoints(cur_pos);
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

    std::vector<Eigen::Vector3d> viewpoint_tiers[4];
    std::vector<double> yaw_tiers[4];
    std::vector<int> visib_tiers[4];
    const double clearance_levels[4] = {
        min_candidate_occupied_clearance_, 0.50, 0.45, 0.35};
    int reject_not_free = 0;
    int reject_occ = 0;
    int reject_unknown = 0;
    int reject_frontier_close = 0;
    int reject_uav_close = 0;
    int reject_vis = 0;

    Eigen::Vector3d sample_center = Eigen::Vector3d::Zero();
    int center_count = 0;
    for (auto &cell : ftr.cells) {
        if (fabs(cell(2) - map_->fly_height_) <= 0.6) {
            sample_center += cell;
            center_count++;
        }
    }
    if (center_count > 0) sample_center /= (double)center_count;
    else sample_center = ftr.average;
    sample_center(2) = map_->fly_height_;

    const auto &cells = ftr.filtered_cells.empty() ? ftr.cells : ftr.filtered_cells;
    const double dr = candidate_rnum_ > 0 ? (candidate_rmax_ - candidate_rmin_) / candidate_rnum_
                                          : map_->resolution_;
    auto addViewpointCandidate = [&](const Eigen::Vector3d &sample_pos) {
        if ((sample_pos - cur_pos).head<2>().norm() + 1e-3 < min_candidate_dist_) {
            reject_uav_close++;
            return;
        }
        if (!map_->isInMap(sample_pos)) { reject_not_free++; return; }

        Eigen::Vector3i sidx;
        map_->posToIndex(sample_pos, sidx);
        if (!map_->isFree2D(sidx(0), sidx(1))) { reject_not_free++; return; }

        if (isTooCloseToFrontierCells(sample_pos, cells)) {
            reject_frontier_close++;
            return;
        }

        bool near_unknown = isNearUnknown(sample_pos);
        if (near_unknown) reject_unknown++;

        double avg_yaw = averageYawToFrontier(sample_pos, cells);
        double center_yaw = atan2(sample_center(1) - sample_pos(1),
                                  sample_center(0) - sample_pos(0));
        const bool center_visible = !map_->isRayOccluded(sample_pos, sample_center);
        wrapYaw(avg_yaw);
        wrapYaw(center_yaw);

        std::vector<double> yaw_candidates;
        auto addYawCandidate = [&](double yaw) {
            wrapYaw(yaw);
            for (double old_yaw : yaw_candidates) {
                if (fabs(atan2(sin(yaw - old_yaw), cos(yaw - old_yaw))) < 0.08)
                    return;
            }
            yaw_candidates.push_back(yaw);
        };
        addYawCandidate(avg_yaw);
        if (center_visible) addYawCandidate(center_yaw);
        double yaw = avg_yaw;
        int visib = -1;
        auto evalYawCandidates = [&]() {
            for (double cand_yaw : yaw_candidates) {
                int cand_visib = countVisibleCells(sample_pos, cand_yaw, cells);
                if (cand_visib > visib) {
                    visib = cand_visib;
                    yaw = cand_yaw;
                }
            }
        };
        evalYawCandidates();
        if (visib < std::max(min_visib_num_ * 3, 8)) {
            const int old_size = (int)yaw_candidates.size();
            for (double dyaw : {0.35, -0.35}) {
                addYawCandidate(avg_yaw + dyaw);
                if (center_visible) addYawCandidate(center_yaw + dyaw);
            }
            for (int yi = old_size; yi < (int)yaw_candidates.size(); ++yi) {
                double cand_yaw = yaw_candidates[yi];
                int cand_visib = countVisibleCells(sample_pos, cand_yaw, cells);
                if (cand_visib > visib) {
                    visib = cand_visib;
                    yaw = cand_yaw;
                }
            }
        }
        if (visib < 0) visib = 0;
        // 所属前沿簇的直接可见性是硬门槛；未知收益不能把隔墙视点救回来。
        if (visib < min_visib_num_) {
            reject_vis++;
            return;
        }
        int unknown_gain = countVisibleUnknown(sample_pos, yaw);
        int view_score = visib + unknown_gain;
        ftr.visib_num = std::max(ftr.visib_num, view_score);
        if (near_unknown) view_score = std::max(1, view_score - 1);

        int clearance_tier = -1;
        for (int tier = 0; tier < 4; ++tier) {
            if (!isNearOccupied(sample_pos, clearance_levels[tier])) {
                clearance_tier = tier;
                break;
            }
        }
        if (clearance_tier < 0) {
            reject_occ++;
            return;
        }
        viewpoint_tiers[clearance_tier].push_back(sample_pos);
        yaw_tiers[clearance_tier].push_back(yaw);
        visib_tiers[clearance_tier].push_back(view_score);
    };

    for (double rc = candidate_rmin_; rc <= candidate_rmax_ + 1e-3; rc += std::max(dr, map_->resolution_)) {
        double dphi = rc < 1.0 ? candidate_dphi_ * 2.0 : candidate_dphi_;
        for (double phi = -M_PI; phi < M_PI; phi += dphi) {
            Eigen::Vector3d sample_pos = sample_center + rc * Eigen::Vector3d(cos(phi), sin(phi), 0);
            sample_pos(2) = map_->fly_height_;
            addViewpointCandidate(sample_pos);
        }
    }

    bool have_any_viewpoint = false;
    for (int tier = 0; tier < 4; ++tier)
        have_any_viewpoint = have_any_viewpoint || !viewpoint_tiers[tier].empty();
    if (!have_any_viewpoint) {
        Eigen::Vector3d to_frontier = sample_center - cur_pos;
        to_frontier(2) = 0.0;
        if (to_frontier.norm() > 1e-3) {
            Eigen::Vector3d dir = to_frontier.normalized();
            const double max_step = std::min(sensing_range_ * 0.85, candidate_rmax_ + 1.5);
            for (double d = std::max(min_candidate_dist_, 0.6);
                 d <= max_step + 1e-3;
                 d += std::max(map_->resolution_, 0.25)) {
                Eigen::Vector3d sample_pos = cur_pos + d * dir;
                sample_pos(2) = map_->fly_height_;
                addViewpointCandidate(sample_pos);
            }
        }
    }

    int selected_tier = -1;
    for (int tier = 0; tier < 4; ++tier) {
        if (!viewpoint_tiers[tier].empty()) {
            selected_tier = tier;
            break;
        }
    }
    if (selected_tier < 0) {
        ROS_DEBUG_THROTTLE(5.0,
            "[FrontierFinder] No viewpoint at or above 0.35m clearance: "
            "cluster=(%.2f,%.2f), rejected=%d.",
            sample_center(0), sample_center(1), reject_occ);
        return;
    }
    auto &viewpoints = viewpoint_tiers[selected_tier];
    auto &yaws = yaw_tiers[selected_tier];
    auto &visib_nums = visib_tiers[selected_tier];
    if (selected_tier > 0) {
        ROS_WARN_THROTTLE(2.0,
            "[FrontierFinder] Viewpoint clearance fallback: cluster=(%.2f,%.2f), "
            "selected_tier=%.2fm, candidates=%zu.",
            sample_center(0), sample_center(1), clearance_levels[selected_tier],
            viewpoints.size());
    }

    std::vector<int> order(viewpoints.size());
    for (int i = 0; i < (int)order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return visib_nums[a] > visib_nums[b];
    });
    for (int idx : order) {
        ROS_ASSERT_MSG(visib_nums[idx] >= min_visib_num_,
                       "Stored viewpoint must directly see its frontier cluster");
        ftr.viewpoints.push_back(viewpoints[idx]);
        ftr.viewpoint_yaws.push_back(yaws[idx]);
        ftr.viewpoint_visib_nums.push_back(visib_nums[idx]);
    }

    if (ftr.viewpoints.empty()) {
        ROS_DEBUG_THROTTLE(5.0,
            "[FrontierFinder] No viewpoint for frontier cluster: cells=%zu center=(%.2f, %.2f, %.2f), "
            "best_score=%d reject_not_free=%d reject_near_occ=%d reject_near_unknown=%d "
            "reject_near_frontier=%d reject_uav_close=%d reject_low_gain=%d",
            ftr.cells.size(), sample_center(0), sample_center(1), sample_center(2),
            ftr.visib_num, reject_not_free, reject_occ, reject_unknown,
            reject_frontier_close, reject_uav_close, reject_vis);
    }
}

bool FrontierFinder::isNearUnknown(const Eigen::Vector3d &pos) {
    const int vox_num = floor(min_candidate_clearance_ / map_->resolution_);
    for (int dx = -vox_num; dx <= vox_num; ++dx) {
        for (int dy = -vox_num; dy <= vox_num; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                Eigen::Vector3d p = pos + Eigen::Vector3d(dx * map_->resolution_,
                                                          dy * map_->resolution_,
                                                          dz * map_->resolution_);
                if (!map_->isInMap(p)) continue;
                if (map_->isUnknown(p)) return true;
            }
        }
    }
    return false;
}

bool FrontierFinder::isNearOccupied(const Eigen::Vector3d &pos, double clearance) {
    Eigen::Vector3i idx;
    map_->posToIndex(pos, idx);
    if (map_->getDistance2D(idx(0), idx(1)) + 1e-3 < clearance) return true;
    if (map_->getDistance(pos) + 1e-3 < clearance) return true;
    const int vox_num = ceil(clearance / map_->resolution_);
    for (int dx = -vox_num; dx <= vox_num; ++dx) {
        for (int dy = -vox_num; dy <= vox_num; ++dy) {
            for (int dz = -vox_num; dz <= vox_num; ++dz) {
                Eigen::Vector3d p = pos + Eigen::Vector3d(dx * map_->resolution_,
                                                          dy * map_->resolution_,
                                                          dz * map_->resolution_);
                if (!map_->isInMap(p)) continue;
                if ((p - pos).norm() <= clearance + 1e-3 && map_->isOccupied(p))
                    return true;
            }
        }
    }
    return false;
}

bool FrontierFinder::isTooCloseToFrontierCells(const Eigen::Vector3d &pos,
                                               const std::vector<Eigen::Vector3d> &cells) {
    if (cells.empty()) return false;
    const double min_dist = std::max(min_viewpoint_frontier_dist_,
                                     2.0 * map_->resolution_);
    if (min_dist <= 1e-3) return false;

    const double min_dist2 = min_dist * min_dist;
    const int stride = std::max(1, (int)cells.size() / 250);
    for (int i = 0; i < (int)cells.size(); i += stride) {
        Eigen::Vector2d d = pos.head<2>() - cells[i].head<2>();
        if (d.squaredNorm() < min_dist2) return true;
    }
    return false;
}

void FrontierFinder::wrapYaw(double &yaw) {
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
}

double FrontierFinder::averageYawToFrontier(const Eigen::Vector3d &pos,
                                            const std::vector<Eigen::Vector3d> &cells) {
    if (cells.empty()) return 0.0;
    Eigen::Vector2d visible_direction = Eigen::Vector2d::Zero();
    for (int i = 0; i < (int)cells.size(); ++i) {
        if (map_->isRayOccluded(pos, cells[i])) continue;
        Eigen::Vector3d dir = cells[i] - pos;
        dir(2) = 0.0;
        if (dir.norm() < 1e-3) continue;
        dir.normalize();
        visible_direction += dir.head<2>();
    }
    if (visible_direction.norm() < 1e-3) return 0.0;
    double avg_yaw = atan2(visible_direction(1), visible_direction(0));
    wrapYaw(avg_yaw);
    return avg_yaw;
}

int FrontierFinder::countVisibleCells(const Eigen::Vector3d &pos, double yaw,
                                       const std::vector<Eigen::Vector3d> &cells) {
    int visib = 0;
    double half_fov = sensing_fov_h_ / 2.0;
    Eigen::Vector3d view_dir(cos(yaw), sin(yaw), 0);
    const int stride = std::max(1, (int)cells.size() / 140);
    for (int ci = 0; ci < (int)cells.size(); ci += stride) {
        const auto &cell = cells[ci];
        Eigen::Vector3d dir = cell - pos;
        double dist = dir.norm();
        if (dist > sensing_range_) continue;
        dir(2) = 0;
        if (dir.norm() < 1e-3) continue;
        double angle = acos(std::min(1.0, std::max(-1.0, dir.normalized().dot(view_dir))));
        if (angle > half_fov) continue;

        if (map_->isRayOccluded(pos, cell)) continue;

        visib += stride;
    }
    return visib;
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
        const auto &vis_cells = frontiers_[i].filtered_cells.empty()
                                    ? frontiers_[i].cells
                                    : frontiers_[i].filtered_cells;

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
            double yaw = best_idx < (int)frontiers_[i].viewpoint_yaws.size()
                             ? frontiers_[i].viewpoint_yaws[best_idx]
                             : averageYawToFrontier(vp, frontiers_[i].filtered_cells.empty()
                                                          ? frontiers_[i].cells
                                                          : frontiers_[i].filtered_cells);

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
