#include "two_uav_coverage_search.h"

bool CoverageSearchManager::isStuck() {
    // ★ 如果已经非常接近目标，不算卡住（正常到达）
    if (has_goal_) {
        double dist_to_goal = (uav_pos_.head<2>() - current_goal_.head<2>()).norm();
        if (dist_to_goal < goal_reach_dist_ * 1.5) return false;
    }

    double move_dist = (uav_pos_ - last_pos_).norm();
    if (move_dist > 0.3) {
        last_pos_ = uav_pos_;
        last_move_time_ = ros::Time::now();
        return false;
    }
    double time_elapsed = (ros::Time::now() - last_move_time_).toSec();
    if (time_elapsed > 10.0) {
        last_move_time_ = ros::Time::now();
        return true;
    }
    return false;
}

bool CoverageSearchManager::isPositionSafe(const Eigen::Vector3d &pos) {
    Eigen::Vector3i idx;
    coverage_map_.posToIndex(pos, idx);
    return coverage_map_.isFree2D(idx(0), idx(1));
}

bool CoverageSearchManager::isNearFrontier(const Eigen::Vector3d &pos) {
    Eigen::Vector3i idx;
    coverage_map_.posToIndex(pos, idx);
    int check_range = (int)ceil(std::min(1.2, sensing_range_ * 0.4) / coverage_map_.resolution_);
    for (int dx = -check_range; dx <= check_range; dx++) {
        for (int dy = -check_range; dy <= check_range; dy++) {
            for (int dz = -check_range; dz <= check_range; dz++) {
                if (dx * dx + dy * dy + dz * dz > check_range * check_range) continue;
                Eigen::Vector3i nidx = idx + Eigen::Vector3i(dx, dy, dz);
                if (coverage_map_.isUnknownIndex(nidx)) return true;
            }
        }
    }
    return false;
}

bool CoverageSearchManager::tryPrepareRollingHandoff() {
    if (!rolling_snapshot_mode_) {
        if (!rolling_worker_running_ && rolling_worker_.joinable()) {
            rolling_worker_.join();
        }
        {
            std::lock_guard<std::mutex> lock(rolling_result_mutex_);
            if (completed_pending_traj_.frontier_generation != 0) {
                if (completed_pending_traj_.ready &&
                    completed_pending_traj_.source_generation == rolling_generation_ &&
                    has_traj_ && has_goal_) {
                    pending_traj_ = std::move(completed_pending_traj_);
                    completed_pending_traj_ = PendingTrajectory();
                    ROS_INFO("[CoverageSearch] Rolling successor accepted from frontier generation %llu.",
                             (unsigned long long)pending_traj_.frontier_generation);
                    return true;
                }
                if (completed_pending_traj_.ready) {
                    ROS_WARN("[CoverageSearch] Discard stale rolling result: source=%llu, active=%llu.",
                             (unsigned long long)completed_pending_traj_.source_generation,
                             (unsigned long long)rolling_generation_);
                } else {
                    ROS_INFO("[CoverageSearch] No safe successor in frontier generation %llu; "
                             "continue old trajectory and wait for a newer generation.",
                             (unsigned long long)completed_pending_traj_.frontier_generation);
                }
                completed_pending_traj_ = PendingTrajectory();
            }
        }

        if (!has_traj_ || !has_goal_ || pending_traj_.ready ||
            rolling_worker_running_ || traj_points_.size() < 3 ||
            !active_time_spline_.valid) {
            return false;
        }
        const double progress = std::max(0.0, std::min(1.0,
            (ros::Time::now() - traj_start_time_).toSec() /
            std::max(1e-3, active_time_spline_.duration)));
        if (progress < rolling_preplan_progress_) return false;

        const uint64_t frontier_generation = frontier_finder_.update_generation_;
        if (frontier_generation == 0 ||
            frontier_generation <= rolling_last_attempt_frontier_generation_) return false;

        // 工作线程只接触这一时刻的地图、前沿和活动轨迹副本；主线程继续发布旧轨迹。
        std::shared_ptr<CoverageSearchManager> planner(new CoverageSearchManager());
        planner->coverage_map_ = coverage_map_;
        planner->frontier_finder_ = frontier_finder_;
        planner->astar2d_.init(&planner->coverage_map_);
        planner->uav_pos_ = uav_pos_;
        planner->uav_vel_ = uav_vel_;
        planner->uav_yaw_ = uav_yaw_;
        planner->fly_height_ = fly_height_;
        planner->sensing_range_ = sensing_range_;
        planner->max_vel_ = max_vel_;
        planner->max_acc_ = max_acc_;
        planner->max_yaw_rate_ = max_yaw_rate_;
        planner->goal_reach_dist_ = goal_reach_dist_;
        planner->sim_mode_ = sim_mode_;
        planner->current_goal_ = current_goal_;
        planner->current_goal_yaw_ = current_goal_yaw_;
        planner->has_goal_ = has_goal_;
        planner->has_traj_ = has_traj_;
        planner->committed_heading_ = committed_heading_;
        planner->has_committed_heading_ = has_committed_heading_;
        planner->failed_goal_ = failed_goal_;
        planner->failed_goal_time_ = failed_goal_time_;
        planner->has_failed_goal_ = has_failed_goal_;
        planner->astar_path_ = astar_path_;
        planner->astar_path_idx_ = astar_path_idx_;
        planner->traj_points_ = traj_points_;
        planner->traj_vels_ = traj_vels_;
        planner->traj_accs_ = traj_accs_;
        planner->traj_yaws_ = traj_yaws_;
        planner->active_time_spline_ = active_time_spline_;
        planner->traj_idx_ = traj_idx_;
        planner->traj_dt_ = traj_dt_;
        planner->traj_start_time_ = traj_start_time_;
        planner->traj_point_reach_time_ = traj_point_reach_time_;
        planner->cmd_yaw_smoothed_ = cmd_yaw_smoothed_;
        planner->cmd_yaw_inited_ = cmd_yaw_inited_;
        planner->frontier_targets_ = frontier_targets_;
        planner->frontier_target_yaws_ = frontier_target_yaws_;
        planner->frontier_target_idx_ = frontier_target_idx_;
        planner->start_time_ = start_time_;
        planner->goal_commit_time_ = goal_commit_time_;
        planner->same_goal_replan_count_ = same_goal_replan_count_;
        planner->traj_step_size_ = traj_step_size_;
        planner->traj_advance_dist_ = traj_advance_dist_;
        planner->traj_point_dwell_timeout_ = traj_point_dwell_timeout_;
        planner->traj_cut_clearance_ = traj_cut_clearance_;
        planner->rolling_preplan_progress_ = rolling_preplan_progress_;
        planner->rolling_handoff_goal_dist_ = rolling_handoff_goal_dist_;
        planner->rolling_prepare_in_progress_ = false;
        planner->rolling_snapshot_mode_ = true;

        rolling_last_attempt_frontier_generation_ = frontier_generation;
        rolling_worker_running_ = true;
        const uint64_t source_generation = rolling_generation_;
        try {
            rolling_worker_ = std::thread([this, planner, source_generation,
                                           frontier_generation]() {
                planner->tryPrepareRollingHandoff();
                PendingTrajectory result = std::move(planner->pending_traj_);
                result.source_generation = source_generation;
                result.frontier_generation = frontier_generation;
                {
                    std::lock_guard<std::mutex> lock(rolling_result_mutex_);
                    completed_pending_traj_ = std::move(result);
                }
                rolling_worker_running_ = false;
            });
            ROS_INFO("[CoverageSearch] Rolling monitor: progress=%.1f%%, frontier_generation=%llu, "
                     "trajectory_generation=%llu.",
                     100.0 * progress, (unsigned long long)frontier_generation,
                     (unsigned long long)source_generation);
        } catch (const std::exception &e) {
            rolling_worker_running_ = false;
            ROS_ERROR("[CoverageSearch] Failed to start rolling worker: %s", e.what());
        }
        return false;
    }

    if (!has_traj_ || !has_goal_ || pending_traj_.ready ||
        !active_time_spline_.valid || traj_points_.size() < 3) {
        return false;
    }

    const int n = (int)traj_points_.size();
    const double progress = std::max(0.0, std::min(1.0,
        (ros::Time::now() - traj_start_time_).toSec() /
        std::max(1e-3, active_time_spline_.duration)));
    if (progress < rolling_preplan_progress_) return false;

    // 在旧轨迹85%~95%窗口内，用De Boor解析值选择已经完成观测和偏航、
    // 但仍保留非零速度的交接状态。
    int handoff_idx = -1;
    double handoff_time = -1.0;
    Eigen::Vector3d handoff_pos, handoff_vel, handoff_acc;
    double handoff_yaw = 0.0, handoff_yaw_rate = 0.0, handoff_yaw_acc = 0.0;
    double best_handoff_score = 1e9;
    for (double fraction = 0.85; fraction <= 0.9501; fraction += 0.01) {
        if (fraction <= progress + 0.02) continue;
        const double t = fraction * active_time_spline_.duration;
        Eigen::Vector3d p, v, a;
        double yaw = 0.0, yaw_rate = 0.0, yaw_acc = 0.0;
        if (!evaluateTimeBspline(active_time_spline_, t, p, v, a,
                                 yaw, yaw_rate, yaw_acc)) continue;
        const double goal_dist = (p.head<2>() - current_goal_.head<2>()).norm();
        const double yaw_error = std::fabs(std::atan2(
            std::sin(current_goal_yaw_ - yaw), std::cos(current_goal_yaw_ - yaw)));
        const double speed = v.head<2>().norm();
        if (goal_dist > rolling_handoff_goal_dist_ || yaw_error > 0.15 || speed < 0.10)
            continue;
        const double score = goal_dist + 0.5 * yaw_error - 0.05 * speed;
        if (score < best_handoff_score) {
            best_handoff_score = score;
            handoff_time = t;
            handoff_pos = p;
            handoff_vel = v;
            handoff_acc = a;
            handoff_yaw = yaw;
            handoff_yaw_rate = yaw_rate;
            handoff_yaw_acc = yaw_acc;
        }
    }
    if (handoff_time < 0.0) {
        ROS_INFO_THROTTLE(2.0,
            "[CoverageSearch] No De Boor handoff in 85%%~95%% satisfying observation, "
            "yaw and nonzero-speed constraints; wait for terminal stop.");
        return false;
    }
    handoff_idx = std::min(n - 2, std::max(traj_idx_ + 1,
        (int)std::lround(handoff_time / std::max(1e-3, traj_dt_))));

    const Eigen::Vector3d old_uav_pos = uav_pos_;
    const Eigen::Vector3d old_uav_vel = uav_vel_;
    const double old_uav_yaw = uav_yaw_;
    const Eigen::Vector3d old_goal = current_goal_;
    const double old_goal_yaw = current_goal_yaw_;
    const bool old_has_goal = has_goal_;
    const bool old_has_traj = has_traj_;
    const std::vector<Eigen::Vector3d> old_astar_path = astar_path_;
    const int old_astar_idx = astar_path_idx_;
    const std::vector<Eigen::Vector3d> old_traj_points = traj_points_;
    const std::vector<Eigen::Vector3d> old_traj_vels = traj_vels_;
    const std::vector<Eigen::Vector3d> old_traj_accs = traj_accs_;
    const std::vector<double> old_traj_yaws = traj_yaws_;
    const TimeBspline old_time_spline = active_time_spline_;
    const int old_traj_idx = traj_idx_;
    const ros::Time old_traj_start = traj_start_time_;
    const ros::Time old_point_time = traj_point_reach_time_;
    const double old_cmd_yaw = cmd_yaw_smoothed_;
    const bool old_cmd_yaw_inited = cmd_yaw_inited_;
    const std::vector<Eigen::Vector3d> old_frontier_targets = frontier_targets_;
    const std::vector<double> old_frontier_yaws = frontier_target_yaws_;
    const int old_frontier_idx = frontier_target_idx_;
    const int old_same_goal_replans = same_goal_replan_count_;
    const ros::Time old_goal_commit = goal_commit_time_;
    const bool old_has_failed_goal = has_failed_goal_;
    const Eigen::Vector3d old_failed_goal = failed_goal_;
    const ros::Time old_failed_goal_time = failed_goal_time_;
    const bool old_planning_state_valid = planning_start_state_valid_;
    const Eigen::Vector3d old_planning_start_acc = planning_start_acc_;
    const double old_planning_start_yaw_rate = planning_start_yaw_rate_;
    const double old_planning_start_yaw_acc = planning_start_yaw_acc_;

    PendingTrajectory prepared;
    const auto planning_start = std::chrono::steady_clock::now();
    uav_pos_ = handoff_pos;
    uav_vel_ = handoff_vel;
    uav_yaw_ = handoff_yaw;
    planning_start_state_valid_ = true;
    planning_start_acc_ = handoff_acc;
    planning_start_yaw_rate_ = handoff_yaw_rate;
    planning_start_yaw_acc_ = handoff_yaw_acc;

    rolling_prepare_in_progress_ = true;
    bool selected = selectNextFrontier();
    int next_idx = -1;
    if (selected) {
        for (int i = 0; i < (int)frontier_targets_.size(); ++i) {
            // 只排除数值上等同于旧目标的点；不再排除旧目标0.8m内的新视点。
            if ((frontier_targets_[i].head<2>() - old_goal.head<2>()).norm() >= 0.10) {
                next_idx = i;
                break;
            }
        }
    }

    if (next_idx >= 0) {
        current_goal_ = frontier_targets_[next_idx];
        current_goal_(2) = fly_height_;
        current_goal_yaw_ = frontier_target_yaws_[next_idx];
        has_goal_ = true;
        if (planPathToGoal()) {
            generateBsplineTraj();
            if (has_traj_ && active_time_spline_.valid) {
                prepared.ready = true;
                prepared.handoff_idx = handoff_idx;
                prepared.handoff_time = handoff_time;
                prepared.goal = current_goal_;
                prepared.goal_yaw = current_goal_yaw_;
                prepared.astar_path = astar_path_;
                prepared.points = traj_points_;
                prepared.vels = traj_vels_;
                prepared.accs = traj_accs_;
                prepared.yaws = traj_yaws_;
                prepared.time_spline = active_time_spline_;
            }
        }
    }
    rolling_prepare_in_progress_ = false;

    // 规划过程只允许写入待交接缓冲；旧轨迹以及真实状态继续作为活动状态。
    uav_pos_ = old_uav_pos;
    uav_vel_ = old_uav_vel;
    uav_yaw_ = old_uav_yaw;
    current_goal_ = old_goal;
    current_goal_yaw_ = old_goal_yaw;
    has_goal_ = old_has_goal;
    has_traj_ = old_has_traj;
    astar_path_ = old_astar_path;
    astar_path_idx_ = old_astar_idx;
    traj_points_ = old_traj_points;
    traj_vels_ = old_traj_vels;
    traj_accs_ = old_traj_accs;
    traj_yaws_ = old_traj_yaws;
    active_time_spline_ = old_time_spline;
    traj_idx_ = old_traj_idx;
    traj_start_time_ = old_traj_start;
    traj_point_reach_time_ = old_point_time;
    cmd_yaw_smoothed_ = old_cmd_yaw;
    cmd_yaw_inited_ = old_cmd_yaw_inited;
    frontier_targets_ = old_frontier_targets;
    frontier_target_yaws_ = old_frontier_yaws;
    frontier_target_idx_ = old_frontier_idx;
    same_goal_replan_count_ = old_same_goal_replans;
    goal_commit_time_ = old_goal_commit;
    has_failed_goal_ = old_has_failed_goal;
    failed_goal_ = old_failed_goal;
    failed_goal_time_ = old_failed_goal_time;
    planning_start_state_valid_ = old_planning_state_valid;
    planning_start_acc_ = old_planning_start_acc;
    planning_start_yaw_rate_ = old_planning_start_yaw_rate;
    planning_start_yaw_acc_ = old_planning_start_yaw_acc;

    const double planning_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - planning_start).count();
    if (!prepared.ready) {
        ROS_WARN("[CoverageSearch] Rolling preplan failed after %.1f ms; retain old trajectory.",
                 planning_ms);
        return false;
    }

    pending_traj_ = std::move(prepared);
    ROS_INFO("[CoverageSearch] Rolling trajectory ready: DeBoor handoff=%.2fs (%d/%d), "
             "next_goal=(%.2f,%.2f), next_points=%zu, planning=%.1f ms.",
             pending_traj_.handoff_time, pending_traj_.handoff_idx, n, pending_traj_.goal(0),
             pending_traj_.goal(1), pending_traj_.points.size(), planning_ms);
    return true;
}

bool CoverageSearchManager::checkObstacleAhead(Eigen::Vector3d &avoid_dir) {
    avoid_dir.setZero();
    bool obstacle_found = false;

    double check_dist = obstacle_check_dist_;
    Eigen::Vector3d fly_dir;
    if (has_goal_) {
        fly_dir = current_goal_ - uav_pos_;
        fly_dir(2) = 0;
        if (fly_dir.norm() > 0.1) fly_dir.normalize();
        else fly_dir = Eigen::Vector3d(cos(uav_yaw_), sin(uav_yaw_), 0);
    } else {
        fly_dir = Eigen::Vector3d(cos(uav_yaw_), sin(uav_yaw_), 0);
    }

    double min_obs_dist = 1e6;
    Eigen::Vector3d closest_obstacle_dir(0, 0, 0);

    for (int i = -6; i <= 6; i++) {
        double angle = uav_yaw_ + i * (M_PI / 12.0);
        Eigen::Vector3d check_dir(cos(angle), sin(angle), 0);

        for (double d = coverage_map_.resolution_; d <= check_dist; d += coverage_map_.resolution_) {
            Eigen::Vector3d check_pos = uav_pos_ + d * check_dir;
            check_pos(2) = fly_height_;

            // ★ 地图边界外不算障碍物（避免在地图边缘误触发避障）
            if (!coverage_map_.isInMap(check_pos)) continue;

            Eigen::Vector3i idx;
            coverage_map_.posToIndex(check_pos, idx);
            if (coverage_map_.isOccupied2D(idx(0), idx(1))) {
                if (d < min_obs_dist) { min_obs_dist = d; closest_obstacle_dir = check_dir; }
                obstacle_found = true;
                break;
            }
        }
    }

    if (obstacle_found && min_obs_dist < safe_distance_) {
        Eigen::Vector3d avoid_left(-closest_obstacle_dir(1), closest_obstacle_dir(0), 0);
        Eigen::Vector3d avoid_right(closest_obstacle_dir(1), -closest_obstacle_dir(0), 0);

        double left_clear = 0, right_clear = 0;
        for (double d = 0; d <= check_dist; d += coverage_map_.resolution_) {
            Eigen::Vector3d lp = uav_pos_ + d * avoid_left; lp(2) = fly_height_;
            Eigen::Vector3i li; coverage_map_.posToIndex(lp, li);
            if (coverage_map_.isInMap2D(li(0), li(1)) && coverage_map_.isFree2D(li(0), li(1)))
                left_clear = d;
            else break;
        }
        for (double d = 0; d <= check_dist; d += coverage_map_.resolution_) {
            Eigen::Vector3d rp = uav_pos_ + d * avoid_right; rp(2) = fly_height_;
            Eigen::Vector3i ri; coverage_map_.posToIndex(rp, ri);
            if (coverage_map_.isInMap2D(ri(0), ri(1)) && coverage_map_.isFree2D(ri(0), ri(1)))
                right_clear = d;
            else break;
        }

        if (left_clear >= right_clear) avoid_dir = avoid_left;
        else avoid_dir = avoid_right;

        if (left_clear < safe_distance_ && right_clear < safe_distance_) {
            avoid_dir = -fly_dir;
        }

        return true;
    }

    return false;
}

// ============================================================
// ★ 核心方法1：选择下一个前沿簇目标
// ============================================================
bool CoverageSearchManager::selectNextFrontier() {
    int ftr_count = frontier_finder_.getFrontierCount();

    if (ftr_count > 0) {
        frontier_targets_.clear();
        frontier_target_yaws_.clear();

        struct TargetCandidate {
            Eigen::Vector3d pos;
            double yaw;
            double euclid_dist;
            bool recent;
        };

        std::vector<TargetCandidate> candidates;
        int clusters_without_view = 0;
        int skipped_not_free = 0;
        int skipped_too_close = 0;
        int reach_direct_failed = 0;
        int reach_astar_tried = 0;
        int reach_astar_failed = 0;
        int reach_astar_timeout = 0;
        int reach_astar_no_path = 0;
        int reach_astar_endpoint_blocked = 0;
        int reach_astar_unknown_rejected = 0;
        int reach_astar_skipped = 0;
        int reach_budget_break = 0;
        const uint64_t current_frontier_generation = frontier_finder_.update_generation_;

        for (auto &ftr : frontier_finder_.frontiers_) {
            if (ftr.viewpoints.empty()) {
                clusters_without_view++;
                continue;
            }

            // 不再只取按可见收益排序的前6个；全量汇总后由全局最近候选池限流。
            for (int i = 0; i < (int)ftr.viewpoints.size(); ++i) {
                const Eigen::Vector3d &vp = ftr.viewpoints[i];
                double dist = (uav_pos_.head<2>() - vp.head<2>()).norm();

                // 仅防止重复选择完全相同的旧视点，不排除其附近的新前沿视点。
                if (rolling_prepare_in_progress_ &&
                    (vp.head<2>() - current_goal_.head<2>()).norm() < 0.10) {
                    continue;
                }

                Eigen::Vector3i idx;
                coverage_map_.posToIndex(vp, idx);
                if (!coverage_map_.isInMap2D(idx(0), idx(1)) ||
                    !coverage_map_.isFree2D(idx(0), idx(1))) {
                    skipped_not_free++;
                    continue;
                }

                if (dist < 0.55) {
                    skipped_too_close++;
                    continue;
                }

                const bool recent = current_frontier_generation > 0 &&
                    ftr.changed_generation + 1 >= current_frontier_generation;
                candidates.push_back({vp, ftr.viewpoint_yaws[i], dist, recent});
            }

        }

        if (!candidates.empty()) {
            auto by_euclid = [](const TargetCandidate &a, const TargetCandidate &b) {
                return a.euclid_dist < b.euclid_dist;
            };
            std::sort(candidates.begin(), candidates.end(), by_euclid);
            std::vector<TargetCandidate> recent_candidates;
            std::vector<TargetCandidate> older_candidates;
            for (const auto &candidate : candidates) {
                (candidate.recent ? recent_candidates : older_candidates).push_back(candidate);
            }

            struct ReachableCandidate {
                Eigen::Vector3d pos;
                double yaw;
                double path_len;
                double final_cost;
                double path_time;
                double direction_time;
                double yaw_time;
            };
            std::vector<ReachableCandidate> reachable;
            std::vector<Eigen::Vector3d> astar_attempted;

            auto evaluatePool = [&](const std::vector<TargetCandidate> &pool,
                                    int max_checks,
                                    int max_astar_checks,
                                    double timeout_ms,
                                    double budget_ms,
                                    bool allow_retry = false) {
                int astar_checks = 0;
                auto t0 = std::chrono::high_resolution_clock::now();
                astar2d_.setSearchTimeout(timeout_ms);
                const int checks = std::min(max_checks, (int)pool.size());
                for (int ci = 0; ci < checks; ++ci) {
                    const auto &cand = pool[ci];
                    double path_len = cand.euclid_dist;
                    std::vector<Eigen::Vector3d> reach_path;
                    Eigen::Vector3d initial_path_dir = cand.pos - uav_pos_;
                    initial_path_dir(2) = 0.0;
                    bool reachable_direct = astar2d_.isPathTraversable(
                        uav_pos_, cand.pos, &path_len);
                    if (!reachable_direct) {
                        reach_direct_failed++;
                        auto now = std::chrono::high_resolution_clock::now();
                        double elapsed_ms = std::chrono::duration<double, std::milli>(now - t0).count();
                        if (elapsed_ms > budget_ms) {
                            reach_budget_break++;
                            break;
                        }
                        bool already_attempted = false;
                        for (const auto &attempted : astar_attempted) {
                            if ((attempted.head<2>() - cand.pos.head<2>()).norm() < 0.1) {
                                already_attempted = true;
                                break;
                            }
                        }
                        if (already_attempted && !allow_retry) {
                            reach_astar_skipped++;
                            continue;
                        }
                        if (astar_checks >= max_astar_checks) {
                            reach_astar_skipped++;
                            continue;
                        }
                        astar_checks++;
                        reach_astar_tried++;
                        if (!already_attempted) astar_attempted.push_back(cand.pos);
                        double astar_len = cand.euclid_dist;
                        if (!astar2d_.search(uav_pos_, cand.pos, reach_path,
                                             &astar_len)) {
                            reach_astar_failed++;
                            switch (astar2d_.lastFailureReason()) {
                                case Astar2D::ASTAR_TIMEOUT:
                                    reach_astar_timeout++;
                                    break;
                                case Astar2D::ASTAR_NO_PATH:
                                    reach_astar_no_path++;
                                    break;
                                case Astar2D::ASTAR_ENDPOINT_BLOCKED:
                                    reach_astar_endpoint_blocked++;
                                    break;
                                case Astar2D::ASTAR_UNKNOWN_REJECTED:
                                    reach_astar_unknown_rejected++;
                                    break;
                                default:
                                    break;
                            }
                            continue;
                        }
                        path_len = std::max(cand.euclid_dist, astar_len);
                        for (int pi = 1; pi < (int)reach_path.size(); ++pi) {
                            Eigen::Vector3d segment = reach_path[pi] - reach_path[0];
                            segment(2) = 0.0;
                            if (segment.norm() > 0.10) {
                                initial_path_dir = segment;
                                break;
                            }
                        }
                    }

                    double path_time = 0.0;
                    double direction_time = 0.0;
                    double yaw_time = 0.0;
                    const double final_cost = computeFrontierCost(
                        cand.pos, cand.yaw, path_len, initial_path_dir,
                        &path_time, &direction_time, &yaw_time);
                    reachable.push_back({
                        cand.pos,
                        cand.yaw,
                        path_len,
                        final_cost,
                        path_time,
                        direction_time,
                        yaw_time
                    });
                }
                astar2d_.setSearchTimeout(250.0);
            };

            auto evaluateCandidateSet = [&](const std::vector<TargetCandidate> &candidate_set) {
                if (candidate_set.empty()) return;
                const int primary_count = std::min((int)candidate_set.size(),
                    rolling_prepare_in_progress_ ? 24 : 48);
                std::vector<TargetCandidate> primary_pool(candidate_set.begin(),
                                                          candidate_set.begin() + primary_count);
                evaluatePool(primary_pool, primary_count,
                    rolling_prepare_in_progress_ ? 8 : 16,
                    rolling_prepare_in_progress_ ? 80.0 : 150.0,
                    rolling_prepare_in_progress_ ? 650.0 : 1800.0);

                if (reachable.empty()) {
                    ROS_WARN_THROTTLE(2.0,
                        "[CoverageSearch] Candidate A* checks found no reachable viewpoint; "
                        "retry up to 3 nearest candidates with 250 ms.");
                    const int rescue_count = std::min((int)candidate_set.size(), 12);
                    evaluatePool(candidate_set, rescue_count,
                        rolling_prepare_in_progress_ ? 2 : 3,
                        rolling_prepare_in_progress_ ? 150.0 : 250.0,
                        rolling_prepare_in_progress_ ? 350.0 : 800.0, true);
                }
            };

            // 先只在本次与上一次增量更新产生/改变的前沿中选择；确实没有
            // 可达解时才回退到更老的全局前沿，避免近处新前沿被远处旧簇抢走。
            evaluateCandidateSet(recent_candidates);
            const bool used_recent_pool = !reachable.empty();
            if (reachable.empty()) evaluateCandidateSet(older_candidates);

            ROS_INFO_THROTTLE(1.0,
                "[CoverageSearch] Frontier generation priority: current=%llu, "
                "recent_viewpoints=%zu, older_viewpoints=%zu, selected_pool=%s.",
                (unsigned long long)current_frontier_generation,
                recent_candidates.size(), older_candidates.size(),
                used_recent_pool ? "recent-two" : "global-fallback");

            if (!reachable.empty()) {
                std::sort(reachable.begin(), reachable.end(),
                          [](const ReachableCandidate &a, const ReachableCandidate &b) {
                              if (fabs(a.final_cost - b.final_cost) > 1e-3)
                                  return a.final_cost < b.final_cost;
                              return a.path_len < b.path_len;
                          });
                ROS_INFO("[CoverageSearch] Best viewpoint bottleneck=%.3fs "
                         "[path=%.3f, direction=%.3f, yaw=%.3f], geodesic=%.2fm",
                         reachable.front().final_cost,
                         reachable.front().path_time,
                         reachable.front().direction_time,
                         reachable.front().yaw_time,
                         reachable.front().path_len);

                const int target_num = std::min(10, (int)reachable.size());
                for (int i = 0; i < target_num; ++i) {
                    frontier_targets_.push_back(reachable[i].pos);
                    frontier_target_yaws_.push_back(reachable[i].yaw);
                }
            }
        }

        if (frontier_targets_.empty()) {
            cout << YELLOW << "[CoverageSearch] Frontiers exist but no usable viewpoint. clusters="
                 << ftr_count << ", no_view=" << clusters_without_view
                 << ", not_free=" << skipped_not_free
                 << ", too_close=" << skipped_too_close
                 << ", reach_direct_failed=" << reach_direct_failed
                 << ", astar_tried=" << reach_astar_tried
                 << ", astar_failed=" << reach_astar_failed
                 << ", astar_timeout=" << reach_astar_timeout
                 << ", astar_no_path=" << reach_astar_no_path
                 << ", astar_endpoint_blocked=" << reach_astar_endpoint_blocked
                 << ", astar_unknown_rejected=" << reach_astar_unknown_rejected
                 << ", astar_skipped=" << reach_astar_skipped
                 << ", budget_break=" << reach_budget_break << TAIL << endl;
            return false;
        }

        filterFailedGoal();

        if (!frontier_targets_.empty()) {
            frontier_target_idx_ = 0;
            cout << GREEN << "[CoverageSearch] Selected " << frontier_targets_.size()
                 << " frontier targets. Best: (" << frontier_targets_[0](0) << ","
                 << frontier_targets_[0](1) << ") dist="
                 << (uav_pos_ - frontier_targets_[0]).norm()
                 << " yaw_diff=" << fabs(atan2(sin(frontier_target_yaws_[0] - uav_yaw_),
                                                cos(frontier_target_yaws_[0] - uav_yaw_)))
                 << TAIL << endl;
            return true;
        }

        cout << YELLOW << "[CoverageSearch] Usable frontier viewpoints were filtered out." << TAIL << endl;
        return false;
    }

    double coverage = coverage_map_.getKnownSpaceRatio();
    cout << GREEN << "[CoverageSearch] No frontier viewpoint target. Known: "
         << coverage * 100.0 << "%" << TAIL << endl;
    return false;
}

// ============================================================
// ★ 核心方法2：A*规划到目标的路径
// ============================================================
bool CoverageSearchManager::planPathToGoal() {
    if (!has_goal_) return false;

    // 先检查直线是否可通行
    if (astar2d_.isPathTraversable(uav_pos_, current_goal_)) {
        astar_path_.clear();
        // ★ 必须同时包含起点和终点，这样Hermite插值才能生成完整的平滑轨迹
        astar_path_.push_back(uav_pos_);
        astar_path_.push_back(current_goal_);
        astar_path_idx_ = 0;
        return true;
    }

    // A*搜索
    bool found = astar2d_.search(uav_pos_, current_goal_, astar_path_);
    if (found && !astar_path_.empty()) {
        astar_path_idx_ = 0;
        return true;
    }

    return false;
}
// 三项均为时间、权重均为1，最终取瓶颈最大值。
double CoverageSearchManager::computeFrontierCost(const Eigen::Vector3d &vp,
                                                  double vp_yaw, double path_len,
                                                  const Eigen::Vector3d &initial_path_dir,
                                                  double *path_time_out,
                                                  double *direction_time_out,
                                                  double *yaw_time_out) {
    const double path_time = path_len / std::max(0.1, max_vel_);
    double direction_time = 0.0;
    Eigen::Vector3d vel = uav_vel_;
    vel(2) = 0.0;
    Eigen::Vector3d path_dir = initial_path_dir;
    path_dir(2) = 0.0;
    if (path_dir.norm() < 1e-3) path_dir = vp - uav_pos_;
    path_dir(2) = 0.0;
    if (vel.norm() > 0.1 && path_dir.norm() > 1e-3) {
        double dot = vel.normalized().dot(path_dir.normalized());
        double direction_diff = acos(std::min(1.0, std::max(-1.0, dot)));
        direction_time = direction_diff / std::max(0.1, max_yaw_rate_);
    }

    double yaw_diff = fabs(atan2(sin(vp_yaw - uav_yaw_), cos(vp_yaw - uav_yaw_)));
    const double yaw_time = yaw_diff / std::max(0.1, max_yaw_rate_);
    if (path_time_out) *path_time_out = path_time;
    if (direction_time_out) *direction_time_out = direction_time;
    if (yaw_time_out) *yaw_time_out = yaw_time;
    const double bottleneck = std::max(path_time, std::max(direction_time, yaw_time));
    ROS_ASSERT_MSG(std::isfinite(bottleneck) && bottleneck >= 0.0,
                   "Frontier bottleneck cost must be finite and non-negative");
    return bottleneck;
}

// 视点净空按 0.55 -> 0.50 -> 0.45 -> 0.35 m 分层。主路径仍保持请求的
// 高净空，只在目标附近平滑下降到该视点所属层级。
double CoverageSearchManager::requiredTrajectoryClearance(
    const Eigen::Vector3d &pos, double requested_clearance) {
    const double requested = std::max(0.35, requested_clearance);
    if (!has_goal_) return requested;

    Eigen::Vector3i goal_idx;
    coverage_map_.posToIndex(current_goal_, goal_idx);
    if (!coverage_map_.isInMap2D(goal_idx(0), goal_idx(1))) return requested;
    const double goal_clearance =
        coverage_map_.getDistance2D(goal_idx(0), goal_idx(1));
    double goal_tier = 0.35;
    for (double tier : {0.55, 0.50, 0.45, 0.35}) {
        if (goal_clearance + 1e-3 >= tier) {
            goal_tier = tier;
            break;
        }
    }
    const double goal_dist =
        (pos.head<2>() - current_goal_.head<2>()).norm();
    return std::max(0.35, std::min(requested, goal_tier + 0.5 * goal_dist));
}

// ★ 执行期把 raw free 和 ESDF clearance 都当作硬安全约束。
//   位置控制会沿当前点到指令点拉直飞，所以这里必须检查整条直线段，
//   不能只检查离散轨迹点。
bool CoverageSearchManager::pathToTargetBlocked(const Eigen::Vector3d &to,
                                                const Eigen::Vector3d &from,
                                                double min_clearance) {
    Eigen::Vector3d start = from;
    Eigen::Vector3d seg = to - start;
    seg(2) = 0;
    double dist = seg.norm();
    const double requested_clearance = std::max(
        std::max(0.35, coverage_map_.esdf_safe_distance_), min_clearance);
    if (dist < 0.05) {
        for (const Eigen::Vector3d &p : {start, to}) {
            Eigen::Vector3i idx;
            coverage_map_.posToIndex(p, idx);
            if (!coverage_map_.isInMap2D(idx(0), idx(1)) ||
                !coverage_map_.isFree2D(idx(0), idx(1)) ||
                coverage_map_.getDistance2D(idx(0), idx(1)) <
                    requiredTrajectoryClearance(p, requested_clearance)) {
                return true;
            }
        }
        return false;
    }
    Eigen::Vector3d dir = seg / dist;
    double step = std::max(0.05, 0.5 * coverage_map_.resolution_);
    Eigen::Vector3i start_idx;
    coverage_map_.posToIndex(start, start_idx);
    double prev_clearance = coverage_map_.isInMap2D(start_idx(0), start_idx(1))
        ? coverage_map_.getDistance2D(start_idx(0), start_idx(1)) : 0.0;
    bool escaping = (start.head<2>() - uav_pos_.head<2>()).norm() < 0.30 &&
                    (!coverage_map_.isFree2D(start_idx(0), start_idx(1)) ||
                     prev_clearance < requiredTrajectoryClearance(start, requested_clearance));
    const int steps = std::max(1, (int)std::ceil(dist / step));
    for (int i = 0; i <= steps; ++i) {
        Eigen::Vector3d p = start + ((double)i / steps) * seg;
        p(2) = fly_height_;
        if (!coverage_map_.isInMap(p)) return true;
        Eigen::Vector3i idx;
        coverage_map_.posToIndex(p, idx);
        double clearance = coverage_map_.getDistance2D(idx(0), idx(1));
        const double hard_clearance =
            requiredTrajectoryClearance(p, requested_clearance);
        if (escaping) {
            bool in_start_cell = idx(0) == start_idx(0) && idx(1) == start_idx(1);
            if (!in_start_cell) {
                if (!coverage_map_.isFree2D(idx(0), idx(1)) ||
                    clearance + 1e-3 < prev_clearance) {
                    return true;
                }
                prev_clearance = clearance;
                if (clearance >= hard_clearance) escaping = false;
            }
            continue;
        }
        if (!coverage_map_.isFree2D(idx(0), idx(1)) || clearance < hard_clearance)
            return true;
    }
    return false;
}

bool CoverageSearchManager::currentGoalUnsafe(std::string &reason) {
    if (!has_goal_) return false;
    if (!coverage_map_.isInMap(current_goal_)) {
        reason = "goal is outside map";
        return true;
    }

    Eigen::Vector3i idx;
    coverage_map_.posToIndex(current_goal_, idx);
    if (!coverage_map_.isInMap2D(idx(0), idx(1)) ||
        !coverage_map_.isFree2D(idx(0), idx(1))) {
        reason = "goal is no longer free";
        return true;
    }

    const double cutoff = requiredTrajectoryClearance(
        current_goal_, std::max(traj_cut_clearance_, coverage_map_.esdf_safe_distance_));
    if (coverage_map_.getDistance2D(idx(0), idx(1)) + 1e-3 < cutoff) {
        reason = "goal ESDF clearance below cutoff";
        return true;
    }
    return false;
}

void CoverageSearchManager::abortCurrentGoalForSafety(const std::string &reason) {
    cout << YELLOW << "[CoverageSearch] Cut current goal: " << reason
         << ". Select next frontier viewpoint." << TAIL << endl;
    if (has_goal_) {
        failed_goal_ = current_goal_;
        failed_goal_time_ = ros::Time::now();
        has_failed_goal_ = true;
    }

    uav_command_.header.stamp = ros::Time::now();
    uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
    uav_command_.position_ref[0] = uav_pos_(0);
    uav_command_.position_ref[1] = uav_pos_(1);
    uav_command_.position_ref[2] = fly_height_;
    uav_command_.yaw_ref = uav_yaw_;
    uav_command_.Command_ID++;
    uav_cmd_pub_.publish(uav_command_);

    has_traj_ = false;
    has_goal_ = false;
    frontier_targets_.clear();
    frontier_target_yaws_.clear();
    frontier_target_idx_ = 0;
    same_goal_replan_count_ = 0;
    replan_count_++;
}

// ★ 失败目标短期排除：3s 内排除距 failed_goal_ 1.0m 以内的候选，避免反复选同一个不可达目标
void CoverageSearchManager::filterFailedGoal() {
    if (!has_failed_goal_) return;
    double elapsed = (ros::Time::now() - failed_goal_time_).toSec();
    if (elapsed > 3.0) { has_failed_goal_ = false; return; }
    std::vector<Eigen::Vector3d> pos; std::vector<double> yw;
    for (int i = 0; i < (int)frontier_targets_.size(); i++) {
        if ((frontier_targets_[i] - failed_goal_).head<2>().norm() < 1.0) continue;
        pos.push_back(frontier_targets_[i]);
        yw.push_back(frontier_target_yaws_[i]);
    }
    frontier_targets_ = pos;
    frontier_target_yaws_ = yw;
}
