#include "two_uav_coverage_search.h"

namespace {
constexpr double kRollingPeriodicInterval = 1.0;
}

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

bool CoverageSearchManager::tryPrepareRollingHandoff(bool force_new_goal) {
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

        if (!has_traj_ || !has_goal_ || (!force_new_goal && !active_goal_frontier_valid_) ||
            pending_traj_.ready || rolling_worker_running_ ||
            traj_points_.size() < 3 || !active_time_spline_.valid) {
            return false;
        }
        const ros::Time now = ros::Time::now();
        const double elapsed = std::max(0.0, (now - traj_start_time_).toSec());
        const double remaining = active_time_spline_.duration - elapsed;
        const double handoff_lead = std::max(0.05, replan_time_);
        if (remaining <= handoff_lead) return false;
        const bool periodic_due = elapsed >= kRollingPeriodicInterval &&
            (rolling_last_attempt_time_.isZero() ||
             (now - rolling_last_attempt_time_).toSec() >= kRollingPeriodicInterval);
        const bool terminal_due = remaining <= 0.50;
        if (!force_new_goal && !periodic_due && !terminal_due) return false;
        const uint64_t frontier_generation = frontier_finder_.update_generation_;

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
        planner->max_yaw_acc_ = max_yaw_acc_;
        planner->replan_time_ = replan_time_;
        planner->goal_reach_dist_ = goal_reach_dist_;
        planner->sim_mode_ = sim_mode_;
        planner->cooperative_mode_ = cooperative_mode_;
        planner->local_swarm_tasks_ = local_swarm_tasks_;
        planner->remote_swarm_tasks_ = remote_swarm_tasks_;
        planner->local_swarm_task_received_ = local_swarm_task_received_;
        planner->remote_swarm_task_received_ = remote_swarm_task_received_;
        planner->current_goal_ = current_goal_;
        planner->current_goal_yaw_ = current_goal_yaw_;
        planner->current_goal_task_id_ = current_goal_task_id_;
        planner->active_goal_frontier_ = active_goal_frontier_;
        planner->active_goal_frontier_valid_ = active_goal_frontier_valid_;
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
        planner->frontier_target_task_ids_ = frontier_target_task_ids_;
        planner->frontier_target_idx_ = frontier_target_idx_;
        planner->start_time_ = start_time_;
        planner->goal_commit_time_ = goal_commit_time_;
        planner->same_goal_replan_count_ = same_goal_replan_count_;
        planner->traj_step_size_ = traj_step_size_;
        planner->traj_advance_dist_ = traj_advance_dist_;
        planner->traj_point_dwell_timeout_ = traj_point_dwell_timeout_;
        planner->traj_cut_clearance_ = traj_cut_clearance_;
        planner->rolling_terminal_speed_ratio_ = rolling_terminal_speed_ratio_;
        planner->rolling_terminal_acc_ratio_ = rolling_terminal_acc_ratio_;
        planner->rolling_prepare_in_progress_ = false;
        planner->rolling_snapshot_mode_ = true;

        rolling_last_attempt_time_ = now;
        ++rolling_replan_count_;
        rolling_worker_running_ = true;
        const uint64_t source_generation = rolling_generation_;
        try {
            rolling_worker_ = std::thread([this, planner, source_generation,
                                           frontier_generation, force_new_goal]() {
                planner->tryPrepareRollingHandoff(force_new_goal);
                PendingTrajectory result = std::move(planner->pending_traj_);
                result.result_ready_time = ros::Time::now();
                result.source_generation = source_generation;
                result.frontier_generation = frontier_generation;
                {
                    std::lock_guard<std::mutex> lock(rolling_result_mutex_);
                    completed_pending_traj_ = std::move(result);
                }
                rolling_worker_running_ = false;
            });
            ROS_INFO("[CoverageSearch] Rolling replan #%llu: trigger=%s, remaining=%.2fs, "
                     "frontier_generation=%llu.",
                     (unsigned long long)rolling_replan_count_,
                     force_new_goal ? "frontier-cleared" :
                         (terminal_due ? "terminal" : "periodic"), remaining,
                     (unsigned long long)frontier_generation);
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
    const double elapsed = std::max(0.0, (ros::Time::now() - traj_start_time_).toSec());
    const double handoff_time = elapsed + std::max(0.05, replan_time_);
    if (handoff_time > active_time_spline_.duration) return false;
    const int handoff_idx = std::min(n - 1, std::max(0, (int)std::lround(
        handoff_time / std::max(1e-3, active_time_spline_.duration) * (n - 1))));
    Eigen::Vector3d handoff_pos, handoff_vel, handoff_acc;
    double handoff_yaw = 0.0, handoff_yaw_rate = 0.0, handoff_yaw_acc = 0.0;
    if (!evaluateTimeBspline(active_time_spline_, handoff_time,
                             handoff_pos, handoff_vel, handoff_acc,
                             handoff_yaw, handoff_yaw_rate, handoff_yaw_acc)) {
        ROS_WARN("[CoverageSearch] Cannot evaluate rolling handoff state.");
        return false;
    }

    const FrontierFinder::FrontierCluster *active_frontier = nullptr;
    if (!force_new_goal) {
        for (const auto &frontier : frontier_finder_.frontiers_) {
            if (frontierMatchesTask(frontier, active_goal_frontier_) &&
                !frontier.viewpoints.empty()) {
                active_frontier = &frontier;
                break;
            }
        }
    }
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
    const std::vector<uint64_t> old_frontier_task_ids = frontier_target_task_ids_;
    const int old_frontier_idx = frontier_target_idx_;
    const uint64_t old_goal_task_id = current_goal_task_id_;
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
    bool successor_selected = false;
    if (active_frontier) {
        current_goal_ = active_frontier->viewpoints.front();
        current_goal_(2) = fly_height_;
        current_goal_yaw_ = active_frontier->viewpoint_yaws.empty()
            ? handoff_yaw : active_frontier->viewpoint_yaws.front();
        current_goal_task_id_ = active_goal_frontier_.task_id;
        successor_selected = true;
    } else if (force_new_goal && selectNextFrontier() && !frontier_targets_.empty()) {
        current_goal_ = frontier_targets_.front();
        current_goal_(2) = fly_height_;
        current_goal_yaw_ = frontier_target_yaws_.front();
        current_goal_task_id_ = frontier_target_task_ids_.front();
        successor_selected = true;
    }
    has_goal_ = successor_selected;
    astar2d_.setSearchTimeout(50.0);
    if (successor_selected && planPathToGoal()) {
        generateBsplineTraj();
        if (has_traj_ && active_time_spline_.valid) {
            prepared.ready = true;
            prepared.handoff_idx = handoff_idx;
            prepared.handoff_time = handoff_time;
            prepared.goal = current_goal_;
            prepared.goal_yaw = current_goal_yaw_;
            prepared.task_id = current_goal_task_id_;
            prepared.astar_path = astar_path_;
            prepared.points = traj_points_;
            prepared.vels = traj_vels_;
            prepared.accs = traj_accs_;
            prepared.yaws = traj_yaws_;
            prepared.time_spline = active_time_spline_;
        }
    }
    astar2d_.setSearchTimeout(250.0);
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
    frontier_target_task_ids_ = old_frontier_task_ids;
    frontier_target_idx_ = old_frontier_idx;
    current_goal_task_id_ = old_goal_task_id;
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
    ROS_INFO("[CoverageSearch] Rolling trajectory ready: handoff=%.2fs (%d/%d), "
             "goal=(%.2f,%.2f), next_points=%zu, planning=%.1f ms.",
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
        frontier_target_task_ids_.clear();

        struct TargetCandidate {
            FrontierFinder::FrontierCluster *frontier;
            Eigen::Vector3d pos;
            double yaw;
            double euclid_dist;
            uint64_t task_id;
            Eigen::Vector3d backup_pos;
            double backup_yaw;
            bool has_backup;
            bool local_reserved;
        };

        std::vector<TargetCandidate> candidates;
        int clusters_without_view = 0;
        int skipped_peer_lease = 0;
        int free_candidates = 0;
        int skipped_not_free = 0;
        int reach_astar_tried = 0;
        int reach_astar_failed = 0;
        int reach_astar_timeout = 0;
        int reach_astar_no_path = 0;
        int reach_astar_endpoint_blocked = 0;
        int reach_astar_unknown_rejected = 0;
        int reach_astar_skipped = 0;
        int reach_budget_break = 0;
        for (auto &ftr : frontier_finder_.frontiers_) {
            if (ftr.viewpoints.empty()) {
                clusters_without_view++;
                continue;
            }
            const uint64_t lease_task_id = leasedTaskIdForFrontier(ftr);
            const bool local_reserved = localReservationTaskIdForFrontier(ftr) != 0;
            if (lease_task_id == 0 && isFrontierLeasedToPeer(ftr)) {
                skipped_peer_lease++;
                continue;
            }
            const bool is_free = lease_task_id == 0;
            if (is_free) ++free_candidates;

            // 竞价和成本排序只使用最佳视点；次视点仅在该最佳视点规划失败时
            // 作为同一前沿簇的立即备选。
            const Eigen::Vector3d &vp = ftr.viewpoints.front();
            const double dist = (uav_pos_.head<2>() - vp.head<2>()).norm();

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

            const double yaw = ftr.viewpoint_yaws.empty() ? uav_yaw_ : ftr.viewpoint_yaws.front();
            const bool has_backup = ftr.viewpoints.size() > 1;
            const Eigen::Vector3d &backup_vp = has_backup ? ftr.viewpoints[1] : vp;
            const double backup_yaw = ftr.viewpoint_yaws.size() > 1
                ? ftr.viewpoint_yaws[1] : yaw;
            candidates.push_back({&ftr, vp, yaw, dist,
                                  is_free ? frontierTaskId(ftr) : lease_task_id,
                                  backup_vp, backup_yaw, has_backup, local_reserved});

        }

        if (!candidates.empty()) {
            auto by_euclid = [](const TargetCandidate &a, const TargetCandidate &b) {
                return a.euclid_dist < b.euclid_dist;
            };
            std::sort(candidates.begin(), candidates.end(), by_euclid);

            struct ReachableCandidate {
                FrontierFinder::FrontierCluster *frontier;
                Eigen::Vector3d pos;
                double yaw;
                uint64_t task_id;
                double path_len;
                double final_cost;
                double path_time;
                double direction_time;
                double yaw_time;
                Eigen::Vector3d backup_pos;
                double backup_yaw;
                bool has_backup;
                bool local_reserved;
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
                        cand.frontier,
                        cand.pos,
                        cand.yaw,
                        cand.task_id,
                        path_len,
                        final_cost,
                        path_time,
                        direction_time,
                        yaw_time,
                        cand.backup_pos,
                        cand.backup_yaw,
                        cand.has_backup,
                        cand.local_reserved
                    });
                }
                astar2d_.setSearchTimeout(250.0);
            };

            auto evaluateCandidateSet = [&](const std::vector<TargetCandidate> &candidate_set) {
                if (candidate_set.empty()) return;
                const int primary_count = std::min((int)candidate_set.size(),
                    rolling_prepare_in_progress_ ? 12 : 48);
                std::vector<TargetCandidate> primary_pool(candidate_set.begin(),
                                                          candidate_set.begin() + primary_count);
                evaluatePool(primary_pool, primary_count,
                    rolling_prepare_in_progress_ ? 3 : 16,
                    rolling_prepare_in_progress_ ? 35.0 : 150.0,
                    rolling_prepare_in_progress_ ? 100.0 : 1800.0);

                if (reachable.empty()) {
                    ROS_WARN_THROTTLE(2.0,
                        "[CoverageSearch] Candidate A* checks found no reachable viewpoint; "
                        "retry up to 3 nearest candidates with 250 ms.");
                    const int rescue_count = std::min((int)candidate_set.size(),
                        rolling_prepare_in_progress_ ? 4 : 12);
                    evaluatePool(candidate_set, rescue_count,
                        rolling_prepare_in_progress_ ? 1 : 3,
                        rolling_prepare_in_progress_ ? 35.0 : 250.0,
                        rolling_prepare_in_progress_ ? 50.0 : 800.0, true);
                }
            };

            auto collectVisibleTargets = [&]() {
                if (reachable.empty()) return false;
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

                int selected_clusters = 0;
                for (const auto &candidate : reachable) {
                    if (selected_clusters >= 5) break;
                    auto &frontier = *candidate.frontier;
                    const bool primary_visible = frontier_finder_.isViewpointVisible(
                        frontier, candidate.pos, candidate.yaw);
                    const bool backup_visible = candidate.has_backup &&
                        frontier_finder_.isViewpointVisible(
                            frontier, candidate.backup_pos, candidate.backup_yaw);

                    if (!primary_visible && !backup_visible) {
                        frontier_finder_.refreshViewpoints(frontier, uav_pos_);
                        ROS_WARN_THROTTLE(1.0,
                            "[CoverageSearch] Cached viewpoints are occluded; refreshed frontier viewpoints.");
                        continue;
                    }

                    if (primary_visible) {
                        frontier_targets_.push_back(candidate.pos);
                        frontier_target_yaws_.push_back(candidate.yaw);
                        frontier_target_task_ids_.push_back(candidate.task_id);
                    }
                    if (backup_visible) {
                        frontier_targets_.push_back(candidate.backup_pos);
                        frontier_target_yaws_.push_back(candidate.backup_yaw);
                        frontier_target_task_ids_.push_back(candidate.task_id);
                    }
                    ++selected_clusters;
                }
                return !frontier_targets_.empty();
            };

            // A locally discovered frontier is a five-second local commitment,
            // not merely a tie-breaker against ordinary own/FREE frontiers.
            std::vector<TargetCandidate> local_reservation_candidates;
            std::vector<TargetCandidate> ordinary_candidates;
            for (const auto &candidate : candidates) {
                (candidate.local_reserved ? local_reservation_candidates : ordinary_candidates)
                    .push_back(candidate);
            }

            evaluateCandidateSet(local_reservation_candidates);
            const bool local_target_ready = collectVisibleTargets();
            if (!local_target_ready) {
                reachable.clear();
                evaluateCandidateSet(ordinary_candidates);
                collectVisibleTargets();
            }
        }

        if (frontier_targets_.empty()) {
            cout << YELLOW << "[CoverageSearch] No local frontier target. clusters="
                 << ftr_count << ", no_view=" << clusters_without_view
                 << ", peer_held=" << skipped_peer_lease
                 << ", free=" << free_candidates
                 << ", not_free=" << skipped_not_free;
            if (reach_astar_tried > 0) {
                cout << ", astar=" << reach_astar_failed << "/" << reach_astar_tried;
                if (reach_astar_timeout > 0) cout << ", timeout=" << reach_astar_timeout;
                if (reach_astar_no_path > 0) cout << ", no_path=" << reach_astar_no_path;
                if (reach_astar_endpoint_blocked > 0)
                    cout << ", endpoint_blocked=" << reach_astar_endpoint_blocked;
                if (reach_astar_unknown_rejected > 0)
                    cout << ", unknown_rejected=" << reach_astar_unknown_rejected;
            }
            if (reach_astar_skipped > 0) cout << ", astar_skipped=" << reach_astar_skipped;
            if (reach_budget_break > 0) cout << ", budget_limited=1";
            cout << TAIL << endl;
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

    // 所有正常任务都经过带低净空代价的 A*，不能因一条仅满足硬阈值的
    // 直线而贴墙飞行。直线段检查仍只用于执行期安全验证。
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

// B样条轨迹与执行期统一使用 0.45m 硬安全距离；视点分层仅用于选点。
double CoverageSearchManager::requiredTrajectoryClearance(
    const Eigen::Vector3d &, double) {
    return std::max(0.45, coverage_map_.esdf_safe_distance_);
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
    releaseCurrentGoal(reason, true, true);
}

void CoverageSearchManager::releaseCurrentGoal(const std::string &reason,
                                               bool mark_failed,
                                               bool count_as_replan) {
    cout << YELLOW << "[CoverageSearch] Release current goal: " << reason
         << ". Select next frontier viewpoint." << TAIL << endl;
    if (mark_failed && has_goal_) {
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
    frontier_target_task_ids_.clear();
    frontier_target_idx_ = 0;
    eraseLocalReservation(current_goal_task_id_);
    current_goal_task_id_ = 0;
    same_goal_replan_count_ = 0;
    active_goal_frontier_valid_ = false;
    active_goal_frontier_covered_pending_ = false;
    active_goal_frontier_cells_.clear();
    active_goal_frontier_checked_generation_ = 0;
    pending_traj_ = PendingTrajectory();
    rolling_last_attempt_time_ = ros::Time(0);
    ++rolling_generation_;
    if (count_as_replan) ++replan_count_;
}

// ★ 失败目标短期排除：3s 内排除距 failed_goal_ 1.0m 以内的候选，避免反复选同一个不可达目标
void CoverageSearchManager::filterFailedGoal() {
    if (!has_failed_goal_) return;
    double elapsed = (ros::Time::now() - failed_goal_time_).toSec();
    if (elapsed > 3.0) { has_failed_goal_ = false; return; }
    std::vector<Eigen::Vector3d> pos; std::vector<double> yw; std::vector<uint64_t> task_ids;
    for (int i = 0; i < (int)frontier_targets_.size(); i++) {
        if ((frontier_targets_[i] - failed_goal_).head<2>().norm() < 1.0) continue;
        pos.push_back(frontier_targets_[i]);
        yw.push_back(frontier_target_yaws_[i]);
        task_ids.push_back(frontier_target_task_ids_[i]);
    }
    frontier_targets_ = pos;
    frontier_target_yaws_ = yw;
    frontier_target_task_ids_ = task_ids;
}
