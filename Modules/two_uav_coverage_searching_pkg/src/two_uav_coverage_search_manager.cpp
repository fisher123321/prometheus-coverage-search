#include "two_uav_coverage_search.h"

#include <map>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

CoverageSearchManager::~CoverageSearchManager() {
    trajectory_timer_.stop();
    if (trajectory_spinner_) trajectory_spinner_->stop();
    if (rolling_worker_.joinable()) rolling_worker_.join();
}

// ============================================================
// ★ CoverageSearchManager 实现 - 前沿簇驱动探索
// 核心逻辑：找前沿簇 → 选最近的前沿簇 → A*规划路径 → B样条平滑 → 执行轨迹
// ============================================================
void CoverageSearchManager::init(ros::NodeHandle &nh) {
    nh.param("uav_id", uav_id_, 1);
    nh.param("coverage_search/sim_mode", sim_mode_, true);
    nh.param("coverage_search/fly_height", fly_height_, 1.5);
    nh.param("coverage_search/sensing_range", sensing_range_, 5.0);
    nh.param("coverage_search/sensing_fov_h", sensing_fov_h_, 1.5708);
    nh.param("coverage_search/sensing_fov_v", sensing_fov_v_, 1.287);
    nh.param("coverage_search/max_vel", max_vel_, 1.0);
    nh.param("coverage_search/max_acc", max_acc_, 0.8);
    nh.param("coverage_search/max_yaw_rate", max_yaw_rate_, 0.5);
    nh.param("coverage_search/max_yaw_acc", max_yaw_acc_, 1.8);
    nh.param("coverage_search/replan_time", replan_time_, 0.25);
    nh.param("coverage_search/replan_min_execute_time", replan_min_execute_time_, 0.5);
    nh.param("coverage_search/replan_periodic_time", replan_periodic_time_, 1.5);
    nh.param("coverage_search/replan_remaining_time", replan_remaining_time_, 0.5);
    nh.param("coverage_search/goal_reach_dist", goal_reach_dist_, 0.25);
    nh.param("coverage_search/auto_start", auto_start_, true);
    nh.param("coverage_search/cooperative_mode", cooperative_mode_, false);
    nh.param("coverage_search/external_goal_only", external_goal_only_, false);
    nh.param("coverage_search/global_coverage_source", global_coverage_source_, 0);
    nh.param("coverage_search/map_epoch", map_epoch_, 1);
    nh.param("coverage_search/swarm_chunk_size", swarm_chunk_size_, 2.0);
    nh.param("coverage_search/swarm_chunk_delta_period", swarm_chunk_delta_period_, 0.5);
    nh.param("coverage_search/peer_state_timeout", peer_state_timeout_, 0.6);
    nh.param("coverage_search/swarm_tx_prefix", swarm_tx_prefix_, string("/two_uav/tx"));
    nh.param("coverage_search/swarm_rx_prefix", swarm_rx_prefix_, string("/two_uav/rx"));
    nh.param("coverage_search/map_input_source", map_input_source_, 3);
    nh.param("coverage_search/global_pcl_topic", global_pcl_topic_, string("/map_generator/global_cloud"));
    nh.param("coverage_search/local_pcl_topic", local_pcl_topic_, string("/map_generator/local_cloud"));
    nh.param("coverage_search/scan_pcl_topic", scan_pcl_topic_, string("/uav1/prometheus/scan_point_cloud"));
    nh.param("coverage_search/depth_pcl_topic", depth_pcl_topic_, string("/uav1/camera/depth/color/points"));
    nh.param("coverage_search/camera_offset_x", camera_offset_(0), 0.095);
    nh.param("coverage_search/camera_offset_y", camera_offset_(1), 0.0);
    nh.param("coverage_search/camera_offset_z", camera_offset_(2), 0.0);
    nh.param("coverage_search/camera_pitch", camera_pitch_, 0.35);
    nh.param("coverage_search/fov_vis_range", fov_vis_range_, 1.5);
    nh.param("coverage_search/obstacle_check_dist", obstacle_check_dist_, 1.5);
    nh.param("coverage_search/safe_distance", safe_distance_, 0.8);
    // ★ 按位置推进的轨迹执行参数
    nh.param("coverage_search/traj_step_size", traj_step_size_, 0.5);
    nh.param("coverage_search/traj_advance_dist", traj_advance_dist_, 0.3);
    nh.param("coverage_search/traj_point_dwell_timeout", traj_point_dwell_timeout_, 5.0);
    nh.param("coverage_search/completion_known_ratio", completion_known_ratio_, 0.99);
    nh.param("coverage_search/residual_scan_yaw_rate", residual_scan_yaw_rate_, 0.6);
    nh.param("coverage_search/swarm_bid_period", swarm_bid_period_, 2.0);
    nh.param("coverage_search/swarm_bid_max_tasks", swarm_bid_max_tasks_, 16);
    nh.param("coverage_search/swarm_bid_max_astar", swarm_bid_max_astar_, 6);
    nh.param("coverage_search/atsp_enabled", atsp_enabled_, true);
    nh.param("coverage_search/atsp_max_clusters", atsp_max_clusters_, 5);
    nh.param("coverage_search/atsp_refine_clusters", atsp_refine_clusters_, 3);
    nh.param("coverage_search/atsp_viewpoints_per_cluster", atsp_viewpoints_per_cluster_, 3);
    nh.param("coverage_search/atsp_edge_timeout_ms", atsp_edge_timeout_ms_, 100.0);
    atsp_max_clusters_ = std::max(1, std::min(5, atsp_max_clusters_));
    atsp_refine_clusters_ = std::max(1, std::min(atsp_max_clusters_, atsp_refine_clusters_));
    atsp_viewpoints_per_cluster_ = std::max(1, atsp_viewpoints_per_cluster_);
    atsp_edge_timeout_ms_ = std::max(20.0, atsp_edge_timeout_ms_);
    replan_min_execute_time_ = std::max(0.0, replan_min_execute_time_);
    replan_periodic_time_ = std::max(replan_min_execute_time_, replan_periodic_time_);
    replan_remaining_time_ = std::max(replan_time_, replan_remaining_time_);
    nh.param("coverage_search/peer_avoidance_radius", peer_avoidance_radius_, 0.75);
    nh.param("coverage_search/peer_avoidance_duration", peer_avoidance_duration_, 2.0);
    uav_name_ = "/uav" + std::to_string(uav_id_);

    coverage_map_.init(nh);
    planning_tf_buffer_.reset(new tf2_ros::Buffer);
    planning_tf_listener_.reset(new tf2_ros::TransformListener(*planning_tf_buffer_));
    // 轨迹安全阈值与 ESDF 保持同一个值，不单独提供配置项。
    traj_cut_clearance_ = coverage_map_.esdf_safe_distance_;
    frontier_finder_.init(nh, &coverage_map_);
    hierarchical_grid_.init(nh, &coverage_map_);
    astar2d_.init(&coverage_map_);

    uav_state_sub_ = nh.subscribe<prometheus_msgs::UAVState>(
        uav_name_ + "/prometheus/state", 1, &CoverageSearchManager::uavStateCb, this);
    uav_control_state_sub_ = nh.subscribe<prometheus_msgs::UAVControlState>(
        uav_name_ + "/prometheus/control_state", 1, &CoverageSearchManager::uavControlStateCb, this);

    if (map_input_source_ == 0) {
        global_pcl_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
            global_pcl_topic_, 1, &CoverageSearchManager::globalPclCb, this);
    } else if (map_input_source_ == 1) {
        local_pcl_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
            uav_name_ + local_pcl_topic_, 1, &CoverageSearchManager::localPclCb, this);
    } else if (map_input_source_ == 2) {
        scan_pcl_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
            scan_pcl_topic_, 1, &CoverageSearchManager::scanPclCb, this);
    } else if (map_input_source_ == 3) {
        depth_pcl_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
            depth_pcl_topic_, 1, &CoverageSearchManager::depthPclCb, this);
    }

    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        uav_name_ + "/prometheus/coverage_search/goal", 1, &CoverageSearchManager::goalCb, this);
    peer_collision_replan_sub_ = nh.subscribe<std_msgs::Bool>(
        uav_name_ + "/prometheus/coverage_search/peer_collision_replan", 1,
        &CoverageSearchManager::peerCollisionReplanCb, this);

    const string command_topic = cooperative_mode_
        ? uav_name_ + "/prometheus/coverage_search/raw_command"
        : uav_name_ + "/prometheus/command";
    uav_cmd_pub_ = nh.advertise<prometheus_msgs::UAVCommand>(command_topic, 1);
    path_vis_pub_ = nh.advertise<visualization_msgs::Marker>(
        uav_name_ + "/prometheus/coverage_search/path_vis", 20);
    fov_vis_pub_ = nh.advertise<visualization_msgs::Marker>(
        uav_name_ + "/prometheus/coverage_search/fov_vis", 1, true);
    coverage_status_pub_ = nh.advertise<visualization_msgs::Marker>(
        uav_name_ + "/prometheus/coverage_search/status", 1);
    uav_label_pub_ = nh.advertise<visualization_msgs::Marker>(
        uav_name_ + "/prometheus/coverage_search/uav_label", 1);
    completion_ready_pub_ = nh.advertise<std_msgs::Bool>(
        uav_name_ + "/prometheus/coverage_search/completion_ready", 1, true);

    if (cooperative_mode_) {
        swarm_frontier_pub_ = nh.advertise<prometheus_two_uav_coverage_search::SwarmFrontierArray>(
            uav_name_ + "/prometheus/coverage_search/swarm_frontiers", 2);
        swarm_bid_pub_ = nh.advertise<prometheus_two_uav_coverage_search::SwarmBidArray>(
            uav_name_ + "/prometheus/coverage_search/swarm_bids", 2);
        swarm_traj_pub_ = nh.advertise<prometheus_two_uav_coverage_search::SwarmTrajectory>(
            uav_name_ + "/prometheus/coverage_search/swarm_trajectory", 2);
        map_chunk_pub_ = nh.advertise<prometheus_two_uav_coverage_search::SwarmMapChunk>(
            swarm_tx_prefix_ + "/map_chunk", 10);
        map_request_pub_ = nh.advertise<prometheus_two_uav_coverage_search::SwarmMapRequest>(
            swarm_tx_prefix_ + "/map_request", 2);
        remote_chunk_sub_ = nh.subscribe(swarm_rx_prefix_ + "/map_chunk", 10,
            &CoverageSearchManager::remoteChunkCb, this);
        remote_frontier_sub_ = nh.subscribe(swarm_rx_prefix_ + "/frontier", 2,
            &CoverageSearchManager::remoteFrontierCb, this);
        local_task_sub_ = nh.subscribe(swarm_tx_prefix_ + "/task", 2,
            &CoverageSearchManager::swarmTaskCb, this);
        remote_task_sub_ = nh.subscribe(swarm_rx_prefix_ + "/task", 2,
            &CoverageSearchManager::swarmTaskCb, this);
        remote_state_sub_ = nh.subscribe(swarm_rx_prefix_ + "/state", 10,
            &CoverageSearchManager::remoteStateCb, this);
        map_request_sub_ = nh.subscribe(swarm_rx_prefix_ + "/map_request", 2,
            &CoverageSearchManager::mapRequestCb, this);
        swarm_chunk_cells_x_ = std::max(1, static_cast<int>(std::round(
            swarm_chunk_size_ / coverage_map_.resolution_)));
        swarm_chunk_cells_y_ = swarm_chunk_cells_x_;
        const int chunks_x = (coverage_map_.grid_size_(0) + swarm_chunk_cells_x_ - 1) /
                             swarm_chunk_cells_x_;
        const int chunks_y = (coverage_map_.grid_size_(1) + swarm_chunk_cells_y_ - 1) /
                             swarm_chunk_cells_y_;
        const int chunks = chunks_x * chunks_y;
        const int voxel_count = coverage_map_.grid_size_(0) * coverage_map_.grid_size_(1) *
                                coverage_map_.grid_size_(2);
        swarm_last_sent_evidence_.assign(voxel_count, 0);
        swarm_chunk_revision_.assign(chunks, 0);
        swarm_peer_chunk_revision_.assign(chunks, 0);
        swarm_chunk_snapshot_stage_.assign(chunks, 0);
        swarm_chunk_force_snapshot_.assign(chunks, false);
        swarm_data_timer_ = nh.createTimer(ros::Duration(swarm_chunk_delta_period_),
            &CoverageSearchManager::swarmDataCb, this);
    }

    // Match FUEL's 20 Hz safety/replan monitor while keeping command publication at 100 Hz.
    mainloop_timer_ = nh.createTimer(ros::Duration(0.05), &CoverageSearchManager::mainloopCb, this);
    trajectory_nh_.reset(new ros::NodeHandle(nh));
    trajectory_nh_->setCallbackQueue(&trajectory_callback_queue_);
    trajectory_timer_ = trajectory_nh_->createTimer(
        ros::Duration(0.01), &CoverageSearchManager::trajectoryCb, this);
    trajectory_spinner_.reset(new ros::AsyncSpinner(1, &trajectory_callback_queue_));
    trajectory_spinner_->start();
    frontier_timer_ = nh.createTimer(ros::Duration(0.3), &CoverageSearchManager::frontierCb, this);

    exec_state_ = EXEC_STATE::INIT;
    odom_ready_ = false; drone_ready_ = false; sensor_ready_ = false;
    has_goal_ = false;
    uav_pos_.setZero(); uav_vel_.setZero(); uav_yaw_ = 0.0;
    last_pos_.setZero(); last_move_time_ = ros::Time::now();
    astar_path_idx_ = 0;
    traj_idx_ = 0; has_traj_ = false; traj_dt_ = 0.1;
    frontier_target_idx_ = 0;
    traj_start_time_ = ros::Time::now();
    hover_wait_start_ = ros::Time::now();
    replan_count_ = 0;
    has_goal_ = false;
    has_committed_heading_ = false; committed_heading_ = 0.0;
    has_failed_goal_ = false; failed_goal_.setZero();
    consecutive_plan_failures_ = 0;
    same_goal_replan_count_ = 0;
    goal_deadline_sec_ = 30.0;
    goal_commit_time_ = ros::Time::now();
    cmd_yaw_smoothed_ = 0.0; cmd_yaw_inited_ = false;
    rolling_prepare_in_progress_ = false;
    pending_traj_ = PendingTrajectory();
    completed_pending_traj_ = PendingTrajectory();
    rolling_generation_ = 0;
    rolling_replan_requested_ = false;
    rolling_last_attempt_time_ = ros::Time(0);
    rolling_replan_count_ = 0;
    active_goal_frontier_valid_ = false;
    active_goal_frontier_covered_pending_ = false;
    active_goal_frontier_checked_generation_ = 0;
    rolling_worker_running_ = false;
    traj_point_reach_time_ = ros::Time::now();
    completion_ready_ = false;
    peer_completion_ready_ = false;
    completion_ready_pub_.publish(std_msgs::Bool());
    coverage_25_time_ = ros::Time(0);
    coverage_50_time_ = ros::Time(0);
    coverage_75_time_ = ros::Time(0);
    residual_scan_yaw_ = 0.0;
    last_residual_search_time_ = ros::Time(0);
    cout << GREEN << "[CoverageSearch] init. UAV: " << uav_name_
         << ", Fly height: " << fly_height_
         << ", Sensing range: " << sensing_range_ << TAIL << endl;
}

void CoverageSearchManager::uavStateCb(const prometheus_msgs::UAVState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    uav_state_ = *msg;
    if (uav_state_.connected && uav_state_.armed) drone_ready_ = true;
    else drone_ready_ = false;
    if (uav_state_.odom_valid) odom_ready_ = true;
    else odom_ready_ = false;
    uav_pos_ = Eigen::Vector3d(msg->position[0], msg->position[1], fly_height_);
    uav_vel_ = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    uav_yaw_ = msg->attitude[2];
}

void CoverageSearchManager::uavControlStateCb(const prometheus_msgs::UAVControlState::ConstPtr &msg) {
    uav_control_state_ = *msg;
}

void CoverageSearchManager::globalPclCb(const sensor_msgs::PointCloud2ConstPtr &msg) {
    sensor_ready_ = true;
    coverage_map_.updateFromGlobalPcl(msg);
}

void CoverageSearchManager::localPclCb(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (!odom_ready_) return;
    sensor_ready_ = true;
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = uav_state_.position[0];
    odom.pose.pose.position.y = uav_state_.position[1];
    odom.pose.pose.position.z = uav_state_.position[2];
    odom.pose.pose.orientation = uav_state_.attitude_q;
    coverage_map_.updateFromLocalPcl(msg, odom);
}

void CoverageSearchManager::scanPclCb(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (!odom_ready_) return;
    sensor_ready_ = true;
    // ★ 使用fly_height_作为传感器z值，确保射线投射在正确的z层进行
    Eigen::Vector3d sensor_pos(uav_state_.position[0], uav_state_.position[1],
                                fly_height_);
    double sensor_yaw = uav_yaw_;
    coverage_map_.updateFromScanPcl(msg, sensor_pos, sensor_yaw);
}

void CoverageSearchManager::depthPclCb(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (!odom_ready_) return;
    sensor_ready_ = true;

    Eigen::Matrix3d R_wc;
    Eigen::Vector3d camera_pos;
    bool capture_pose_used = false;
    if (planning_tf_buffer_ && !msg->header.stamp.isZero() &&
        !msg->header.frame_id.empty() && msg->header.frame_id != "world") {
        try {
            const geometry_msgs::TransformStamped tf = planning_tf_buffer_->lookupTransform(
                "world", msg->header.frame_id, msg->header.stamp, ros::Duration(0.02));
            Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                                 tf.transform.rotation.y, tf.transform.rotation.z);
            if (q.norm() > 1e-3) {
                q.normalize();
                R_wc = q.toRotationMatrix();
                camera_pos = Eigen::Vector3d(tf.transform.translation.x,
                                              tf.transform.translation.y,
                                              tf.transform.translation.z);
                capture_pose_used = true;
                ROS_INFO_ONCE("[CoverageSearch] CoverageMap uses capture-time camera TF for depth integration.");
            }
        } catch (const tf2::TransformException &) {
            ROS_WARN_THROTTLE(1.0,
                "[CoverageSearch] No capture-time TF world -> %s at %.3f; using latest state.",
                msg->header.frame_id.c_str(), msg->header.stamp.toSec());
        }
    }
    if (!capture_pose_used) {
        Eigen::Quaterniond q_wb(uav_state_.attitude_q.w,
                                uav_state_.attitude_q.x,
                                uav_state_.attitude_q.y,
                                uav_state_.attitude_q.z);
        Eigen::Matrix3d R_wb;
        if (q_wb.norm() > 1e-3) {
            q_wb.normalize();
            R_wb = q_wb.toRotationMatrix();
        } else {
            double cy = cos(uav_yaw_), sy = sin(uav_yaw_);
            R_wb << cy, -sy, 0,
                    sy,  cy, 0,
                     0,   0, 1;
        }
        Eigen::Matrix3d R_opt_to_body;
        R_opt_to_body << 0,  0, 1,
                        -1,  0, 0,
                         0, -1, 0;
        R_wc = R_wb *
            Eigen::AngleAxisd(camera_pitch_, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            R_opt_to_body;
        const Eigen::Vector3d body_pos(uav_state_.position[0], uav_state_.position[1],
                                       uav_state_.position[2]);
        camera_pos = body_pos + R_wb * camera_offset_;
    }
    coverage_map_.updateFromDepthPcl(msg, camera_pos, R_wc);
}

void CoverageSearchManager::goalCb(const geometry_msgs::PoseStampedConstPtr &msg) {
    if (external_goal_only_) {
        current_goal_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, fly_height_);
        const geometry_msgs::Quaternion &q = msg->pose.orientation;
        current_goal_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        has_goal_ = true;
        current_goal_task_id_ = 0;
        has_traj_ = false;
        pending_traj_ = PendingTrajectory();
        same_goal_replan_count_ = 0;
        goal_commit_time_ = ros::Time::now();
    }
    if (exec_state_ == EXEC_STATE::INIT) {
        exec_state_ = EXEC_STATE::EXPLORING;
        start_time_ = ros::Time::now();
        finish_time_ = ros::Time(0);
        coverage_25_time_ = coverage_50_time_ = coverage_75_time_ = ros::Time(0);
        setCompletionReady(false);
        cout << GREEN << "[CoverageSearch] Triggered! Start exploring." << TAIL << endl;
    }
}

void CoverageSearchManager::remoteStateCb(
    const prometheus_two_uav_coverage_search::SwarmState::ConstPtr &msg) {
    if (!cooperative_mode_ || static_cast<int>(msg->uav_id) == uav_id_) return;
    if (!msg->odom_valid) {
        peer_state_valid_ = false;
        peer_completion_ready_ = false;
        coverage_map_.setDynamicPeerVolume(Eigen::Vector3d::Zero(), false);
        return;
    }
    peer_pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    peer_state_received_ = ros::Time::now();
    peer_state_valid_ = true;
    peer_completion_ready_ = msg->completion_ready;
    coverage_map_.setDynamicPeerVolume(peer_pos_, true);
}

void CoverageSearchManager::peerCollisionReplanCb(const std_msgs::Bool::ConstPtr &msg) {
    if (!cooperative_mode_ || !msg->data) return;
    peer_collision_replan_requested_ = true;
    peer_avoidance_until_ = ros::Time::now() + ros::Duration(peer_avoidance_duration_);
}

void CoverageSearchManager::setCompletionReady(bool ready) {
    if (completion_ready_ == ready) return;
    completion_ready_ = ready;
    std_msgs::Bool msg;
    msg.data = ready;
    completion_ready_pub_.publish(msg);
}

void CoverageSearchManager::frontierCb(const ros::TimerEvent &e) {
    updateFrontiers();
}

void CoverageSearchManager::trajectoryCb(const ros::TimerEvent &) {
    executeRealtimeTrajectory();
}

void CoverageSearchManager::updateFrontiers() {
    if (!sensor_ready_ || !coverage_map_.map_ready_) return;

    coverage_map_.updateDistanceFields();
    frontier_finder_.searchFrontiers(uav_pos_);
    updateLocalFrontierReservations();
    int ftr_count = frontier_finder_.getFrontierCount();
    frontier_finder_.publishFrontiers();
    std::vector<Eigen::Vector3d> frontier_avgs;
    frontier_finder_.getFrontierAverages(frontier_avgs);
    if (ftr_count == 0) {
        hierarchical_grid_.updateFromMap();
    }
    hierarchical_grid_.inputFrontiers(frontier_avgs);
    // [FrontierTiming] outer update log intentionally disabled during replan profiling.
}

void CoverageSearchManager::updateLocalFrontierReservations() {
    const ros::Time now = ros::Time::now();
    local_frontier_reservations_.erase(
        std::remove_if(local_frontier_reservations_.begin(), local_frontier_reservations_.end(),
            [&](const LocalFrontierReservation &reservation) {
                return reservation.task.lease_expire_time <= now;
            }),
        local_frontier_reservations_.end());

    int discovered = 0;
    int created = 0;
    double priority_distance = std::numeric_limits<double>::infinity();
    uint64_t priority_task_id = 0;
    for (auto &frontier : frontier_finder_.frontiers_) {
        if (!frontier.local_discovery) continue;
        frontier.local_discovery = false;
        ++discovered;
        if (reserveLocalFrontier(frontier)) ++created;
        const double distance = (frontier.average.head<2>() - uav_pos_.head<2>()).norm();
        if (distance < priority_distance) {
            priority_distance = distance;
            priority_task_id = frontierTaskId(frontier);
        }
    }
    if (discovered > 0) {
        local_discovery_priority_task_id_ = priority_task_id;
        // Keep executing the old trajectory while a fresh ATSP is solved
        // from its future handoff state.  A new local frontier invalidates a
        // prepared successor, never the active trajectory.
        pending_traj_ = PendingTrajectory();
        ++rolling_generation_;
        rolling_replan_requested_ = true;
        if (cooperative_mode_) publishSwarmFrontiers();
        ROS_INFO("[CoverageSearch] Directly assigned %d local frontier update(s) "
                 "(%d new 8.0s lease(s)); task %llu is next-tour priority.",
                 discovered, created,
                 static_cast<unsigned long long>(priority_task_id));
    }
}

int CoverageSearchManager::chunkIdForAddress(int address) const {
    const int yz = coverage_map_.grid_size_(1) * coverage_map_.grid_size_(2);
    const int x = address / yz;
    const int y = (address % yz) / coverage_map_.grid_size_(2);
    const int chunks_x = (coverage_map_.grid_size_(0) + swarm_chunk_cells_x_ - 1) /
                         swarm_chunk_cells_x_;
    return (y / swarm_chunk_cells_y_) * chunks_x + x / swarm_chunk_cells_x_;
}

void CoverageSearchManager::appendChunk(
    int chunk_id, bool snapshot,
    prometheus_two_uav_coverage_search::SwarmMapChunk &msg) {
    const int chunks_x = (coverage_map_.grid_size_(0) + swarm_chunk_cells_x_ - 1) /
                         swarm_chunk_cells_x_;
    const int chunk_x = chunk_id % chunks_x;
    const int chunk_y = chunk_id / chunks_x;
    const int x0 = chunk_x * swarm_chunk_cells_x_;
    const int x1 = std::min(coverage_map_.grid_size_(0), x0 + swarm_chunk_cells_x_);
    const int y0 = chunk_y * swarm_chunk_cells_y_;
    const int y1 = std::min(coverage_map_.grid_size_(1), y0 + swarm_chunk_cells_y_);
    msg.header.stamp = ros::Time::now();
    msg.source_uav_id = uav_id_;
    msg.map_epoch = map_epoch_;
    msg.chunk_id = chunk_id;
    msg.revision = swarm_chunk_revision_[chunk_id] + 1;
    msg.snapshot = snapshot;
    int known = 0;
    int total = 0;
    for (int x = x0; x < x1; ++x) {
        for (int y = y0; y < y1; ++y) {
            for (int z = 0; z < coverage_map_.grid_size_(2); ++z) {
                const int address = coverage_map_.toAddress(Eigen::Vector3i(x, y, z));
                const int8_t evidence = coverage_map_.getLocalEvidence(address);
                ++total;
                if (evidence != 0) ++known;
                if (snapshot || evidence != swarm_last_sent_evidence_[address]) {
                    msg.addresses.push_back(address);
                    msg.evidence.push_back(evidence);
                    swarm_last_sent_evidence_[address] = evidence;
                }
            }
        }
    }
    msg.known_percent = total == 0 ? 0 : static_cast<uint8_t>(100 * known / total);
}

void CoverageSearchManager::publishSwarmMapDelta() {
    if (!coverage_map_.map_ready_) return;
    for (size_t chunk = 0; chunk < swarm_chunk_revision_.size(); ++chunk) {
        const uint8_t stage = swarm_chunk_snapshot_stage_[chunk];
        const uint8_t threshold = stage == 0 ? 25 : stage == 1 ? 50 : stage == 2 ? 75 : 100;
        prometheus_two_uav_coverage_search::SwarmMapChunk snapshot;
        appendChunk(static_cast<int>(chunk), false, snapshot);
        const bool has_change = !snapshot.addresses.empty();
        const bool reached_threshold = snapshot.known_percent >= threshold && stage < 4;
        if (swarm_chunk_force_snapshot_[chunk] || reached_threshold) {
            prometheus_two_uav_coverage_search::SwarmMapChunk full;
            appendChunk(static_cast<int>(chunk), true, full);
            full.revision = ++swarm_chunk_revision_[chunk];
            map_chunk_pub_.publish(full);
            swarm_chunk_force_snapshot_[chunk] = false;
            if (reached_threshold) ++swarm_chunk_snapshot_stage_[chunk];
        } else if (has_change) {
            snapshot.revision = ++swarm_chunk_revision_[chunk];
            map_chunk_pub_.publish(snapshot);
        }
    }
}

void CoverageSearchManager::publishSwarmFrontiers() {
    prometheus_two_uav_coverage_search::SwarmFrontierArray out;
    out.header.stamp = ros::Time::now();
    out.source_uav_id = uav_id_;
    out.map_epoch = map_epoch_;
    out.revision = ++swarm_frontier_revision_;
    const ros::Time now = ros::Time::now();
    const size_t limit = std::min<size_t>(32, frontier_finder_.frontiers_.size());
    std::vector<bool> selected(frontier_finder_.frontiers_.size(), false);
    std::vector<size_t> selected_indices;
    selected_indices.reserve(limit);
    const auto is_active = [&](const FrontierFinder::FrontierCluster &frontier) {
        for (const auto &reservation : local_frontier_reservations_) {
            if (reservation.active && frontierMatchesTask(frontier, reservation.task)) return true;
        }
        return false;
    };
    const auto is_leased = [&](const FrontierFinder::FrontierCluster &frontier) {
        const auto contains = [&](const prometheus_two_uav_coverage_search::SwarmTaskArray &tasks) {
            for (const auto &task : tasks.tasks) {
                if (task.lease_expire_time > now && frontierMatchesTask(frontier, task)) return true;
            }
            return false;
        };
        return contains(local_swarm_tasks_) || contains(remote_swarm_tasks_);
    };
    for (size_t i = 0; i < frontier_finder_.frontiers_.size(); ++i) {
        if (!is_active(frontier_finder_.frontiers_[i]) &&
            !is_leased(frontier_finder_.frontiers_[i])) continue;
        selected[i] = true;
        selected_indices.push_back(i);
    }
    std::vector<size_t> remaining_indices;
    remaining_indices.reserve(frontier_finder_.frontiers_.size());
    for (size_t i = 0; i < frontier_finder_.frontiers_.size(); ++i) {
        if (!selected[i]) remaining_indices.push_back(i);
    }
    std::sort(remaining_indices.begin(), remaining_indices.end(),
              [&](size_t lhs, size_t rhs) {
                  const auto &a = frontier_finder_.frontiers_[lhs].average;
                  const auto &b = frontier_finder_.frontiers_[rhs].average;
                  const double da = (a.head<2>() - uav_pos_.head<2>()).squaredNorm();
                  const double db = (b.head<2>() - uav_pos_.head<2>()).squaredNorm();
                  return da == db ? lhs < rhs : da < db;
              });
    for (const size_t i : remaining_indices) {
        if (selected_indices.size() >= limit) break;
        selected_indices.push_back(i);
    }
    for (const size_t i : selected_indices) {
        const auto &frontier = frontier_finder_.frontiers_[i];
        if (frontier.viewpoints.empty()) continue;
        prometheus_two_uav_coverage_search::SwarmFrontier item;
        Eigen::Vector3i index;
        coverage_map_.posToIndex(frontier.average, index);
        if (!coverage_map_.isInMapIndex(index)) continue;
        item.chunk_id = chunkIdForAddress(coverage_map_.toAddress(index));
        item.task_id = frontierTaskId(frontier);
        item.frontier_source_uav_id = uav_id_;
        item.cluster_version = frontier.source_revision;
        item.frontier_cell_count = frontier.cells.size();
        item.centroid.x = frontier.average(0); item.centroid.y = frontier.average(1); item.centroid.z = frontier.average(2);
        item.box_min.x = frontier.box_min_(0); item.box_min.y = frontier.box_min_(1); item.box_min.z = frontier.box_min_(2);
        item.box_max.x = frontier.box_max_(0); item.box_max.y = frontier.box_max_(1); item.box_max.z = frontier.box_max_(2);
        const size_t viewpoint_count = std::min<size_t>(3, frontier.viewpoints.size());
        for (size_t view = 0; view < viewpoint_count; ++view) {
            geometry_msgs::Pose pose;
            pose.position.x = frontier.viewpoints[view](0);
            pose.position.y = frontier.viewpoints[view](1);
            pose.position.z = fly_height_;
            const double yaw = view < frontier.viewpoint_yaws.size()
                ? frontier.viewpoint_yaws[view] : uav_yaw_;
            pose.orientation.w = std::cos(0.5 * yaw);
            pose.orientation.z = std::sin(0.5 * yaw);
            item.candidate_viewpoints.push_back(pose);
        }
        item.information_gain = std::max(1, frontier.visib_num);
        for (auto &reservation : local_frontier_reservations_) {
            if (!frontierMatchesTask(frontier, reservation.task)) continue;
            if (reservation.task.lease_expire_time > now) {
                item.reservation_owner_uav_id = uav_id_;
                item.reservation_expire_time = reservation.task.lease_expire_time;
                break;
            }
        }
        out.frontiers.push_back(item);
    }
    local_swarm_frontiers_ = out;
    swarm_frontier_pub_.publish(out);
}

void CoverageSearchManager::publishSwarmBids() {
    const ros::Time now = ros::Time::now();
    if (!last_swarm_bid_time_.isZero() &&
        (now - last_swarm_bid_time_).toSec() < swarm_bid_period_) return;
    last_swarm_bid_time_ = now;

    // Bid liveness is independent of map readiness.  An empty array means
    // "online but no task yet"; no message means a broken bridge/node.
    prometheus_two_uav_coverage_search::SwarmBidArray out;
    out.header.stamp = now;
    out.source_uav_id = uav_id_;
    out.map_epoch = map_epoch_;
    out.frontier_revision = std::max(local_swarm_frontiers_.revision,
                                     remote_swarm_frontiers_.revision);
    out.revision = ++swarm_bid_revision_;
    if (!coverage_map_.map_ready_) {
        swarm_bid_pub_.publish(out);
        return;
    }

    // A task is bid with the actual A* length in this UAV's fused map, not
    // with its Euclidean chord.  This is the quantity the coordinator must
    // compare when walls make one aircraft's apparent "near" task expensive.
    std::map<uint64_t, prometheus_two_uav_coverage_search::SwarmFrontier> tasks;
    std::map<uint64_t, uint32_t> task_sources;
    auto merge = [&tasks, &task_sources](const prometheus_two_uav_coverage_search::SwarmFrontierArray &source) {
        for (const auto &frontier : source.frontiers) {
            const auto found = task_sources.find(frontier.task_id);
            if (found == task_sources.end() || source.source_uav_id < found->second) {
                tasks[frontier.task_id] = frontier;
                task_sources[frontier.task_id] = source.source_uav_id;
            }
        }
    };
    // Prefer our current candidate representation when both maps describe an
    // identical grid viewpoint.  The task ID itself is deterministic.
    merge(local_swarm_frontiers_);
    merge(remote_swarm_frontiers_);

    struct RankedTask {
        prometheus_two_uav_coverage_search::SwarmFrontier frontier;
        double rank;
    };
    std::vector<RankedTask> ranked;
    ranked.reserve(tasks.size());
    const auto has_live_lease = [&](uint64_t task_id) {
        const auto contains = [&](const prometheus_two_uav_coverage_search::SwarmTaskArray &leases,
                                  const ros::Time &received) {
            if (received.isZero() || (now - received).toSec() > 5.0) return false;
            for (const auto &lease : leases.tasks) {
                if (lease.task_id == task_id && lease.lease_expire_time > now) return true;
            }
            return false;
        };
        return contains(local_swarm_tasks_, local_swarm_task_received_) ||
               contains(remote_swarm_tasks_, remote_swarm_task_received_);
    };
    for (const auto &entry : tasks) {
        const auto &frontier = entry.second;
        // A local eight-second reservation is visible to the peer but is not
        // an auction candidate.  The source coordinator converts it directly
        // into a lease.
        if (frontier.reservation_owner_uav_id != 0 &&
            frontier.reservation_expire_time > now) continue;
        if (has_live_lease(frontier.task_id)) continue;
        if (frontier.candidate_viewpoints.empty()) continue;
        const auto &bid_view = frontier.candidate_viewpoints.front();
        const double dx = bid_view.position.x - uav_pos_(0);
        const double dy = bid_view.position.y - uav_pos_(1);
        // Rank only decides which bounded set receives an expensive A* query;
        // it does not become the bid cost.
        ranked.push_back({frontier, std::hypot(dx, dy)});
    }
    std::sort(ranked.begin(), ranked.end(), [](const RankedTask &a, const RankedTask &b) {
        return a.rank < b.rank;
    });

    const size_t limit = std::min<size_t>(std::max(1, swarm_bid_max_tasks_), ranked.size());
    int astar_attempts = 0;
    astar2d_.setSearchTimeout(20.0);
    for (size_t i = 0; i < limit; ++i) {
        const auto &frontier = ranked[i].frontier;
        if (frontier.candidate_viewpoints.empty()) continue;
        const auto &bid_view = frontier.candidate_viewpoints.front();
        Eigen::Vector3d goal(bid_view.position.x, bid_view.position.y, fly_height_);
        double path_len = (goal.head<2>() - uav_pos_.head<2>()).norm();
        bool reachable = astar2d_.isPathTraversable(uav_pos_, goal, &path_len);
        if (!reachable && astar_attempts < swarm_bid_max_astar_) {
            ++astar_attempts;
            std::vector<Eigen::Vector3d> path;
            reachable = astar2d_.search(uav_pos_, goal, path, &path_len);
        }
        prometheus_two_uav_coverage_search::SwarmBid bid;
        bid.task_id = frontier.task_id;
        bid.task_version = out.frontier_revision;
        bid.bidder_uav_id = uav_id_;
        bid.reachable = reachable;
        bid.cost = reachable ? std::max(0.0, path_len)
                             : std::numeric_limits<float>::infinity();
        out.bids.push_back(bid);
    }
    swarm_bid_pub_.publish(out);
}

void CoverageSearchManager::publishSwarmTrajectory() {
    prometheus_two_uav_coverage_search::SwarmTrajectory out;
    out.header.stamp = ros::Time::now();
    out.source_uav_id = uav_id_;
    out.sequence = uav_command_.Command_ID;
    out.active_task_id = has_goal_ ? current_goal_task_id_ : 0;
    out.active_task_distance = has_goal_ && current_goal_task_id_ != 0
        ? (uav_pos_.head<2>() - current_goal_.head<2>()).norm() : 0.0;
    out.dt = traj_dt_;
    const int end = std::min<int>(traj_points_.size(), traj_idx_ + 30);
    for (int i = std::max(0, traj_idx_); i < end; ++i) {
        geometry_msgs::Point point;
        point.x = traj_points_[i](0); point.y = traj_points_[i](1); point.z = traj_points_[i](2);
        out.points.push_back(point);
        geometry_msgs::Vector3 velocity;
        if (i < static_cast<int>(traj_vels_.size())) {
            velocity.x = traj_vels_[i](0); velocity.y = traj_vels_[i](1); velocity.z = traj_vels_[i](2);
        }
        out.velocities.push_back(velocity);
    }
    swarm_traj_pub_.publish(out);
}

void CoverageSearchManager::swarmDataCb(const ros::TimerEvent &) {
    if (!cooperative_mode_) return;
    publishSwarmFrontiers();
    publishSwarmBids();
    publishSwarmMapDelta();
}

void CoverageSearchManager::remoteChunkCb(
    const prometheus_two_uav_coverage_search::SwarmMapChunkConstPtr &msg) {
    if (!cooperative_mode_ || msg->source_uav_id == static_cast<uint32_t>(uav_id_) ||
        msg->map_epoch != static_cast<uint32_t>(map_epoch_) ||
        msg->chunk_id >= swarm_peer_chunk_revision_.size() ||
        msg->addresses.size() != msg->evidence.size()) return;
    const uint32_t have = swarm_peer_chunk_revision_[msg->chunk_id];
    if (!msg->snapshot && msg->revision > have + 1) {
        prometheus_two_uav_coverage_search::SwarmMapRequest request;
        request.header.stamp = ros::Time::now();
        request.requester_uav_id = uav_id_;
        request.map_epoch = map_epoch_;
        request.chunk_id = msg->chunk_id;
        request.have_revision = have;
        map_request_pub_.publish(request);
        return;
    }
    if (msg->revision < have) return;
    const int voxel_count = coverage_map_.grid_size_(0) * coverage_map_.grid_size_(1) *
                            coverage_map_.grid_size_(2);
    bool changed = false;
    Eigen::Vector3i changed_min, changed_max;
    for (size_t i = 0; i < msg->addresses.size(); ++i) {
        if (msg->addresses[i] >= voxel_count ||
            chunkIdForAddress(msg->addresses[i]) != static_cast<int>(msg->chunk_id)) continue;
        if (!coverage_map_.setRemoteEvidence(msg->addresses[i], msg->evidence[i])) continue;
        const int yz = coverage_map_.grid_size_(1) * coverage_map_.grid_size_(2);
        const int x = msg->addresses[i] / yz;
        const int remain = msg->addresses[i] % yz;
        const Eigen::Vector3i idx(x, remain / coverage_map_.grid_size_(2),
                                  remain % coverage_map_.grid_size_(2));
        if (!changed) {
            changed_min = idx;
            changed_max = idx;
            changed = true;
        } else {
            changed_min = changed_min.cwiseMin(idx);
            changed_max = changed_max.cwiseMax(idx);
        }
    }
    if (changed) coverage_map_.markRemoteUpdatedBox(changed_min, changed_max);
    swarm_peer_chunk_revision_[msg->chunk_id] = msg->revision;
}

void CoverageSearchManager::remoteFrontierCb(
    const prometheus_two_uav_coverage_search::SwarmFrontierArrayConstPtr &msg) {
    if (!cooperative_mode_ || msg->source_uav_id == static_cast<uint32_t>(uav_id_) ||
        msg->map_epoch != static_cast<uint32_t>(map_epoch_)) return;
    remote_swarm_frontiers_ = *msg;
    remote_swarm_frontier_received_ = ros::Time::now();
}

void CoverageSearchManager::swarmTaskCb(
    const prometheus_two_uav_coverage_search::SwarmTaskArrayConstPtr &msg) {
    if (!cooperative_mode_ || msg->map_epoch != static_cast<uint32_t>(map_epoch_)) return;
    if (msg->source_uav_id == static_cast<uint32_t>(uav_id_)) {
        local_swarm_tasks_ = *msg;
        local_swarm_task_received_ = ros::Time::now();
    } else {
        remote_swarm_tasks_ = *msg;
        remote_swarm_task_received_ = ros::Time::now();
    }
}

uint64_t CoverageSearchManager::frontierTaskId(
    const FrontierFinder::FrontierCluster &frontier) {
    ROS_ASSERT_MSG(frontier.task_id != 0,
                   "Source frontier must receive a stable task ID before use");
    return frontier.task_id;
}

bool CoverageSearchManager::isFrontierLeasedToSelf(
    const FrontierFinder::FrontierCluster &frontier) {
    return leasedTaskIdForFrontier(frontier) != 0;
}

bool CoverageSearchManager::isFrontierLeasedToPeer(
    const FrontierFinder::FrontierCluster &frontier) {
    if (!cooperative_mode_) return false;
    const ros::Time now = ros::Time::now();
    const auto has_peer_task = [&](const prometheus_two_uav_coverage_search::SwarmTaskArray &tasks,
                                   const ros::Time &received_time) {
        if (received_time.isZero() || (now - received_time).toSec() > 5.0) return false;
        for (const auto &task : tasks.tasks) {
            if (task.winner_uav_id != static_cast<uint32_t>(uav_id_) &&
                task.lease_expire_time > now &&
                (frontierMatchesTask(frontier, task) || frontierOverlapsTask(frontier, task))) {
                return true;
            }
        }
        return false;
    };
    if (has_peer_task(local_swarm_tasks_, local_swarm_task_received_) ||
        has_peer_task(remote_swarm_tasks_, remote_swarm_task_received_)) {
        return true;
    }
    for (const auto &remote : remote_swarm_frontiers_.frontiers) {
        if (remote.reservation_owner_uav_id == 0 ||
            remote.reservation_owner_uav_id == static_cast<uint32_t>(uav_id_) ||
            remote.reservation_expire_time <= now) continue;
        prometheus_two_uav_coverage_search::SwarmTask reservation;
        reservation.task_id = remote.task_id;
        reservation.frontier_source_uav_id = remote.frontier_source_uav_id;
        reservation.centroid = remote.centroid;
        reservation.box_min = remote.box_min;
        reservation.box_max = remote.box_max;
        if (frontierOverlapsTask(frontier, reservation)) return true;
    }
    return false;
}

bool CoverageSearchManager::frontierMatchesTask(
    const FrontierFinder::FrontierCluster &frontier,
    const prometheus_two_uav_coverage_search::SwarmTask &task) {
    return task.frontier_source_uav_id == static_cast<uint32_t>(uav_id_) &&
           frontierTaskId(frontier) == task.task_id;
}

bool CoverageSearchManager::frontierOverlapsTask(
    const FrontierFinder::FrontierCluster &frontier,
    const prometheus_two_uav_coverage_search::SwarmTask &task) const {
    const double dx = frontier.average(0) - task.centroid.x;
    const double dy = frontier.average(1) - task.centroid.y;
    if (std::hypot(dx, dy) > 0.5) return false;
    const double overlap_x = std::max(0.0, std::min(frontier.box_max_(0), task.box_max.x) -
                                      std::max(frontier.box_min_(0), task.box_min.x));
    const double overlap_y = std::max(0.0, std::min(frontier.box_max_(1), task.box_max.y) -
                                      std::max(frontier.box_min_(1), task.box_min.y));
    const double frontier_area = (frontier.box_max_(0) - frontier.box_min_(0)) *
                                 (frontier.box_max_(1) - frontier.box_min_(1));
    const double task_area = (task.box_max.x - task.box_min.x) *
                             (task.box_max.y - task.box_min.y);
    const double smaller_area = std::min(frontier_area, task_area);
    return smaller_area > 1e-6 && overlap_x * overlap_y >= 0.75 * smaller_area;
}

bool CoverageSearchManager::getSelfLeasedRemoteTask(
    uint64_t task_id, prometheus_two_uav_coverage_search::SwarmTask &task) {
    if (!cooperative_mode_) return false;
    const ros::Time now = ros::Time::now();
    const auto find = [&](const prometheus_two_uav_coverage_search::SwarmTaskArray &tasks,
                          const ros::Time &received) {
        if (received.isZero() || (now - received).toSec() > 5.0) return false;
        for (const auto &candidate : tasks.tasks) {
            if (candidate.task_id == task_id &&
                candidate.frontier_source_uav_id != static_cast<uint32_t>(uav_id_) &&
                candidate.winner_uav_id == static_cast<uint32_t>(uav_id_) &&
                candidate.lease_expire_time > now &&
                !candidate.candidate_viewpoints.empty()) {
                task = candidate;
                return true;
            }
        }
        return false;
    };
    return find(local_swarm_tasks_, local_swarm_task_received_) ||
           find(remote_swarm_tasks_, remote_swarm_task_received_);
}

bool CoverageSearchManager::isRemoteTaskViewpointValid(
    const prometheus_two_uav_coverage_search::SwarmTask &task,
    const Eigen::Vector3d &point, double yaw) {
    Eigen::Vector3i index;
    coverage_map_.posToIndex(point, index);
    if (!coverage_map_.isInMap2D(index(0), index(1)) ||
        !coverage_map_.isFree2D(index(0), index(1)) ||
        coverage_map_.getDistance2D(index(0), index(1)) + 1e-3 <
            requiredTrajectoryClearance(point, traj_cut_clearance_)) return false;
    bool candidate_found = false;
    for (const auto &pose : task.candidate_viewpoints) {
        const double pose_yaw = std::atan2(2.0 * (pose.orientation.w * pose.orientation.z +
                                                   pose.orientation.x * pose.orientation.y),
                                           1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
                                                        pose.orientation.z * pose.orientation.z));
        if (std::hypot(point(0) - pose.position.x, point(1) - pose.position.y) < 0.10 &&
            std::fabs(std::atan2(std::sin(yaw - pose_yaw), std::cos(yaw - pose_yaw))) < 0.20) {
            candidate_found = true;
            break;
        }
    }
    if (!candidate_found) return false;
    const Eigen::Vector3d centroid(task.centroid.x, task.centroid.y, task.centroid.z);
    return !coverage_map_.isRayOccluded(point, centroid);
}

bool CoverageSearchManager::hasConfirmedSelfLease(
    const FrontierFinder::FrontierCluster &frontier) {
    if (!cooperative_mode_) return true;
    // A locally sensed frontier is already this UAV's eight-second lease.  It
    // must be eligible for the fresh ATSP immediately; coordinator echo is not
    // a prerequisite.
    if (localReservationTaskIdForFrontier(frontier) != 0) return true;
    const ros::Time now = ros::Time::now();
    if (local_swarm_task_received_.isZero() ||
        (now - local_swarm_task_received_).toSec() > 5.0) return false;
    for (const auto &task : local_swarm_tasks_.tasks) {
        if (task.winner_uav_id == static_cast<uint32_t>(uav_id_) &&
            task.lease_expire_time > now && frontierMatchesTask(frontier, task)) {
            return true;
        }
    }
    return false;
}

uint64_t CoverageSearchManager::leasedTaskIdForFrontier(
    const FrontierFinder::FrontierCluster &frontier) {
    if (!cooperative_mode_) return frontierTaskId(frontier);
    const ros::Time now = ros::Time::now();
    const bool local_fresh = !local_swarm_task_received_.isZero() &&
        (now - local_swarm_task_received_).toSec() <= 5.0;
    const bool remote_fresh = !remote_swarm_task_received_.isZero() &&
        (now - remote_swarm_task_received_).toSec() <= 5.0;
    uint32_t owner = std::numeric_limits<uint32_t>::max();
    uint64_t selected_task_id = 0;
    bool owner_conflict = false;
    const auto merge = [&](const prometheus_two_uav_coverage_search::SwarmTaskArray &tasks,
                           bool fresh) {
        if (!fresh) return;
        for (const auto &task : tasks.tasks) {
            if (task.lease_expire_time <= now || !frontierMatchesTask(frontier, task)) continue;
            if (owner != std::numeric_limits<uint32_t>::max() &&
                task.winner_uav_id != owner) {
                owner_conflict = true;
                continue;
            }
            if (owner == std::numeric_limits<uint32_t>::max()) {
                owner = task.winner_uav_id;
                selected_task_id = task.task_id;
            }
        }
    };
    merge(local_swarm_tasks_, local_fresh);
    merge(remote_swarm_tasks_, remote_fresh);
    if (owner_conflict) return 0;
    if (owner != std::numeric_limits<uint32_t>::max()) {
        return owner == static_cast<uint32_t>(uav_id_) ? selected_task_id : 0;
    }

    const uint64_t local_reservation = localReservationTaskIdForFrontier(frontier);
    if (local_reservation != 0) return local_reservation;
    for (const auto &remote : remote_swarm_frontiers_.frontiers) {
        if (remote.reservation_owner_uav_id == 0 ||
            remote.reservation_owner_uav_id == static_cast<uint32_t>(uav_id_) ||
            remote.reservation_expire_time <= now) continue;
        prometheus_two_uav_coverage_search::SwarmTask reservation;
        reservation.task_id = remote.task_id;
        reservation.frontier_source_uav_id = remote.frontier_source_uav_id;
        reservation.centroid = remote.centroid;
        reservation.box_min = remote.box_min;
        reservation.box_max = remote.box_max;
        if (frontierOverlapsTask(frontier, reservation)) return 0;
    }
    return 0;
}

uint64_t CoverageSearchManager::localReservationTaskIdForFrontier(
    const FrontierFinder::FrontierCluster &frontier) {
    const ros::Time now = ros::Time::now();
    for (const auto &reservation : local_frontier_reservations_) {
        if ((reservation.active || reservation.task.lease_expire_time > now) &&
            frontierMatchesTask(frontier, reservation.task)) {
            return reservation.task.task_id;
        }
    }
    return 0;
}

bool CoverageSearchManager::reserveLocalFrontier(
    const FrontierFinder::FrontierCluster &frontier) {
    const ros::Time now = ros::Time::now();
    for (auto &reservation : local_frontier_reservations_) {
        if (!frontierMatchesTask(frontier, reservation.task)) continue;
        return false;
    }

    LocalFrontierReservation reservation;
    reservation.task.task_id = frontierTaskId(frontier);
    reservation.task.frontier_source_uav_id = uav_id_;
    reservation.task.winner_uav_id = uav_id_;
    reservation.task.cluster_version = frontier.source_revision;
    reservation.task.frontier_cell_count = frontier.cells.size();
    reservation.task.centroid.x = frontier.average(0);
    reservation.task.centroid.y = frontier.average(1);
    reservation.task.centroid.z = frontier.average(2);
    reservation.task.box_min.x = frontier.box_min_(0);
    reservation.task.box_min.y = frontier.box_min_(1);
    reservation.task.box_min.z = frontier.box_min_(2);
    reservation.task.box_max.x = frontier.box_max_(0);
    reservation.task.box_max.y = frontier.box_max_(1);
    reservation.task.box_max.z = frontier.box_max_(2);
    const size_t viewpoint_count = std::min<size_t>(3, frontier.viewpoints.size());
    for (size_t view = 0; view < viewpoint_count; ++view) {
        geometry_msgs::Pose pose;
        pose.position.x = frontier.viewpoints[view](0);
        pose.position.y = frontier.viewpoints[view](1);
        pose.position.z = fly_height_;
        const double yaw = view < frontier.viewpoint_yaws.size()
            ? frontier.viewpoint_yaws[view] : uav_yaw_;
        pose.orientation.w = std::cos(0.5 * yaw);
        pose.orientation.z = std::sin(0.5 * yaw);
        reservation.task.candidate_viewpoints.push_back(pose);
    }
    reservation.task.lease_expire_time = now + ros::Duration(8.0);
    local_frontier_reservations_.push_back(reservation);
    return true;
}

bool CoverageSearchManager::reserveLocalFrontierForGoal(
    uint64_t task_id, const Eigen::Vector3d &goal) {
    for (const auto &frontier : frontier_finder_.frontiers_) {
        if (frontierTaskId(frontier) != task_id) continue;
        bool matches_goal = false;
        for (const auto &viewpoint : frontier.viewpoints) {
            if ((viewpoint.head<2>() - goal.head<2>()).norm() < 0.10) {
                matches_goal = true;
                break;
            }
        }
        if (!matches_goal || leasedTaskIdForFrontier(frontier) != 0 ||
            isFrontierLeasedToPeer(frontier)) continue;
        return reserveLocalFrontier(frontier);
    }
    return false;
}

void CoverageSearchManager::markLocalReservationActive(uint64_t task_id) {
    for (auto &reservation : local_frontier_reservations_) {
        if (reservation.task.task_id == task_id) {
            reservation.active = true;
        }
    }
}

bool CoverageSearchManager::hasActiveLocalReservation(uint64_t task_id) const {
    const ros::Time now = ros::Time::now();
    for (const auto &reservation : local_frontier_reservations_) {
        if (reservation.task.task_id == task_id && reservation.active &&
            reservation.task.lease_expire_time > now) return true;
    }
    return false;
}

void CoverageSearchManager::eraseLocalReservation(uint64_t task_id) {
    local_frontier_reservations_.erase(
        std::remove_if(local_frontier_reservations_.begin(), local_frontier_reservations_.end(),
            [&](const LocalFrontierReservation &reservation) {
                return reservation.task.task_id == task_id;
            }),
        local_frontier_reservations_.end());
}

bool CoverageSearchManager::isAtspTargetValid(const Eigen::Vector3d &target,
                                               double target_yaw, uint64_t task_id) {
    Eigen::Vector3i target_index;
    coverage_map_.posToIndex(target, target_index);
    if (!coverage_map_.isInMap2D(target_index(0), target_index(1)) ||
        !coverage_map_.isFree2D(target_index(0), target_index(1)) ||
        coverage_map_.getDistance2D(target_index(0), target_index(1)) + 1e-3 <
            requiredTrajectoryClearance(target, traj_cut_clearance_)) {
        return false;
    }
    for (const auto &frontier : frontier_finder_.frontiers_) {
        if (frontierTaskId(frontier) != task_id || !hasConfirmedSelfLease(frontier) ||
            isFrontierLeasedToPeer(frontier)) continue;
        for (size_t view = 0; view < frontier.viewpoints.size(); ++view) {
            const double yaw = view < frontier.viewpoint_yaws.size()
                ? frontier.viewpoint_yaws[view] : target_yaw;
            if ((frontier.viewpoints[view].head<2>() - target.head<2>()).norm() < 0.10 &&
                std::fabs(std::atan2(std::sin(yaw - target_yaw),
                                     std::cos(yaw - target_yaw))) < 0.20 &&
                frontier_finder_.isViewpointVisible(frontier, frontier.viewpoints[view], yaw)) {
                return true;
            }
        }
    }
    prometheus_two_uav_coverage_search::SwarmTask remote_task;
    if (!getSelfLeasedRemoteTask(task_id, remote_task)) return false;

    // A coordinator lease alone is not a live frontier.  The source UAV must
    // still advertise the exact entity; an empty/newer source summary removes
    // the viewpoint from our executable queue before its old lease expires.
    const ros::Time now = ros::Time::now();
    if (remote_swarm_frontier_received_.isZero() ||
        (now - remote_swarm_frontier_received_).toSec() > 1.0) return false;
    const auto source_it = std::find_if(remote_swarm_frontiers_.frontiers.begin(),
                                        remote_swarm_frontiers_.frontiers.end(),
        [&](const prometheus_two_uav_coverage_search::SwarmFrontier &frontier) {
            return frontier.task_id == remote_task.task_id &&
                   frontier.frontier_source_uav_id == remote_task.frontier_source_uav_id &&
                   !frontier.candidate_viewpoints.empty();
        });
    return source_it != remote_swarm_frontiers_.frontiers.end() &&
           isRemoteTaskViewpointValid(remote_task, target, target_yaw);
}

bool CoverageSearchManager::currentGoalLeaseValid() {
    if (!cooperative_mode_ || current_goal_task_id_ == 0) return true;
    // Ownership and viewpoint liveness are deliberately separate.  A
    // refreshed/disappeared viewpoint requests a rolling ATSP replan; it is
    // not a reason to stop the old, still collision-free trajectory.
    if (hasActiveLocalReservation(current_goal_task_id_)) return true;
    prometheus_two_uav_coverage_search::SwarmTask remote_task;
    if (getSelfLeasedRemoteTask(current_goal_task_id_, remote_task)) return true;
    const ros::Time now = ros::Time::now();
    if (local_swarm_task_received_.isZero() ||
        (now - local_swarm_task_received_).toSec() > 5.0) return false;
    for (const auto &task : local_swarm_tasks_.tasks) {
        if (task.task_id == current_goal_task_id_ &&
            task.winner_uav_id == static_cast<uint32_t>(uav_id_) &&
            task.lease_expire_time > now) return true;
    }
    return false;
}

bool CoverageSearchManager::captureActiveGoalFrontier() {
    active_goal_frontier_valid_ = false;
    active_goal_frontier_covered_pending_ = false;
    active_goal_frontier_cells_.clear();
    active_goal_frontier_checked_generation_ = frontier_finder_.update_generation_;
    if (current_goal_task_id_ == 0) return false;

    prometheus_two_uav_coverage_search::SwarmTask remote_task;
    if (getSelfLeasedRemoteTask(current_goal_task_id_, remote_task)) {
        active_goal_frontier_ = remote_task;
        // The source owns remote frontier cells, so this UAV must not infer
        // completion from its fused map.  Lease/lifecycle changes invalidate
        // the target through isAtspTargetValid instead.
        return true;
    }

    for (const auto &frontier : frontier_finder_.frontiers_) {
        if (frontierTaskId(frontier) != current_goal_task_id_) continue;
        bool owns_goal = false;
        for (const auto &viewpoint : frontier.viewpoints) {
            if ((viewpoint.head<2>() - current_goal_.head<2>()).norm() < 0.10) {
                owns_goal = true;
                break;
            }
        }
        if (!owns_goal) continue;
        active_goal_frontier_.task_id = current_goal_task_id_;
        active_goal_frontier_.centroid.x = frontier.average(0);
        active_goal_frontier_.centroid.y = frontier.average(1);
        active_goal_frontier_.centroid.z = frontier.average(2);
        active_goal_frontier_.box_min.x = frontier.box_min_(0);
        active_goal_frontier_.box_min.y = frontier.box_min_(1);
        active_goal_frontier_.box_min.z = frontier.box_min_(2);
        active_goal_frontier_.box_max.x = frontier.box_max_(0);
        active_goal_frontier_.box_max.y = frontier.box_max_(1);
        active_goal_frontier_.box_max.z = frontier.box_max_(2);
        active_goal_frontier_cells_ = frontier.cells;
        active_goal_frontier_valid_ = true;
        return true;
    }
    return false;
}

bool CoverageSearchManager::currentGoalFrontierCovered() {
    if (!active_goal_frontier_valid_ || active_goal_frontier_cells_.empty() || !has_goal_)
        return false;
    if (active_goal_frontier_covered_pending_) return true;
    const uint64_t generation = frontier_finder_.update_generation_;
    if (generation == 0 || generation <= active_goal_frontier_checked_generation_) return false;
    active_goal_frontier_checked_generation_ = generation;
    if (!frontier_finder_.isFrontierCellsCovered(active_goal_frontier_cells_, 0.20))
        return false;

    // A periodic successor for the now-covered goal must not win the race
    // against its forced replacement.  Keep this condition latched until a
    // new goal captures its own frontier signature.
    active_goal_frontier_covered_pending_ = true;
    pending_traj_ = PendingTrajectory();
    ++rolling_generation_;
    rolling_replan_requested_ = true;
    ROS_INFO("[CoverageSearch] Active frontier covered; force-replan request latched.");
    return true;
}

void CoverageSearchManager::mapRequestCb(
    const prometheus_two_uav_coverage_search::SwarmMapRequestConstPtr &msg) {
    if (!cooperative_mode_ || msg->requester_uav_id == static_cast<uint32_t>(uav_id_) ||
        msg->map_epoch != static_cast<uint32_t>(map_epoch_) ||
        msg->chunk_id >= swarm_chunk_force_snapshot_.size()) return;
    swarm_chunk_force_snapshot_[msg->chunk_id] = true;
}
// ============================================================
// ★ 主循环 - 简化FSM：INIT → EXPLORING → FINISH
// ============================================================
void CoverageSearchManager::mainloopCb(const ros::TimerEvent &e) {
    const bool peer_fresh = cooperative_mode_ && peer_state_valid_ &&
        (ros::Time::now() - peer_state_received_).toSec() <= peer_state_timeout_;
    if (!peer_fresh) peer_completion_ready_ = false;
    coverage_map_.setDynamicPeerVolume(peer_pos_, peer_fresh);
    const bool peer_avoidance_active = peer_fresh &&
        ros::Time::now() < peer_avoidance_until_;
    coverage_map_.setDynamicPeerAvoidance(peer_pos_, peer_avoidance_active,
                                           peer_avoidance_radius_);
    if (peer_fresh && coverage_map_.map_ready_) {
        if (last_peer_clear_valid_ &&
            (last_peer_clear_pos_ - peer_pos_).norm() > 0.05) {
            coverage_map_.clearDynamicPeerVolume(last_peer_clear_pos_);
        }
        coverage_map_.clearDynamicPeerVolume(peer_pos_);
        last_peer_clear_pos_ = peer_pos_;
        last_peer_clear_valid_ = true;
    } else if (!peer_fresh) {
        last_peer_clear_valid_ = false;
    }

    // UAV ID 标签 — 独立于 FSM 状态，只要节点运行就发布
    if (std::isfinite(uav_pos_(0)) && std::isfinite(uav_pos_(1)) && std::isfinite(uav_pos_(2))) {
        visualization_msgs::Marker label;
        label.header.frame_id = "world";
        label.header.stamp = ros::Time::now();
        label.ns = "uav_label";
        label.id = 0;
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::Marker::ADD;
        label.pose.position.x = uav_pos_(0);
        label.pose.position.y = uav_pos_(1);
        label.pose.position.z = uav_pos_(2) + 0.6;
        label.pose.orientation.w = 1.0;
        label.scale.z = 0.45;
        label.color.r = 0.0;
        label.color.g = 1.0;
        label.color.b = 0.0;
        label.color.a = 1.0;
        label.lifetime = ros::Duration(0.5);
        label.text = "UAV" + std::to_string(uav_id_);
        uav_label_pub_.publish(label);
    }

    bool command_ready = (uav_control_state_.control_state ==
                          prometheus_msgs::UAVControlState::COMMAND_CONTROL);
    if (!odom_ready_ || !drone_ready_ || !sensor_ready_ || !command_ready) {
        static int wait_cnt = 0;
        if (++wait_cnt >= 10) {
            wait_cnt = 0;
            cout << YELLOW << "[CoverageSearch] Waiting... odom:" << odom_ready_
                 << " drone:" << drone_ready_ << " sensor:" << sensor_ready_
                 << " command:" << command_ready << TAIL << endl;
        }
        return;
    }

    if (consumeRealtimeTrajectoryCompletion()) {
        has_traj_ = false;
        has_goal_ = false;
        pending_traj_ = PendingTrajectory();
        rolling_replan_requested_ = false;
        ROS_INFO("[CoverageSearch] Trajectory terminal P/V/A hold converged; select next frontier.");
    }
    if (has_traj_) {
        maintainActiveTrajectory();
    } else {
        disarmRealtimeTrajectory();
    }
    if (cooperative_mode_) publishSwarmTrajectory();

    // 机体真实占据的体积不可能同时是静态障碍；清除自反射/旧深度噪点，
    // 但不清除外部0.35m安全膨胀区。
    coverage_map_.clearRobotVolume(Eigen::Vector3d(
        uav_state_.position[0], uav_state_.position[1], uav_state_.position[2]));
    coverage_map_.publishMap();
    publishCoverageStatus();

    switch (exec_state_) {
    case EXEC_STATE::INIT: {
        if (coverage_map_.map_ready_) {
            // ★ 每1秒搜索一次前沿（不是每0.1s），避免CPU过载
            static ros::Time last_ftr_search = ros::Time::now();
            bool need_search = (ros::Time::now() - last_ftr_search).toSec() > 1.0;
            if (need_search) {
                last_ftr_search = ros::Time::now();
                frontier_finder_.searchFrontiers(uav_pos_);
            }
            int ftr_count = frontier_finder_.getFrontierCount();

            if (auto_start_) {
                // ★ 必须等到至少有一个前沿簇才开始探索
                if (ftr_count > 0) {
                    exec_state_ = EXEC_STATE::EXPLORING;
                    start_time_ = ros::Time::now();
                    finish_time_ = ros::Time(0);
                    coverage_25_time_ = coverage_50_time_ = coverage_75_time_ = ros::Time(0);
                    setCompletionReady(false);
                    // 普通目标选择不使用方向硬过滤。
                    has_committed_heading_ = false;
                    cout << GREEN << "[CoverageSearch] Map ready. " << ftr_count
                         << " frontiers detected. Auto start exploring!" << TAIL << endl;
                } else {
                    // ★ 地图有数据但没检测到前沿：发 Move 到当前 (x,y) + fly_height
                    // 用 Move 而非 Current_Pos_Hover，确保无人机爬升到 fly_height（避免落在地面）
                    uav_command_.header.stamp = ros::Time::now();
                    uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                    uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                    uav_command_.position_ref[0] = uav_pos_(0);
                    uav_command_.position_ref[1] = uav_pos_(1);
                    uav_command_.position_ref[2] = fly_height_;
                    uav_command_.yaw_ref = uav_yaw_;
                    uav_command_.Command_ID++;
                    uav_cmd_pub_.publish(uav_command_);
                }
            } else {
                // 等待外部触发
                uav_command_.header.stamp = ros::Time::now();
                uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                uav_command_.position_ref[0] = uav_pos_(0);
                uav_command_.position_ref[1] = uav_pos_(1);
                uav_command_.position_ref[2] = fly_height_;
                uav_command_.yaw_ref = uav_yaw_;
                uav_command_.Command_ID++;
                uav_cmd_pub_.publish(uav_command_);
            }
        }
        break;
    }

    case EXEC_STATE::EXPLORING: {
        // ★★★ 核心循环：前沿簇驱动探索 ★★★
        // 简化逻辑：轨迹执行 → 完成 → 重新搜索前沿 → 选目标 → 规划 → 执行
        const double coverage = coverage_map_.getKnownSpaceRatio();
        const auto finish_if_complete = [&]() {
            if (coverage <= completion_known_ratio_ || !frontier_targets_.empty()) {
                return false;
            }
            setCompletionReady(true);
            // Freeze statistics at the first successful completion condition,
            // even if cooperative shutdown is still waiting for the peer.
            if (finish_time_.isZero()) finish_time_ = ros::Time::now();
            const bool peer_ready = peer_fresh && peer_completion_ready_;
            has_traj_ = false;
            has_goal_ = false;
            pending_traj_ = PendingTrajectory();
            disarmRealtimeTrajectory();
            if (!cooperative_mode_ || peer_ready) {
                exec_state_ = EXEC_STATE::FINISH;
                const double total = start_time_.isZero() ? 0.0 :
                    (finish_time_ - start_time_).toSec();
                const auto stage_time = [&](const ros::Time &stamp) {
                    return stamp.isZero() || start_time_.isZero() ? -1.0 :
                        (stamp - start_time_).toSec();
                };
                cout << GREEN << "[CoverageSearch] FINISHED: coverage="
                     << coverage * 100.0 << "%, total=" << total
                     << "s, T25=" << stage_time(coverage_25_time_)
                     << "s, T50=" << stage_time(coverage_50_time_)
                     << "s, T75=" << stage_time(coverage_75_time_) << "s"
                     << TAIL << endl;
            } else {
                uav_command_.header.stamp = ros::Time::now();
                uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                uav_command_.position_ref[0] = uav_pos_(0);
                uav_command_.position_ref[1] = uav_pos_(1);
                uav_command_.position_ref[2] = fly_height_;
                uav_command_.yaw_ref = uav_yaw_;
                uav_command_.Command_ID++;
                uav_cmd_pub_.publish(uav_command_);
                ROS_INFO_THROTTLE(2.0,
                    "[CoverageSearch] Coverage %.2f%% > 99%%; waiting for peer completion.",
                    coverage * 100.0);
            }
            return true;
        };
        setCompletionReady(false);

        // 安全保护
        if (replan_count_ > 100) {
            cout << YELLOW << "[CoverageSearch] Too many replans (" << replan_count_
                 << "); clear stale target state and enter residual recovery instead of false finish."
                 << TAIL << endl;
            has_goal_ = false;
            has_traj_ = false;
            frontier_targets_.clear();
            frontier_target_yaws_.clear();
            frontier_target_task_ids_.clear();
            current_goal_task_id_ = 0;
            frontier_target_idx_ = 0;
            atsp_tour_active_ = false;
            replan_count_ = 0;
        }

        if (peer_collision_replan_requested_) {
            peer_collision_replan_requested_ = false;
            if (has_traj_ && has_goal_) {
                ++rolling_generation_;  // Invalidate a successor made before the peer became blocked.
                pending_traj_ = PendingTrajectory();
                has_traj_ = false;
                disarmRealtimeTrajectory();
                ROS_WARN("[CoverageSearch] Peer trajectory conflict: replan current goal with peer exclusion %.2fm.",
                         peer_avoidance_radius_);
            }
        }

        // 1. 执行当前轨迹至目标；到达后才按真实状态重新选择下一视点。
        if (has_traj_) break;

        // 当前轨迹因完成或安全检查退出后，不能把上一条轨迹的待交接结果带入重规划。
        pending_traj_ = PendingTrajectory();

        // 协同模式的任务由外部共识竞价指定；没有获胜任务时原地保持，不能退回单机贪心选点。
        if (external_goal_only_ && !has_goal_) {
            uav_command_.header.stamp = ros::Time::now();
            uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command_.position_ref[0] = uav_pos_(0);
            uav_command_.position_ref[1] = uav_pos_(1);
            uav_command_.position_ref[2] = fly_height_;
            uav_command_.yaw_ref = uav_yaw_;
            uav_command_.Command_ID++;
            uav_cmd_pub_.publish(uav_command_);
            break;
        }

        // ★ 改动1：若仍有目标(轨迹被挡废弃但目标保留)，先用更新后的地图对"同一目标"重规划 A*
        //   这样局部新增障碍只会触发绕行，不会换目标 → 避免前后跳变
        //   但限次：连续重规划超 5 次仍无前进 → 放弃该目标
        //   ★★ 硬截止：目标选定超 goal_deadline_sec_ 秒仍未到达 → 强制放弃
        //      防止无人机在不可达目标上"挪动一点点就清零计数"而永远不放弃
        const auto discard_atsp_tour = [&]() {
            if (!atsp_tour_active_) return;
            frontier_targets_.clear();
            frontier_target_yaws_.clear();
            frontier_target_task_ids_.clear();
            frontier_target_idx_ = 0;
            atsp_tour_active_ = false;
        };
        if (has_goal_) {
            double goal_elapsed = (ros::Time::now() - goal_commit_time_).toSec();
            std::string cut_reason;
            if (currentGoalUnsafe(cut_reason)) {
                abortCurrentGoalForSafety(cut_reason);
            } else if (goal_elapsed > goal_deadline_sec_) {
                // ★ 硬截止：超时仍未到达 → 强制放弃
                cout << YELLOW << "[CoverageSearch] Goal deadline (" << goal_deadline_sec_
                     << "s) exceeded for (" << current_goal_(0) << "," << current_goal_(1)
                     << "). Give up, select next frontier." << TAIL << endl;
                failed_goal_ = current_goal_;
                failed_goal_time_ = ros::Time::now();
                has_failed_goal_ = true;
                has_goal_ = false;
                discard_atsp_tour();
                same_goal_replan_count_ = 0;
                // 落到 selectNextFrontier
            } else {
                same_goal_replan_count_++;
                if (same_goal_replan_count_ > 5) {
                    cout << YELLOW << "[CoverageSearch] Same goal replanned " << same_goal_replan_count_
                         << " times without progress. Give up goal." << TAIL << endl;
                    failed_goal_ = current_goal_;
                    failed_goal_time_ = ros::Time::now();
                    has_failed_goal_ = true;
                    has_goal_ = false;
                    discard_atsp_tour();
                    same_goal_replan_count_ = 0;
                } else if (planPathToGoal()) {
                    generateBsplineTraj();
                    if (has_traj_) {
                        consecutive_plan_failures_ = 0;
                        cout << GREEN << "[CoverageSearch] Replanned to SAME goal ("
                             << current_goal_(0) << "," << current_goal_(1) << ") around new obstacle, traj="
                             << traj_points_.size() << " pts" << TAIL << endl;
                        break;
                    }
                    failed_goal_ = current_goal_;
                    failed_goal_time_ = ros::Time::now();
                    has_failed_goal_ = true;
                    has_goal_ = false;
                    discard_atsp_tour();
                    same_goal_replan_count_ = 0;
                    cout << YELLOW << "[CoverageSearch] Re-plan to goal failed, give up. Will re-select." << TAIL << endl;
                } else {
                    failed_goal_ = current_goal_;
                    failed_goal_time_ = ros::Time::now();
                    has_failed_goal_ = true;
                    has_goal_ = false;
                    discard_atsp_tour();
                    same_goal_replan_count_ = 0;
                    cout << YELLOW << "[CoverageSearch] Re-plan to goal failed (A*), give up. Will re-select." << TAIL << endl;
                }
            }
        }

        // 2. 没有目标 → 先消费最新AABB地图更新，再选择目标
        updateFrontiers();
        if (!cooperative_mode_ && frontier_finder_.getFrontierCount() == 0 &&
            coverage_map_.getKnownSpaceRatio() < completion_known_ratio_ &&
            (ros::Time::now() - last_residual_search_time_).toSec() >= 2.0) {
            last_residual_search_time_ = ros::Time::now();
            frontier_finder_.searchResidualFrontiers(uav_pos_);
        }
        auto select_t0 = std::chrono::high_resolution_clock::now();
        // ATSP is re-solved from the current state; its previous tour tail is
        // never an executable target queue.
        bool selected_frontier = selectNextFrontier();
        auto select_t1 = std::chrono::high_resolution_clock::now();
        double select_ms = std::chrono::duration<double, std::milli>(select_t1 - select_t0).count();
        if (select_ms > 120.0) {
            ROS_WARN_THROTTLE(2.0, "[CoverageSearch] target selection took %.1f ms, frontiers=%d",
                              select_ms, frontier_finder_.getFrontierCount());
        }
        if (!selected_frontier && finish_if_complete()) break;
        if (!selected_frontier) {
            int current_frontiers = frontier_finder_.getFrontierCount();
            if (current_frontiers > 0) {
                setCompletionReady(false);
                double observe_yaw = uav_yaw_;
                double best_dist = 1e9;
                Eigen::Vector3d recovery_target = Eigen::Vector3d::Zero();
                for (const auto &frontier : frontier_finder_.frontiers_) {
                    if (atsp_enabled_ && !hasConfirmedSelfLease(frontier)) continue;
                    if (!isFrontierLeasedToSelf(frontier)) continue;
                    const Eigen::Vector3d &avg = frontier.average;
                    double dist = (avg.head<2>() - uav_pos_.head<2>()).norm();
                    if (dist < best_dist && dist > 1e-3) {
                        best_dist = dist;
                        recovery_target = avg;
                        observe_yaw = atan2(avg(1) - uav_pos_(1), avg(0) - uav_pos_(0));
                    }
                }

                Eigen::Vector3d recovery_goal;
                std::vector<Eigen::Vector3d> recovery_path;
                if (best_dist < 1e9 && astar2d_.findReachableApproach(
                        uav_pos_, recovery_target, recovery_goal, recovery_path)) {
                    current_goal_ = recovery_goal;
                    current_goal_yaw_ = observe_yaw;
                    current_goal_task_id_ = 0;
                    astar_path_ = recovery_path;
                    has_goal_ = true;
                    same_goal_replan_count_ = 0;
                    goal_commit_time_ = ros::Time::now();
                    generateBsplineTraj();
                    if (has_traj_) {
                        ROS_INFO("[CoverageSearch] Recovery approach: safe goal=(%.2f,%.2f), "
                                 "toward frontier=(%.2f,%.2f), path=%zu.",
                                 recovery_goal(0), recovery_goal(1), recovery_target(0),
                                 recovery_target(1), recovery_path.size());
                        break;
                    }
                    has_goal_ = false;
                }

                uav_command_.header.stamp = ros::Time::now();
                uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                uav_command_.position_ref[0] = uav_pos_(0);
                uav_command_.position_ref[1] = uav_pos_(1);
                uav_command_.position_ref[2] = fly_height_;
                uav_command_.yaw_ref = observe_yaw;
                uav_command_.Command_ID++;
                uav_cmd_pub_.publish(uav_command_);

                static int observe_log_cnt = 0;
                if (++observe_log_cnt >= 10) {
                    observe_log_cnt = 0;
                    cout << YELLOW << "[CoverageSearch] Frontiers exist but no reachable viewpoint yet. "
                         << "Observe nearest frontier in place, yaw=" << observe_yaw << TAIL << endl;
                }
                break;
            }

            // No frontier and coverage is still at or below 99%: keep observing.
            residual_scan_yaw_ = std::atan2(
                std::sin(residual_scan_yaw_ + residual_scan_yaw_rate_ * 0.1),
                std::cos(residual_scan_yaw_ + residual_scan_yaw_rate_ * 0.1));
            uav_command_.header.stamp = ros::Time::now();
            uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command_.position_ref[0] = uav_pos_(0);
            uav_command_.position_ref[1] = uav_pos_(1);
            uav_command_.position_ref[2] = fly_height_;
            uav_command_.yaw_ref = residual_scan_yaw_;
            uav_command_.Command_ID++;
            uav_cmd_pub_.publish(uav_command_);
            ROS_WARN_THROTTLE(2.0,
                "[CoverageSearch] Residual recovery scan: coverage=%.2f%%, completion > %.2f%%.",
                coverage * 100.0, completion_known_ratio_ * 100.0);
            break;
        }
        // Coverage has not reached 99%, so keep normal frontier planning.

        // 3. 选到了前沿目标，逐个尝试规划，直到成功
        while (frontier_target_idx_ < (int)frontier_targets_.size()) {
            current_goal_ = frontier_targets_[frontier_target_idx_];
            current_goal_(2) = fly_height_;
            current_goal_yaw_ = frontier_target_yaws_[frontier_target_idx_];
            current_goal_task_id_ = frontier_target_task_ids_[frontier_target_idx_];
            has_goal_ = true;
            const bool reserved_free_goal =
                reserveLocalFrontierForGoal(current_goal_task_id_, current_goal_);
            same_goal_replan_count_ = 0;  // 新目标，重置同目标重规划计数
            goal_commit_time_ = ros::Time::now();  // ★ 记录目标选定时刻，用于硬截止

            bool path_found = planPathToGoal();
            if (path_found) {
                generateBsplineTraj();
                if (has_traj_) {
                    if (!captureActiveGoalFrontier()) {
                        ROS_WARN("[CoverageSearch] Active frontier signature unavailable; "
                                 "rolling task-completion check is disabled for this goal.");
                    }
                    markLocalReservationActive(current_goal_task_id_);
                    consecutive_plan_failures_ = 0;
                    if (!has_committed_heading_) {
                        Eigen::Vector3d to_goal = current_goal_ - uav_pos_;
                        to_goal(2) = 0.0;
                        if (to_goal.norm() > 0.2) {
                            committed_heading_ = atan2(to_goal(1), to_goal(0));
                            has_committed_heading_ = true;
                        }
                    }
                    cout << GREEN << "[CoverageSearch] Heading to frontier #"
                         << frontier_target_idx_ << " ("
                         << current_goal_(0) << "," << current_goal_(1) << ") dist="
                         << (uav_pos_.head<2>() - current_goal_.head<2>()).norm()
                         << "m, clearance_tier="
                         << requiredTrajectoryClearance(current_goal_, traj_cut_clearance_)
                         << "m, traj=" << traj_points_.size() << " pts" << TAIL << endl;
                    break;
                }
                cout << YELLOW << "[CoverageSearch] Frontier #" << frontier_target_idx_
                     << " has an A* path, but trajectory generation rejected it. Trying next."
                     << TAIL << endl;
            } else {
                cout << YELLOW << "[CoverageSearch] Frontier #" << frontier_target_idx_
                     << " has no safe A* path. Trying next." << TAIL << endl;
            }
            if (reserved_free_goal) eraseLocalReservation(current_goal_task_id_);
            if (atsp_tour_active_) {
                // A failed edge invalidates the directed cost model.  Do not
                // skip ahead in a stale tour; rebuild it from the latest map.
                frontier_targets_.clear();
                frontier_target_yaws_.clear();
                frontier_target_task_ids_.clear();
                frontier_target_idx_ = 0;
                atsp_tour_active_ = false;
                has_goal_ = false;
                break;
            }
            frontier_target_idx_++;
            has_goal_ = false;
        }

        // 所有候选目标都规划失败 → 悬停等待地图更新。
        if (frontier_target_idx_ >= (int)frontier_targets_.size()) {
            consecutive_plan_failures_++;
            cout << YELLOW << "[CoverageSearch] Selected targets produced no executable trajectory ("
                 << consecutive_plan_failures_ << "). Wait for map update." << TAIL << endl;
            // 悬停在当前位置(用 Move 到 fly_height，确保不掉高)
            uav_command_.header.stamp = ros::Time::now();
            uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Move;
            uav_command_.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
            uav_command_.position_ref[0] = uav_pos_(0);
            uav_command_.position_ref[1] = uav_pos_(1);
            uav_command_.position_ref[2] = fly_height_;
            uav_command_.yaw_ref = uav_yaw_;
            uav_command_.Command_ID++;
            uav_cmd_pub_.publish(uav_command_);
            if (consecutive_plan_failures_ >= 2) {
                has_committed_heading_ = false;
                consecutive_plan_failures_ = 0;
            }
            frontier_targets_.clear();
            frontier_target_yaws_.clear();
            frontier_target_task_ids_.clear();
            frontier_target_idx_ = 0;
            current_goal_task_id_ = 0;
            replan_count_++;
        }
        break;
    }

    case EXEC_STATE::FINISH: {
        uav_command_.header.stamp = ros::Time::now();
        uav_command_.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
        uav_command_.Command_ID++;
        uav_cmd_pub_.publish(uav_command_);
        ROS_INFO_THROTTLE(2.0, "[CoverageSearch] FINISHED! Holding position; no auto landing.");
        break;
    }
    }

    publishVisualization();
}

void CoverageSearchManager::publishVisualization() {
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "coverage_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.1;
    path_marker.color.r = 0.0; path_marker.color.g = 0.0;
    path_marker.color.b = 1.0; path_marker.color.a = 1.0;
    path_marker.lifetime = ros::Duration(2.0);

    if (has_goal_) {
        geometry_msgs::Point p;
        p.x = uav_pos_(0); p.y = uav_pos_(1); p.z = fly_height_;
        path_marker.points.push_back(p);
        p.x = current_goal_(0); p.y = current_goal_(1); p.z = fly_height_;
        path_marker.points.push_back(p);
    }
    path_vis_pub_.publish(path_marker);

    // 发布B样条轨迹可视化
    if (has_traj_ && !traj_points_.empty()) {
        visualization_msgs::Marker traj_marker;
        traj_marker.header.frame_id = "world";
        traj_marker.header.stamp = ros::Time::now();
        traj_marker.ns = "bspline_traj";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
        traj_marker.action = visualization_msgs::Marker::ADD;
        traj_marker.scale.x = 0.08;
        traj_marker.color.r = 1.0; traj_marker.color.g = 0.5;
        traj_marker.color.b = 0.0; traj_marker.color.a = 1.0;
        traj_marker.lifetime = ros::Duration(1.0);

        geometry_msgs::Point p;
        for (int i = max(0, traj_idx_ - 5); i < (int)traj_points_.size(); i++) {
            p.x = traj_points_[i](0); p.y = traj_points_[i](1); p.z = fly_height_;
            traj_marker.points.push_back(p);
        }
        path_vis_pub_.publish(traj_marker);
    }

    // ★ 发布A*路径可视化（已删除，只保留蓝色路径线和橙色B样条轨迹）

    // ★ 实际目标视点：洋红色球和同色偏航箭头。
    if (has_goal_) {
        visualization_msgs::Marker goal_marker;
        goal_marker.header.frame_id = "world";
        goal_marker.header.stamp = ros::Time::now();
        goal_marker.ns = "goal_point";
        goal_marker.id = 0;
        goal_marker.type = visualization_msgs::Marker::SPHERE;
        goal_marker.action = visualization_msgs::Marker::ADD;
        goal_marker.pose.position.x = current_goal_(0);
        goal_marker.pose.position.y = current_goal_(1);
        goal_marker.pose.position.z = fly_height_;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.scale.x = 0.20; goal_marker.scale.y = 0.20; goal_marker.scale.z = 0.20;
        goal_marker.color.r = 1.0; goal_marker.color.g = 0.0;
        goal_marker.color.b = 1.0; goal_marker.color.a = 1.0;
        goal_marker.lifetime = ros::Duration(2.0);
        path_vis_pub_.publish(goal_marker);

        visualization_msgs::Marker goal_yaw_marker;
        goal_yaw_marker.header = goal_marker.header;
        goal_yaw_marker.ns = "goal_view_yaw";
        goal_yaw_marker.id = 0;
        goal_yaw_marker.type = visualization_msgs::Marker::ARROW;
        goal_yaw_marker.action = visualization_msgs::Marker::ADD;
        goal_yaw_marker.scale.x = 0.04;
        goal_yaw_marker.scale.y = 0.10;
        goal_yaw_marker.scale.z = 0.13;
        goal_yaw_marker.color = goal_marker.color;
        goal_yaw_marker.lifetime = ros::Duration(2.0);
        geometry_msgs::Point p0, p1;
        p0.x = current_goal_(0); p0.y = current_goal_(1); p0.z = fly_height_;
        p1.x = p0.x + 0.55 * cos(current_goal_yaw_);
        p1.y = p0.y + 0.55 * sin(current_goal_yaw_);
        p1.z = p0.z;
        goal_yaw_marker.points.push_back(p0);
        goal_yaw_marker.points.push_back(p1);
        path_vis_pub_.publish(goal_yaw_marker);
    }

    // ★ 发布轨迹点标记（橙色小球列表，当前执行点高亮为白色）
    if (has_traj_ && !traj_points_.empty()) {
        visualization_msgs::Marker tp_marker;
        tp_marker.header.frame_id = "world";
        tp_marker.header.stamp = ros::Time::now();
        tp_marker.ns = "traj_points";
        tp_marker.id = 0;
        tp_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        tp_marker.action = visualization_msgs::Marker::ADD;
        tp_marker.scale.x = 0.15; tp_marker.scale.y = 0.15; tp_marker.scale.z = 0.15;
        tp_marker.lifetime = ros::Duration(1.0);
        for (int i = traj_idx_; i < (int)traj_points_.size(); i++) {
            geometry_msgs::Point p;
            p.x = traj_points_[i](0); p.y = traj_points_[i](1); p.z = fly_height_;
            tp_marker.points.push_back(p);
            std_msgs::ColorRGBA c;
            if (i == traj_idx_) {
                c.r = 1.0; c.g = 1.0; c.b = 1.0; c.a = 1.0;  // 当前点白色
            } else {
                c.r = 1.0; c.g = 0.5; c.b = 0.0; c.a = 0.6;  // 其他点橙色
            }
            tp_marker.colors.push_back(c);
        }
        path_vis_pub_.publish(tp_marker);
    }

}

void CoverageSearchManager::publishCameraFov() {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.ns = "depth_camera_fov";
    mk.id = 0;
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.025;
    mk.color.r = 0.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 0.95;
    mk.lifetime = ros::Duration(0.0);

    Eigen::Quaterniond q_wb(uav_state_.attitude_q.w,
                            uav_state_.attitude_q.x,
                            uav_state_.attitude_q.y,
                            uav_state_.attitude_q.z);
    Eigen::Matrix3d R_wb;
    if (q_wb.norm() > 1e-3) {
        q_wb.normalize();
        R_wb = q_wb.toRotationMatrix();
    } else {
        const double yaw = uav_state_.attitude[2];
        const double cy = cos(yaw), sy = sin(yaw);
        R_wb << cy, -sy, 0,
                sy,  cy, 0,
                 0,   0, 1;
    }

    Eigen::Matrix3d R_opt_to_body;
    R_opt_to_body << 0,  0, 1,
                    -1,  0, 0,
                     0, -1, 0;
    Eigen::Matrix3d R_mount =
        Eigen::AngleAxisd(camera_pitch_, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d R_wc = R_wb * R_mount * R_opt_to_body;
    Eigen::Vector3d body_pos(uav_state_.position[0], uav_state_.position[1], uav_state_.position[2]);
    Eigen::Vector3d origin = body_pos + R_wb * camera_offset_;

    double z = std::min(fov_vis_range_, sensing_range_);
    double hx = z * tan(sensing_fov_h_ * 0.5);
    double hy = z * tan(sensing_fov_v_ * 0.5);
    std::vector<Eigen::Vector3d> corners_cam = {
        Eigen::Vector3d(-hx, -hy, z),
        Eigen::Vector3d( hx, -hy, z),
        Eigen::Vector3d( hx,  hy, z),
        Eigen::Vector3d(-hx,  hy, z)
    };

    std::vector<Eigen::Vector3d> corners_w;
    for (auto &p_cam : corners_cam) corners_w.push_back(origin + R_wc * p_cam);

    auto addLine = [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        geometry_msgs::Point p;
        p.x = a(0); p.y = a(1); p.z = a(2); mk.points.push_back(p);
        p.x = b(0); p.y = b(1); p.z = b(2); mk.points.push_back(p);
    };
    for (auto &c : corners_w) addLine(origin, c);
    for (int i = 0; i < 4; ++i) addLine(corners_w[i], corners_w[(i + 1) % 4]);

    fov_vis_pub_.publish(mk);
}

void CoverageSearchManager::publishCoverageStatus() {
    if (cooperative_mode_ && global_coverage_source_ != 0 && uav_id_ != global_coverage_source_) return;
    if (start_time_.isZero()) return;
    double coverage = coverage_map_.getKnownSpaceRatio();
    const ros::Time now = ros::Time::now();
    if (coverage >= 0.25 && coverage_25_time_.isZero()) coverage_25_time_ = now;
    if (coverage >= 0.50 && coverage_50_time_.isZero()) coverage_50_time_ = now;
    if (coverage >= 0.75 && coverage_75_time_.isZero()) coverage_75_time_ = now;
    const ros::Time end_time = finish_time_.isZero() ? now : finish_time_;
    const double duration = (end_time - start_time_).toSec();
    int frontier_count = frontier_finder_.getFrontierCount();

    static int print_cnt = 0;
    if (++print_cnt >= 4) {
        print_cnt = 0;
        cout << GREEN << (cooperative_mode_ ? "[CoverageSearch] Global fused" : "[CoverageSearch]")
             << " State: " << exec_state_
             << ", Known: " << coverage * 100.0 << "%"
             << ", Frontiers: " << frontier_count
             << ", Duration: " << duration << "s"
             << ", T25: " << (coverage_25_time_.isZero() ? -1.0 : (coverage_25_time_ - start_time_).toSec()) << "s"
             << ", T50: " << (coverage_50_time_.isZero() ? -1.0 : (coverage_50_time_ - start_time_).toSec()) << "s"
             << ", T75: " << (coverage_75_time_.isZero() ? -1.0 : (coverage_75_time_ - start_time_).toSec()) << "s"
             << ", Ready: " << completion_ready_
             << ", Peer ready: " << peer_completion_ready_
             << ", Replan: " << replan_count_ << TAIL << endl;
    }
}
