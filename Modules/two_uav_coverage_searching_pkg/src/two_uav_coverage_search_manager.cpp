#include "two_uav_coverage_search.h"
#include "two_uav_auction_protocol.h"

#include <map>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

CoverageSearchManager::~CoverageSearchManager() {
    trajectory_timer_.stop();
    raw_heartbeat_timer_.stop();
    if (trajectory_spinner_) trajectory_spinner_->stop();
    if (heartbeat_spinner_) heartbeat_spinner_->stop();
    if (auction_spinner_) auction_spinner_->stop();
    {
        std::lock_guard<std::mutex> lock(auction_worker_mutex_);
        auction_worker_stop_ = true;
    }
    auction_worker_cv_.notify_one();
    if (auction_worker_.joinable()) auction_worker_.join();
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
    nh.param("coverage_search/atsp_enabled", atsp_enabled_, true);
    nh.param("coverage_search/atsp_max_clusters", atsp_max_clusters_, 5);
    nh.param("coverage_search/atsp_refine_clusters", atsp_refine_clusters_, 3);
    nh.param("coverage_search/atsp_viewpoints_per_cluster", atsp_viewpoints_per_cluster_, 3);
    nh.param("coverage_search/atsp_edge_timeout_ms", atsp_edge_timeout_ms_, 100.0);
    nh.param("coverage_search/atsp_planning_budget_ms", atsp_planning_budget_ms_, 250.0);
    nh.param("coverage_search/auction_astar_timeout_ms", auction_astar_timeout_ms_, 50.0);
    atsp_max_clusters_ = std::max(1, std::min(5, atsp_max_clusters_));
    atsp_refine_clusters_ = std::max(1, std::min(atsp_max_clusters_, atsp_refine_clusters_));
    atsp_viewpoints_per_cluster_ = std::max(1, atsp_viewpoints_per_cluster_);
    atsp_edge_timeout_ms_ = std::max(20.0, atsp_edge_timeout_ms_);
    atsp_planning_budget_ms_ = std::max(50.0, atsp_planning_budget_ms_);
    auction_astar_timeout_ms_ = std::max(20.0, auction_astar_timeout_ms_);
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
        swarm_traj_pub_ = nh.advertise<prometheus_two_uav_coverage_search::SwarmTrajectory>(
            uav_name_ + "/prometheus/coverage_search/swarm_trajectory", 2);
        auction_assignment_pub_ = nh.advertise<prometheus_two_uav_coverage_search::SwarmAuctionAssignment>(
            uav_name_ + "/prometheus/coverage_search/auction_assignment", 2);
        frontier_transfer_ack_pub_ = nh.advertise<prometheus_two_uav_coverage_search::SwarmFrontierTransferAck>(
            swarm_tx_prefix_ + "/frontier_transfer_ack", 2);
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
        auction_nh_.reset(new ros::NodeHandle(nh));
        auction_nh_->setCallbackQueue(&auction_callback_queue_);
        auction_task_set_sub_ = auction_nh_->subscribe(
            uav_name_ + "/prometheus/coverage_search/auction_task_set", 2,
            &CoverageSearchManager::auctionTaskSetCb, this);
        remote_frontier_transfer_ack_sub_ = nh.subscribe(
            swarm_rx_prefix_ + "/frontier_transfer_ack", 2,
            &CoverageSearchManager::remoteFrontierTransferAckCb, this);
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
    heartbeat_nh_.reset(new ros::NodeHandle(nh));
    heartbeat_nh_->setCallbackQueue(&heartbeat_callback_queue_);
    raw_heartbeat_timer_ = heartbeat_nh_->createTimer(
        ros::Duration(0.05), &CoverageSearchManager::rawHeartbeatCb, this);
    heartbeat_spinner_.reset(new ros::AsyncSpinner(1, &heartbeat_callback_queue_));
    heartbeat_spinner_->start();
    auction_worker_ = std::thread(&CoverageSearchManager::auctionWorkerLoop, this);
    if (auction_nh_) {
        auction_spinner_.reset(new ros::AsyncSpinner(1, &auction_callback_queue_));
        auction_spinner_->start();
    }
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
    std::lock_guard<std::mutex> lock(coverage_map_update_mutex_);
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
    std::lock_guard<std::mutex> lock(coverage_map_update_mutex_);
    coverage_map_.updateFromLocalPcl(msg, odom);
}

void CoverageSearchManager::scanPclCb(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (!odom_ready_) return;
    sensor_ready_ = true;
    // ★ 使用fly_height_作为传感器z值，确保射线投射在正确的z层进行
    Eigen::Vector3d sensor_pos(uav_state_.position[0], uav_state_.position[1],
                                fly_height_);
    double sensor_yaw = uav_yaw_;
    std::lock_guard<std::mutex> lock(coverage_map_update_mutex_);
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
    std::lock_guard<std::mutex> lock(coverage_map_update_mutex_);
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
        std::lock_guard<std::mutex> lock(coverage_map_update_mutex_);
        coverage_map_.setDynamicPeerVolume(Eigen::Vector3d::Zero(), false);
        return;
    }
    peer_pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    peer_state_received_ = ros::Time::now();
    peer_state_valid_ = true;
    peer_completion_ready_ = msg->completion_ready;
    std::lock_guard<std::mutex> lock(coverage_map_update_mutex_);
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

void CoverageSearchManager::rawHeartbeatCb(const ros::TimerEvent &) {
    {
        std::lock_guard<std::mutex> lock(realtime_trajectory_mutex_);
        if (realtime_trajectory_ && !realtime_trajectory_->points.empty()) return;
    }
    Eigen::Vector3d pos;
    double yaw = 0.0;
    bool odom_ready = false;
    {
        std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
        pos = uav_pos_;
        yaw = uav_yaw_;
        odom_ready = odom_ready_;
    }
    if (!odom_ready || !pos.allFinite() || !std::isfinite(yaw)) return;
    prometheus_msgs::UAVCommand command;
    command.header.stamp = ros::Time::now();
    command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    command.Control_Level = prometheus_msgs::UAVCommand::DEFAULT_CONTROL;
    command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
    command.position_ref[0] = pos(0);
    command.position_ref[1] = pos(1);
    command.position_ref[2] = fly_height_;
    command.yaw_ref = yaw;
    command.Command_ID = realtime_command_id_.fetch_add(1) + 1;
    uav_cmd_pub_.publish(command);
}

void CoverageSearchManager::publishStartupHold() {
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

void CoverageSearchManager::updateFrontiers() {
    if (!sensor_ready_ || !coverage_map_.map_ready_) return;

    {
        std::lock_guard<std::mutex> lock(coverage_map_update_mutex_);
        coverage_map_.updateDistanceFields();
    }
    frontier_finder_.searchFrontiers(uav_pos_);
    retireFrontierTasks(frontier_finder_.consumeRetiredTaskIds());
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
    double priority_distance = std::numeric_limits<double>::infinity();
    uint64_t priority_task_id = 0;
    for (auto &frontier : frontier_finder_.frontiers_) {
        if (!frontier.local_discovery) continue;
        frontier.local_discovery = false;
        ++discovered;
        reserveLocalFrontier(frontier);
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
    std::vector<bool> selected(frontier_finder_.frontiers_.size(), false);
    std::vector<size_t> selected_indices;
    selected_indices.reserve(frontier_finder_.frontiers_.size());
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
    // The allocation protocol must agree on the complete active task set.
    // Full cells are no longer advertised here, so carrying all descriptors
    // is materially cheaper than the old nearest-32 truncation.
    for (size_t i = 0; i < frontier_finder_.frontiers_.size(); ++i) {
        if (!selected[i]) selected_indices.push_back(i);
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
        item.frontier_source_uav_id = frontier.source_uav_id;
        item.cluster_version = frontier.source_revision;
        item.frontier_cell_count = frontier.cells.size();
        item.centroid.x = frontier.average(0); item.centroid.y = frontier.average(1); item.centroid.z = frontier.average(2);
        item.box_min.x = frontier.box_min_(0); item.box_min.y = frontier.box_min_(1); item.box_min.z = frontier.box_min_(2);
        item.box_max.x = frontier.box_max_(0); item.box_max.y = frontier.box_max_(1); item.box_max.z = frontier.box_max_(2);
        // Allocation uses only the best representative viewpoint. Once the
        // peer owns the task, send it the complete cluster and all viewpoints.
        const bool transfer_to_peer = isFrontierLeasedToPeer(frontier);
        if (transfer_to_peer) {
            item.frontier_cells.reserve(frontier.cells.size());
            for (const auto &cell : frontier.cells) {
                geometry_msgs::Point point;
                point.x = cell(0); point.y = cell(1); point.z = cell(2);
                item.frontier_cells.push_back(point);
            }
        }
        const double hard_clearance = std::max(0.45, coverage_map_.esdf_safe_distance_);
        size_t representative_view = 0;
        for (size_t view = 0; view < frontier.viewpoints.size(); ++view) {
            Eigen::Vector3i view_index;
            coverage_map_.posToIndex(frontier.viewpoints[view], view_index);
            if (coverage_map_.isInMap2D(view_index(0), view_index(1)) &&
                coverage_map_.isFree2D(view_index(0), view_index(1)) &&
                coverage_map_.getDistance2D(view_index(0), view_index(1)) + 1e-3 >=
                    hard_clearance &&
                frontier_finder_.isViewpointVisible(
                    frontier, frontier.viewpoints[view],
                    view < frontier.viewpoint_yaws.size()
                        ? frontier.viewpoint_yaws[view] : uav_yaw_)) {
                representative_view = view;
                break;
            }
        }
        const size_t viewpoint_count = transfer_to_peer ? frontier.viewpoints.size() : 1;
        for (size_t offset = 0; offset < viewpoint_count; ++offset) {
            const size_t view = offset == 0 ? representative_view
                : (offset <= representative_view ? offset - 1 : offset);
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
    publishSwarmMapDelta();
}

void CoverageSearchManager::auctionTaskSetCb(
    const prometheus_two_uav_coverage_search::SwarmAuctionTaskSetConstPtr &msg) {
    if (!cooperative_mode_ || msg->source_uav_id != static_cast<uint32_t>(uav_id_) ||
        msg->map_epoch != static_cast<uint32_t>(map_epoch_)) return;
    {
        std::lock_guard<std::mutex> lock(auction_assignment_mutex_);
        if (last_auction_assignment_.task_set_hash == msg->task_set_hash &&
            last_auction_assignment_.uav1_state_sequence == msg->uav1_state_sequence &&
            last_auction_assignment_.uav2_state_sequence == msg->uav2_state_sequence &&
            two_uav_auction::assignmentShapeValid(last_auction_assignment_)) {
            auction_assignment_pub_.publish(last_auction_assignment_);
            return;
        }
    }
    publishAuctionAssignment(*msg);
}

CoverageSearchManager::AuctionMapSnapshot CoverageSearchManager::captureAuctionMapSnapshot() {
    std::lock_guard<std::mutex> lock(coverage_map_update_mutex_);
    AuctionMapSnapshot snapshot;
    if (!coverage_map_.map_ready_) return snapshot;
    snapshot.nx = coverage_map_.grid_size_(0);
    snapshot.ny = coverage_map_.grid_size_(1);
    snapshot.resolution = coverage_map_.resolution_;
    snapshot.origin = coverage_map_.origin_;
    snapshot.walkable.assign(snapshot.nx * snapshot.ny, 0);
    const double clearance = std::max(0.45, coverage_map_.esdf_safe_distance_);
    for (int x = 0; x < snapshot.nx; ++x) {
        for (int y = 0; y < snapshot.ny; ++y) {
            const uint8_t walkable =
                coverage_map_.isFree2D(x, y, false) &&
                coverage_map_.getDistance2D(x, y) + 1e-3 >= clearance;
            snapshot.walkable[x * snapshot.ny + y] = walkable;
            snapshot.walkable_count += walkable;
            snapshot.walkable_hash ^= walkable;
            snapshot.walkable_hash *= 1099511628211ULL;
        }
    }
    return snapshot;
}

bool CoverageSearchManager::auctionSnapshotPathCost(const AuctionMapSnapshot &map,
                                                     const Eigen::Vector3d &start,
                                                     const Eigen::Vector3d &goal,
                                                     double timeout_ms, double *cost,
                                                     uint8_t *failure_mask) {
    if (cost) *cost = std::numeric_limits<double>::infinity();
    if (failure_mask) *failure_mask = 0;
    const auto index_of = [&](const Eigen::Vector3d &point, int *x, int *y) {
        *x = static_cast<int>(std::floor((point(0) - map.origin(0)) / map.resolution));
        *y = static_cast<int>(std::floor((point(1) - map.origin(1)) / map.resolution));
        return *x >= 0 && *x < map.nx && *y >= 0 && *y < map.ny;
    };
    const auto walkable = [&](int x, int y) {
        return x >= 0 && x < map.nx && y >= 0 && y < map.ny &&
            map.walkable[x * map.ny + y] != 0;
    };
    int sx = 0, sy = 0, gx = 0, gy = 0;
    if (!index_of(start, &sx, &sy) || !index_of(goal, &gx, &gy) || !walkable(gx, gy)) {
        if (failure_mask) *failure_mask |= 1u << 1;
        return false;
    }
    double escape_cost = 0.0;
    if (!walkable(sx, sy)) {
        int best_x = -1, best_y = -1, best_dist2 = std::numeric_limits<int>::max();
        for (int dx = -5; dx <= 5; ++dx) for (int dy = -5; dy <= 5; ++dy) {
            if (!walkable(sx + dx, sy + dy)) continue;
            const int dist2 = dx * dx + dy * dy;
            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                best_x = sx + dx;
                best_y = sy + dy;
            }
        }
        if (best_x < 0) {
            if (failure_mask) *failure_mask |= 1u << 1;
            return false;
        }
        escape_cost = std::sqrt(static_cast<double>(best_dist2)) * map.resolution;
        sx = best_x;
        sy = best_y;
    }
    struct OpenCell {
        double f;
        int key;
        bool operator<(const OpenCell &other) const { return f > other.f; }
    };
    const auto key = [&](int x, int y) { return x * map.ny + y; };
    const auto heuristic = [&](int x, int y) {
        return std::hypot(x - gx, y - gy) * map.resolution;
    };
    const int total = map.nx * map.ny;
    std::vector<double> distance(total, std::numeric_limits<double>::infinity());
    std::priority_queue<OpenCell> open;
    distance[key(sx, sy)] = 0.0;
    open.push({heuristic(sx, sy), key(sx, sy)});
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::microseconds(static_cast<int64_t>(timeout_ms * 1000.0));
    const int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};
    while (!open.empty()) {
        if (std::chrono::steady_clock::now() >= deadline) {
            if (failure_mask) *failure_mask |= 1u << 0;
            return false;
        }
        const OpenCell current = open.top();
        open.pop();
        const int x = current.key / map.ny, y = current.key % map.ny;
        if (current.f > distance[current.key] + heuristic(x, y) + 1e-9) continue;
        if (x == gx && y == gy) {
            if (cost) *cost = distance[current.key] + escape_cost;
            return true;
        }
        for (int direction = 0; direction < 8; ++direction) {
            const int nx = x + dx[direction], ny = y + dy[direction];
            if (!walkable(nx, ny)) continue;
            if ((dx[direction] != 0 && dy[direction] != 0) &&
                (!walkable(x + dx[direction], y) || !walkable(x, y + dy[direction]))) continue;
            const int next = key(nx, ny);
            const double next_distance = distance[current.key] +
                (dx[direction] == 0 || dy[direction] == 0 ? map.resolution
                                                           : std::sqrt(2.0) * map.resolution);
            if (next_distance + 1e-9 >= distance[next]) continue;
            distance[next] = next_distance;
            open.push({next_distance + heuristic(nx, ny), next});
        }
    }
    if (failure_mask) *failure_mask |= 1u << 2;
    return false;
}

void CoverageSearchManager::publishAuctionAssignment(
    const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet &task_set) {
    const auto callback_started = std::chrono::steady_clock::now();
    const auto job_is_current = [&](const AuctionJob &candidate) {
        return candidate.task_set.task_set_hash == task_set.task_set_hash &&
            candidate.task_set.uav1_state_sequence == task_set.uav1_state_sequence &&
            candidate.task_set.uav2_state_sequence == task_set.uav2_state_sequence;
    };
    {
        std::lock_guard<std::mutex> lock(auction_worker_mutex_);
        if ((auction_job_ready_ && job_is_current(auction_job_)) ||
            (auction_active_task_set_hash_ == task_set.task_set_hash &&
             auction_active_uav1_state_sequence_ == task_set.uav1_state_sequence &&
             auction_active_uav2_state_sequence_ == task_set.uav2_state_sequence)) return;
    }
    AuctionJob job;
    job.task_set = task_set;
    job.callback_started = callback_started;
    if (!task_set.header.stamp.isZero()) {
        job.receive_age_ms = std::max(0.0,
            (ros::Time::now() - task_set.header.stamp).toSec() * 1000.0);
    }
    const auto snapshot_started = std::chrono::steady_clock::now();
    job.map = captureAuctionMapSnapshot();
    if (job.map.walkable.empty()) {
        ROS_WARN("[CoverageSearch] UAV %d cannot evaluate distributed round %llu: map not ready.",
                 uav_id_, static_cast<unsigned long long>(task_set.task_set_hash));
        return;
    }
    job.snapshot_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - snapshot_started).count();
    job.uav1_start = Eigen::Vector3d(task_set.uav1_position.x,
                                    task_set.uav1_position.y,
                                    task_set.uav1_position.z);
    job.uav2_start = Eigen::Vector3d(task_set.uav2_position.x,
                                    task_set.uav2_position.y,
                                    task_set.uav2_position.z);
    job.peer_state_age_s = peer_state_received_.isZero() ?
        std::numeric_limits<double>::infinity() :
        std::max(0.0, (ros::Time::now() - peer_state_received_).toSec());
    job.enqueued = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(auction_worker_mutex_);
        if ((auction_job_ready_ && job_is_current(auction_job_)) ||
            (auction_active_task_set_hash_ == task_set.task_set_hash &&
             auction_active_uav1_state_sequence_ == task_set.uav1_state_sequence &&
             auction_active_uav2_state_sequence_ == task_set.uav2_state_sequence)) return;
        job.sequence = ++auction_job_sequence_;
        auction_job_ = std::move(job);
        auction_job_ready_ = true;
    }
    auction_worker_cv_.notify_one();
}

void CoverageSearchManager::auctionWorkerLoop() {
    while (true) {
        AuctionJob job;
        {
            std::unique_lock<std::mutex> lock(auction_worker_mutex_);
            auction_worker_cv_.wait(lock, [&] { return auction_worker_stop_ || auction_job_ready_; });
            if (auction_worker_stop_) return;
            job = auction_job_;
            auction_job_ready_ = false;
            auction_active_task_set_hash_ = job.task_set.task_set_hash;
            auction_active_uav1_state_sequence_ = job.task_set.uav1_state_sequence;
            auction_active_uav2_state_sequence_ = job.task_set.uav2_state_sequence;
        }
        const auto evaluation_started = std::chrono::steady_clock::now();
        const double queue_ms = std::chrono::duration<double, std::milli>(
            evaluation_started - job.enqueued).count();
        prometheus_two_uav_coverage_search::SwarmAuctionAssignment out;
        out.header.stamp = ros::Time::now();
        out.source_uav_id = uav_id_;
        out.map_epoch = map_epoch_;
        out.task_set_hash = job.task_set.task_set_hash;
        out.uav1_state_sequence = job.task_set.uav1_state_sequence;
        out.uav2_state_sequence = job.task_set.uav2_state_sequence;
        out.final_result = false;
    struct VehicleBid {
        bool reachable = false;
        bool timed_out = false;
        uint8_t failure_mask = 0;
        double path_cost = std::numeric_limits<double>::infinity();
        double timeout_euclid = std::numeric_limits<double>::infinity();
        double nearest_euclid = std::numeric_limits<double>::infinity();
    };
    const auto evaluate = [&](const Eigen::Vector3d &start,
                              const prometheus_two_uav_coverage_search::SwarmFrontier &frontier) {
        VehicleBid bid;
        const auto &view = frontier.candidate_viewpoints.front().position;
        const Eigen::Vector3d goal(view.x, view.y, fly_height_);
        const double euclid = (goal.head<2>() - start.head<2>()).norm();
        bid.nearest_euclid = euclid;
        double path_cost = std::numeric_limits<double>::infinity();
        uint8_t failure_mask = 0;
        const bool reachable = auctionSnapshotPathCost(job.map, start, goal,
                                                        auction_astar_timeout_ms_,
                                                        &path_cost, &failure_mask);
        if (reachable && std::isfinite(path_cost)) {
            bid.reachable = true;
            bid.path_cost = path_cost;
        } else if (failure_mask & (1u << 0)) {
            bid.timed_out = true;
            bid.failure_mask |= 1u << 0;
            bid.timeout_euclid = euclid;
        } else if (failure_mask & (1u << 1)) {
            bid.failure_mask |= 1u << 1;
        } else if (failure_mask & (1u << 2)) {
            bid.failure_mask |= 1u << 2;
        }
        return bid;
    };
    const auto failureSummary = [](uint8_t mask) {
        if (mask == 0) return std::string("none");
        std::string result;
        const auto append = [&](const char *reason) {
            if (!result.empty()) result += '|';
            result += reason;
        };
        if (mask & (1u << 0)) append("timeout");
        if (mask & (1u << 1)) append("endpoint_blocked");
        if (mask & (1u << 2)) append("no_path");
        if (mask & (1u << 3)) append("unknown_rejected");
        return result;
    };

    for (const auto &frontier : job.task_set.frontiers) {
        const VehicleBid bid1 = evaluate(job.uav1_start, frontier);
        const VehicleBid bid2 = evaluate(job.uav2_start, frontier);
        uint32_t owner = 0;
        uint32_t viewpoint = 0;
        uint8_t mode = prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_UNREACHABLE;
        double cost1 = bid1.path_cost;
        double cost2 = bid2.path_cost;
        if (bid1.reachable || bid2.reachable) {
            if (bid1.reachable && (!bid2.reachable || cost1 < cost2 - 1e-3)) owner = 1;
            else if (bid2.reachable && (!bid1.reachable || cost2 < cost1 - 1e-3)) owner = 2;
            else owner = 1;
            mode = prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_ASTAR;
        } else if (bid1.timed_out || bid2.timed_out) {
            cost1 = bid1.timed_out ? bid1.timeout_euclid : std::numeric_limits<double>::infinity();
            cost2 = bid2.timed_out ? bid2.timeout_euclid : std::numeric_limits<double>::infinity();
            if (cost1 < cost2 - 1e-3) owner = 1;
            else if (cost2 < cost1 - 1e-3) owner = 2;
            else owner = 1;
            mode = prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_EUCLIDEAN_TIMEOUT;
        }
        out.task_ids.push_back(frontier.task_id);
        out.frontier_source_uav_ids.push_back(frontier.frontier_source_uav_id);
        out.cluster_versions.push_back(frontier.cluster_version);
        out.owner_uav_ids.push_back(owner);
        out.viewpoint_indices.push_back(viewpoint);
        out.assignment_modes.push_back(mode);
        const char *mode_name = mode == prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_ASTAR
            ? "astar" : mode == prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_EUCLIDEAN_TIMEOUT
            ? "euclidean_timeout" : "unreachable";
        ROS_INFO("[AuctionCost] evaluator=UAV%d round=%llu task=%llu representative_view=0 "
                 "C1=%.3f reachable1=%s failures1=%s "
                 "C2=%.3f reachable2=%s failures2=%s owner=%u mode=%s.",
                 uav_id_, static_cast<unsigned long long>(job.task_set.task_set_hash),
                 static_cast<unsigned long long>(frontier.task_id),
                 cost1, bid1.reachable ? "yes" : "no", failureSummary(bid1.failure_mask).c_str(),
                 cost2, bid2.reachable ? "yes" : "no", failureSummary(bid2.failure_mask).c_str(),
                 owner, mode_name);
    }
        out.assignment_hash = two_uav_auction::assignmentHash(out);
        const auto evaluation_finished = std::chrono::steady_clock::now();
        bool superseded = false;
        {
            std::lock_guard<std::mutex> lock(auction_worker_mutex_);
            superseded = job.sequence != auction_job_sequence_;
            if (!superseded) {
                auction_active_task_set_hash_ = 0;
                auction_active_uav1_state_sequence_ = 0;
                auction_active_uav2_state_sequence_ = 0;
            }
        }
        ROS_INFO("[AuctionEvalTiming] evaluator=UAV%d round=%llu tasks=%zu "
                 "receive_age=%.1fms snapshot=%.1fms queue=%.1fms compute=%.1fms total=%.1fms "
                 "peer_age=%.3fs start1=(%.2f,%.2f) start2=(%.2f,%.2f) "
                 "walkable=%zu/%zu map_hash=%llu superseded=%s.",
                 uav_id_, static_cast<unsigned long long>(job.task_set.task_set_hash),
                 job.task_set.frontiers.size(), job.receive_age_ms, job.snapshot_ms, queue_ms,
                 std::chrono::duration<double, std::milli>(
                     evaluation_finished - evaluation_started).count(),
                 std::chrono::duration<double, std::milli>(
                     evaluation_finished - job.callback_started).count(),
                 job.peer_state_age_s, job.uav1_start.x(), job.uav1_start.y(),
                 job.uav2_start.x(), job.uav2_start.y(), job.map.walkable_count,
                 job.map.walkable.size(),
                 static_cast<unsigned long long>(job.map.walkable_hash),
                 superseded ? "yes" : "no");
        if (superseded) continue;
        {
            std::lock_guard<std::mutex> lock(auction_assignment_mutex_);
            last_auction_assignment_ = out;
        }
        auction_assignment_pub_.publish(out);
    }
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
    std::lock_guard<std::mutex> lock(coverage_map_update_mutex_);
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
    synchronizeTransferredFrontiers();
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
    synchronizeTransferredFrontiers();
}

void CoverageSearchManager::remoteFrontierTransferAckCb(
    const prometheus_two_uav_coverage_search::SwarmFrontierTransferAckConstPtr &msg) {
    if (!cooperative_mode_ || msg->source_uav_id == static_cast<uint32_t>(uav_id_) ||
        msg->map_epoch != static_cast<uint32_t>(map_epoch_)) return;
    remote_transfer_ack_versions_[msg->task_id] = std::max(
        remote_transfer_ack_versions_[msg->task_id], msg->cluster_version);
    synchronizeTransferredFrontiers();
}

void CoverageSearchManager::synchronizeTransferredFrontiers() {
    if (!cooperative_mode_) return;
    const ros::Time now = ros::Time::now();
    const auto fresh = [&](const ros::Time &received) {
        return !received.isZero() && (now - received).toSec() <= 5.0;
    };
    const auto peer_advertisement = [&](const prometheus_two_uav_coverage_search::SwarmTask &task) {
        return std::find_if(remote_swarm_frontiers_.frontiers.begin(),
                            remote_swarm_frontiers_.frontiers.end(),
                            [&](const prometheus_two_uav_coverage_search::SwarmFrontier &frontier) {
                                return frontier.task_id == task.task_id &&
                                    frontier.frontier_source_uav_id == task.frontier_source_uav_id;
                            });
    };
    const auto synchronize = [&](const prometheus_two_uav_coverage_search::SwarmTaskArray &tasks,
                                 bool is_fresh) {
        if (!is_fresh) return;
        for (const auto &task : tasks.tasks) {
            if (task.lease_expire_time <= now) continue;
            if (retired_frontier_ids_.count(task.task_id) != 0) continue;
            const auto copy = peer_advertisement(task);
            if (task.winner_uav_id == static_cast<uint32_t>(uav_id_)) {
                bool active = std::any_of(frontier_finder_.frontiers_.begin(),
                                          frontier_finder_.frontiers_.end(),
                    [&](const FrontierFinder::FrontierCluster &frontier) {
                        return frontier.task_id == task.task_id &&
                            frontier.source_revision >= task.cluster_version;
                    });
                if (!active)
                    active = frontier_finder_.reactivateSleepingFrontier(
                        task.task_id, task.cluster_version);
                const bool complete_copy = copy != remote_swarm_frontiers_.frontiers.end() &&
                    !copy->frontier_cells.empty() &&
                    copy->cluster_version >= task.cluster_version;
                if (!active && complete_copy)
                    active = frontier_finder_.adoptFrontier(*copy);

                if (!active && copy == remote_swarm_frontiers_.frontiers.end()) {
                    ROS_WARN_THROTTLE(2.0,
                        "[FrontierTransfer] UAV %d owns task=%llu but peer descriptor is missing.",
                        uav_id_, static_cast<unsigned long long>(task.task_id));
                } else if (!active && !complete_copy) {
                    ROS_WARN_THROTTLE(2.0,
                        "[FrontierTransfer] UAV %d owns task=%llu but peer sent descriptor only: "
                        "required_version=%u received_version=%u cells=%zu.",
                        uav_id_, static_cast<unsigned long long>(task.task_id),
                        task.cluster_version, copy->cluster_version,
                        copy->frontier_cells.size());
                } else if (active && complete_copy) {
                    const auto sent = sent_transfer_ack_versions_.find(task.task_id);
                    if (sent == sent_transfer_ack_versions_.end() ||
                        sent->second < task.cluster_version) {
                        prometheus_two_uav_coverage_search::SwarmFrontierTransferAck ack;
                        ack.header.stamp = now;
                        ack.source_uav_id = uav_id_;
                        ack.map_epoch = map_epoch_;
                        ack.task_id = task.task_id;
                        ack.frontier_source_uav_id = task.frontier_source_uav_id;
                        ack.cluster_version = task.cluster_version;
                        frontier_transfer_ack_pub_.publish(ack);
                        sent_transfer_ack_versions_[task.task_id] = task.cluster_version;
                        ROS_INFO("[FrontierTransfer] UAV %d activated task=%llu version=%u "
                                 "and acknowledged peer.",
                                 uav_id_, static_cast<unsigned long long>(task.task_id),
                                 task.cluster_version);
                    }
                }
            }
            // Only an explicit ACK from the receiver can retire the current
            // holder's active copy. The immutable source ID is identity, not ownership.
            // This keeps a recoverable sleeping backup if the
            // complete-cell transfer was lost or only partially received.
            if (task.winner_uav_id != static_cast<uint32_t>(uav_id_) &&
                remote_transfer_ack_versions_[task.task_id] >= task.cluster_version) {
                if (frontier_finder_.moveFrontierToSleeping(task.task_id)) {
                    ROS_INFO("[FrontierTransfer] UAV %d moved acknowledged task=%llu "
                             "version=%u to sleeping.",
                             uav_id_, static_cast<unsigned long long>(task.task_id),
                             task.cluster_version);
                }
            }
        }
    };
    synchronize(local_swarm_tasks_, fresh(local_swarm_task_received_));
    synchronize(remote_swarm_tasks_, fresh(remote_swarm_task_received_));
}

void CoverageSearchManager::retireFrontierTasks(const std::vector<uint64_t> &task_ids) {
    if (task_ids.empty()) return;
    retired_frontier_ids_.insert(task_ids.begin(), task_ids.end());
    const auto retired = [&](uint64_t task_id) {
        return std::find(task_ids.begin(), task_ids.end(), task_id) != task_ids.end();
    };
    local_frontier_reservations_.erase(
        std::remove_if(local_frontier_reservations_.begin(), local_frontier_reservations_.end(),
                       [&](const LocalFrontierReservation &reservation) {
                           return retired(reservation.task.task_id);
                       }),
        local_frontier_reservations_.end());
    if (retired(local_discovery_priority_task_id_)) local_discovery_priority_task_id_ = 0;

    bool affects_executable_plan = retired(current_goal_task_id_) ||
        retired(pending_traj_.task_id);
    for (const uint64_t task_id : pending_traj_.frontier_target_task_ids) {
        affects_executable_plan = affects_executable_plan || retired(task_id);
    }
    if (affects_executable_plan) {
        pending_traj_ = PendingTrajectory();
        ++rolling_generation_;
        rolling_replan_requested_ = true;
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
    return task.frontier_source_uav_id == frontier.source_uav_id &&
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
    const auto contains_self_lease = [&](const prometheus_two_uav_coverage_search::SwarmTaskArray &tasks,
                                         const ros::Time &received) {
        if (received.isZero() || (now - received).toSec() > 5.0) return false;
        for (const auto &task : tasks.tasks) {
            if (task.winner_uav_id == static_cast<uint32_t>(uav_id_) &&
                task.lease_expire_time > now && frontierMatchesTask(frontier, task)) return true;
        }
        return false;
    };
    if (contains_self_lease(local_swarm_tasks_, local_swarm_task_received_) ||
        contains_self_lease(remote_swarm_tasks_, remote_swarm_task_received_)) return true;
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
    reservation.task.frontier_source_uav_id = frontier.source_uav_id;
    reservation.task.winner_uav_id = uav_id_;
    reservation.task.allocation_state =
        prometheus_two_uav_coverage_search::SwarmTask::ALLOCATION_LOCAL_RESERVATION;
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
    if (retired_frontier_ids_.count(task_id) != 0) return false;
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
    if (retired_frontier_ids_.count(current_goal_task_id_) != 0) return false;
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
        active_goal_frontier_.frontier_source_uav_id = frontier.source_uav_id;
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
    prometheus_two_uav_coverage_search::SwarmTask remote_task;
    if (getSelfLeasedRemoteTask(current_goal_task_id_, remote_task)) {
        active_goal_frontier_ = remote_task;
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
    if (!frontier_finder_.isFrontierCellsCovered(
            active_goal_frontier_cells_, frontier_finder_.retire_changed_ratio_))
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
    {
        std::lock_guard<std::mutex> lock(coverage_map_update_mutex_);
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

    const bool command_ready = (uav_control_state_.control_state ==
                                prometheus_msgs::UAVControlState::COMMAND_CONTROL);
    if (!odom_ready_ || !drone_ready_ || !command_ready) {
        static int wait_cnt = 0;
        if (++wait_cnt >= 10) {
            wait_cnt = 0;
            cout << YELLOW << "[CoverageSearch] Waiting... odom:" << odom_ready_
                 << " drone:" << drone_ready_ << " sensor:" << sensor_ready_
                 << " command:" << command_ready << TAIL << endl;
        }
        return;
    }

    // The coordinator relays only this topic.  Keep it alive while the first
    // depth cloud is being transformed and integrated into the local map.
    if (!sensor_ready_ || !coverage_map_.map_ready_) {
        publishStartupHold();
        ROS_INFO_THROTTLE(1.0,
            "[CoverageSearch] Raw command heartbeat active; waiting for sensor=%d map=%d.",
            sensor_ready_, coverage_map_.map_ready_);
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
    {
        std::lock_guard<std::mutex> lock(coverage_map_update_mutex_);
        coverage_map_.clearRobotVolume(Eigen::Vector3d(
            uav_state_.position[0], uav_state_.position[1], uav_state_.position[2]));
    }
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
                    publishStartupHold();
                }
            } else {
                // 等待外部触发
                publishStartupHold();
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
                    break;
                }
            } else {
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
