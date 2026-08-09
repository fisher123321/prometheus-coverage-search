#ifndef TWO_UAV_COVERAGE_SEARCH_H
#define TWO_UAV_COVERAGE_SEARCH_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <random>
#include <set>
#include <functional>
#include <chrono>
#include <condition_variable>
#include <cassert>
#include <atomic>
#include <mutex>
#include <thread>
#include <cstdint>
#include <memory>
#include <limits>
#include <map>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/Bool.h>

#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVControlState.h>
#include <prometheus_two_uav_coverage_search/SwarmFrontierArray.h>
#include <prometheus_two_uav_coverage_search/SwarmMapChunk.h>
#include <prometheus_two_uav_coverage_search/SwarmMapRequest.h>
#include <prometheus_two_uav_coverage_search/SwarmState.h>
#include <prometheus_two_uav_coverage_search/SwarmTaskArray.h>
#include <prometheus_two_uav_coverage_search/SwarmTrajectory.h>
#include <prometheus_two_uav_coverage_search/SwarmAuctionTaskSet.h>
#include <prometheus_two_uav_coverage_search/SwarmAuctionAssignment.h>
#include <prometheus_two_uav_coverage_search/SwarmFrontierTransferAck.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "two_uav_coverage_console_colors.h"

using namespace std;
using namespace Eigen;

// ============================================================
// 占据地图类 - 深度相机3D栅格地图，用于覆盖搜索
// ============================================================
class CoverageMap {
public:
    CoverageMap() {}
    ~CoverageMap() {}

    void init(ros::NodeHandle &nh);

    // 地图更新
    void updateFromGlobalPcl(const sensor_msgs::PointCloud2ConstPtr &msg);
    void updateFromLocalPcl(const sensor_msgs::PointCloud2ConstPtr &msg, const nav_msgs::Odometry &odom);
    void updateFromScanPcl(const sensor_msgs::PointCloud2ConstPtr &msg, const Eigen::Vector3d &sensor_pos, double sensor_yaw);
    void updateFromDepthPcl(const sensor_msgs::PointCloud2ConstPtr &msg,
                            const Eigen::Vector3d &camera_pos,
                            const Eigen::Matrix3d &R_wc);
    void clearRobotVolume(const Eigen::Vector3d &body_pos);
    void setDynamicPeerVolume(const Eigen::Vector3d &body_pos, bool active);
    void setDynamicPeerAvoidance(const Eigen::Vector3d &body_pos, bool active,
                                 double radius);
    void clearDynamicPeerVolume(const Eigen::Vector3d &body_pos);
    bool isInDynamicPeerVolume(const Eigen::Vector3d &pos) const;
    bool isInDynamicPeerAvoidance(const Eigen::Vector3d &pos) const;

    // 栅格状态查询
    bool isInMap(const Eigen::Vector3d &pos);
    bool isOccupied(const Eigen::Vector3d &pos);
    bool isUnknown(const Eigen::Vector3d &pos);
    bool isFree(const Eigen::Vector3d &pos);
    bool isInMapIndex(const Eigen::Vector3i &idx);
    bool isOccupiedIndex(const Eigen::Vector3i &idx);
    bool isUnknownIndex(const Eigen::Vector3i &idx);
    bool isFreeIndex(const Eigen::Vector3i &idx);
    bool isRayOccluded(const Eigen::Vector3d &start,
                       const Eigen::Vector3d &end);
    void collectVisibleUnknown(const Eigen::Vector3d &start,
                               const Eigen::Vector3d &end,
                               std::set<int> &unknown_addresses);

    // 2D索引查询
    bool isOccupied2D(int x, int y);
    bool isFree2D(int x, int y);
    bool isUnknown2D(int x, int y);
    bool isInMap2D(int x, int y);

    // 位置与索引转换
    void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &idx);
    void indexToPos(const Eigen::Vector3i &idx, Eigen::Vector3d &pos);
    int  toAddress(const Eigen::Vector3i &idx);

    // 固定探索矩形内的已知空间比率；分母不随探索过程变化
    double getKnownSpaceRatio();
    int    getUnknownCount();
    int    getFreeCount();
    int    getTotalCount();

    void updateDistanceFields(bool force = false);
    double getDistance(const Eigen::Vector3d &pos);
    double getDistance2D(int x, int y);
    double getTSDF(const Eigen::Vector3d &pos);
    bool isDistanceFieldReady() const { return esdf_ready_; }
    void publishMap();
    bool getUpdatedBox(Eigen::Vector3d &update_min,
                       Eigen::Vector3d &update_max,
                       bool reset = true);
    void getRemoteUpdatedBoxes(
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &boxes,
        bool reset = true);
    void markRemoteUpdatedBox(const Eigen::Vector3i &min_idx,
                              const Eigen::Vector3i &max_idx);
    int8_t getLocalEvidence(int address) const;
    // Returns true only when remote evidence changes the fused occupancy state.
    bool setRemoteEvidence(int address, int8_t evidence);

    // 参数（public以便Astar2D访问）
    double resolution_, inv_resolution_;
    Eigen::Vector3d origin_, map_size_3d_, min_range_, max_range_;
    Eigen::Vector3i grid_size_;
    double fly_height_;
    double depth_min_range_, depth_max_range_, depth_height_tolerance_;
    double depth_ground_ignore_height_;
    double depth_fov_h_, depth_fov_v_;
    double esdf_max_distance_, esdf_safe_distance_, tsdf_trunc_dist_;
    bool sim_mode_;
    bool map_ready_;

    std::vector<uint8_t> occupancy_buffer_;
    std::vector<uint8_t> observed_buffer_;
    std::vector<int8_t> occupancy_evidence_buffer_;
    std::vector<float> esdf_buffer_;
    std::vector<float> tsdf_buffer_;
    std::vector<float> esdf_tmp1_;
    std::vector<float> esdf_tmp2_;

    int min_z_idx_, max_z_idx_, fly_z_idx_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_pcl_;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;

    ros::Publisher coverage_vis_pub_, fused_map_vis_pub_;
    ros::Timer map_pub_timer_;
    ros::Time last_fused_map_vis_time_;

private:
    bool setOccupancy(const Eigen::Vector3d &pos, uint8_t occ,
                      bool mark_local_update = true);
    void integrateHit(const Eigen::Vector3d &pos);
    void integrateMiss(const Eigen::Vector3i &idx);
    void beginOccupancyUpdate();
    bool updateOccupancyFromEvidence(const Eigen::Vector3i &idx,
                                     bool mark_local_update = true);
    void markUpdatedIndex(const Eigen::Vector3i &idx, int margin = 0);
    void markDistanceFieldDirtyIndex(const Eigen::Vector3i &idx, int margin = 0);
    using IndexBox = std::pair<Eigen::Vector3i, Eigen::Vector3i>;
    void appendRemoteBox(std::vector<IndexBox> &boxes,
                         const Eigen::Vector3i &min_idx,
                         const Eigen::Vector3i &max_idx);
    void clearVolume(const Eigen::Vector3d &body_pos,
                     double radius, double half_height,
                     bool mark_local_update);
    void pubMapTimerCb(const ros::TimerEvent &e);
    void raycastFree(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                     bool include_end = false, bool stop_at_occupied = true);
    bool traceRay(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                  bool include_end, bool integrate_free,
                  std::set<int> *unknown_addresses = nullptr,
                  bool projected_occlusion = false,
                  bool stop_at_occupied = true);
    void raycastCameraFovFree(const Eigen::Vector3d &camera_pos,
                              const Eigen::Matrix3d &R_wc,
                              const pcl::PointCloud<pcl::PointXYZ> &real_points);
    int queue_size_;
    int st_it_;
    double robot_clear_radius_, robot_clear_half_height_;
    double peer_clear_radius_, peer_clear_half_height_;
    Eigen::Vector3d dynamic_peer_pos_ = Eigen::Vector3d::Zero();
    bool dynamic_peer_valid_ = false;
    Eigen::Vector3d dynamic_peer_avoidance_pos_ = Eigen::Vector3d::Zero();
    double dynamic_peer_avoidance_radius_ = 0.0;
    bool dynamic_peer_avoidance_valid_ = false;
    unsigned int occupancy_update_epoch_ = 0;
    std::vector<unsigned int> occupancy_update_stamp_;
    std::vector<int8_t> remote_evidence_buffer_;
    std::map<int, pcl::PointCloud<pcl::PointXYZ>> point_cloud_pair_;
    bool updated_bbox_valid_ = false;
    bool esdf_dirty_ = true;
    bool esdf_ready_ = false;
    Eigen::Vector3i updated_min_idx_, updated_max_idx_;
    bool df_bbox_valid_ = false;
    Eigen::Vector3i df_min_idx_, df_max_idx_;
    std::vector<IndexBox> remote_updated_boxes_;
    std::vector<IndexBox> remote_df_boxes_;
};

// ============================================================
// 2D A* 搜索器
// ============================================================
class Astar2D {
public:
    enum FailureReason {
        ASTAR_OK = 0,
        ASTAR_ENDPOINT_BLOCKED,
        ASTAR_TIMEOUT,
        ASTAR_NO_PATH,
        ASTAR_UNKNOWN_REJECTED
    };

    Astar2D() {}
    ~Astar2D() {}

    void init(CoverageMap *map);

    bool search(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                std::vector<Eigen::Vector3d> &path,
                double *path_len = nullptr,
                int *unknown_cells = nullptr,
                double *unknown_ratio = nullptr);

    bool isPathClear(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);
    bool isPathTraversable(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                           double *path_len = nullptr,
                           int *unknown_cells = nullptr,
                           double *unknown_ratio = nullptr);
    bool findReachableApproach(const Eigen::Vector3d &start, const Eigen::Vector3d &target,
                               Eigen::Vector3d &approach,
                               std::vector<Eigen::Vector3d> &path);

    int toAddress2D(int x, int y);
    void setSearchTimeout(double timeout_ms) { search_timeout_ms_ = timeout_ms; }
    FailureReason lastFailureReason() const { return last_failure_reason_; }

private:
    CoverageMap *map_;
    double search_timeout_ms_;
    FailureReason last_failure_reason_;

    int dx_[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy_[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    double move_cost_[8] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};

    double heuristic(const Eigen::Vector2i &from, const Eigen::Vector2i &to);
    bool isWalkable(int x, int y);
    bool analyzePathUnknown(const std::vector<Eigen::Vector2i> &idx_path,
                            double &path_len,
                            int &unknown_cells,
                            double &unknown_ratio);
};

// ============================================================
// 前沿查找器
// ============================================================
class FrontierFinder {
public:
    FrontierFinder() {}
    ~FrontierFinder() {}

    void init(ros::NodeHandle &nh, CoverageMap *map);
    void searchFrontiers(const Eigen::Vector3d &cur_pos);
    void getFrontierAverages(std::vector<Eigen::Vector3d> &averages);
    void getFrontierClusters(std::vector<std::vector<Eigen::Vector3d>> &clusters);
    int  getFrontierCount();
    void computeViewpoints(const Eigen::Vector3d &cur_pos);
    void getTopViewpoints(const Eigen::Vector3d &cur_pos,
                          double cur_yaw,
                          std::vector<Eigen::Vector3d> &points,
                          std::vector<double> &yaws);
    void setMap(CoverageMap *map) { map_ = map; }
    void publishFrontiers();
    bool adoptFrontier(const prometheus_two_uav_coverage_search::SwarmFrontier &frontier);
    bool moveFrontierToSleeping(uint64_t task_id);
    std::vector<uint64_t> consumeRetiredTaskIds();
    bool isFrontierCellsCovered(const std::vector<Eigen::Vector3d> &cells,
                                double changed_fraction);
    double lastAabbSearchMs() const { return last_aabb_search_ms_; }
    double lastViewpointMs() const { return last_viewpoint_ms_; }
    int lastLocalUpdateBoxCount() const { return last_local_update_box_count_; }
    int lastRemoteUpdateBoxCount() const { return last_remote_update_box_count_; }
    int lastMergedBoxCount() const { return last_merged_box_count_; }

    int cluster_min_;
    double cluster_size_xy_;
    double candidate_radius_;
    double candidate_clearance_;
    int down_sample_;
    double sensing_range_;
    double sensing_fov_h_;
    double sensing_fov_v_;
    double sensing_pitch_;
    double min_frontier_height_;
    double max_frontier_height_;
    double max_frontier_above_flight_;
    int frontier_z_half_layers_;
    int split_min_cells_;
    double split_min_aspect_ratio_;
    double split_max_aspect_ratio_;
    int min_frontier_z_layers_;
    double min_frontier_z_extent_;
    int min_frontier_xy_cells_;
    double min_frontier_neighbor_ratio_;
    int vis_down_sample_;
    int vis_max_points_;
    int vis_max_total_points_;
    double retire_changed_ratio_ = 0.15;

    struct FrontierCluster {
        std::vector<Eigen::Vector3d> cells;
        std::vector<Eigen::Vector3d> filtered_cells;
        Eigen::Vector3d average;
        Eigen::Vector3d box_min_, box_max_;
        std::vector<Eigen::Vector3d> viewpoints;
        std::vector<double> viewpoint_yaws;
        std::vector<int> viewpoint_visib_nums;
        int visib_num = 0;
        // Immutable frontier identity, assigned only after clustering and PCA
        // splitting. A task handoff never changes either field.
        uint64_t task_id = 0;
        uint32_t source_uav_id = 0;
        uint32_t source_revision = 0;
        uint64_t changed_generation = 0;
        bool local_discovery = false;
    };

    bool isViewpointVisible(const FrontierCluster &ftr, const Eigen::Vector3d &pos,
                            double yaw);
    void refreshViewpoints(FrontierCluster &ftr, const Eigen::Vector3d &cur_pos);

    std::vector<FrontierCluster> frontiers_;
    std::vector<FrontierCluster> sleeping_frontiers_;
    uint64_t update_generation_ = 0;

private:
    CoverageMap *map_;
    int source_uav_id_ = 1;
    uint64_t next_task_sequence_ = 1;
    void expandFrontier(const Eigen::Vector3i &seed, std::vector<Eigen::Vector3d> &cluster_cells);
    bool haveOverlap(const Eigen::Vector3d &min1, const Eigen::Vector3d &max1,
                     const Eigen::Vector3d &min2, const Eigen::Vector3d &max2);
    void resetFrontierFlag(const FrontierCluster &ftr);
    void splitLargeFrontiers();
    void assignNewTaskIdentities(const std::vector<FrontierCluster> &previous);
    void computeFrontierInfo(FrontierCluster &ftr);
    bool isValidFrontierCluster(const std::vector<Eigen::Vector3d> &cells,
                                bool relaxed = false);
    void downsample(const std::vector<Eigen::Vector3d> &cluster_in,
                    std::vector<Eigen::Vector3d> &cluster_out);
    void sampleViewpoints(FrontierCluster &ftr, const Eigen::Vector3d &cur_pos);
    int countVisibleUnknown(const Eigen::Vector3d &pos, double yaw);
    bool getRepresentativeFrontierCell(const FrontierCluster &ftr,
                                       Eigen::Vector3d &target);
    bool isFrontierCell(const Eigen::Vector3i &idx);
    std::vector<bool> frontier_flag_;
    ros::Publisher frontier_vis_pub_;
    std::set<int> published_frontier_marker_ids_;
    std::vector<uint64_t> retired_task_ids_;
    double last_aabb_search_ms_ = 0.0;
    double last_viewpoint_ms_ = 0.0;
    int last_local_update_box_count_ = 0;
    int last_remote_update_box_count_ = 0;
    int last_merged_box_count_ = 0;
};

// ============================================================
// 分层网格
// ============================================================
class HierarchicalGrid {
public:
    HierarchicalGrid() {}
    ~HierarchicalGrid() {}

    void init(ros::NodeHandle &nh, CoverageMap *map);
    void updateFromMap();
    void inputFrontiers(const std::vector<Eigen::Vector3d> &frontier_averages);
    void calculateCostMatrix(const Eigen::Vector3d &cur_pos,
                             Eigen::MatrixXd &cost_matrix,
                             std::vector<Eigen::Vector3d> &cell_centers);
    void solveCoveragePathGreedy(const Eigen::Vector3d &cur_pos,
                                  std::vector<Eigen::Vector3d> &coverage_path);
    void getActiveCellIds(std::vector<int> &ids);
    Eigen::Vector3d getCellCenter(int id);
    void publishGrid();

    double cell_size_;
    int min_unknown_num_;
    double unknown_penalty_;

    struct GridCell {
        int id_;
        Eigen::Vector3d center_;
        Eigen::Vector3d bbox_min_, bbox_max_;
        int unknown_num_, free_num_, frontier_num_;
        bool is_active_;
        std::vector<int> frontier_ids_;
        std::vector<Eigen::Vector3d> frontier_vps_;
    };

    std::vector<GridCell> grid_cells_;
    std::vector<int> active_cell_ids_;

private:
    CoverageMap *map_;
    int num_cells_x_, num_cells_y_;
    Eigen::Vector3d grid_origin_;
    ros::Publisher grid_vis_pub_;
};

// ============================================================
// ★ 覆盖搜索管理器 - 前沿簇驱动探索
// 核心逻辑：找前沿簇 → 选最近的前沿簇 → A*规划路径 → B样条平滑 → 执行轨迹
// ============================================================
class CoverageSearchManager {
public:
    CoverageSearchManager() {}
    ~CoverageSearchManager();

    void init(ros::NodeHandle &nh);

private:
    CoverageMap coverage_map_;
    FrontierFinder frontier_finder_;
    HierarchicalGrid hierarchical_grid_;
    Astar2D astar2d_;
    std::unique_ptr<tf2_ros::Buffer> planning_tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> planning_tf_listener_;

    ros::Subscriber uav_state_sub_, uav_control_state_sub_;
    ros::Subscriber global_pcl_sub_, local_pcl_sub_, scan_pcl_sub_, depth_pcl_sub_, goal_sub_;
    ros::Subscriber remote_chunk_sub_, map_request_sub_, remote_frontier_sub_, remote_state_sub_;
    ros::Subscriber local_task_sub_, remote_task_sub_, peer_collision_replan_sub_, auction_task_set_sub_;
    ros::Subscriber remote_frontier_transfer_ack_sub_;
    ros::Publisher uav_cmd_pub_, path_vis_pub_, fov_vis_pub_, coverage_status_pub_, uav_label_pub_, completion_ready_pub_;
    ros::Publisher swarm_frontier_pub_, swarm_traj_pub_, map_chunk_pub_, map_request_pub_;
    ros::Publisher auction_assignment_pub_;
    ros::Publisher frontier_transfer_ack_pub_;
    // The command timer must never wait behind depth integration or frontier
    // extraction.  It has its own callback queue and emits only immutable
    // trajectory samples; map/planning state remains single-threaded.
    ros::CallbackQueue trajectory_callback_queue_;
    std::unique_ptr<ros::NodeHandle> trajectory_nh_;
    std::unique_ptr<ros::AsyncSpinner> trajectory_spinner_;
    // Raw-command safety must not wait behind map updates or planning callbacks.
    ros::CallbackQueue heartbeat_callback_queue_;
    std::unique_ptr<ros::NodeHandle> heartbeat_nh_;
    std::unique_ptr<ros::AsyncSpinner> heartbeat_spinner_;
    ros::Timer mainloop_timer_, trajectory_timer_, frontier_timer_, swarm_data_timer_;
    ros::Timer raw_heartbeat_timer_;

    struct AuctionMapSnapshot {
        int nx = 0, ny = 0;
        double resolution = 0.0;
        Eigen::Vector3d origin = Eigen::Vector3d::Zero();
        std::vector<uint8_t> walkable;
    };
    struct AuctionJob {
        uint64_t sequence = 0;
        prometheus_two_uav_coverage_search::SwarmAuctionTaskSet task_set;
        AuctionMapSnapshot map;
        Eigen::Vector3d uav1_start = Eigen::Vector3d::Zero();
        Eigen::Vector3d uav2_start = Eigen::Vector3d::Zero();
    };
    std::mutex auction_worker_mutex_, auction_assignment_mutex_;
    std::condition_variable auction_worker_cv_;
    std::thread auction_worker_;
    AuctionJob auction_job_;
    bool auction_job_ready_ = false, auction_worker_stop_ = false;
    uint64_t auction_job_sequence_ = 0;
    uint64_t auction_active_task_set_hash_ = 0;
    uint64_t auction_active_uav1_state_sequence_ = 0;
    uint64_t auction_active_uav2_state_sequence_ = 0;

    prometheus_msgs::UAVState uav_state_;
    prometheus_msgs::UAVControlState uav_control_state_;
    prometheus_msgs::UAVCommand uav_command_;
    Eigen::Vector3d uav_pos_, uav_vel_;
    double uav_yaw_;
    bool odom_ready_, drone_ready_, sensor_ready_;
    Eigen::Vector3d peer_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_peer_clear_pos_ = Eigen::Vector3d::Zero();
    ros::Time peer_state_received_;
    bool peer_state_valid_ = false;
    bool last_peer_clear_valid_ = false;
    bool peer_collision_replan_requested_ = false;
    ros::Time peer_avoidance_until_;
    int uav_id_;
    string uav_name_;

    double fly_height_, sensing_range_, sensing_fov_h_, sensing_fov_v_;
    double max_vel_, max_acc_, max_yaw_rate_, max_yaw_acc_, replan_time_, goal_reach_dist_;
    // FUEL-style execution triggers.  The handoff itself still uses replan_time_.
    double replan_min_execute_time_ = 0.5;
    double replan_periodic_time_ = 1.5;
    double replan_remaining_time_ = 0.5;
    double peer_avoidance_radius_ = 0.75, peer_avoidance_duration_ = 2.0;
    bool sim_mode_, auto_start_, cooperative_mode_, external_goal_only_;
    int global_coverage_source_;
    int map_input_source_;
    int map_epoch_;
    double swarm_chunk_size_, swarm_chunk_delta_period_;
    double peer_state_timeout_ = 0.6;
    string swarm_tx_prefix_, swarm_rx_prefix_;
    int swarm_chunk_cells_x_, swarm_chunk_cells_y_;
    uint32_t swarm_frontier_revision_ = 0;
    std::vector<int8_t> swarm_last_sent_evidence_;
    std::vector<uint32_t> swarm_chunk_revision_;
    std::vector<uint32_t> swarm_peer_chunk_revision_;
    std::vector<uint8_t> swarm_chunk_snapshot_stage_;
    std::vector<bool> swarm_chunk_force_snapshot_;
    bool atsp_enabled_ = true;
    int atsp_max_clusters_ = 5;
    int atsp_refine_clusters_ = 3;
    int atsp_viewpoints_per_cluster_ = 3;
    double atsp_edge_timeout_ms_ = 100.0;
    double auction_astar_timeout_ms_ = 50.0;
    double atsp_planning_budget_ms_ = 250.0;
    bool atsp_tour_active_ = false;
    prometheus_two_uav_coverage_search::SwarmFrontierArray local_swarm_frontiers_;
    prometheus_two_uav_coverage_search::SwarmFrontierArray remote_swarm_frontiers_;
    prometheus_two_uav_coverage_search::SwarmTaskArray local_swarm_tasks_;
    prometheus_two_uav_coverage_search::SwarmTaskArray remote_swarm_tasks_;
    prometheus_two_uav_coverage_search::SwarmAuctionTaskSet last_auction_task_set_;
    prometheus_two_uav_coverage_search::SwarmAuctionAssignment last_auction_assignment_;
    std::map<uint64_t, uint32_t> remote_transfer_ack_versions_;
    std::map<uint64_t, uint32_t> sent_transfer_ack_versions_;
    ros::Time local_swarm_task_received_, remote_swarm_task_received_,
        remote_swarm_frontier_received_;
    string global_pcl_topic_, local_pcl_topic_, scan_pcl_topic_, depth_pcl_topic_;
    Eigen::Vector3d camera_offset_;
    double camera_pitch_;
    double fov_vis_range_;

    // ★ 简化FSM：INIT → EXPLORING → FINISH
    enum EXEC_STATE { INIT, EXPLORING, FINISH };
    EXEC_STATE exec_state_;

    // ★ 当前目标前沿簇
    Eigen::Vector3d current_goal_;
    double current_goal_yaw_;
    bool has_goal_;
    // ★ 执行期前向抢占使用的平滑运动方向，不参与普通目标硬过滤
    double committed_heading_;
    bool has_committed_heading_;
    // ★ 失败目标短期排除：re-plan 失败的目标点 N 秒内不再被选中，避免死循环
    Eigen::Vector3d failed_goal_;
    ros::Time failed_goal_time_;
    bool has_failed_goal_;
    // ★ 连续规划失败计数
    int consecutive_plan_failures_;
    // ★ 同目标重规划限次：被挡后对同一目标重规划，超过 2 次仍立刻被挡→放弃该目标
    int same_goal_replan_count_;
    // ★ 目标硬截止时间：选定目标时刻 + N 秒仍未到达 → 强制放弃，避免卡死在一个不可达目标上
    ros::Time goal_commit_time_;
    double goal_deadline_sec_;

    // ★ A*路径
    std::vector<Eigen::Vector3d> astar_path_;
    int astar_path_idx_;

    // ★ B样条轨迹执行
    // 轨迹点序列（A*路径经B样条平滑后的密集轨迹点）
    std::vector<Eigen::Vector3d> traj_points_;
    std::vector<Eigen::Vector3d> traj_vels_;
    std::vector<Eigen::Vector3d> traj_accs_;
    std::vector<double> traj_yaws_;
    int traj_idx_;
    bool has_traj_;
    double traj_dt_;  // 轨迹点时间间隔
    // ★ 按位置推进轨迹所需状态
    ros::Time traj_point_reach_time_;   // 进入当前 traj 点的时刻，用于单点停留超时
    double cmd_yaw_smoothed_;           // 平滑后的偏航指令（已限速）
    bool cmd_yaw_inited_;               // 平滑偏航是否已用真实姿态初始化

    struct TimeBspline {
        bool valid = false;
        int degree = 3;
        // duration is the position-trajectory duration. Position and yaw use
        // independent time axes; yaw completes before the terminal P/V/A=0
        // position boundary.
        double duration = 0.0;
        double position_duration = 0.0;
        double yaw_duration = 0.0;
        std::vector<double> knots;      // position knots
        std::vector<double> yaw_knots;  // yaw knots
        std::vector<Eigen::Vector3d> position_ctrl;
        std::vector<Eigen::Vector3d> yaw_ctrl;
    } active_time_spline_;

    struct RealtimeTrajectory {
        uint64_t generation = 0;
        TimeBspline time_spline;
        std::vector<Eigen::Vector3d> points;
        std::vector<Eigen::Vector3d> velocities;
        std::vector<Eigen::Vector3d> accelerations;
        std::vector<double> yaws;
        ros::Time start_time;
        Eigen::Vector3d goal = Eigen::Vector3d::Zero();
        double goal_yaw = 0.0;
        double sample_dt = 0.1;
    };
    std::mutex realtime_trajectory_mutex_;
    std::shared_ptr<const RealtimeTrajectory> realtime_trajectory_;
    std::mutex vehicle_state_mutex_;
    std::atomic<uint64_t> realtime_trajectory_generation_{0};
    std::atomic<uint64_t> realtime_completed_generation_{0};
    std::atomic<uint32_t> realtime_command_id_{0};
    std::atomic<float> realtime_yaw_rate_ref_{0.0f};

    // 旧轨迹执行期间预先生成，抵达交接点后一次性切换。
    struct PendingTrajectory {
        bool ready = false;
        int handoff_idx = -1;
        double handoff_time = -1.0;
        Eigen::Vector3d goal = Eigen::Vector3d::Zero();
        double goal_yaw = 0.0;
        uint64_t task_id = 0;
        std::vector<Eigen::Vector3d> astar_path;
        std::vector<Eigen::Vector3d> points;
        std::vector<Eigen::Vector3d> vels;
        std::vector<Eigen::Vector3d> accs;
        std::vector<double> yaws;
        TimeBspline time_spline;
        uint64_t source_generation = 0;
        uint64_t frontier_generation = 0;
        std::vector<Eigen::Vector3d> frontier_targets;
        std::vector<double> frontier_target_yaws;
        std::vector<uint64_t> frontier_target_task_ids;
        int frontier_target_idx = 0;
        bool atsp_tour_active = false;
        ros::Time result_ready_time;
    } pending_traj_;
    bool rolling_prepare_in_progress_;
    bool rolling_snapshot_mode_ = false;
    uint64_t rolling_generation_ = 0;
    // Coalesces frontier/map changes while the single rolling worker is busy.
    // The active trajectory remains executable until a successor is accepted.
    bool rolling_replan_requested_ = false;
    ros::Time rolling_last_attempt_time_;
    uint64_t rolling_replan_count_ = 0;
    bool planning_start_state_valid_ = false;
    Eigen::Vector3d planning_start_acc_ = Eigen::Vector3d::Zero();
    double planning_start_yaw_rate_ = 0.0;
    double planning_start_yaw_acc_ = 0.0;
    double traj_esdf_ms_ = 0.0;
    double traj_geometry_ms_ = 0.0;
    double traj_profile_ms_ = 0.0;
    struct AtspTiming {
        bool valid = false;
        bool solved = false;
        double total_ms = 0.0;
        double order_dp_ms = 0.0;
        size_t cluster_count = 0;
        const char *reason = "";
    } rolling_atsp_timing_;
    std::atomic<bool> rolling_worker_running_{false};
    std::thread rolling_worker_;
    std::mutex rolling_result_mutex_;
    PendingTrajectory completed_pending_traj_;

    // ★ 前沿簇队列
    std::vector<Eigen::Vector3d> frontier_targets_;
    std::vector<double> frontier_target_yaws_;
    std::vector<uint64_t> frontier_target_task_ids_;
    int frontier_target_idx_;
    // The next locally observed frontier must be visited before a stale tail
    // of the previous ATSP tour.  It is cleared once inserted into a new tour.
    uint64_t local_discovery_priority_task_id_ = 0;
    uint64_t current_goal_task_id_ = 0;
    struct LocalFrontierReservation {
        prometheus_two_uav_coverage_search::SwarmTask task;
        bool active = false;
    };
    std::vector<LocalFrontierReservation> local_frontier_reservations_;
    std::set<uint64_t> retired_frontier_ids_;
    prometheus_two_uav_coverage_search::SwarmTask active_goal_frontier_;
    std::vector<Eigen::Vector3d> active_goal_frontier_cells_;
    bool active_goal_frontier_valid_ = false;
    bool active_goal_frontier_covered_pending_ = false;
    uint64_t active_goal_frontier_checked_generation_ = 0;

    ros::Time start_time_, finish_time_;
    ros::Time coverage_25_time_, coverage_50_time_, coverage_75_time_;
    ros::Time traj_start_time_;  // 轨迹开始时间，用于超时检测
    ros::Time hover_wait_start_; // 悬停等待开始时间
    int replan_count_;

    // 完成仅由固定区域覆盖率决定；协同模式仍等待对机完成共识。
    bool completion_ready_ = false;
    bool peer_completion_ready_ = false;
    double completion_known_ratio_;
    double residual_scan_yaw_rate_;
    double residual_scan_yaw_;

    // ★ 轨迹执行参数（按位置推进）
    double traj_step_size_;          // 轨迹点间距
    double traj_advance_dist_;       // 推进到下一点所需的距离阈值
    double traj_point_dwell_timeout_; // 单点最长停留时间
    double traj_cut_clearance_;      // 与 map/esdf_safe_distance 同步的执行期阈值

    // 卡住检测
    Eigen::Vector3d last_pos_;
    ros::Time last_move_time_;
    bool isStuck();

    // 避障
    double obstacle_check_dist_;
    double safe_distance_;
    bool checkObstacleAhead(Eigen::Vector3d &avoid_dir);
    bool isPositionSafe(const Eigen::Vector3d &pos);

    // 回调
    void uavStateCb(const prometheus_msgs::UAVState::ConstPtr &msg);
    void uavControlStateCb(const prometheus_msgs::UAVControlState::ConstPtr &msg);
    void globalPclCb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void localPclCb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void scanPclCb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void depthPclCb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void goalCb(const geometry_msgs::PoseStampedConstPtr &msg);
    void remoteStateCb(const prometheus_two_uav_coverage_search::SwarmState::ConstPtr &msg);
    void peerCollisionReplanCb(const std_msgs::Bool::ConstPtr &msg);
    void remoteChunkCb(const prometheus_two_uav_coverage_search::SwarmMapChunkConstPtr &msg);
    void remoteFrontierCb(const prometheus_two_uav_coverage_search::SwarmFrontierArrayConstPtr &msg);
    void swarmTaskCb(const prometheus_two_uav_coverage_search::SwarmTaskArrayConstPtr &msg);
    void auctionTaskSetCb(const prometheus_two_uav_coverage_search::SwarmAuctionTaskSetConstPtr &msg);
    void remoteFrontierTransferAckCb(
        const prometheus_two_uav_coverage_search::SwarmFrontierTransferAckConstPtr &msg);
    void mapRequestCb(const prometheus_two_uav_coverage_search::SwarmMapRequestConstPtr &msg);

    void mainloopCb(const ros::TimerEvent &e);
    void trajectoryCb(const ros::TimerEvent &e);
    void frontierCb(const ros::TimerEvent &e);
    void publishStartupHold();
    void updateFrontiers();
    void updateLocalFrontierReservations();
    void synchronizeTransferredFrontiers();
    void retireFrontierTasks(const std::vector<uint64_t> &task_ids);
    void setCompletionReady(bool ready);

    // ★ 核心方法
    bool selectNextFrontier();       // 选择下一个前沿簇目标
    bool selectAtspTour();
    bool hasConfirmedSelfLease(const FrontierFinder::FrontierCluster &frontier);
    bool isAtspTargetValid(const Eigen::Vector3d &target, double yaw, uint64_t task_id);
    bool planPathToGoal();           // A*规划到目标的路径
    void generateBsplineTraj();      // 从A*路径生成B样条轨迹
    void executeTrajectory();        // 执行B样条轨迹
    void executeRealtimeTrajectory();
    void armRealtimeTrajectory();
    void disarmRealtimeTrajectory();
    bool consumeRealtimeTrajectoryCompletion();
    void maintainActiveTrajectory();
    double terminalBrakingDistance() const;
    bool reachedGoal();
    // ★ 三次均匀B样条评估（仅用于几何候选生成）
    Eigen::Vector3d evaluateBspline(const std::vector<Eigen::Vector3d> &ctrl_pts, double t);
    Eigen::Vector3d deBoor(const std::vector<Eigen::Vector3d> &ctrl_pts,
                          const std::vector<double> &knots,
                          int degree, double time) const;
    bool evaluateTimeBspline(const TimeBspline &spline, double time,
                             Eigen::Vector3d &position,
                             Eigen::Vector3d &velocity,
                             Eigen::Vector3d &acceleration,
                             double &yaw, double &yaw_rate,
                             double &yaw_acceleration) const;
    bool buildTimeParameterizedSpline(const Eigen::Vector3d &start_acc,
                                      double start_yaw_rate,
                                      double start_yaw_acceleration);
    bool optimizeTimeBspline2D(TimeBspline &spline, bool rolling,
                               std::string &reason);
    // 三项统一为时间并取瓶颈最大值：路径、运动方向转向、偏航。
    double computeFrontierCost(const Eigen::Vector3d &vp, double vp_yaw,
                               double path_len,
                               const Eigen::Vector3d &initial_path_dir,
                               double *path_time = nullptr,
                               double *direction_time = nullptr,
                               double *yaw_time = nullptr);
    double computeTransitionCost(const Eigen::Vector3d &from_pos,
                                 const Eigen::Vector3d &from_vel,
                                 double from_yaw,
                                 const Eigen::Vector3d &to_pos,
                                 double to_yaw,
                                 double path_len,
                                 const Eigen::Vector3d &initial_path_dir) const;
    double requiredTrajectoryClearance(const Eigen::Vector3d &pos,
                                       double requested_clearance);
    // ★ 检查从 uav_pos 到 target 的直线是否被占据格阻挡（用于轨迹执行时避障）
    bool pathToTargetBlocked(const Eigen::Vector3d &to, const Eigen::Vector3d &from,
                             double min_clearance = -1.0);
    bool currentGoalUnsafe(std::string &reason);
    bool activeTrajectoryUnsafe(std::string &reason);
    void abortCurrentGoalForSafety(const std::string &reason);
    void releaseCurrentGoal(const std::string &reason, bool mark_failed,
                            bool count_as_replan);
    // ★ 目标点前沿验证：目标点附近必须有unknown区域
    bool isNearFrontier(const Eigen::Vector3d &pos);
    bool tryPrepareRollingHandoff(bool force_new_goal = false,
                                  bool handoff_at_terminal = false);
    bool buildContinuousBridge(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &start_vel,
                               const Eigen::Vector3d &start_acc,
                               double start_yaw, double start_yaw_rate,
                               double start_yaw_acceleration);
    bool activatePendingTrajectory();
    // ★ 失败目标短期排除：移除距 failed_goal_ 1.0m 内、且在 3s 内的候选
    void filterFailedGoal();
    void publishVisualization();
    void publishCameraFov();
    void publishCoverageStatus();
    void swarmDataCb(const ros::TimerEvent &e);
    void publishSwarmFrontiers();
    void publishAuctionAssignment(
        const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet &task_set);
    AuctionMapSnapshot captureAuctionMapSnapshot();
    static bool auctionSnapshotPathCost(const AuctionMapSnapshot &map,
                                        const Eigen::Vector3d &start,
                                        const Eigen::Vector3d &goal,
                                        double timeout_ms, double *cost,
                                        uint8_t *failure_mask);
    void auctionWorkerLoop();
    void rawHeartbeatCb(const ros::TimerEvent &e);
    void publishSwarmTrajectory();
    void publishSwarmMapDelta();
    int chunkIdForAddress(int address) const;
    void appendChunk(int chunk_id, bool snapshot,
                     prometheus_two_uav_coverage_search::SwarmMapChunk &msg);
    uint64_t frontierTaskId(const FrontierFinder::FrontierCluster &frontier);
    bool isFrontierLeasedToSelf(const FrontierFinder::FrontierCluster &frontier);
    bool isFrontierLeasedToPeer(const FrontierFinder::FrontierCluster &frontier);
    uint64_t leasedTaskIdForFrontier(const FrontierFinder::FrontierCluster &frontier);
    bool frontierMatchesTask(const FrontierFinder::FrontierCluster &frontier,
                             const prometheus_two_uav_coverage_search::SwarmTask &task);
    bool frontierOverlapsTask(const FrontierFinder::FrontierCluster &frontier,
                              const prometheus_two_uav_coverage_search::SwarmTask &task) const;
    bool getSelfLeasedRemoteTask(uint64_t task_id,
                                 prometheus_two_uav_coverage_search::SwarmTask &task);
    bool isRemoteTaskViewpointValid(
        const prometheus_two_uav_coverage_search::SwarmTask &task,
        const Eigen::Vector3d &point, double yaw);
    bool currentGoalLeaseValid();
    bool captureActiveGoalFrontier();
    bool currentGoalFrontierCovered();
    uint64_t localReservationTaskIdForFrontier(
        const FrontierFinder::FrontierCluster &frontier);
    bool reserveLocalFrontier(const FrontierFinder::FrontierCluster &frontier);
    bool reserveLocalFrontierForGoal(uint64_t task_id, const Eigen::Vector3d &goal);
    void markLocalReservationActive(uint64_t task_id);
    bool hasActiveLocalReservation(uint64_t task_id) const;
    void eraseLocalReservation(uint64_t task_id);

};

#endif // TWO_UAV_COVERAGE_SEARCH_H
