#include "coverage_search.h"
#include <limits>

namespace {
template <typename FGet, typename FSet>
void fillESDF1D(FGet f_get, FSet f_set, int start, int end) {
    if (end < start) return;
    bool has_finite = false;
    for (int q = start; q <= end; ++q) {
        if (f_get(q) < 1e8) {
            has_finite = true;
            break;
        }
    }
    if (!has_finite) {
        for (int q = start; q <= end; ++q) f_set(q, 1e9);
        return;
    }

    const int n = end - start + 1;
    std::vector<int> v(n, 0);
    std::vector<double> z(n + 1, 0.0);

    int k = 0;
    v[0] = start;
    z[0] = -std::numeric_limits<double>::max();
    z[1] = std::numeric_limits<double>::max();

    for (int q = start + 1; q <= end; ++q) {
        double s = 0.0;
        do {
            double fq = f_get(q);
            double fv = f_get(v[k]);
            s = ((fq + q * q) - (fv + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
            if (s > z[k]) break;
            k--;
        } while (k >= 0);
        if (k < 0) {
            k = 0;
            s = -std::numeric_limits<double>::max();
        }
        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = std::numeric_limits<double>::max();
    }

    k = 0;
    for (int q = start; q <= end; ++q) {
        while (z[k + 1] < q) k++;
        double val = (q - v[k]) * (q - v[k]) + f_get(v[k]);
        f_set(q, val);
    }
}
}
// ============================================================
// CoverageMap 实现 - 新增2D查询方法
// ============================================================
void CoverageMap::init(ros::NodeHandle &nh) {
    nh.param("coverage_search/sim_mode", sim_mode_, true);
    nh.param("coverage_search/fly_height", fly_height_, 1.5);
    nh.param("map/resolution", resolution_, 0.2);
    nh.param("map/origin_x", origin_(0), -15.0);
    nh.param("map/origin_y", origin_(1), -15.0);
    nh.param("map/origin_z", origin_(2), -0.5);
    nh.param("map/map_size_x", map_size_3d_(0), 30.0);
    nh.param("map/map_size_y", map_size_3d_(1), 30.0);
    nh.param("map/map_size_z", map_size_3d_(2), 3.0);
    nh.param("map/queue_size", queue_size_, 5);
    nh.param("sensor/depth_min_range", depth_min_range_, 0.3);
    nh.param("sensor/depth_max_range", depth_max_range_, 3.0);
    nh.param("sensor/depth_height_tolerance", depth_height_tolerance_, 0.6);
    nh.param("sensor/depth_ground_ignore_height", depth_ground_ignore_height_, 0.25);
    nh.param("coverage_search/sensing_fov_h", depth_fov_h_, 1.571);
    nh.param("coverage_search/sensing_fov_v", depth_fov_v_, 1.287);
    nh.param("map/esdf_max_distance", esdf_max_distance_, 6.0);
    nh.param("map/esdf_safe_distance", esdf_safe_distance_, 0.35);
    nh.param("map/tsdf_trunc_dist", tsdf_trunc_dist_, 1.0);
    nh.param("map/debug_unknown_boundary_vis", debug_unknown_boundary_vis_, true);
    nh.param("map/robot_clear_radius", robot_clear_radius_, 0.22);
    nh.param("map/robot_clear_half_height", robot_clear_half_height_, 0.20);

    inv_resolution_ = 1.0 / resolution_;
    for (int i = 0; i < 3; ++i) {
        grid_size_(i) = ceil(map_size_3d_(i) / resolution_);
    }

    int buf_size = grid_size_(0) * grid_size_(1) * grid_size_(2);
    occupancy_buffer_.resize(buf_size, 0);
    occupancy_evidence_buffer_.resize(buf_size, 0);
    occupancy_update_stamp_.resize(buf_size, 0);
    esdf_buffer_.assign(buf_size, (float)esdf_max_distance_);
    tsdf_buffer_.assign(buf_size, (float)tsdf_trunc_dist_);
    esdf_tmp1_.assign(buf_size, 0.0f);
    esdf_tmp2_.assign(buf_size, 0.0f);
    updated_bbox_valid_ = false;
    esdf_dirty_ = true;
    esdf_ready_ = false;
    updated_min_idx_ = Eigen::Vector3i::Zero();
    updated_max_idx_ = Eigen::Vector3i::Zero();

    min_range_ = origin_;
    max_range_ = origin_ + map_size_3d_;

    // ★ 缓存2D查询的z层范围索引
    {
        Eigen::Vector3i tmp;
        posToIndex(Eigen::Vector3d(0, 0, fly_height_), tmp);
        fly_z_idx_ = max(0, min(tmp(2), grid_size_(2) - 1));
        posToIndex(Eigen::Vector3d(0, 0, fly_height_ - resolution_), tmp);
        min_z_idx_ = max(0, tmp(2));
        posToIndex(Eigen::Vector3d(0, 0, fly_height_ + resolution_), tmp);
        max_z_idx_ = min(tmp(2), grid_size_(2) - 1);
    }

    global_pcl_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    st_it_ = 0;
    map_ready_ = false;

    string uav_name;
    int uav_id;
    nh.param("uav_id", uav_id, 1);
    uav_name = "/uav" + std::to_string(uav_id);

    map_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        uav_name + "/prometheus/coverage_search/map_vis", 1);
    coverage_vis_pub_ = nh.advertise<visualization_msgs::Marker>(
        uav_name + "/prometheus/coverage_search/coverage_status", 1);
    map_pub_timer_ = nh.createTimer(ros::Duration(2.0), &CoverageMap::pubMapTimerCb, this);

    cout << GREEN << "[CoverageMap] init. Resolution: " << resolution_
         << ", Size: " << map_size_3d_.transpose()
         << ", Grid: " << grid_size_.transpose() << TAIL << endl;
}

// ★ 新增：2D索引查询（使用缓存的z层索引，避免重复计算）
bool CoverageMap::isOccupied2D(int x, int y) {
    if (!isInMap2D(x, y)) return true;
    for (int z = min_z_idx_; z <= max_z_idx_; z++) {
        Eigen::Vector3i idx(x, y, z);
        int adr = toAddress(idx);
        if (occupancy_buffer_[adr] == 2) return true;
    }
    return false;
}

bool CoverageMap::isFree2D(int x, int y) {
    if (!isInMap2D(x, y)) return false;
    if (isOccupied2D(x, y)) return false;
    for (int z = min_z_idx_; z <= max_z_idx_; z++) {
        Eigen::Vector3i idx(x, y, z);
        int adr = toAddress(idx);
        if (occupancy_buffer_[adr] == 1) return true;
    }
    return false;
}

bool CoverageMap::isUnknown2D(int x, int y) {
    if (!isInMap2D(x, y)) return false;
    if (isOccupied2D(x, y)) return false;
    bool has_unknown = false;
    for (int z = min_z_idx_; z <= max_z_idx_; z++) {
        Eigen::Vector3i idx(x, y, z);
        int adr = toAddress(idx);
        if (occupancy_buffer_[adr] == 1) return false;
        if (occupancy_buffer_[adr] == 0) has_unknown = true;
    }
    return has_unknown;
}

bool CoverageMap::isInMap2D(int x, int y) {
    return x >= 0 && x < grid_size_(0) && y >= 0 && y < grid_size_(1);
}

void CoverageMap::updateFromGlobalPcl(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (map_ready_) return;
    map_ready_ = true;
    pcl::fromROSMsg(*msg, *global_pcl_);
    markUpdatedIndex(Eigen::Vector3i(0, 0, 0),
                     std::max(grid_size_(0), std::max(grid_size_(1), grid_size_(2))));
    for (const auto &pt : global_pcl_->points) {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        if (isInMap(p)) setOccupancy(p, 2);
    }
}

void CoverageMap::updateFromLocalPcl(const sensor_msgs::PointCloud2ConstPtr &msg,
                                      const nav_msgs::Odometry &odom) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_pcl);
    beginOccupancyUpdate();

    if (sim_mode_) {
        if (queue_size_ <= 0) {
            *global_pcl_ += *input_pcl;
        } else {
            point_cloud_pair_[st_it_] = *input_pcl;
            st_it_ = (st_it_ + 1) % queue_size_;
            global_pcl_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            for (auto &kv : point_cloud_pair_) {
                *global_pcl_ += kv.second;
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter_.setInputCloud(global_pcl_);
        voxel_filter_.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter_.filter(*filtered);
        global_pcl_ = filtered;
        for (const auto &pt : global_pcl_->points) {
            Eigen::Vector3d p(pt.x, pt.y, pt.z);
            if (isInMap(p)) integrateHit(p);
        }
    }
    map_ready_ = true;
}

void CoverageMap::updateFromScanPcl(const sensor_msgs::PointCloud2ConstPtr &msg,
                                     const Eigen::Vector3d &sensor_pos, double sensor_yaw) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_pcl);

    if (input_pcl->points.empty()) return;
    beginOccupancyUpdate();

    // 下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter_.setInputCloud(input_pcl);
    voxel_filter_.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel_filter_.filter(*filtered);

    // ★ 预计算偏航角旋转矩阵（2D旋转，z不变）
    double cos_yaw = cos(sensor_yaw);
    double sin_yaw = sin(sensor_yaw);

    int points_in_map = 0;
    for (auto &pt : filtered->points) {
        // ★★★ 关键修复：点云坐标是lidar_link局部坐标，需要根据偏航角旋转到世界坐标
        // 2D激光雷达水平扫描，点云z值接近0（在lidar_link坐标系中）
        // 需要先绕z轴旋转（偏航角），再加上传感器世界坐标位置
        Eigen::Vector3d p3d_local(pt.x, pt.y, pt.z);
        // ★ 2D旋转：绕z轴旋转sensor_yaw
        Eigen::Vector3d p3d;
        p3d(0) = cos_yaw * p3d_local(0) - sin_yaw * p3d_local(1);
        p3d(1) = sin_yaw * p3d_local(0) + cos_yaw * p3d_local(1);
        p3d(2) = p3d_local(2);
        // 加上传感器世界坐标位置
        p3d = sensor_pos + p3d;

        if (!isInMap(p3d)) continue;
        points_in_map++;

        // 射线投射：从传感器位置到该点，标记经过的格子为free
        raycastFree(sensor_pos, p3d);

        // 标记该点为占据
        integrateHit(p3d);

    }

    map_ready_ = true;

    // ★ 调试日志：每100次更新输出一次
    static int update_count = 0;
    if (++update_count % 100 == 0) {
        cout << GREEN << "[CoverageMap] ScanPcl update: " << filtered->points.size()
             << " points, " << points_in_map << " in map, sensor_z=" << sensor_pos(2)
             << ", yaw=" << sensor_yaw << TAIL << endl;
    }
}

void CoverageMap::updateFromDepthPcl(const sensor_msgs::PointCloud2ConstPtr &msg,
                                      const Eigen::Vector3d &camera_pos,
                                      const Eigen::Matrix3d &R_wc) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_pcl);
    if (input_pcl->points.empty()) return;
    beginOccupancyUpdate();

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter_.setInputCloud(input_pcl);
    voxel_filter_.setLeafSize(0.08f, 0.08f, 0.08f);
    voxel_filter_.filter(*filtered);

    Eigen::Vector3d ray_start = camera_pos;
    if (!isInMap(ray_start)) return;

    int used_points = 0;
    for (const auto &pt : filtered->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        Eigen::Vector3d p_cam(pt.x, pt.y, pt.z);
        double range = p_cam.norm();
        if (range < depth_min_range_ || range > depth_max_range_) continue;

        Eigen::Vector3d p_world = camera_pos + R_wc * p_cam;
        if (!isInMap(p_world)) continue;
        used_points++;

        raycastFree(ray_start, p_world);
        if (p_world(2) < depth_ground_ignore_height_) continue;
        integrateHit(p_world);
    }

    raycastCameraFovFree(camera_pos, R_wc, *filtered);
    map_ready_ = true;

    static int depth_update_count = 0;
    if (++depth_update_count % 30 == 0) {
        cout << GREEN << "[CoverageMap] DepthPcl update: " << filtered->points.size()
             << " points, " << used_points << " used, range=["
             << depth_min_range_ << "," << depth_max_range_ << "]" << TAIL << endl;
    }
}

void CoverageMap::raycastCameraFovFree(const Eigen::Vector3d &camera_pos,
                                       const Eigen::Matrix3d &R_wc,
                                       const pcl::PointCloud<pcl::PointXYZ> &real_points) {
    if (!isInMap(camera_pos)) return;

    const double half_h = depth_fov_h_ * 0.5;
    const double half_v = depth_fov_v_ * 0.5;
    if (half_h < 1e-3 || half_v < 1e-3) return;
    const int yaw_samples = std::min(45, std::max(17,
        (int)std::ceil(2.0 * depth_max_range_ * std::tan(half_h) / resolution_) + 1));
    const int pitch_samples = std::min(31, std::max(9,
        (int)std::ceil(2.0 * depth_max_range_ * std::tan(half_v) / resolution_) + 1));
    std::vector<double> stop_range(yaw_samples, depth_max_range_);
    std::vector<uint8_t> occluded(yaw_samples, 0);

    // 覆盖规划按 XY 可达性工作：同一水平视线只清到最近的非地面回波，
    // 防止补充射线从墙体点云的竖直采样间隙穿到障碍物后方。
    for (const auto &pt : real_points.points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
            !std::isfinite(pt.z) || pt.z <= 1e-3) continue;
        double range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (range < depth_min_range_ || range > depth_max_range_) continue;
        Eigen::Vector3d p_world = camera_pos + R_wc * Eigen::Vector3d(pt.x, pt.y, pt.z);
        if (p_world(2) < depth_ground_ignore_height_) continue;
        double yaw = std::atan2(pt.x, pt.z);
        double pitch = std::atan2(pt.y, pt.z);
        if (std::abs(yaw) > half_h || std::abs(pitch) > half_v) continue;
        int iy = (int)std::round((yaw + half_h) / (2.0 * half_h) * (yaw_samples - 1));
        for (int dy = -1; dy <= 1; ++dy) {
            int y = iy + dy;
            if (y < 0 || y >= yaw_samples) continue;
            occluded[y] = 1;
            stop_range[y] = std::min(stop_range[y], range);
        }
    }

    for (int iy = 0; iy < yaw_samples; ++iy) {
        double ay = -half_h + 2.0 * half_h * iy / std::max(1, yaw_samples - 1);
        double clear_range = occluded[iy]
                                 ? std::max(depth_min_range_, stop_range[iy] - resolution_)
                                 : depth_max_range_;
        for (int ip = 0; ip < pitch_samples; ++ip) {
            double ap = -half_v + 2.0 * half_v * ip / std::max(1, pitch_samples - 1);
            Eigen::Vector3d dir_cam(tan(ay), tan(ap), 1.0);
            dir_cam.normalize();
            Eigen::Vector3d end = camera_pos + R_wc * (dir_cam * clear_range);
            if (!isInMap(end)) {
                Eigen::Vector3d clipped = camera_pos;
                Eigen::Vector3d dir = (end - camera_pos);
                double len = dir.norm();
                if (len < 1e-3) continue;
                dir /= len;
                for (double d = resolution_; d <= clear_range; d += resolution_) {
                    Eigen::Vector3d p = camera_pos + d * dir;
                    if (!isInMap(p)) break;
                    clipped = p;
                }
                end = clipped;
            }
            // 补充射线与实测射线共用硬遮挡规则，遇到 occupied 必须终止。
            raycastFree(camera_pos, end, true);
        }
    }
}

bool CoverageMap::isInMap(const Eigen::Vector3d &pos) {
    if (pos(0) < min_range_(0) + 1e-4 || pos(1) < min_range_(1) + 1e-4 ||
        pos(2) < min_range_(2) + 1e-4)
        return false;
    if (pos(0) > max_range_(0) - 1e-4 || pos(1) > max_range_(1) - 1e-4 ||
        pos(2) > max_range_(2) - 1e-4)
        return false;
    return true;
}

bool CoverageMap::isOccupied(const Eigen::Vector3d &pos) {
    if (!isInMap(pos)) return true;
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    return occupancy_buffer_[toAddress(idx)] == 2;
}

bool CoverageMap::isUnknown(const Eigen::Vector3d &pos) {
    if (!isInMap(pos)) return false;
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    return occupancy_buffer_[toAddress(idx)] == 0;
}

bool CoverageMap::isFree(const Eigen::Vector3d &pos) {
    if (!isInMap(pos)) return false;
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    return occupancy_buffer_[toAddress(idx)] == 1;
}

bool CoverageMap::isInMapIndex(const Eigen::Vector3i &idx) {
    return idx(0) >= 0 && idx(0) < grid_size_(0) &&
           idx(1) >= 0 && idx(1) < grid_size_(1) &&
           idx(2) >= 0 && idx(2) < grid_size_(2);
}

bool CoverageMap::isOccupiedIndex(const Eigen::Vector3i &idx) {
    if (!isInMapIndex(idx)) return true;
    return occupancy_buffer_[toAddress(idx)] == 2;
}

bool CoverageMap::isUnknownIndex(const Eigen::Vector3i &idx) {
    if (!isInMapIndex(idx)) return false;
    return occupancy_buffer_[toAddress(idx)] == 0;
}

bool CoverageMap::isFreeIndex(const Eigen::Vector3i &idx) {
    if (!isInMapIndex(idx)) return false;
    return occupancy_buffer_[toAddress(idx)] == 1;
}

void CoverageMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &idx) {
    for (int i = 0; i < 3; ++i)
        idx(i) = floor((pos(i) - origin_(i)) * inv_resolution_);
}

void CoverageMap::indexToPos(const Eigen::Vector3i &idx, Eigen::Vector3d &pos) {
    for (int i = 0; i < 3; ++i)
        pos(i) = (idx(i) + 0.5) * resolution_ + origin_(i);
}

int CoverageMap::toAddress(const Eigen::Vector3i &idx) {
    return idx(0) * grid_size_(1) * grid_size_(2) + idx(1) * grid_size_(2) + idx(2);
}

void CoverageMap::setOccupancy(const Eigen::Vector3d &pos, uint8_t occ) {
    if (!isInMap(pos)) return;
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    int adr = toAddress(idx);
    occupancy_evidence_buffer_[adr] = occ == 2 ? 4 : -3;
    if (occupancy_buffer_[adr] == occ) return;
    occupancy_buffer_[adr] = occ;
    markUpdatedIndex(idx, 1);
    markDistanceFieldDirtyIndex(idx, 1);
}

void CoverageMap::updateOccupancyFromEvidence(const Eigen::Vector3i &idx) {
    if (!isInMapIndex(idx)) return;
    int adr = toAddress(idx);
    uint8_t next = occupancy_buffer_[adr];
    if (occupancy_evidence_buffer_[adr] >= 2) next = 2;
    else if (occupancy_evidence_buffer_[adr] <= -1) next = 1;
    if (next == occupancy_buffer_[adr]) return;
    occupancy_buffer_[adr] = next;
    markUpdatedIndex(idx, 1);
    markDistanceFieldDirtyIndex(idx, 1);
}

void CoverageMap::integrateHit(const Eigen::Vector3d &pos) {
    if (!isInMap(pos)) return;
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    int adr = toAddress(idx);
    unsigned int &stamp = occupancy_update_stamp_[adr];
    if (stamp == occupancy_update_epoch_ + 1) return;
    int increment = stamp == occupancy_update_epoch_ ? 2 : 1;
    stamp = occupancy_update_epoch_ + 1;
    occupancy_evidence_buffer_[adr] = std::min<int>(
        4, occupancy_evidence_buffer_[adr] + increment);
    updateOccupancyFromEvidence(idx);
}

void CoverageMap::integrateMiss(const Eigen::Vector3i &idx) {
    if (!isInMapIndex(idx)) return;
    int adr = toAddress(idx);
    unsigned int &stamp = occupancy_update_stamp_[adr];
    if (stamp == occupancy_update_epoch_ || stamp == occupancy_update_epoch_ + 1) return;
    stamp = occupancy_update_epoch_;
    occupancy_evidence_buffer_[adr] = std::max<int>(
        -3, occupancy_evidence_buffer_[adr] - 1);
    updateOccupancyFromEvidence(idx);
}

void CoverageMap::beginOccupancyUpdate() {
    if (occupancy_update_epoch_ > std::numeric_limits<unsigned int>::max() - 3) {
        std::fill(occupancy_update_stamp_.begin(), occupancy_update_stamp_.end(), 0);
        occupancy_update_epoch_ = 2;
    } else {
        occupancy_update_epoch_ += 2;
    }
}

void CoverageMap::clearRobotVolume(const Eigen::Vector3d &body_pos) {
    if (!isInMap(body_pos)) return;
    Eigen::Vector3i center;
    posToIndex(body_pos, center);
    int rxy = std::max(1, (int)std::ceil(
        (robot_clear_radius_ + 0.5 * resolution_) / resolution_));
    int rz = std::max(1, (int)std::ceil(
        (robot_clear_half_height_ + 0.5 * resolution_) / resolution_));
    int cleared_occupied = 0;
    for (int dx = -rxy; dx <= rxy; ++dx) {
        for (int dy = -rxy; dy <= rxy; ++dy) {
            for (int dz = -rz; dz <= rz; ++dz) {
                Eigen::Vector3i idx = center + Eigen::Vector3i(dx, dy, dz);
                if (!isInMapIndex(idx)) continue;
                Eigen::Vector3d p;
                indexToPos(idx, p);
                if ((p.head<2>() - body_pos.head<2>()).norm() >
                        robot_clear_radius_ + 0.5 * resolution_ ||
                    std::fabs(p(2) - body_pos(2)) >
                        robot_clear_half_height_ + 0.5 * resolution_) {
                    continue;
                }
                if (occupancy_buffer_[toAddress(idx)] == 2) cleared_occupied++;
                setOccupancy(p, 1);
            }
        }
    }
    if (cleared_occupied > 0) {
        ROS_WARN_THROTTLE(1.0,
            "[CoverageMap] Cleared %d occupied voxels inside measured robot volume at "
            "(%.2f, %.2f, %.2f).",
            cleared_occupied, body_pos(0), body_pos(1), body_pos(2));
    }
}

void CoverageMap::markUpdatedIndex(const Eigen::Vector3i &idx, int margin) {
    if (!isInMapIndex(idx)) return;
    Eigen::Vector3i min_idx = idx - Eigen::Vector3i::Constant(std::max(0, margin));
    Eigen::Vector3i max_idx = idx + Eigen::Vector3i::Constant(std::max(0, margin));
    for (int k = 0; k < 3; ++k) {
        min_idx(k) = std::max(0, min_idx(k));
        max_idx(k) = std::min(grid_size_(k) - 1, max_idx(k));
    }

    if (!updated_bbox_valid_) {
        updated_min_idx_ = min_idx;
        updated_max_idx_ = max_idx;
        updated_bbox_valid_ = true;
        return;
    }
    for (int k = 0; k < 3; ++k) {
        updated_min_idx_(k) = std::min(updated_min_idx_(k), min_idx(k));
        updated_max_idx_(k) = std::max(updated_max_idx_(k), max_idx(k));
    }
}

void CoverageMap::markDistanceFieldDirtyIndex(const Eigen::Vector3i &idx, int margin) {
    if (!isInMapIndex(idx)) return;
    esdf_dirty_ = true;

    Eigen::Vector3i min_idx = idx - Eigen::Vector3i::Constant(std::max(0, margin));
    Eigen::Vector3i max_idx = idx + Eigen::Vector3i::Constant(std::max(0, margin));
    for (int k = 0; k < 3; ++k) {
        min_idx(k) = std::max(0, min_idx(k));
        max_idx(k) = std::min(grid_size_(k) - 1, max_idx(k));
    }

    if (!df_bbox_valid_) {
        df_min_idx_ = min_idx;
        df_max_idx_ = max_idx;
        df_bbox_valid_ = true;
        return;
    }
    for (int k = 0; k < 3; ++k) {
        df_min_idx_(k) = std::min(df_min_idx_(k), min_idx(k));
        df_max_idx_(k) = std::max(df_max_idx_(k), max_idx(k));
    }
}

bool CoverageMap::getUpdatedBox(Eigen::Vector3d &update_min,
                                Eigen::Vector3d &update_max,
                                bool reset) {
    if (!updated_bbox_valid_) return false;
    Eigen::Vector3d pmin, pmax;
    indexToPos(updated_min_idx_, pmin);
    indexToPos(updated_max_idx_, pmax);
    update_min = pmin - Eigen::Vector3d::Constant(0.5 * resolution_);
    update_max = pmax + Eigen::Vector3d::Constant(0.5 * resolution_);
    for (int k = 0; k < 3; ++k) {
        update_min(k) = std::max(update_min(k), min_range_(k));
        update_max(k) = std::min(update_max(k), max_range_(k));
    }
    if (reset) updated_bbox_valid_ = false;
    return true;
}

void CoverageMap::updateDistanceFields(bool force) {
    if (!force && !esdf_dirty_ && esdf_ready_) return;

    const int nx = grid_size_(0), ny = grid_size_(1), nz = grid_size_(2);
    const int buf_size = nx * ny * nz;
    const float inf = 1e9f;

    auto addr = [ny, nz](int x, int y, int z) { return x * ny * nz + y * nz + z; };
    auto setSignedDistance = [&](int adr, double dist) {
        double clamped = std::min(dist, esdf_max_distance_);
        esdf_buffer_[adr] = (float)clamped;
        double signed_dist = occupancy_buffer_[adr] == 2 ? -resolution_ : clamped;
        signed_dist = std::max(-tsdf_trunc_dist_, std::min(tsdf_trunc_dist_, signed_dist));
        tsdf_buffer_[adr] = (float)signed_dist;
    };

    auto runFullESDF = [&]() {
        if ((int)esdf_tmp1_.size() != buf_size) esdf_tmp1_.assign(buf_size, 0.0f);
        if ((int)esdf_tmp2_.size() != buf_size) esdf_tmp2_.assign(buf_size, 0.0f);

        bool has_occ = false;
        for (int adr = 0; adr < buf_size; ++adr) {
            if (occupancy_buffer_[adr] == 2) {
                has_occ = true;
                break;
            }
        }
        if (!has_occ) {
            std::fill(esdf_buffer_.begin(), esdf_buffer_.end(), (float)esdf_max_distance_);
            std::fill(tsdf_buffer_.begin(), tsdf_buffer_.end(), (float)tsdf_trunc_dist_);
            return;
        }

        for (int y = 0; y < ny; ++y) {
            for (int z = 0; z < nz; ++z) {
                fillESDF1D(
                    [&](int x) {
                        return occupancy_buffer_[addr(x, y, z)] == 2 ? 0.0 : (double)inf;
                    },
                    [&](int x, double val) {
                        esdf_tmp1_[addr(x, y, z)] = (float)std::min(val, (double)inf);
                    },
                    0, nx - 1);
            }
        }

        for (int x = 0; x < nx; ++x) {
            for (int z = 0; z < nz; ++z) {
                fillESDF1D(
                    [&](int y) {
                        return (double)esdf_tmp1_[addr(x, y, z)];
                    },
                    [&](int y, double val) {
                        esdf_tmp2_[addr(x, y, z)] = (float)std::min(val, (double)inf);
                    },
                    0, ny - 1);
            }
        }

        for (int x = 0; x < nx; ++x) {
            for (int y = 0; y < ny; ++y) {
                fillESDF1D(
                    [&](int z) {
                        return (double)esdf_tmp2_[addr(x, y, z)];
                    },
                    [&](int z, double val) {
                        double dist = resolution_ * std::sqrt(std::min(val, (double)inf));
                        setSignedDistance(addr(x, y, z), dist);
                    },
                    0, nz - 1);
            }
        }
    };

    auto runLocalESDF = [&](const Eigen::Vector3i &dirty_min,
                            const Eigen::Vector3i &dirty_max) -> bool {
        const int r = std::max(1, (int)std::ceil(esdf_max_distance_ * inv_resolution_));
        Eigen::Vector3i target_min = dirty_min - Eigen::Vector3i::Constant(r);
        Eigen::Vector3i target_max = dirty_max + Eigen::Vector3i::Constant(r);
        Eigen::Vector3i source_min = target_min - Eigen::Vector3i::Constant(r);
        Eigen::Vector3i source_max = target_max + Eigen::Vector3i::Constant(r);
        for (int k = 0; k < 3; ++k) {
            target_min(k) = std::max(0, target_min(k));
            target_max(k) = std::min(grid_size_(k) - 1, target_max(k));
            source_min(k) = std::max(0, source_min(k));
            source_max(k) = std::min(grid_size_(k) - 1, source_max(k));
        }

        int lx = source_max(0) - source_min(0) + 1;
        int ly = source_max(1) - source_min(1) + 1;
        int lz = source_max(2) - source_min(2) + 1;
        if (lx <= 0 || ly <= 0 || lz <= 0) return false;
        const int local_size = lx * ly * lz;
        if (local_size > (int)(0.65 * buf_size)) return false;

        auto laddr = [ly, lz](int x, int y, int z) { return x * ly * lz + y * lz + z; };
        std::vector<float> local1(local_size, inf), local2(local_size, inf);

        bool has_occ = false;
        for (int x = 0; x < lx && !has_occ; ++x) {
            for (int y = 0; y < ly && !has_occ; ++y) {
                for (int z = 0; z < lz; ++z) {
                    Eigen::Vector3i g = source_min + Eigen::Vector3i(x, y, z);
                    if (occupancy_buffer_[addr(g(0), g(1), g(2))] == 2) {
                        has_occ = true;
                        break;
                    }
                }
            }
        }
        if (!has_occ) {
            for (int x = target_min(0); x <= target_max(0); ++x)
                for (int y = target_min(1); y <= target_max(1); ++y)
                    for (int z = target_min(2); z <= target_max(2); ++z) {
                        int adr = addr(x, y, z);
                        esdf_buffer_[adr] = (float)esdf_max_distance_;
                        tsdf_buffer_[adr] = (float)tsdf_trunc_dist_;
                    }
            return true;
        }

        for (int y = 0; y < ly; ++y) {
            for (int z = 0; z < lz; ++z) {
                fillESDF1D(
                    [&](int x) {
                        Eigen::Vector3i g = source_min + Eigen::Vector3i(x, y, z);
                        return occupancy_buffer_[addr(g(0), g(1), g(2))] == 2 ? 0.0 : (double)inf;
                    },
                    [&](int x, double val) {
                        local1[laddr(x, y, z)] = (float)std::min(val, (double)inf);
                    },
                    0, lx - 1);
            }
        }

        for (int x = 0; x < lx; ++x) {
            for (int z = 0; z < lz; ++z) {
                fillESDF1D(
                    [&](int y) {
                        return (double)local1[laddr(x, y, z)];
                    },
                    [&](int y, double val) {
                        local2[laddr(x, y, z)] = (float)std::min(val, (double)inf);
                    },
                    0, ly - 1);
            }
        }

        for (int x = 0; x < lx; ++x) {
            int gx = source_min(0) + x;
            if (gx < target_min(0) || gx > target_max(0)) continue;
            for (int y = 0; y < ly; ++y) {
                int gy = source_min(1) + y;
                if (gy < target_min(1) || gy > target_max(1)) continue;
                fillESDF1D(
                    [&](int z) {
                        return (double)local2[laddr(x, y, z)];
                    },
                    [&](int z, double val) {
                        int gz = source_min(2) + z;
                        if (gz < target_min(2) || gz > target_max(2)) return;
                        double dist = resolution_ * std::sqrt(std::min(val, (double)inf));
                        setSignedDistance(addr(gx, gy, gz), dist);
                    },
                    0, lz - 1);
            }
        }
        return true;
    };

    bool updated_local = false;
    if (!force && esdf_ready_ && df_bbox_valid_) {
        updated_local = runLocalESDF(df_min_idx_, df_max_idx_);
    }
    if (!updated_local) runFullESDF();

    df_bbox_valid_ = false;
    esdf_dirty_ = false;
    esdf_ready_ = true;
}

double CoverageMap::getDistance(const Eigen::Vector3d &pos) {
    if (!isInMap(pos)) return 0.0;
    updateDistanceFields();
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    if (!isInMapIndex(idx)) return 0.0;
    return esdf_buffer_[toAddress(idx)];
}

double CoverageMap::getDistance2D(int x, int y) {
    if (!isInMap2D(x, y)) return 0.0;
    updateDistanceFields();
    double min_dist = esdf_max_distance_;
    for (int z = min_z_idx_; z <= max_z_idx_; ++z) {
        min_dist = std::min(min_dist, (double)esdf_buffer_[toAddress(Eigen::Vector3i(x, y, z))]);
    }
    return min_dist;
}

double CoverageMap::getTSDF(const Eigen::Vector3d &pos) {
    if (!isInMap(pos)) return -tsdf_trunc_dist_;
    updateDistanceFields();
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    if (!isInMapIndex(idx)) return -tsdf_trunc_dist_;
    return tsdf_buffer_[toAddress(idx)];
}

double CoverageMap::getKnownSpaceRatio() {
    const int total = getTotalCount();
    return total > 0 ? 1.0 - (double)getUnknownCount() / total : 0.0;
}

int CoverageMap::getUnknownCount() {
    int cnt = 0;
    for (int x = 0; x < grid_size_(0); x++)
        for (int y = 0; y < grid_size_(1); y++)
            if (isUnknown2D(x, y)) cnt++;
    return cnt;
}

int CoverageMap::getFreeCount() {
    int cnt = 0;
    for (int x = 0; x < grid_size_(0); x++)
        for (int y = 0; y < grid_size_(1); y++)
            if (isFree2D(x, y)) cnt++;
    return cnt;
}

int CoverageMap::getTotalCount() { return grid_size_(0) * grid_size_(1); }

void CoverageMap::raycastFree(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                              bool include_end) {
    traceRay(start, end, include_end, true);
}

bool CoverageMap::isRayOccluded(const Eigen::Vector3d &start,
                                const Eigen::Vector3d &end) {
    return traceRay(start, end, true, false, nullptr, true);
}

void CoverageMap::collectVisibleUnknown(const Eigen::Vector3d &start,
                                        const Eigen::Vector3d &end,
                                        std::set<int> &unknown_addresses) {
    traceRay(start, end, true, false, &unknown_addresses, true);
}

bool CoverageMap::traceRay(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                           bool include_end, bool integrate_free,
                           std::set<int> *unknown_addresses,
                           bool projected_occlusion) {
    if (!isInMap(start)) return true;

    Eigen::Vector3i cell, end_cell;
    posToIndex(start, cell);
    posToIndex(end, end_cell);
    const Eigen::Vector3d ray = end - start;
    if (ray.squaredNorm() < 1e-12) return false;

    Eigen::Vector3i step = Eigen::Vector3i::Zero();
    Eigen::Vector3d t_max = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    Eigen::Vector3d t_delta = t_max;
    for (int axis = 0; axis < 3; ++axis) {
        if (std::fabs(ray(axis)) < 1e-12) continue;
        step(axis) = ray(axis) > 0.0 ? 1 : -1;
        const double boundary = origin_(axis) +
            (cell(axis) + (step(axis) > 0 ? 1 : 0)) * resolution_;
        t_max(axis) = (boundary - start(axis)) / ray(axis);
        t_delta(axis) = resolution_ / std::fabs(ray(axis));
    }

    auto visit = [&](const Eigen::Vector3i &idx) {
        if (!isInMapIndex(idx)) return 0;
        const bool is_end = idx == end_cell;
        if (is_end && !include_end) return 2;
        const int address = toAddress(idx);
        const bool occupied = occupancy_buffer_[address] == 2 ||
            (projected_occlusion && isOccupied2D(idx(0), idx(1)));
        if (occupied) {
            if (integrate_free) {
                // ponytail: occupied is the single hard-occlusion rule shared by
                // mapping and viewpoint gain; add decay only with stale-map evidence.
                ROS_WARN_THROTTLE(2.0,
                    "[CoverageMap] Occlusion guard stopped ray at occupied voxel "
                    "(%d,%d,%d), evidence=%d; cells behind it were ignored.",
                    idx(0), idx(1), idx(2),
                    (int)occupancy_evidence_buffer_[address]);
            }
            return 1;
        }
        if (unknown_addresses && occupancy_buffer_[address] == 0)
            unknown_addresses->insert(address);
        if (integrate_free) integrateMiss(idx);
        return is_end ? 2 : 0;
    };

    for (;;) {
        const int result = visit(cell);
        if (result == 1) return true;
        if (result == 2) return false;

        const double next_t = t_max.minCoeff();
        if (next_t > 1.0 + 1e-12) return false;
        for (int axis = 0; axis < 3; ++axis) {
            if (t_max(axis) <= next_t + 1e-12) {
                cell(axis) += step(axis);
                t_max(axis) += t_delta(axis);
            }
        }
        if (!isInMapIndex(cell)) return false;
    }
}

void CoverageMap::publishMap() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "coverage_status";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = min_range_(0) + map_size_3d_(0) / 2;
    marker.pose.position.y = min_range_(1) - 1.0;
    marker.pose.position.z = fly_height_;
    marker.scale.z = 0.5;
    marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 1.0;

    double ratio = getKnownSpaceRatio();
    char text[256];
    snprintf(text, sizeof(text), "Known: %.1f%%  Free: %d  Unknown: %d",
             ratio * 100.0, getFreeCount(), getUnknownCount());
    marker.text = text;
    coverage_vis_pub_.publish(marker);
    publishUnknownBoundaryDebug();
}

void CoverageMap::pubMapTimerCb(const ros::TimerEvent &e) { publishMap(); }

void CoverageMap::publishUnknownBoundaryDebug() {
    if (map_vis_pub_.getNumSubscribers() < 1) return;

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    m.ns = "internal_unknown_free_boundary_2d";
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = debug_unknown_boundary_vis_ ? visualization_msgs::Marker::ADD
                                           : visualization_msgs::Marker::DELETE;
    m.scale.x = resolution_ * 0.75;
    m.scale.y = resolution_ * 0.75;
    m.scale.z = 0.04;
    m.color.r = 0.0;
    m.color.g = 0.95;
    m.color.b = 1.0;
    m.color.a = 0.65;
    m.lifetime = ros::Duration(2.5);

    if (debug_unknown_boundary_vis_) {
        const int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        const int max_points = 3000;
        for (int x = 1; x < grid_size_(0) - 1 && (int)m.points.size() < max_points; ++x) {
            for (int y = 1; y < grid_size_(1) - 1 && (int)m.points.size() < max_points; ++y) {
                if (!isUnknown2D(x, y)) continue;
                bool near_free = false;
                for (const auto &d : dirs) {
                    if (isFree2D(x + d[0], y + d[1])) {
                        near_free = true;
                        break;
                    }
                }
                if (!near_free) continue;
                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, y, fly_z_idx_), pos);
                geometry_msgs::Point p;
                p.x = pos(0);
                p.y = pos(1);
                p.z = 0.08;
                m.points.push_back(p);
            }
        }
    }

    markers.markers.push_back(m);
    map_vis_pub_.publish(markers);
}
