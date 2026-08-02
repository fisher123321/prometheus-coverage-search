#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_two_uav_coverage_search/SwarmState.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

namespace {

struct TimedBodyPose {
    ros::Time stamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond attitude;
};

bool interpolatePose(const std::deque<TimedBodyPose>& history, const ros::Time& stamp,
                     double max_pose_gap, TimedBodyPose& pose) {
    if (history.empty() || stamp.isZero()) return false;
    for (size_t i = 1; i < history.size(); ++i) {
        const TimedBodyPose& before = history[i - 1];
        const TimedBodyPose& after = history[i];
        if (stamp < before.stamp) return false;
        if (stamp > after.stamp) continue;
        const double span = (after.stamp - before.stamp).toSec();
        if (span <= 0.0 || span > max_pose_gap) return false;
        const double alpha = (stamp - before.stamp).toSec() / span;
        pose.stamp = stamp;
        pose.position = before.position + alpha * (after.position - before.position);
        pose.attitude = before.attitude.slerp(alpha, after.attitude).normalized();
        return true;
    }
    return false;
}

void poseInterpolationSelfTest() {
    std::deque<TimedBodyPose> poses;
    poses.push_back({ros::Time(10.0), Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()});
    poses.push_back({ros::Time(10.1), Eigen::Vector3d(1.0, 0.0, 0.0),
                     Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()))});
    TimedBodyPose result;
    assert(interpolatePose(poses, ros::Time(10.05), 0.12, result));
    assert(std::fabs(result.position.x() - 0.5) < 1e-9);
    assert(std::fabs(std::fabs(result.attitude.angularDistance(Eigen::Quaterniond::Identity())) -
                     M_PI_2) < 1e-9);
    assert(!interpolatePose(poses, ros::Time(9.9), 0.12, result));
    assert(!interpolatePose(poses, ros::Time(10.05), 0.02, result));
}

}  // namespace

class DepthCloudDownsampleNode {
public:
    explicit DepthCloudDownsampleNode(ros::NodeHandle &nh) {
        nh.param<std::string>("input_topic", input_topic_, "/uav1/camera/depth/color/points");
        nh.param<std::string>("output_topic", output_topic_, "/uav1/camera/depth/color/points_downsampled");
        nh.param<std::string>("octomap_output_topic", octomap_output_topic_, "");
        nh.param("leaf_size", leaf_size_, 0.10);
        nh.param("min_range", min_range_, 0.2);
        nh.param("max_range", max_range_, 4.5);
        nh.param("max_rate", max_rate_, 10.0);
        nh.param("add_fov_clear_points", add_fov_clear_points_, false);
        nh.param("fov_h", fov_h_, 1.5708);
        nh.param("fov_v", fov_v_, 1.287);
        nh.param("clear_point_range", clear_point_range_, max_range_ + 0.15);
        nh.param("clear_yaw_samples", clear_yaw_samples_, 25);
        nh.param("clear_pitch_samples", clear_pitch_samples_, 15);
        nh.param("clear_block_margin", clear_block_margin_, 1);
        nh.param<std::string>("local_state_topic", local_state_topic_, "");
        nh.param<std::string>("peer_state_topic", peer_state_topic_, "");
        nh.param("peer_clear_radius", peer_clear_radius_, 0.40);
        nh.param("peer_clear_half_height", peer_clear_half_height_, 0.30);
        nh.param("peer_state_timeout", peer_state_timeout_, 0.60);
        nh.param("camera_offset_x", camera_offset_(0), 0.095);
        nh.param("camera_offset_y", camera_offset_(1), 0.0);
        nh.param("camera_offset_z", camera_offset_(2), 0.0);
        nh.param("camera_pitch", camera_pitch_, 0.35);
        nh.param<std::string>("pose_topic", pose_topic_, "");
        nh.param<std::string>("sync_frame", sync_frame_, "");
        nh.param<std::string>("world_frame", world_frame_, "world");
        nh.param("pose_history_sec", pose_history_sec_, 2.0);
        nh.param("max_pose_interp_gap", max_pose_interp_gap_, 0.04);
        nh.param("cloud_sync_queue_size", cloud_sync_queue_size_, 4);
        nh.param("temporal_hit_confirmation", temporal_hit_confirmation_, false);
        nh.param("temporal_hit_voxel", temporal_hit_voxel_, 0.10);
        nh.param("temporal_hit_min_frames", temporal_hit_min_frames_, 3);
        nh.param("max_mapping_angular_rate", max_mapping_angular_rate_, 0.0);

        pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        if (!octomap_output_topic_.empty()) {
            octomap_pub_ = nh.advertise<sensor_msgs::PointCloud2>(octomap_output_topic_, 1);
        }
        sub_ = nh.subscribe(input_topic_, 1, &DepthCloudDownsampleNode::cloudCb, this);
        if (!local_state_topic_.empty() && !peer_state_topic_.empty()) {
            local_state_sub_ = nh.subscribe(local_state_topic_, 1,
                &DepthCloudDownsampleNode::localStateCb, this);
            peer_state_sub_ = nh.subscribe(peer_state_topic_, 10,
                &DepthCloudDownsampleNode::peerStateCb, this);
        }
        if (!pose_topic_.empty() && !sync_frame_.empty()) {
            pose_sub_ = nh.subscribe(pose_topic_, 50, &DepthCloudDownsampleNode::poseCb, this);
        } else if (!pose_topic_.empty() || !sync_frame_.empty()) {
            ROS_WARN("[DepthCloudDownsample] pose_topic and sync_frame must both be set; using legacy TF.");
            pose_topic_.clear();
            sync_frame_.clear();
        }

        ROS_INFO("[DepthCloudDownsample] %s -> %s leaf=%.2f range=[%.2f, %.2f] max_rate=%.1f",
                 input_topic_.c_str(), output_topic_.c_str(), leaf_size_,
                 min_range_, max_range_, max_rate_);
        if (!octomap_output_topic_.empty()) {
            ROS_INFO("[DepthCloudDownsample] octomap output=%s fov_clear=%s clear_range=%.2f",
                     octomap_output_topic_.c_str(),
                     add_fov_clear_points_ ? "true" : "false",
                     clear_point_range_);
        }
        if (!pose_topic_.empty()) {
            ROS_INFO("[DepthCloudDownsample] Planning/OctoMap pose sync: %s -> %s, max edge age %.3fs.",
                     pose_topic_.c_str(), sync_frame_.c_str(), max_pose_interp_gap_);
        }
        if (temporal_hit_confirmation_) {
            ROS_INFO("[DepthCloudDownsample] OctoMap requires %d consecutive depth hits per %.2fm cell.",
                     temporal_hit_min_frames_, temporal_hit_voxel_);
        }
        if (max_mapping_angular_rate_ > 0.0) {
            ROS_INFO("[DepthCloudDownsample] OctoMap pauses above %.2f rad/s body rotation.",
                     max_mapping_angular_rate_);
        }
    }

private:
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr &msg) {
        if (!sync_frame_.empty()) {
            pending_clouds_.push_back(msg);
            while (pending_clouds_.size() > static_cast<size_t>(std::max(1, cloud_sync_queue_size_))) {
                ROS_WARN_THROTTLE(1.0,
                    "[DepthCloudDownsample] Dropped unsynchronized cloud at %.3f (pose stream too late).",
                    pending_clouds_.front()->header.stamp.toSec());
                pending_clouds_.pop_front();
            }
            processPendingClouds();
            return;
        }
        processCloud(msg);
    }

    void processCloud(const sensor_msgs::PointCloud2ConstPtr &msg) {
        const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        if (!last_pub_time_.isZero() && max_rate_ > 0.0 &&
            stamp >= last_pub_time_ &&
            (stamp - last_pub_time_).toSec() < 1.0 / max_rate_) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ranged(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        ranged->reserve(cloud->size());
        const double min_r2 = min_range_ * min_range_;
        const double max_r2 = max_range_ * max_range_;
        for (const auto &pt : cloud->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            const double r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            if (r2 < min_r2 || r2 > max_r2) continue;
            ranged->points.push_back(pt);
        }
        ranged->width = ranged->points.size();
        ranged->height = 1;
        ranged->is_dense = false;
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(ranged);
        voxel.setLeafSize((float)leaf_size_, (float)leaf_size_, (float)leaf_size_);
        voxel.filter(*filtered);
        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(*filtered, out);
        out.header = msg->header;
        if (!sync_frame_.empty()) {
            if (!publishSynchronizedCameraTf(msg->header.stamp)) {
                ROS_WARN_THROTTLE(1.0,
                    "[DepthCloudDownsample] Dropped planning cloud: no capture-time pose at %.3f.",
                    msg->header.stamp.toSec());
                last_pub_time_ = stamp;
                return;
            }
            out.header.frame_id = sync_frame_;
        }
        pub_.publish(out);

        if (octomap_pub_) {
            if (!mappingPoseStable(msg->header.stamp)) {
                ROS_WARN_THROTTLE(1.0,
                    "[DepthCloudDownsample] Skipped OctoMap cloud during fast body rotation.");
                last_pub_time_ = stamp;
                return;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr raw_octomap_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr octomap_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            const int suppressed = removePeerPoints(*filtered, *raw_octomap_cloud);
            filterTemporalHits(*raw_octomap_cloud, msg->header.stamp, *octomap_cloud);
            if (add_fov_clear_points_) addFovClearPoints(*raw_octomap_cloud, *octomap_cloud);
            if (suppressed > 0) {
                ROS_INFO_THROTTLE(1.0,
                    "[DepthCloudDownsample] suppressed %d peer-body points from OctoMap input.",
                    suppressed);
            }

            sensor_msgs::PointCloud2 octomap_out;
            pcl::toROSMsg(*octomap_cloud, octomap_out);
            octomap_out.header = msg->header;
            if (!sync_frame_.empty()) octomap_out.header.frame_id = sync_frame_;
            octomap_pub_.publish(octomap_out);
        }
        last_pub_time_ = stamp;
    }

    void localStateCb(const prometheus_msgs::UAVState::ConstPtr &msg) {
        local_state_ = *msg;
        local_state_valid_ = msg->odom_valid;
    }

    void peerStateCb(const prometheus_two_uav_coverage_search::SwarmState::ConstPtr &msg) {
        peer_pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        peer_state_valid_ = msg->odom_valid;
        peer_state_received_ = ros::Time::now();
    }

    void poseCb(const nav_msgs::Odometry::ConstPtr& msg) {
        if (msg->header.stamp.isZero()) return;
        const auto& q = msg->pose.pose.orientation;
        Eigen::Quaterniond attitude(q.w, q.x, q.y, q.z);
        if (attitude.norm() <= 1e-6) return;
        attitude.normalize();
        TimedBodyPose pose{msg->header.stamp,
                            Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                            msg->pose.pose.position.z),
                            attitude};
        if (!pose_history_.empty() && pose.stamp < pose_history_.back().stamp) {
            ROS_WARN_THROTTLE(1.0,
                "[DepthCloudDownsample] Ignoring out-of-order pose stamp %.3f < %.3f.",
                pose.stamp.toSec(), pose_history_.back().stamp.toSec());
            return;
        }
        pose_history_.push_back(pose);
        const ros::Time oldest = pose.stamp - ros::Duration(std::max(0.1, pose_history_sec_));
        while (pose_history_.size() > 1 && pose_history_.front().stamp < oldest)
            pose_history_.pop_front();
        processPendingClouds();
    }

    void processPendingClouds() {
        while (!pending_clouds_.empty()) {
            const sensor_msgs::PointCloud2ConstPtr& cloud = pending_clouds_.front();
            if (pose_history_.empty() || cloud->header.stamp >= pose_history_.back().stamp) return;
            if (cloud->header.stamp < pose_history_.front().stamp) {
                ROS_WARN_THROTTLE(1.0,
                    "[DepthCloudDownsample] Dropped cloud at %.3f: pose history starts at %.3f.",
                    cloud->header.stamp.toSec(), pose_history_.front().stamp.toSec());
                pending_clouds_.pop_front();
                continue;
            }
            processCloud(cloud);
            pending_clouds_.pop_front();
        }
    }

    bool mappingPoseStable(const ros::Time& stamp) const {
        if (max_mapping_angular_rate_ <= 0.0 || sync_frame_.empty()) return true;
        for (size_t i = 1; i < pose_history_.size(); ++i) {
            const TimedBodyPose& before = pose_history_[i - 1];
            const TimedBodyPose& after = pose_history_[i];
            if (stamp < before.stamp) return false;
            if (stamp > after.stamp) continue;
            const double dt = (after.stamp - before.stamp).toSec();
            return dt > 1e-4 && before.attitude.angularDistance(after.attitude) / dt <=
                max_mapping_angular_rate_;
        }
        return false;
    }

    bool publishSynchronizedCameraTf(const ros::Time& stamp) {
        if (sync_frame_.empty()) return true;
        Eigen::Vector3d camera;
        Eigen::Matrix3d R_wc;
        if (!cameraTransformAt(stamp, camera, R_wc)) return false;

        geometry_msgs::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = world_frame_;
        tf.child_frame_id = sync_frame_;
        tf.transform.translation.x = camera.x();
        tf.transform.translation.y = camera.y();
        tf.transform.translation.z = camera.z();
        const Eigen::Quaterniond q_wc(R_wc);
        tf.transform.rotation.x = q_wc.x();
        tf.transform.rotation.y = q_wc.y();
        tf.transform.rotation.z = q_wc.z();
        tf.transform.rotation.w = q_wc.w();
        sync_tf_pub_.sendTransform(tf);
        return true;
    }

    bool cameraTransformAt(const ros::Time& stamp, Eigen::Vector3d& camera,
                           Eigen::Matrix3d& R_wc) const {
        if (sync_frame_.empty()) return false;
        TimedBodyPose pose;
        if (!interpolatePose(pose_history_, stamp, max_pose_interp_gap_, pose)) return false;

        const Eigen::Matrix3d R_wb = pose.attitude.toRotationMatrix();
        Eigen::Matrix3d R_opt_to_body;
        R_opt_to_body << 0, 0, 1, -1, 0, 0, 0, -1, 0;
        R_wc = R_wb *
            Eigen::AngleAxisd(camera_pitch_, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            R_opt_to_body;
        camera = pose.position + R_wb * camera_offset_;
        return true;
    }

    static int64_t hitCellKey(const Eigen::Vector3d& point, double voxel) {
        const int64_t bias = 1 << 20;
        const int64_t mask = (1 << 21) - 1;
        const int64_t x = static_cast<int64_t>(std::floor(point.x() / voxel)) + bias;
        const int64_t y = static_cast<int64_t>(std::floor(point.y() / voxel)) + bias;
        const int64_t z = static_cast<int64_t>(std::floor(point.z() / voxel)) + bias;
        return ((x & mask) << 42) | ((y & mask) << 21) | (z & mask);
    }

    void filterTemporalHits(const pcl::PointCloud<pcl::PointXYZ>& input,
                            const ros::Time& stamp, pcl::PointCloud<pcl::PointXYZ>& output) {
        output.clear();
        if (!temporal_hit_confirmation_ || sync_frame_.empty()) {
            output = input;
            return;
        }
        Eigen::Vector3d camera;
        Eigen::Matrix3d R_wc;
        if (!cameraTransformAt(stamp, camera, R_wc)) return;
        std::unordered_map<int64_t, uint8_t> current_streaks;
        current_streaks.reserve(input.size());
        output.reserve(input.size());
        for (const auto& point : input.points) {
            const Eigen::Vector3d world = camera + R_wc * Eigen::Vector3d(point.x, point.y, point.z);
            const int64_t key = hitCellKey(world, temporal_hit_voxel_);
            const auto previous = last_hit_streaks_.find(key);
            const uint8_t streak = previous == last_hit_streaks_.end() ? 1 :
                static_cast<uint8_t>(std::min(255, previous->second + 1));
            current_streaks[key] = streak;
            if (streak >= temporal_hit_min_frames_) output.points.push_back(point);
        }
        output.width = output.points.size();
        output.height = 1;
        output.is_dense = false;
        last_hit_streaks_.swap(current_streaks);
    }

    int removePeerPoints(const pcl::PointCloud<pcl::PointXYZ> &input,
                         pcl::PointCloud<pcl::PointXYZ> &output) const {
        output.clear();
        output.reserve(input.size());
        const bool peer_fresh = peer_state_valid_ && local_state_valid_ &&
            (ros::Time::now() - peer_state_received_).toSec() <= peer_state_timeout_;
        if (!peer_fresh) {
            output = input;
            return 0;
        }

        Eigen::Quaterniond q_wb(local_state_.attitude_q.w, local_state_.attitude_q.x,
                                local_state_.attitude_q.y, local_state_.attitude_q.z);
        if (q_wb.norm() <= 1e-3) {
            output = input;
            return 0;
        }
        q_wb.normalize();
        const Eigen::Matrix3d R_wb = q_wb.toRotationMatrix();
        Eigen::Matrix3d R_opt_to_body;
        R_opt_to_body << 0, 0, 1, -1, 0, 0, 0, -1, 0;
        const Eigen::Matrix3d R_wc = R_wb *
            Eigen::AngleAxisd(camera_pitch_, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            R_opt_to_body;
        const Eigen::Vector3d camera_pos(local_state_.position[0], local_state_.position[1],
                                          local_state_.position[2]);
        const Eigen::Vector3d origin = camera_pos + R_wb * camera_offset_;

        int suppressed = 0;
        for (const auto &pt : input.points) {
            const Eigen::Vector3d world = origin + R_wc * Eigen::Vector3d(pt.x, pt.y, pt.z);
            if ((world.head<2>() - peer_pos_.head<2>()).norm() <= peer_clear_radius_ &&
                std::fabs(world(2) - peer_pos_(2)) <= peer_clear_half_height_) {
                ++suppressed;
                continue;
            }
            output.points.push_back(pt);
        }
        output.width = output.points.size();
        output.height = 1;
        output.is_dense = false;
        return suppressed;
    }

    void addFovClearPoints(const pcl::PointCloud<pcl::PointXYZ> &real_points,
                           pcl::PointCloud<pcl::PointXYZ> &out) const {
        if (clear_yaw_samples_ < 2 || clear_pitch_samples_ < 2) return;

        const double half_h = 0.5 * fov_h_;
        const double half_v = 0.5 * fov_v_;
        std::vector<uint8_t> blocked(clear_yaw_samples_ * clear_pitch_samples_, 0);

        auto mark = [&](int iy, int ip) {
            for (int dy = -clear_block_margin_; dy <= clear_block_margin_; ++dy) {
                for (int dp = -clear_block_margin_; dp <= clear_block_margin_; ++dp) {
                    int y = iy + dy;
                    int p = ip + dp;
                    if (y < 0 || y >= clear_yaw_samples_ || p < 0 || p >= clear_pitch_samples_) continue;
                    blocked[y * clear_pitch_samples_ + p] = 1;
                }
            }
        };

        for (const auto &pt : real_points.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (r < min_range_ || r > max_range_ || pt.z <= 1e-3) continue;
            double yaw = std::atan2(pt.x, pt.z);
            double pitch = std::atan2(pt.y, pt.z);
            if (std::abs(yaw) > half_h || std::abs(pitch) > half_v) continue;
            int iy = (int)std::round((yaw + half_h) / (2.0 * half_h) * (clear_yaw_samples_ - 1));
            int ip = (int)std::round((pitch + half_v) / (2.0 * half_v) * (clear_pitch_samples_ - 1));
            mark(iy, ip);
        }

        out.points.reserve(out.points.size() + clear_yaw_samples_ * clear_pitch_samples_);
        for (int iy = 0; iy < clear_yaw_samples_; ++iy) {
            double yaw = -half_h + 2.0 * half_h * iy / (clear_yaw_samples_ - 1);
            for (int ip = 0; ip < clear_pitch_samples_; ++ip) {
                if (blocked[iy * clear_pitch_samples_ + ip]) continue;
                double pitch = -half_v + 2.0 * half_v * ip / (clear_pitch_samples_ - 1);
                double tx = std::tan(yaw);
                double ty = std::tan(pitch);
                double inv_norm = 1.0 / std::sqrt(tx * tx + ty * ty + 1.0);
                pcl::PointXYZ p;
                p.x = clear_point_range_ * tx * inv_norm;
                p.y = clear_point_range_ * ty * inv_norm;
                p.z = clear_point_range_ * inv_norm;
                out.points.push_back(p);
            }
        }
        out.width = out.points.size();
        out.height = 1;
        out.is_dense = false;
    }

    ros::Subscriber sub_;
    ros::Subscriber local_state_sub_;
    ros::Subscriber peer_state_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher pub_;
    ros::Publisher octomap_pub_;
    ros::Time last_pub_time_;
    tf2_ros::TransformBroadcaster sync_tf_pub_;
    std::string input_topic_;
    std::string output_topic_;
    std::string octomap_output_topic_;
    double leaf_size_;
    double min_range_;
    double max_range_;
    double max_rate_;
    bool add_fov_clear_points_;
    double fov_h_;
    double fov_v_;
    double clear_point_range_;
    int clear_yaw_samples_;
    int clear_pitch_samples_;
    int clear_block_margin_;
    std::string local_state_topic_;
    std::string peer_state_topic_;
    double peer_clear_radius_;
    double peer_clear_half_height_;
    double peer_state_timeout_;
    Eigen::Vector3d camera_offset_;
    double camera_pitch_;
    std::string pose_topic_;
    std::string sync_frame_;
    std::string world_frame_;
    double pose_history_sec_;
    double max_pose_interp_gap_;
    int cloud_sync_queue_size_;
    bool temporal_hit_confirmation_;
    double temporal_hit_voxel_;
    int temporal_hit_min_frames_;
    double max_mapping_angular_rate_;
    std::deque<TimedBodyPose> pose_history_;
    std::deque<sensor_msgs::PointCloud2ConstPtr> pending_clouds_;
    std::unordered_map<int64_t, uint8_t> last_hit_streaks_;
    prometheus_msgs::UAVState local_state_;
    bool local_state_valid_ = false;
    Eigen::Vector3d peer_pos_ = Eigen::Vector3d::Zero();
    bool peer_state_valid_ = false;
    ros::Time peer_state_received_;
};

int main(int argc, char **argv) {
    if (argc == 2 && std::string(argv[1]) == "--self-test") {
        poseInterpolationSelfTest();
        return 0;
    }
    ros::init(argc, argv, "two_uav_depth_cloud_downsample_node");
    ros::NodeHandle nh("~");
    DepthCloudDownsampleNode node(nh);
    ros::spin();
    return 0;
}
