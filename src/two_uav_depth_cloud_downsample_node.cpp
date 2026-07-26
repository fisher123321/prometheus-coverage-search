#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_two_uav_coverage_search/SwarmState.h>
#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <vector>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class DepthCloudDownsampleNode {
public:
    explicit DepthCloudDownsampleNode(ros::NodeHandle &nh) {
        nh.param<std::string>("input_topic", input_topic_, "/uav1/camera/depth/color/points");
        nh.param<std::string>("output_topic", output_topic_, "/uav1/camera/depth/color/points_downsampled");
        nh.param<std::string>("octomap_output_topic", octomap_output_topic_, "");
        nh.param("leaf_size", leaf_size_, 0.10);
        nh.param("min_range", min_range_, 0.2);
        nh.param("max_range", max_range_, 3.2);
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

        ROS_INFO("[DepthCloudDownsample] %s -> %s leaf=%.2f range=[%.2f, %.2f] max_rate=%.1f",
                 input_topic_.c_str(), output_topic_.c_str(), leaf_size_,
                 min_range_, max_range_, max_rate_);
        if (!octomap_output_topic_.empty()) {
            ROS_INFO("[DepthCloudDownsample] octomap output=%s fov_clear=%s clear_range=%.2f",
                     octomap_output_topic_.c_str(),
                     add_fov_clear_points_ ? "true" : "false",
                     clear_point_range_);
        }
    }

private:
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr &msg) {
        ros::Time now = ros::Time::now();
        if (!last_pub_time_.isZero() && max_rate_ > 0.0 &&
            (now - last_pub_time_).toSec() < 1.0 / max_rate_) {
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
            double r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
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
        pub_.publish(out);

        if (octomap_pub_) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr octomap_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            const int suppressed = removePeerPoints(*filtered, *octomap_cloud);
            if (add_fov_clear_points_) addFovClearPoints(*octomap_cloud, *octomap_cloud);
            if (suppressed > 0) {
                ROS_INFO_THROTTLE(1.0,
                    "[DepthCloudDownsample] suppressed %d peer-body points from OctoMap input.",
                    suppressed);
            }

            sensor_msgs::PointCloud2 octomap_out;
            pcl::toROSMsg(*octomap_cloud, octomap_out);
            octomap_out.header = msg->header;
            octomap_pub_.publish(octomap_out);
        }
        last_pub_time_ = now;
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
    ros::Publisher pub_;
    ros::Publisher octomap_pub_;
    ros::Time last_pub_time_;
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
    prometheus_msgs::UAVState local_state_;
    bool local_state_valid_ = false;
    Eigen::Vector3d peer_pos_ = Eigen::Vector3d::Zero();
    bool peer_state_valid_ = false;
    ros::Time peer_state_received_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "two_uav_depth_cloud_downsample_node");
    ros::NodeHandle nh("~");
    DepthCloudDownsampleNode node(nh);
    ros::spin();
    return 0;
}
