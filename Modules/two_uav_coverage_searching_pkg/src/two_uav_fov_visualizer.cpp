#include <ros/ros.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <prometheus_msgs/UAVState.h>
#include <visualization_msgs/Marker.h>

class FovVisualizer {
public:
    explicit FovVisualizer(ros::NodeHandle &nh) {
        int uav_id;
        nh.param("uav_id", uav_id, 1);
        nh.param("sensing_range", sensing_range_, 4.5);
        nh.param("sensing_fov_h", sensing_fov_h_, 1.571);
        nh.param("sensing_fov_v", sensing_fov_v_, 1.287);
        nh.param("camera_offset_x", camera_offset_(0), 0.095);
        nh.param("camera_offset_y", camera_offset_(1), 0.0);
        nh.param("camera_offset_z", camera_offset_(2), 0.0);
        nh.param("camera_pitch", camera_pitch_, 0.35);
        nh.param("fov_vis_range", fov_vis_range_, 1.5);
        const std::string uav_name = "/uav" + std::to_string(uav_id);
        pub_ = nh.advertise<visualization_msgs::Marker>(
            uav_name + "/prometheus/coverage_search/fov_vis", 1);
        sub_ = nh.subscribe(uav_name + "/prometheus/state", 1, &FovVisualizer::stateCb, this);
    }

private:
    void stateCb(const prometheus_msgs::UAVState::ConstPtr &state) {
        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.ns = "depth_camera_fov";
        mk.id = 0;
        mk.type = visualization_msgs::Marker::LINE_LIST;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.w = 1.0;
        mk.scale.x = 0.025;
        mk.color.a = 0.95;

        Eigen::Quaterniond q(state->attitude_q.w, state->attitude_q.x,
                             state->attitude_q.y, state->attitude_q.z);
        Eigen::Matrix3d R_wb;
        if (q.norm() > 1e-3) {
            q.normalize();
            R_wb = q.toRotationMatrix();
        } else {
            const double yaw = state->attitude[2];
            R_wb << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
        }
        Eigen::Matrix3d R_opt_to_body;
        R_opt_to_body << 0, 0, 1, -1, 0, 0, 0, -1, 0;
        const Eigen::Matrix3d R_wc = R_wb *
            Eigen::AngleAxisd(camera_pitch_, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            R_opt_to_body;
        const Eigen::Vector3d origin(state->position[0], state->position[1], state->position[2]);
        const Eigen::Vector3d camera = origin + R_wb * camera_offset_;
        const double range = std::min(fov_vis_range_, sensing_range_);
        const double hx = range * tan(sensing_fov_h_ * 0.5);
        const double hy = range * tan(sensing_fov_v_ * 0.5);
        const Eigen::Vector3d corners[] = {
            {-hx, -hy, range}, {hx, -hy, range}, {hx, hy, range}, {-hx, hy, range}};
        Eigen::Vector3d world_corners[4];
        for (int i = 0; i < 4; ++i) world_corners[i] = camera + R_wc * corners[i];
        const auto line = [&mk](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
            geometry_msgs::Point p;
            p.x = a.x(); p.y = a.y(); p.z = a.z(); mk.points.push_back(p);
            p.x = b.x(); p.y = b.y(); p.z = b.z(); mk.points.push_back(p);
        };
        for (const auto &corner : world_corners) line(camera, corner);
        for (int i = 0; i < 4; ++i) line(world_corners[i], world_corners[(i + 1) % 4]);
        pub_.publish(mk);
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
    Eigen::Vector3d camera_offset_;
    double sensing_range_, sensing_fov_h_, sensing_fov_v_, camera_pitch_, fov_vis_range_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "two_uav_fov_visualizer");
    ros::NodeHandle nh("~");
    FovVisualizer visualizer(nh);
    ros::spin();
    return 0;
}
