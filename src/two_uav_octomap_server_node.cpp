#include <ros/ros.h>
#include <octomap_server/OctomapServer.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "two_uav_octomap_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  octomap_server::OctomapServer server(private_nh, nh);
  ros::spin();
  return 0;
}
