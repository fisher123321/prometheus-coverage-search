#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

namespace {

// The output is a coverage map, not an obstacle map: every voxel known by
// either source is written as free so RViz renders known=white, unknown=dark.
void mergeKnownLeaves(const octomap::OcTree& source, octomap::OcTree* output) {
  const double resolution = output->getResolution();
  for (octomap::OcTree::leaf_iterator it = source.begin_leafs(), end = source.end_leafs();
       it != end; ++it) {
    const unsigned int steps = std::max(1u, static_cast<unsigned int>(
        std::lround(it.getSize() / resolution)));
    const double half = 0.5 * it.getSize();
    const double min_x = it.getX() - half + 0.5 * resolution;
    const double min_y = it.getY() - half + 0.5 * resolution;
    const double min_z = it.getZ() - half + 0.5 * resolution;
    for (unsigned int x = 0; x < steps; ++x) {
      for (unsigned int y = 0; y < steps; ++y) {
        for (unsigned int z = 0; z < steps; ++z) {
          output->updateNode(octomap::point3d(min_x + x * resolution,
                                               min_y + y * resolution,
                                               min_z + z * resolution), false, true);
        }
      }
    }
  }
}

bool knownUnionSelfTest() {
  octomap::OcTree source(0.1), output(0.1);
  const octomap::point3d point(1.0f, 2.0f, 1.5f);
  source.updateNode(point, true);
  source.updateInnerOccupancy();
  mergeKnownLeaves(source, &output);
  output.updateInnerOccupancy();
  const octomap::OcTreeNode* node = output.search(point);
  return node != nullptr && !output.isNodeOccupied(node);
}

class ExploredMapFuser {
 public:
  void init(ros::NodeHandle& nh) {
    nh_ = nh;
    nh_.param<std::string>("uav1_input", uav1_input_, "/uav1/octomap_binary");
    nh_.param<std::string>("uav2_input", uav2_input_, "/uav2/octomap_binary");
    nh_.param<std::string>("output", output_topic_, "/two_uav/fused_explored_octomap_binary");
    nh_.param<std::string>("frame_id", frame_id_, "world");
    uav1_sub_ = nh_.subscribe(uav1_input_, 1, &ExploredMapFuser::mapCb, this);
    uav2_sub_ = nh_.subscribe(uav2_input_, 1, &ExploredMapFuser::mapCb, this);
    output_pub_ = nh_.advertise<octomap_msgs::Octomap>(output_topic_, 1, true);
  }

 private:
  void mapCb(const octomap_msgs::Octomap::ConstPtr& msg) {
    std::unique_ptr<octomap::AbstractOcTree> decoded(octomap_msgs::msgToMap(*msg));
    const auto* source = dynamic_cast<const octomap::OcTree*>(decoded.get());
    if (source == nullptr) {
      ROS_WARN_THROTTLE(2.0, "[explored_map_fuser] ignored non-OcTree input on %s", msg->header.frame_id.c_str());
      return;
    }
    if (!map_) map_.reset(new octomap::OcTree(source->getResolution()));
    if (std::fabs(map_->getResolution() - source->getResolution()) > 1e-6) {
      ROS_WARN_THROTTLE(2.0, "[explored_map_fuser] ignored map with resolution %.3f; expected %.3f",
                        source->getResolution(), map_->getResolution());
      return;
    }
    mergeKnownLeaves(*source, map_.get());
    map_->updateInnerOccupancy();
    octomap_msgs::Octomap out;
    if (!octomap_msgs::binaryMapToMsg(*map_, out)) return;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = frame_id_;
    output_pub_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::Subscriber uav1_sub_, uav2_sub_;
  ros::Publisher output_pub_;
  std::unique_ptr<octomap::OcTree> map_;
  std::string uav1_input_, uav2_input_, output_topic_, frame_id_;
};

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--self-test") return knownUnionSelfTest() ? 0 : 1;
  ros::init(argc, argv, "two_uav_explored_map_fuser");
  ros::NodeHandle nh("~");
  ExploredMapFuser fuser;
  fuser.init(nh);
  ros::spin();
  return 0;
}
