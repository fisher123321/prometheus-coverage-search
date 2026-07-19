#include "two_uav_coverage_search.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "two_uav_coverage_search_node");
    ros::NodeHandle nh("~");

    CoverageSearchManager manager;
    manager.init(nh);

    cout << GREEN << "[CoverageSearchNode] Started." << TAIL << endl;

    ros::spin();
    return 0;
}
