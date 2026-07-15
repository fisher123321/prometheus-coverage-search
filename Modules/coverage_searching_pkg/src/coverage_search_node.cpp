#include "coverage_search.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "coverage_search_node");
    ros::NodeHandle nh("~");

    CoverageSearchManager manager;
    manager.init(nh);

    cout << GREEN << "[CoverageSearchNode] Started." << TAIL << endl;

    ros::spin();
    return 0;
}
