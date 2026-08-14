#include <gtest/gtest.h>

#include "two_uav_coverage_search.h"

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(CoverageMapExchange, NeutralObservedVoxelIsNotEncodedAsUnknown) {
    CoverageMap map;
    map.occupancy_evidence_buffer_ = {0};
    map.observed_buffer_ = {1};
    map.occupancy_buffer_ = {1};
    EXPECT_EQ(-1, map.getLocalEvidence(0));
    map.occupancy_buffer_[0] = 2;
    EXPECT_EQ(2, map.getLocalEvidence(0));
    map.observed_buffer_[0] = 0;
    EXPECT_EQ(0, map.getLocalEvidence(0));
}

TEST(GoalDeadline, ScalesWithEstimatedTravelTime) {
    EXPECT_DOUBLE_EQ(30.0, goalDeadlineSeconds(5.0));
    EXPECT_DOUBLE_EQ(50.0, goalDeadlineSeconds(30.0));
}

TEST(FrontierContinuation, RequiresNearbyForwardGeometry) {
    EXPECT_GT(frontierContinuationConfidence(0.10, 0.60, 0.40, true,
                                              0.9, true, 0.20), 0.55);
    EXPECT_DOUBLE_EQ(0.0, frontierContinuationConfidence(0.10, 0.60, -0.30,
                                                         true, 1.0, true, 0.20));
    EXPECT_DOUBLE_EQ(0.0, frontierContinuationConfidence(0.70, 0.60, 0.40,
                                                         true, 1.0, true, 0.20));
}

TEST(KinodynamicAstar2D, PreservesMovingStartAndReachesLocalGoal) {
    CoverageMap map;
    map.resolution_ = 0.20;
    map.inv_resolution_ = 5.0;
    map.origin_ = Eigen::Vector3d::Zero();
    map.map_size_3d_ = Eigen::Vector3d(8.0, 8.0, 0.20);
    map.grid_size_ = Eigen::Vector3i(40, 40, 1);
    map.fly_height_ = 0.10;
    map.fly_z_idx_ = map.min_z_idx_ = map.max_z_idx_ = 0;
    map.esdf_safe_distance_ = 0.45;
    map.esdf_max_distance_ = 5.0;
    map.tsdf_trunc_dist_ = 1.0;
    map.map_ready_ = true;
    const int cells = map.grid_size_.prod();
    map.occupancy_buffer_.assign(cells, 1);
    map.observed_buffer_.assign(cells, 1);
    map.occupancy_evidence_buffer_.assign(cells, 1);
    map.esdf_buffer_.assign(cells, map.esdf_max_distance_);
    map.tsdf_buffer_.assign(cells, map.tsdf_trunc_dist_);
    map.updateDistanceFields(true);

    KinodynamicAstar2D planner;
    planner.init(&map);
    const Eigen::Vector3d start(1.0, 1.0, map.fly_height_);
    const Eigen::Vector3d goal(3.5, 2.0, map.fly_height_);
    const std::vector<Eigen::Vector3d> guide{start, goal};
    std::vector<Eigen::Vector3d> path;
    ASSERT_TRUE(planner.search(start, Eigen::Vector3d(0.8, 0.0, 0.0),
                               Eigen::Vector3d::Zero(), goal,
                               Eigen::Vector3d::Zero(), 1.0, 0.8, 250.0, path,
                               &guide, 0.8));
    ASSERT_GE(path.size(), 3u);
    EXPECT_LT((path.front() - start).norm(), 1e-9);
    EXPECT_LT((path.back() - goal).norm(), 1e-9);
    EXPECT_GT(path[1](0), path[0](0));
    const Eigen::Vector2d segment = goal.head<2>() - start.head<2>();
    for (const auto &point : path) {
        const double t = std::max(0.0, std::min(1.0,
            (point.head<2>() - start.head<2>()).dot(segment) / segment.squaredNorm()));
        EXPECT_LE((point.head<2>() - (start.head<2>() + t * segment)).norm(), 0.81);
    }
}
