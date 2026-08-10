#include <gtest/gtest.h>

#include "two_uav_coverage_search.h"

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(FrontierLifecycle, TransferKeepsImmutableIdentity) {
    FrontierFinder finder;
    FrontierFinder::FrontierCluster frontier;
    frontier.task_id = (static_cast<uint64_t>(1) << 48) | 7;
    frontier.source_uav_id = 1;
    frontier.source_revision = 3;
    finder.frontiers_.push_back(frontier);

    ASSERT_TRUE(finder.moveFrontierToSleeping(frontier.task_id));
    ASSERT_TRUE(finder.frontiers_.empty());
    ASSERT_EQ(1u, finder.sleeping_frontiers_.size());
    EXPECT_EQ(frontier.task_id, finder.sleeping_frontiers_.front().task_id);
    EXPECT_EQ(frontier.source_uav_id, finder.sleeping_frontiers_.front().source_uav_id);
    EXPECT_EQ(frontier.source_revision, finder.sleeping_frontiers_.front().source_revision);

    EXPECT_FALSE(finder.reactivateSleepingFrontier(frontier.task_id, 4));
    EXPECT_TRUE(finder.reactivateSleepingFrontier(frontier.task_id, 3));
    ASSERT_EQ(1u, finder.frontiers_.size());
    EXPECT_TRUE(finder.sleeping_frontiers_.empty());
    EXPECT_EQ(frontier.task_id, finder.frontiers_.front().task_id);
    EXPECT_EQ(frontier.source_uav_id, finder.frontiers_.front().source_uav_id);
}
