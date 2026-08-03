#include <gtest/gtest.h>

#include "two_uav_atsp.h"

TEST(OpenAtsp, FindsDirectedMinimumFromCurrentState) {
    const std::vector<std::vector<double>> cost = {
        {0.0, 1.0, 9.0, 9.0},
        {0.0, 0.0, 1.0, 6.0},
        {0.0, 8.0, 0.0, 1.0},
        {0.0, 1.0, 8.0, 0.0},
    };
    std::vector<int> order;
    ASSERT_TRUE(solveOpenAtsp(cost, order));
    EXPECT_EQ((std::vector<int>{0, 1, 2}), order);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
