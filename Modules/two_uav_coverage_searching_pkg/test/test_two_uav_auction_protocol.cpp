#include <gtest/gtest.h>

#include "two_uav_auction_protocol.h"

namespace {

prometheus_two_uav_coverage_search::SwarmFrontier frontier(uint64_t id,
                                                            uint32_t source,
                                                            uint32_t version,
                                                            double x) {
  prometheus_two_uav_coverage_search::SwarmFrontier out;
  out.task_id = id;
  out.frontier_source_uav_id = source;
  out.cluster_version = version;
  out.frontier_cell_count = 12;
  geometry_msgs::Pose view;
  view.position.x = x;
  view.position.y = 2.0;
  view.position.z = 1.5;
  view.orientation.w = 1.0;
  out.candidate_viewpoints.push_back(view);
  return out;
}

prometheus_two_uav_coverage_search::SwarmAuctionAssignment assignment(
    const std::vector<prometheus_two_uav_coverage_search::SwarmFrontier>& tasks,
    const std::vector<uint32_t>& owners) {
  prometheus_two_uav_coverage_search::SwarmAuctionAssignment out;
  for (size_t i = 0; i < tasks.size(); ++i) {
    out.task_ids.push_back(tasks[i].task_id);
    out.frontier_source_uav_ids.push_back(tasks[i].frontier_source_uav_id);
    out.cluster_versions.push_back(tasks[i].cluster_version);
    out.owner_uav_ids.push_back(owners[i]);
    out.viewpoint_indices.push_back(0);
    out.assignment_modes.push_back(
        owners[i] == 0
            ? prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_UNREACHABLE
            : prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_ASTAR);
  }
  out.assignment_hash = two_uav_auction::assignmentHash(out);
  return out;
}

}  // namespace

TEST(TwoUavAuctionProtocol, TaskSetDigestIsOrderIndependentAfterCanonicalSort) {
  std::vector<prometheus_two_uav_coverage_search::SwarmFrontier> first{
      frontier(22, 2, 4, 3.0), frontier(11, 1, 7, 1.0)};
  std::vector<prometheus_two_uav_coverage_search::SwarmFrontier> second = first;
  std::sort(first.begin(), first.end(), two_uav_auction::taskLess);
  std::reverse(second.begin(), second.end());
  std::sort(second.begin(), second.end(), two_uav_auction::taskLess);
  EXPECT_EQ(two_uav_auction::taskSetHash(first), two_uav_auction::taskSetHash(second));
}

TEST(TwoUavAuctionProtocol, TaskSetDigestUsesOnlyRepresentativeViewpoint) {
  auto first = frontier(11, 1, 7, 1.0);
  first.candidate_viewpoints.push_back(first.candidate_viewpoints.front());
  first.candidate_viewpoints.back().position.x = 4.0;
  auto second = first;
  second.candidate_viewpoints.back().position.x = 5.0;
  EXPECT_EQ(two_uav_auction::taskSetHash({first}),
            two_uav_auction::taskSetHash({second}));
  second.candidate_viewpoints.front().position.x = 2.0;
  EXPECT_NE(two_uav_auction::taskSetHash({first}),
            two_uav_auction::taskSetHash({second}));
}

TEST(TwoUavAuctionProtocol, RequiresMutualAcknowledgementOfTheSameOfferPair) {
  prometheus_two_uav_coverage_search::SwarmAuctionTaskSet uav1;
  prometheus_two_uav_coverage_search::SwarmAuctionTaskSet uav2;
  uav1.offer_sequence = 7;
  uav2.offer_sequence = 9;
  EXPECT_FALSE(two_uav_auction::offersMutuallyAcknowledged(uav1, uav2));
  uav1.peer_offer_sequence_ack = 9;
  EXPECT_FALSE(two_uav_auction::offersMutuallyAcknowledged(uav1, uav2));
  uav2.peer_offer_sequence_ack = 7;
  EXPECT_TRUE(two_uav_auction::offersMutuallyAcknowledged(uav1, uav2));
  uav2.offer_sequence = 10;
  EXPECT_FALSE(two_uav_auction::offersMutuallyAcknowledged(uav1, uav2));
}

TEST(TwoUavAuctionProtocol, UnreachableRetryBacksOffAndCaps) {
  EXPECT_DOUBLE_EQ(1.0, two_uav_auction::unreachableRetryDelay(1.0, 8.0, 1));
  EXPECT_DOUBLE_EQ(2.0, two_uav_auction::unreachableRetryDelay(1.0, 8.0, 2));
  EXPECT_DOUBLE_EQ(4.0, two_uav_auction::unreachableRetryDelay(1.0, 8.0, 3));
  EXPECT_DOUBLE_EQ(8.0, two_uav_auction::unreachableRetryDelay(1.0, 8.0, 8));
}

TEST(TwoUavAuctionProtocol, AssignmentDigestDetectsOwnerConflict) {
  std::vector<prometheus_two_uav_coverage_search::SwarmFrontier> tasks{
      frontier(11, 1, 7, 1.0), frontier(22, 2, 4, 3.0)};
  const auto first = assignment(tasks, {1, 2});
  const auto second = assignment(tasks, {2, 2});
  EXPECT_TRUE(two_uav_auction::assignmentShapeValid(first));
  EXPECT_NE(first.assignment_hash, second.assignment_hash);
}

TEST(TwoUavAuctionProtocol, ResolvesEveryOwnerConflictToLowerId) {
  EXPECT_EQ(1u, two_uav_auction::resolveOwner(1, 2, 1));
  EXPECT_EQ(2u, two_uav_auction::resolveOwner(2, 2, 1));
  EXPECT_EQ(2u, two_uav_auction::resolveOwner(0, 2, 1));
  EXPECT_EQ(0u, two_uav_auction::resolveOwner(0, 0, 1));
}

TEST(TwoUavAuctionProtocol, RejectsOwnerNotConfirmedByItsOwnEvaluator) {
  // This is the previous peer-final deadlock: UAV1's map selected UAV2,
  // while UAV2 rejected the endpoint for both vehicles.  The final result
  // must be a valid explicit UNREACHABLE, not owner=2 with that mode.
  EXPECT_EQ(0u, two_uav_auction::resolveConsensusOwner(2, 1, 0, 2, 1));
  EXPECT_EQ(0u, two_uav_auction::resolveConsensusOwner(0, 1, 1, 2, 1));
  EXPECT_EQ(1u, two_uav_auction::resolveConsensusOwner(1, 1, 0, 2, 1));
  EXPECT_EQ(2u, two_uav_auction::resolveConsensusOwner(2, 1, 2, 2, 1));
  EXPECT_EQ(1u, two_uav_auction::resolveConsensusOwner(2, 1, 1, 2, 1));
}

TEST(TwoUavAuctionProtocol, ConfirmedTaskSetIncludesOneSidedUidsAndSourceDescriptor) {
  prometheus_two_uav_coverage_search::SwarmAuctionTaskSet uav1;
  uav1.source_uav_id = 1;
  uav1.map_epoch = 1;
  uav1.offer_sequence = 4;
  uav1.frontiers = {frontier(11, 1, 7, 1.0), frontier(33, 1, 1, 8.0)};
  prometheus_two_uav_coverage_search::SwarmAuctionTaskSet uav2;
  uav2.source_uav_id = 2;
  uav2.map_epoch = 1;
  uav2.offer_sequence = 6;
  uav2.frontiers = {frontier(11, 1, 9, 9.0), frontier(22, 2, 4, 3.0)};

  const auto first = two_uav_auction::confirmedTaskSet(uav1, uav2);
  const auto second = two_uav_auction::confirmedTaskSet(uav2, uav1);
  ASSERT_EQ(3u, first.frontiers.size());
  ASSERT_EQ(3u, second.frontiers.size());
  EXPECT_EQ(11u, first.frontiers.front().task_id);
  EXPECT_EQ(7u, first.frontiers.front().cluster_version);
  EXPECT_EQ(first.task_set_hash, second.task_set_hash);
  uav2.offer_sequence = 7;
  EXPECT_NE(first.task_set_hash,
            two_uav_auction::confirmedTaskSet(uav1, uav2).task_set_hash);
}

TEST(TwoUavAuctionProtocol, ConfirmedTaskSetFreezesMatchingNewestPositions) {
  prometheus_two_uav_coverage_search::SwarmAuctionTaskSet uav1;
  uav1.source_uav_id = 1;
  uav1.offer_sequence = 4;
  uav1.uav1_state_sequence = 10;
  uav1.uav2_state_sequence = 7;
  uav1.uav1_position.x = 1.0;
  uav1.uav2_position.x = 20.0;
  prometheus_two_uav_coverage_search::SwarmAuctionTaskSet uav2;
  uav2.source_uav_id = 2;
  uav2.offer_sequence = 6;
  uav2.uav1_state_sequence = 9;
  uav2.uav2_state_sequence = 12;
  uav2.uav1_position.x = 2.0;
  uav2.uav2_position.x = 21.0;

  const auto first = two_uav_auction::confirmedTaskSet(uav1, uav2);
  const auto second = two_uav_auction::confirmedTaskSet(uav2, uav1);
  EXPECT_EQ(10u, first.uav1_state_sequence);
  EXPECT_EQ(12u, first.uav2_state_sequence);
  EXPECT_DOUBLE_EQ(1.0, first.uav1_position.x);
  EXPECT_DOUBLE_EQ(21.0, first.uav2_position.x);
  EXPECT_EQ(first.task_set_hash, second.task_set_hash);
  uav1.uav1_position.x = 1.5;
  EXPECT_NE(first.task_set_hash,
            two_uav_auction::confirmedTaskSet(uav1, uav2).task_set_hash);
}

TEST(TwoUavAuctionProtocol, SnapshotSurvivesUnrelatedChangesAndOwnRevisionRefresh) {
  const auto stable = frontier(11, 1, 7, 1.0);
  const auto changed = frontier(22, 2, 4, 3.0);
  const std::vector<prometheus_two_uav_coverage_search::SwarmFrontier> snapshot{stable};

  EXPECT_TRUE(two_uav_auction::snapshotStillOffered(
      snapshot, {stable, frontier(22, 2, 5, 3.5)}));
  EXPECT_TRUE(two_uav_auction::snapshotStillOffered(
      snapshot, {frontier(11, 1, 8, 1.0), changed}));
  EXPECT_FALSE(two_uav_auction::snapshotStillOffered(
      snapshot, {changed}));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
