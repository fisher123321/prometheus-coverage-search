#ifndef TWO_UAV_AUCTION_PROTOCOL_H
#define TWO_UAV_AUCTION_PROTOCOL_H

#include <algorithm>
#include <cstdint>
#include <map>
#include <vector>

#include <prometheus_two_uav_coverage_search/SwarmAuctionAssignment.h>
#include <prometheus_two_uav_coverage_search/SwarmAuctionTaskSet.h>
#include <prometheus_two_uav_coverage_search/SwarmFrontier.h>

namespace two_uav_auction {

inline uint64_t mix(uint64_t hash, uint64_t value) {
  // FNV-1a is sufficient here: this is a fast mismatch detector, not a
  // security primitive or a persistent identity.
  hash ^= value;
  return hash * 1099511628211ULL;
}

inline bool taskLess(const prometheus_two_uav_coverage_search::SwarmFrontier& lhs,
                     const prometheus_two_uav_coverage_search::SwarmFrontier& rhs) {
  if (lhs.task_id != rhs.task_id) return lhs.task_id < rhs.task_id;
  if (lhs.frontier_source_uav_id != rhs.frontier_source_uav_id)
    return lhs.frontier_source_uav_id < rhs.frontier_source_uav_id;
  return lhs.cluster_version < rhs.cluster_version;
}

inline bool sameTaskKey(const prometheus_two_uav_coverage_search::SwarmFrontier& lhs,
                        const prometheus_two_uav_coverage_search::SwarmFrontier& rhs) {
  return lhs.task_id == rhs.task_id &&
         lhs.frontier_source_uav_id == rhs.frontier_source_uav_id;
}

// A distributed round owns an immutable task snapshot.  Later changes to a
// different frontier must not restart that round. The UID is the immutable
// entity identity; a refreshed cluster revision does not alter the frozen
// descriptor used by this round. Only UID retirement/disappearance does.
inline bool snapshotStillOffered(
    const std::vector<prometheus_two_uav_coverage_search::SwarmFrontier>& snapshot,
    const std::vector<prometheus_two_uav_coverage_search::SwarmFrontier>& offered) {
  for (const auto& task : snapshot) {
    if (std::none_of(offered.begin(), offered.end(),
                     [&](const prometheus_two_uav_coverage_search::SwarmFrontier& current) {
                       return sameTaskKey(task, current);
                     })) {
      return false;
    }
  }
  return true;
}

inline uint64_t taskSetHash(
    const std::vector<prometheus_two_uav_coverage_search::SwarmFrontier>& frontiers) {
  uint64_t hash = 1469598103934665603ULL;
  hash = mix(hash, frontiers.size());
  for (const auto& frontier : frontiers) {
    hash = mix(hash, frontier.task_id);
    hash = mix(hash, frontier.frontier_source_uav_id);
    hash = mix(hash, frontier.cluster_version);
    hash = mix(hash, frontier.frontier_cell_count);
    // Allocation exchanges one deterministic representative viewpoint only.
    // The remaining viewpoints belong to the full post-assignment transfer.
    if (!frontier.candidate_viewpoints.empty()) {
      const auto& view = frontier.candidate_viewpoints.front();
      const auto& point = view.position;
      const auto& orientation = view.orientation;
      hash = mix(hash, static_cast<uint64_t>(static_cast<int64_t>(point.x * 1000.0)));
      hash = mix(hash, static_cast<uint64_t>(static_cast<int64_t>(point.y * 1000.0)));
      hash = mix(hash, static_cast<uint64_t>(static_cast<int64_t>(point.z * 1000.0)));
      hash = mix(hash, static_cast<uint64_t>(static_cast<int64_t>(orientation.x * 1000000.0)));
      hash = mix(hash, static_cast<uint64_t>(static_cast<int64_t>(orientation.y * 1000000.0)));
      hash = mix(hash, static_cast<uint64_t>(static_cast<int64_t>(orientation.z * 1000000.0)));
      hash = mix(hash, static_cast<uint64_t>(static_cast<int64_t>(orientation.w * 1000000.0)));
    }
  }
  return hash;
}

// Both offers are frozen while this commutative intersection is built. A
// one-sided new task waits for the next round instead of cancelling the tasks
// already confirmed by both UAVs. The discovering UAV's descriptor is
// authoritative; transferred tasks fall back to the lower coordinator ID.
inline prometheus_two_uav_coverage_search::SwarmAuctionTaskSet commonTaskSet(
    const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& first,
    const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& second) {
  prometheus_two_uav_coverage_search::SwarmAuctionTaskSet out;
  out.map_epoch = first.map_epoch;
  out.uav1_state_sequence = std::max(first.uav1_state_sequence, second.uav1_state_sequence);
  out.uav2_state_sequence = std::max(first.uav2_state_sequence, second.uav2_state_sequence);

  std::map<uint64_t, const prometheus_two_uav_coverage_search::SwarmFrontier*> peer;
  for (const auto& frontier : second.frontiers) peer[frontier.task_id] = &frontier;
  for (const auto& local : first.frontiers) {
    const auto found = peer.find(local.task_id);
    if (found == peer.end() || !sameTaskKey(local, *found->second)) continue;
    const auto& remote = *found->second;
    if (local.frontier_source_uav_id == first.source_uav_id) out.frontiers.push_back(local);
    else if (remote.frontier_source_uav_id == second.source_uav_id) out.frontiers.push_back(remote);
    else out.frontiers.push_back(first.source_uav_id < second.source_uav_id ? local : remote);
  }
  std::sort(out.frontiers.begin(), out.frontiers.end(), taskLess);
  out.task_set_hash = taskSetHash(out.frontiers);
  return out;
}

inline bool assignmentShapeValid(
    const prometheus_two_uav_coverage_search::SwarmAuctionAssignment& assignment) {
  const size_t count = assignment.task_ids.size();
  return assignment.frontier_source_uav_ids.size() == count &&
         assignment.cluster_versions.size() == count &&
         assignment.owner_uav_ids.size() == count &&
         assignment.viewpoint_indices.size() == count &&
         assignment.assignment_modes.size() == count;
}

inline uint64_t assignmentHash(
    const prometheus_two_uav_coverage_search::SwarmAuctionAssignment& assignment) {
  if (!assignmentShapeValid(assignment)) return 0;
  uint64_t hash = 1469598103934665603ULL;
  hash = mix(hash, assignment.task_ids.size());
  hash = mix(hash, assignment.uav1_state_sequence);
  hash = mix(hash, assignment.uav2_state_sequence);
  for (size_t i = 0; i < assignment.task_ids.size(); ++i) {
    hash = mix(hash, assignment.task_ids[i]);
    hash = mix(hash, assignment.frontier_source_uav_ids[i]);
    hash = mix(hash, assignment.cluster_versions[i]);
    hash = mix(hash, assignment.owner_uav_ids[i]);
    hash = mix(hash, assignment.viewpoint_indices[i]);
    hash = mix(hash, assignment.assignment_modes[i]);
  }
  return hash;
}

inline uint32_t resolveOwner(uint32_t local_owner, uint32_t peer_owner,
                             uint32_t lower_uav_id) {
  if (local_owner == peer_owner) return local_owner;
  if (local_owner == 0) return peer_owner;
  if (peer_owner == 0) return local_owner;
  return lower_uav_id;
}

}  // namespace two_uav_auction

#endif
