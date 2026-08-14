#ifndef TWO_UAV_AUCTION_PROTOCOL_H
#define TWO_UAV_AUCTION_PROTOCOL_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

#include <prometheus_two_uav_coverage_search/SwarmAuctionAssignment.h>
#include <prometheus_two_uav_coverage_search/SwarmAuctionTaskSet.h>
#include <prometheus_two_uav_coverage_search/SwarmFrontier.h>
#include <prometheus_two_uav_coverage_search/SwarmTask.h>

namespace two_uav_auction {

inline uint64_t mix(uint64_t hash, uint64_t value) {
  // FNV-1a is sufficient here: this is a fast mismatch detector, not a
  // security primitive or a persistent identity.
  hash ^= value;
  return hash * 1099511628211ULL;
}

inline double unreachableRetryDelay(double initial, double maximum,
                                    uint32_t consecutive_failures) {
  const uint32_t exponent = consecutive_failures > 0
      ? std::min<uint32_t>(consecutive_failures - 1, 20) : 0;
  return std::min(maximum, std::ldexp(initial, exponent));
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

inline double poseYaw(const geometry_msgs::Pose& pose) {
  return 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);
}

inline double angleDistance(double lhs, double rhs) {
  return std::fabs(std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs)));
}

inline bool samePhysicalTaskGeometry(
    uint64_t lhs_id, const geometry_msgs::Point& lhs_centroid,
    const geometry_msgs::Point& lhs_box_min, const geometry_msgs::Point& lhs_box_max,
    const std::vector<geometry_msgs::Pose>& lhs_views,
    uint64_t rhs_id, const geometry_msgs::Point& rhs_centroid,
    const geometry_msgs::Point& rhs_box_min, const geometry_msgs::Point& rhs_box_max,
    const std::vector<geometry_msgs::Pose>& rhs_views) {
  if (lhs_id == rhs_id) return true;
  if (lhs_views.empty() || rhs_views.empty()) return false;
  const auto& lhs_view = lhs_views.front();
  const auto& rhs_view = rhs_views.front();
  const double view_distance = std::sqrt(
      std::pow(lhs_view.position.x - rhs_view.position.x, 2) +
      std::pow(lhs_view.position.y - rhs_view.position.y, 2) +
      std::pow(lhs_view.position.z - rhs_view.position.z, 2));
  const double centroid_distance = std::sqrt(
      std::pow(lhs_centroid.x - rhs_centroid.x, 2) +
      std::pow(lhs_centroid.y - rhs_centroid.y, 2) +
      std::pow(lhs_centroid.z - rhs_centroid.z, 2));
  const double gap_x = std::max(0.0, std::max(lhs_box_min.x, rhs_box_min.x) -
                                      std::min(lhs_box_max.x, rhs_box_max.x));
  const double gap_y = std::max(0.0, std::max(lhs_box_min.y, rhs_box_min.y) -
                                      std::min(lhs_box_max.y, rhs_box_max.y));
  return view_distance <= 0.30 && centroid_distance <= 0.80 &&
      std::hypot(gap_x, gap_y) <= 0.30 &&
      angleDistance(poseYaw(lhs_view), poseYaw(rhs_view)) <= 0.35;
}

inline bool samePhysicalTask(
    const prometheus_two_uav_coverage_search::SwarmFrontier& lhs,
    const prometheus_two_uav_coverage_search::SwarmFrontier& rhs) {
  return samePhysicalTaskGeometry(
      lhs.task_id, lhs.centroid, lhs.box_min, lhs.box_max, lhs.candidate_viewpoints,
      rhs.task_id, rhs.centroid, rhs.box_min, rhs.box_max, rhs.candidate_viewpoints);
}

inline bool samePhysicalTask(
    const prometheus_two_uav_coverage_search::SwarmFrontier& frontier,
    const prometheus_two_uav_coverage_search::SwarmTask& task) {
  return samePhysicalTaskGeometry(
      frontier.task_id, frontier.centroid, frontier.box_min, frontier.box_max,
      frontier.candidate_viewpoints, task.task_id, task.centroid, task.box_min,
      task.box_max, task.candidate_viewpoints);
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
    hash = mix(hash, frontier.previous_owner_uav_id);
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

inline bool offersMutuallyAcknowledged(
    const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& first,
    const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& second) {
  return first.offer_sequence != 0 && second.offer_sequence != 0 &&
         first.peer_offer_sequence_ack == second.offer_sequence &&
         second.peer_offer_sequence_ack == first.offer_sequence;
}

inline const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& stateOffer(
    const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& first,
    const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& second,
    uint32_t uav_id) {
  const uint64_t first_sequence = uav_id == 1 ? first.uav1_state_sequence
                                               : first.uav2_state_sequence;
  const uint64_t second_sequence = uav_id == 1 ? second.uav1_state_sequence
                                                : second.uav2_state_sequence;
  if (first_sequence != second_sequence)
    return first_sequence > second_sequence ? first : second;
  if (first.source_uav_id == uav_id) return first;
  if (second.source_uav_id == uav_id) return second;
  return first.source_uav_id < second.source_uav_id ? first : second;
}

// Both offers are frozen while this commutative union is built. A task needs
// one advertisement, not two: its discovering UAV naturally has it in its
// local offer while the other UAV may only learn it through this exchange.
// The discovering UAV's descriptor is authoritative when both offers contain
// the task; transferred tasks fall back to the lower coordinator ID.
inline prometheus_two_uav_coverage_search::SwarmAuctionTaskSet confirmedTaskSet(
    const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& first,
    const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& second) {
  prometheus_two_uav_coverage_search::SwarmAuctionTaskSet out;
  out.map_epoch = first.map_epoch;
  out.uav1_state_sequence = std::max(first.uav1_state_sequence, second.uav1_state_sequence);
  out.uav2_state_sequence = std::max(first.uav2_state_sequence, second.uav2_state_sequence);
  const auto& uav1_state = stateOffer(first, second, 1);
  const auto& uav2_state = stateOffer(first, second, 2);
  out.uav1_position = uav1_state.uav1_position;
  out.uav2_position = uav2_state.uav2_position;
  std::map<uint64_t, const prometheus_two_uav_coverage_search::SwarmFrontier*> peer;
  for (const auto& frontier : second.frontiers) peer[frontier.task_id] = &frontier;
  for (const auto& local : first.frontiers) {
    const auto found = peer.find(local.task_id);
    if (found == peer.end()) {
      out.frontiers.push_back(local);
      continue;
    }
    const auto& remote = *found->second;
    if (!sameTaskKey(local, remote)) {
      peer.erase(found);
      continue;
    }
    if (local.frontier_source_uav_id == first.source_uav_id) out.frontiers.push_back(local);
    else if (remote.frontier_source_uav_id == second.source_uav_id) out.frontiers.push_back(remote);
    else out.frontiers.push_back(first.source_uav_id < second.source_uav_id ? local : remote);
    peer.erase(found);
  }
  for (const auto& entry : peer) out.frontiers.push_back(*entry.second);
  std::sort(out.frontiers.begin(), out.frontiers.end(), taskLess);
  std::vector<prometheus_two_uav_coverage_search::SwarmFrontier> unique;
  unique.reserve(out.frontiers.size());
  for (const auto& frontier : out.frontiers) {
    if (std::none_of(unique.begin(), unique.end(), [&](const auto& kept) {
          return samePhysicalTask(frontier, kept);
        })) unique.push_back(frontier);
  }
  out.frontiers.swap(unique);
  out.task_set_hash = taskSetHash(out.frontiers);
  const uint64_t uav1_offer = first.source_uav_id < second.source_uav_id
      ? first.offer_sequence : second.offer_sequence;
  const uint64_t uav2_offer = first.source_uav_id < second.source_uav_id
      ? second.offer_sequence : first.offer_sequence;
  out.task_set_hash = mix(mix(out.task_set_hash, uav1_offer), uav2_offer);
  const auto mix_point = [&](const geometry_msgs::Point& point) {
    out.task_set_hash = mix(out.task_set_hash,
        static_cast<uint64_t>(static_cast<int64_t>(point.x * 1000.0)));
    out.task_set_hash = mix(out.task_set_hash,
        static_cast<uint64_t>(static_cast<int64_t>(point.y * 1000.0)));
    out.task_set_hash = mix(out.task_set_hash,
        static_cast<uint64_t>(static_cast<int64_t>(point.z * 1000.0)));
  };
  mix_point(out.uav1_position);
  mix_point(out.uav2_position);
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

inline uint32_t applyOwnerHysteresis(uint32_t proposed_owner,
                                     uint32_t previous_owner,
                                     bool uav1_reachable, double uav1_cost,
                                     bool uav2_reachable, double uav2_cost,
                                     double min_distance_improvement,
                                     double min_relative_improvement) {
  if (proposed_owner == 0 || previous_owner < 1 || previous_owner > 2 ||
      proposed_owner == previous_owner) return proposed_owner;
  const bool previous_reachable = previous_owner == 1 ? uav1_reachable : uav2_reachable;
  const bool proposed_reachable = proposed_owner == 1 ? uav1_reachable : uav2_reachable;
  if (!previous_reachable || !proposed_reachable) return proposed_owner;
  const double previous_cost = previous_owner == 1 ? uav1_cost : uav2_cost;
  const double proposed_cost = proposed_owner == 1 ? uav1_cost : uav2_cost;
  if (!std::isfinite(previous_cost) || !std::isfinite(proposed_cost)) return proposed_owner;
  const double improvement = previous_cost - proposed_cost;
  const double relative = previous_cost > 1e-6 ? improvement / previous_cost : 0.0;
  return improvement + 1e-9 >= min_distance_improvement &&
         relative + 1e-9 >= min_relative_improvement
      ? proposed_owner : previous_owner;
}

// A hard endpoint rejection is not a normal bid conflict: assigning that
// UAV anyway would create a lease that its own planner must immediately
// reject.  A one-sided non-zero decision is therefore accepted only when the
// proposed owner is the evaluator that made it.  Ordinary non-zero conflicts
// still follow the deterministic lower-ID rule.
inline uint32_t resolveConsensusOwner(uint32_t first_owner,
                                      uint32_t first_evaluator_id,
                                      uint32_t second_owner,
                                      uint32_t second_evaluator_id,
                                      uint32_t lower_uav_id,
                                      uint32_t previous_owner = 0) {
  if (first_owner == second_owner) return first_owner;
  if (first_owner == 0)
    return second_owner == second_evaluator_id ? second_owner : 0;
  if (second_owner == 0)
    return first_owner == first_evaluator_id ? first_owner : 0;
  if (previous_owner == first_owner || previous_owner == second_owner)
    return previous_owner;
  return lower_uav_id;
}

}  // namespace two_uav_auction

#endif
