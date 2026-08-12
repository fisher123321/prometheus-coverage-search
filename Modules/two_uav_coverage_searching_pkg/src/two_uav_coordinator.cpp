#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVControlState.h>
#include <prometheus_msgs/UAVState.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include <prometheus_two_uav_coverage_search/SwarmFrontierArray.h>
#include <prometheus_two_uav_coverage_search/SwarmAuctionManifest.h>
#include <prometheus_two_uav_coverage_search/SwarmAuctionTaskSet.h>
#include <prometheus_two_uav_coverage_search/SwarmAuctionAssignment.h>
#include <prometheus_two_uav_coverage_search/SwarmState.h>
#include <prometheus_two_uav_coverage_search/SwarmTaskArray.h>
#include <prometheus_two_uav_coverage_search/SwarmTrajectory.h>

#include "two_uav_auction_protocol.h"

namespace {
enum Phase : uint8_t { WAIT_PEER = 0, WAIT_TAKEOFF = 1, ACTIVE = 2, HOLD = 3 };

double distance2d(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool sameViewpoints(const std::vector<geometry_msgs::Pose>& lhs,
                    const std::vector<geometry_msgs::Pose>& rhs) {
  if (lhs.size() != rhs.size()) return false;
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (distance2d(lhs[i].position, rhs[i].position) > 1e-3 ||
        std::fabs(lhs[i].position.z - rhs[i].position.z) > 1e-3 ||
        std::fabs(lhs[i].orientation.z - rhs[i].orientation.z) > 1e-4 ||
        std::fabs(lhs[i].orientation.w - rhs[i].orientation.w) > 1e-4) return false;
  }
  return true;
}

geometry_msgs::Pose labelPose(const prometheus_two_uav_coverage_search::SwarmTask& task) {
  if (!task.candidate_viewpoints.empty()) return task.candidate_viewpoints.front();
  geometry_msgs::Pose pose;
  pose.position = task.centroid;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose labelPose(const prometheus_two_uav_coverage_search::SwarmFrontier& frontier) {
  if (!frontier.candidate_viewpoints.empty()) return frontier.candidate_viewpoints.front();
  geometry_msgs::Pose pose;
  pose.position = frontier.centroid;
  pose.orientation.w = 1.0;
  return pose;
}

const char* assignmentModeName(uint8_t mode) {
  if (mode == prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_ASTAR)
    return "astar";
  if (mode == prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_EUCLIDEAN_TIMEOUT)
    return "euclidean_timeout";
  return "unreachable";
}

double wrapYaw(double yaw) { return std::atan2(std::sin(yaw), std::cos(yaw)); }

void advanceYawReference(double target, double dt, double max_rate, double max_acc,
                         double* yaw, double* yaw_rate) {
  dt = std::max(0.002, std::min(0.05, dt));
  const double error = wrapYaw(target - *yaw);
  const double braking_rate = std::sqrt(2.0 * std::max(1e-3, max_acc) * std::fabs(error));
  const double target_rate = std::copysign(std::min(max_rate, braking_rate), error);
  const double rate_step = std::max(-max_acc * dt,
                                    std::min(max_acc * dt, target_rate - *yaw_rate));
  *yaw_rate = std::max(-max_rate, std::min(max_rate, *yaw_rate + rate_step));
  const double yaw_step = *yaw_rate * dt;
  if (std::fabs(yaw_step) >= std::fabs(error)) {
    *yaw = wrapYaw(target);
    *yaw_rate = 0.0;
  } else {
    *yaw = wrapYaw(*yaw + yaw_step);
  }
}

bool yawRelaySelfTest() {
  double yaw = 0.0, yaw_rate = 0.0, previous_yaw = yaw;
  for (int i = 0; i < 400; ++i) {
    advanceYawReference(M_PI_2, 0.01, 1.0, 1.0, &yaw, &yaw_rate);
    if (std::fabs(yaw_rate) > 1.001 ||
        std::fabs(wrapYaw(yaw - previous_yaw)) > 0.0101) return false;
    previous_yaw = yaw;
  }
  return std::fabs(wrapYaw(M_PI_2 - yaw)) < 1e-3 && std::fabs(yaw_rate) < 1e-3;
}

}  // namespace

class TwoUavCoordinator {
 public:
  TwoUavCoordinator() = default;

  void init(ros::NodeHandle& nh) {
    nh_ = nh;
    nh_.param("uav_id", uav_id_, 1);
    nh_.param("peer_uav_id", peer_uav_id_, 2);
    nh_.param("map_epoch", map_epoch_, 1);
    nh_.param("fly_height", fly_height_, 1.5);
    nh_.param("min_start_separation", min_start_separation_, 1.0);
    nh_.param("safe_separation", safe_separation_, 1.2);
    nh_.param("peer_timeout", peer_timeout_, 0.6);
    nh_.param("trajectory_timeout", trajectory_timeout_, 0.6);
    nh_.param("max_vel", max_vel_, 1.0);
    nh_.param("yaw_max_rate", yaw_max_rate_, 1.3962634);
    nh_.param("yaw_max_acc", yaw_max_acc_, 1.5707963);
    nh_.param("task_bundle_size", task_bundle_size_, 16);
    nh_.param("task_reach_dist", task_reach_dist_, 0.35);
    nh_.param("task_commit_timeout", task_commit_timeout_, 15.0);
    nh_.param("task_retry_cooldown", task_retry_cooldown_, 12.0);
    nh_.param("task_goal_separation", task_goal_separation_, 3.0);
    nh_.param("task_lease_duration", task_lease_duration_, 8.0);
    nh_.param("unreachable_retry_period", unreachable_retry_period_, 1.0);
    nh_.param("unreachable_retry_max_period", unreachable_retry_max_period_, 8.0);
    nh_.param("auction_period", auction_period_, 0.2);
    nh_.param("raw_command_timeout", raw_command_timeout_, 0.6);
    unreachable_retry_period_ = std::max(0.2, unreachable_retry_period_);
    unreachable_retry_max_period_ = std::max(
        unreachable_retry_period_, unreachable_retry_max_period_);
    nh_.param<std::string>("tx_prefix", tx_prefix_, "/two_uav/tx");
    nh_.param<std::string>("rx_prefix", rx_prefix_, "/two_uav/rx");
    const std::string uav = "/uav" + std::to_string(uav_id_);

    state_sub_ = nh_.subscribe(uav + "/prometheus/state", 10, &TwoUavCoordinator::stateCb, this);
    control_sub_ = nh_.subscribe(uav + "/prometheus/control_state", 10,
                                 &TwoUavCoordinator::controlCb, this);
    raw_command_sub_ = nh_.subscribe(uav + "/prometheus/coverage_search/raw_command", 1,
                                      &TwoUavCoordinator::rawCommandCb, this);
    local_frontier_sub_ = nh_.subscribe(uav + "/prometheus/coverage_search/swarm_frontiers", 2,
                                         &TwoUavCoordinator::localFrontierCb, this);
    local_auction_assignment_sub_ = nh_.subscribe(
        uav + "/prometheus/coverage_search/auction_assignment", 2,
        &TwoUavCoordinator::localAuctionAssignmentCb, this);
    local_trajectory_sub_ = nh_.subscribe(uav + "/prometheus/coverage_search/swarm_trajectory", 2,
                                           &TwoUavCoordinator::localTrajectoryCb, this);
    completion_ready_sub_ = nh_.subscribe(uav + "/prometheus/coverage_search/completion_ready", 1,
                                           &TwoUavCoordinator::completionReadyCb, this);
    peer_state_sub_ = nh_.subscribe(rx_prefix_ + "/state", 10, &TwoUavCoordinator::peerStateCb, this);
    peer_frontier_sub_ = nh_.subscribe(rx_prefix_ + "/frontier", 2,
                                        &TwoUavCoordinator::peerFrontierCb, this);
    peer_task_sub_ = nh_.subscribe(rx_prefix_ + "/task", 2, &TwoUavCoordinator::peerTaskCb, this);
    peer_trajectory_sub_ = nh_.subscribe(rx_prefix_ + "/trajectory", 2,
                                          &TwoUavCoordinator::peerTrajectoryCb, this);
    peer_auction_manifest_sub_ = nh_.subscribe(
        rx_prefix_ + "/auction_manifest", 2,
        &TwoUavCoordinator::peerAuctionManifestCb, this);
    peer_auction_task_set_sub_ = nh_.subscribe(
        rx_prefix_ + "/auction_task_set", 2,
        &TwoUavCoordinator::peerAuctionTaskSetCb, this);
    peer_auction_assignment_sub_ = nh_.subscribe(
        rx_prefix_ + "/auction_assignment", 2,
        &TwoUavCoordinator::peerAuctionAssignmentCb, this);

    state_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmState>(tx_prefix_ + "/state", 10);
    frontier_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmFrontierArray>(tx_prefix_ + "/frontier", 2);
    task_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmTaskArray>(tx_prefix_ + "/task", 2);
    local_task_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmTaskArray>(
        uav + "/prometheus/coverage_search/local_tasks", 2);
    trajectory_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmTrajectory>(tx_prefix_ + "/trajectory", 2);
    auction_manifest_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmAuctionManifest>(
        tx_prefix_ + "/auction_manifest", 2);
    auction_assignment_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmAuctionAssignment>(
        tx_prefix_ + "/auction_assignment", 2);
    auction_task_set_sync_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmAuctionTaskSet>(
        tx_prefix_ + "/auction_task_set", 2);
    auction_task_set_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmAuctionTaskSet>(
        uav + "/prometheus/coverage_search/auction_task_set", 2);
    peer_collision_replan_pub_ = nh_.advertise<std_msgs::Bool>(
        uav + "/prometheus/coverage_search/peer_collision_replan", 1);
    command_pub_ = nh_.advertise<prometheus_msgs::UAVCommand>(uav + "/prometheus/command", 1);
    task_label_pub_ = nh_.advertise<visualization_msgs::Marker>(
        uav + "/prometheus/coverage_search/task_labels", 10);
    timer_ = nh_.createTimer(ros::Duration(0.05), &TwoUavCoordinator::timerCb, this);
    command_timer_ = nh_.createTimer(ros::Duration(0.01),
                                     &TwoUavCoordinator::commandTimerCb, this);
    phase_ = WAIT_PEER;
    ROS_INFO("[two_uav_coordinator] UAV %d waits for UAV %d", uav_id_, peer_uav_id_);
  }

 private:
  bool ownReady() const {
    return have_state_ && state_.connected && state_.armed && state_.odom_valid &&
           control_.control_state == prometheus_msgs::UAVControlState::COMMAND_CONTROL;
  }

  bool peerFresh() const {
    return have_peer_ && (ros::Time::now() - peer_received_).toSec() <= peer_timeout_;
  }

  bool rawCommandFresh() const {
    return have_raw_command_ && !last_raw_command_received_.isZero() &&
           (ros::Time::now() - last_raw_command_received_).toSec() <= raw_command_timeout_;
  }

  bool stableAtHeight(const prometheus_msgs::UAVState& state) const {
    const double speed = std::hypot(state.velocity[0], state.velocity[1]);
    return altitudeAtHeight(state.position[2]) && speed <= 0.35;
  }

  bool altitudeAtHeight(double altitude) const {
    return std::fabs(altitude - fly_height_) <= 0.30;
  }

  geometry_msgs::Point ownPoint() const {
    geometry_msgs::Point point;
    point.x = state_.position[0]; point.y = state_.position[1]; point.z = state_.position[2];
    return point;
  }

  bool startSeparationOk() const {
    return peerFresh() && distance2d(ownPoint(), peer_.pose.position) >= min_start_separation_;
  }

  void stateCb(const prometheus_msgs::UAVState::ConstPtr& msg) {
    state_ = *msg;
    have_state_ = true;
  }
  void controlCb(const prometheus_msgs::UAVControlState::ConstPtr& msg) { control_ = *msg; }
  void rawCommandCb(const prometheus_msgs::UAVCommand::ConstPtr& msg) {
    raw_command_ = *msg;
    have_raw_command_ = true;
    last_raw_command_received_ = ros::Time::now();
  }

  void commandTimerCb(const ros::TimerEvent& event) {
    if (!ownReady() || phase_ != ACTIVE || !rawCommandFresh()) return;
    std::string safety_reason;
    if (!peerTrajectorySafe(&safety_reason)) {
      ROS_WARN_THROTTLE(1.0, "[two_uav_coordinator] UAV %d blocks command handoff: %s",
                        uav_id_, safety_reason.c_str());
      phase_ = HOLD;
      publishHold();
      return;
    }
    if (!relay_yaw_inited_) {
      relay_yaw_ = state_.attitude[2];
      relay_yaw_rate_ = 0.0;
      relay_yaw_inited_ = true;
    }
    const double dt = event.current_real.isZero() || event.last_real.isZero()
        ? 0.01 : (event.current_real - event.last_real).toSec();
    advanceYawReference(raw_command_.yaw_ref, dt, yaw_max_rate_, yaw_max_acc_,
                        &relay_yaw_, &relay_yaw_rate_);
    prometheus_msgs::UAVCommand command = raw_command_;
    command.header.stamp = ros::Time::now();
    command.yaw_ref = relay_yaw_;
    command.yaw_rate_ref = relay_yaw_rate_;
    command.Command_ID = ++command_id_;
    command_pub_.publish(command);
  }
  void localFrontierCb(const prometheus_two_uav_coverage_search::SwarmFrontierArray::ConstPtr& msg) {
    if (static_cast<int>(msg->source_uav_id) != uav_id_ ||
        msg->map_epoch != static_cast<uint32_t>(map_epoch_)) return;
    local_frontiers_ = *msg;
    local_frontier_received_ = ros::Time::now();
    claimLocalFrontierReservations();
  }
  void localTrajectoryCb(const prometheus_two_uav_coverage_search::SwarmTrajectory::ConstPtr& msg) {
    local_trajectory_ = *msg;
    local_trajectory_received_ = ros::Time::now();
  }
  void completionReadyCb(const std_msgs::Bool::ConstPtr& msg) { completion_ready_ = msg->data; }
  void peerStateCb(const prometheus_two_uav_coverage_search::SwarmState::ConstPtr& msg) {
    if (static_cast<int>(msg->uav_id) != peer_uav_id_) return;
    peer_ = *msg;
    peer_received_ = ros::Time::now();
    have_peer_ = true;
  }
  void peerFrontierCb(const prometheus_two_uav_coverage_search::SwarmFrontierArray::ConstPtr& msg) {
    if (static_cast<int>(msg->source_uav_id) != peer_uav_id_ ||
        msg->map_epoch != static_cast<uint32_t>(map_epoch_)) return;
    peer_frontiers_ = *msg;
    peer_frontier_received_ = ros::Time::now();
  }
  void peerTaskCb(const prometheus_two_uav_coverage_search::SwarmTaskArray::ConstPtr& msg) {
    if (static_cast<int>(msg->source_uav_id) != peer_uav_id_ ||
        msg->map_epoch != static_cast<uint32_t>(map_epoch_)) return;
    peer_tasks_ = *msg;
    peer_task_received_ = ros::Time::now();
  }
  void peerTrajectoryCb(const prometheus_two_uav_coverage_search::SwarmTrajectory::ConstPtr& msg) {
    if (static_cast<int>(msg->source_uav_id) != peer_uav_id_) return;
    peer_trajectory_ = *msg;
    peer_trajectory_received_ = ros::Time::now();
  }
  void localAuctionAssignmentCb(
      const prometheus_two_uav_coverage_search::SwarmAuctionAssignment::ConstPtr& msg) {
    if (static_cast<int>(msg->source_uav_id) != uav_id_ ||
        msg->map_epoch != static_cast<uint32_t>(map_epoch_)) return;
    local_auction_proposal_ = *msg;
    local_auction_proposal_received_ = ros::Time::now();
  }
  void peerAuctionManifestCb(
      const prometheus_two_uav_coverage_search::SwarmAuctionManifest::ConstPtr& msg) {
    if (static_cast<int>(msg->source_uav_id) != peer_uav_id_ ||
        msg->map_epoch != static_cast<uint32_t>(map_epoch_)) return;
    peer_auction_manifest_ = *msg;
    peer_auction_manifest_received_ = ros::Time::now();
  }
  void peerAuctionTaskSetCb(
      const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet::ConstPtr& msg) {
    if (static_cast<int>(msg->source_uav_id) != peer_uav_id_ ||
        msg->map_epoch != static_cast<uint32_t>(map_epoch_)) return;
    if (msg->task_set_hash != two_uav_auction::taskSetHash(msg->frontiers)) {
      ROS_WARN_THROTTLE(1.0,
          "[AuctionTaskSet] UAV %d rejected peer candidate list with invalid digest.", uav_id_);
      return;
    }
    peer_auction_offer_ = *msg;
    peer_auction_offer_received_ = ros::Time::now();
  }
  void peerAuctionAssignmentCb(
      const prometheus_two_uav_coverage_search::SwarmAuctionAssignment::ConstPtr& msg) {
    if (static_cast<int>(msg->source_uav_id) != peer_uav_id_ ||
        msg->map_epoch != static_cast<uint32_t>(map_epoch_)) return;
    if (msg->final_result) {
      peer_auction_final_ = *msg;
      peer_auction_final_received_ = ros::Time::now();
    } else {
      peer_auction_proposal_ = *msg;
      peer_auction_proposal_received_ = ros::Time::now();
    }
  }

  void publishState() {
    if (!have_state_) return;
    prometheus_two_uav_coverage_search::SwarmState out;
    out.header.stamp = ros::Time::now();
    out.uav_id = uav_id_;
    out.sequence = ++state_sequence_;
    out.ack_sequence = have_peer_ ? peer_.sequence : 0;
    out.odom_valid = state_.odom_valid;
    out.command_ready = ownReady();
    out.completion_ready = completion_ready_;
    out.phase = phase_;
    out.pose.position = ownPoint();
    out.pose.orientation = state_.attitude_q;
    out.velocity.x = state_.velocity[0]; out.velocity.y = state_.velocity[1]; out.velocity.z = state_.velocity[2];
    state_pub_.publish(out);
  }

  bool sampleTrajectory(const prometheus_two_uav_coverage_search::SwarmTrajectory& trajectory,
                        const ros::Time& when, geometry_msgs::Point* point) const {
    if (trajectory.points.empty() || trajectory.dt <= 1e-3 || trajectory.header.stamp.isZero()) {
      return false;
    }
    const double offset = std::max(0.0, (when - trajectory.header.stamp).toSec());
    const double sample = offset / trajectory.dt;
    const int first = static_cast<int>(std::floor(sample));
    if (first >= static_cast<int>(trajectory.points.size())) {
      *point = trajectory.points.back();
      return true;
    }
    const int second = std::min(first + 1, static_cast<int>(trajectory.points.size()) - 1);
    const double alpha = std::min(1.0, std::max(0.0, sample - first));
    const auto& a = trajectory.points[first];
    const auto& b = trajectory.points[second];
    point->x = a.x + alpha * (b.x - a.x);
    point->y = a.y + alpha * (b.y - a.y);
    point->z = a.z + alpha * (b.z - a.z);
    return true;
  }

  bool peerTrajectorySafe(std::string* reason) {
    const ros::Time now = ros::Time::now();
    if (!peerFresh()) return true;
    const bool local_fresh = !local_trajectory_received_.isZero() &&
        (now - local_trajectory_received_).toSec() <= trajectory_timeout_;
    const bool peer_fresh = !peer_trajectory_received_.isZero() &&
        (now - peer_trajectory_received_).toSec() <= trajectory_timeout_;
    const geometry_msgs::Point static_self = ownPoint();
    const geometry_msgs::Point static_peer = peer_.pose.position;

    geometry_msgs::Point self, peer;
    bool high_id_escaping = false;
    double previous_separation = 0.0;
    for (double t = 0.0; t <= 2.0; t += 0.05) {
      const ros::Time when = now + ros::Duration(t);
      self = static_self;
      peer = static_peer;
      if (local_fresh) sampleTrajectory(local_trajectory_, when, &self);
      if (peer_fresh) sampleTrajectory(peer_trajectory_, when, &peer);
      const double separation = std::sqrt(
          (self.x - peer.x) * (self.x - peer.x) +
          (self.y - peer.y) * (self.y - peer.y) +
          (self.z - peer.z) * (self.z - peer.z));
      if (separation >= safe_separation_) {
        high_id_escaping = false;
        continue;
      }

      // Low ID keeps right of way.  Only the high-ID aircraft is blocked and
      // asked to replan, so both coordinators cannot turn one predicted
      // crossing into a mutual hover deadlock.
      if (uav_id_ < peer_uav_id_) continue;

      if (last_collision_replan_request_.isZero() ||
          (now - last_collision_replan_request_).toSec() >= 0.5) {
        std_msgs::Bool request;
        request.data = true;
        peer_collision_replan_pub_.publish(request);
        last_collision_replan_request_ = now;
      }
      // 两机已经进入安全间距内时，高 ID 机不能因 t=0 的既有重叠
      // 永远被拦截；只允许它沿间距单调增加的轨迹退出。
      if (high_id_escaping || t <= 1e-6) {
        if (!high_id_escaping) {
          high_id_escaping = true;
          previous_separation = separation;
          continue;
        }
        if (separation + 1e-3 >= previous_separation) {
          previous_separation = separation;
          continue;
        }
      }
      if (reason) {
        *reason = "predicted peer collision in " + std::to_string(t) +
            "s at " + std::to_string(separation) + "m; " +
            "higher ID replans";
      }
      return false;
    }
    return true;
  }

  void publishHold() {
    if (!have_state_ || !ownReady()) return;
    prometheus_msgs::UAVCommand hold;
    hold.header.stamp = ros::Time::now();
    hold.Agent_CMD = prometheus_msgs::UAVCommand::Move;
    hold.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
    hold.position_ref[0] = state_.position[0];
    hold.position_ref[1] = state_.position[1];
    hold.position_ref[2] = fly_height_;
    hold.yaw_ref = relay_yaw_inited_ ? relay_yaw_ : state_.attitude[2];
    hold.yaw_rate_ref = 0.0;
    hold.Command_ID = ++command_id_;
    command_pub_.publish(hold);
  }

  void publishHoldTrajectory() {
    prometheus_two_uav_coverage_search::SwarmTrajectory hold;
    hold.header.stamp = ros::Time::now();
    hold.source_uav_id = uav_id_;
    hold.sequence = command_id_;
    hold.active_task_id = local_trajectory_.active_task_id;
    hold.active_task_distance = local_trajectory_.active_task_distance;
    hold.dt = 0.05;
    const geometry_msgs::Point point = ownPoint();
    geometry_msgs::Vector3 velocity;
    for (int i = 0; i <= 40; ++i) {
      hold.points.push_back(point);
      hold.velocities.push_back(velocity);
    }
    trajectory_pub_.publish(hold);
  }

  void publishOwnLeases(const ros::Time& now) {
    prometheus_two_uav_coverage_search::SwarmTaskArray local;
    local.header.stamp = now;
    local.source_uav_id = uav_id_;
    local.map_epoch = map_epoch_;
    local.revision = ++task_revision_;
    for (const auto& entry : own_leases_) {
      if (entry.second.lease_expire_time > now) local.tasks.push_back(entry.second);
    }
    local_tasks_ = local;
    auto manager_view = local;
    for (const auto& recent : recently_assigned_frontiers_) {
      const auto owner = last_committed_owners_.find(recent.first);
      const auto& frontier = recent.second.first;
      if (recent.second.second <= now || owner == last_committed_owners_.end() ||
          owner->second == static_cast<uint32_t>(uav_id_) ||
          frontier.frontier_source_uav_id != static_cast<uint32_t>(uav_id_)) continue;
      auto transfer = makeLease(frontier, now, 0);
      transfer.winner_uav_id = owner->second;
      transfer.lease_expire_time = recent.second.second;
      manager_view.tasks.push_back(transfer);
    }
    local_task_pub_.publish(manager_view);
    auto shared = local;
    shared.tasks.erase(std::remove_if(shared.tasks.begin(), shared.tasks.end(),
        [](const prometheus_two_uav_coverage_search::SwarmTask& task) {
          return task.allocation_state ==
              prometheus_two_uav_coverage_search::SwarmTask::ALLOCATION_LOCAL_RESERVATION;
        }), shared.tasks.end());
    local_task_published_ = now;
    task_pub_.publish(shared);
  }

  prometheus_two_uav_coverage_search::SwarmFrontierArray peerVisibleFrontiers(
      const ros::Time& now) const {
    auto shared = local_frontiers_;
    shared.frontiers.erase(std::remove_if(shared.frontiers.begin(), shared.frontiers.end(),
        [&](const prometheus_two_uav_coverage_search::SwarmFrontier& frontier) {
          if (frontier.reservation_owner_uav_id == static_cast<uint32_t>(uav_id_) &&
              frontier.reservation_expire_time > now) return true;
          const auto lease = own_leases_.find(frontier.task_id);
          return lease != own_leases_.end() && lease->second.lease_expire_time > now &&
              lease->second.allocation_state ==
                  prometheus_two_uav_coverage_search::SwarmTask::ALLOCATION_LOCAL_RESERVATION;
        }), shared.frontiers.end());
    return shared;
  }

  void claimLocalFrontierReservations() {
    const ros::Time now = ros::Time::now();
    int claimed = 0;
    int updated = 0;
    for (const auto& frontier : local_frontiers_.frontiers) {
      if (frontier.reservation_owner_uav_id != static_cast<uint32_t>(uav_id_) ||
          frontier.reservation_expire_time <= now) continue;
      const auto existing = own_leases_.find(frontier.task_id);
      if (existing != own_leases_.end() && existing->second.lease_expire_time > now) {
        const auto& previous = existing->second;
        const bool changed = previous.frontier_source_uav_id != frontier.frontier_source_uav_id ||
            previous.cluster_version != frontier.cluster_version ||
            previous.frontier_cell_count != frontier.frontier_cell_count ||
            !sameViewpoints(previous.candidate_viewpoints, frontier.candidate_viewpoints);
        if (!changed) continue;
        auto task = previous;
        task.task_version = std::max(task.task_version, local_frontiers_.revision);
        task.frontier_source_uav_id = frontier.frontier_source_uav_id;
        task.cluster_version = frontier.cluster_version;
        task.frontier_cell_count = frontier.frontier_cell_count;
        task.centroid = frontier.centroid;
        task.box_min = frontier.box_min;
        task.box_max = frontier.box_max;
        task.candidate_viewpoints = frontier.candidate_viewpoints;
        own_leases_[task.task_id] = task;
        ++updated;
        continue;
      }
      if (existing != own_leases_.end()) own_leases_.erase(existing);
      prometheus_two_uav_coverage_search::SwarmTask task;
      task.task_id = frontier.task_id;
      task.frontier_source_uav_id = frontier.frontier_source_uav_id;
      task.task_version = std::max(1u, local_frontiers_.revision);
      task.cluster_version = frontier.cluster_version;
      task.frontier_cell_count = frontier.frontier_cell_count;
      task.winner_uav_id = uav_id_;
      task.allocation_state = prometheus_two_uav_coverage_search::SwarmTask::ALLOCATION_LOCAL_RESERVATION;
      task.cost = 0.0;
      task.centroid = frontier.centroid;
      task.box_min = frontier.box_min;
      task.box_max = frontier.box_max;
      task.candidate_viewpoints = frontier.candidate_viewpoints;
      task.lease_expire_time = now + ros::Duration(task_lease_duration_);
      own_leases_[task.task_id] = task;
      ++claimed;
    }
    if (claimed > 0 || updated > 0) {
      publishOwnLeases(now);
      ROS_INFO("[two_uav_coordinator] UAV %d directly claimed %d and updated %d "
               "local frontier task(s); new leases are %.1fs.", uav_id_, claimed, updated,
               task_lease_duration_);
    }
  }

  prometheus_two_uav_coverage_search::SwarmTask makeLease(
      const prometheus_two_uav_coverage_search::SwarmFrontier& frontier,
      const ros::Time& now, uint32_t viewpoint_index) const {
    prometheus_two_uav_coverage_search::SwarmTask task;
    task.task_id = frontier.task_id;
    task.frontier_source_uav_id = frontier.frontier_source_uav_id;
    task.task_version = std::max(1u, frontier.cluster_version);
    task.cluster_version = frontier.cluster_version;
    task.frontier_cell_count = frontier.frontier_cell_count;
    task.winner_uav_id = uav_id_;
    task.allocation_state = prometheus_two_uav_coverage_search::SwarmTask::ALLOCATION_DISTRIBUTED;
    task.cost = 0.0;
    task.lease_expire_time = now + ros::Duration(task_lease_duration_);
    task.centroid = frontier.centroid;
    task.box_min = frontier.box_min;
    task.box_max = frontier.box_max;
    task.candidate_viewpoints = frontier.candidate_viewpoints;
    if (viewpoint_index < task.candidate_viewpoints.size() && viewpoint_index != 0) {
      std::rotate(task.candidate_viewpoints.begin(),
                  task.candidate_viewpoints.begin() + viewpoint_index,
                  task.candidate_viewpoints.begin() + viewpoint_index + 1);
    }
    return task;
  }

  bool assignmentMatchesTaskSet(
      const prometheus_two_uav_coverage_search::SwarmAuctionAssignment& assignment,
      const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& task_set) const {
    if (assignment.task_set_hash != task_set.task_set_hash ||
        assignment.uav1_state_sequence != task_set.uav1_state_sequence ||
        assignment.uav2_state_sequence != task_set.uav2_state_sequence ||
        !two_uav_auction::assignmentShapeValid(assignment) ||
        assignment.assignment_hash != two_uav_auction::assignmentHash(assignment) ||
        assignment.task_ids.size() != task_set.frontiers.size()) return false;
    for (size_t i = 0; i < task_set.frontiers.size(); ++i) {
      const auto& frontier = task_set.frontiers[i];
      if (assignment.task_ids[i] != frontier.task_id ||
          assignment.frontier_source_uav_ids[i] != frontier.frontier_source_uav_id ||
          assignment.cluster_versions[i] != frontier.cluster_version ||
          assignment.owner_uav_ids[i] > 2 ||
          assignment.viewpoint_indices[i] >= frontier.candidate_viewpoints.size() ||
          assignment.assignment_modes[i] <
              prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_ASTAR ||
          assignment.assignment_modes[i] >
              prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_UNREACHABLE ||
          (assignment.owner_uav_ids[i] == 0) !=
              (assignment.assignment_modes[i] ==
               prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_UNREACHABLE))
        return false;
    }
    return true;
  }

  static std::string taskIds(
      const std::vector<prometheus_two_uav_coverage_search::SwarmFrontier>& tasks) {
    std::ostringstream stream;
    stream << '[';
    for (size_t i = 0; i < tasks.size(); ++i) {
      if (i > 0) stream << ',';
      stream << tasks[i].task_id;
    }
    stream << ']';
    return stream.str();
  }

  bool buildAuctionTaskSet(const ros::Time& now, uint64_t active_task_id,
                           bool active_fresh,
                           prometheus_two_uav_coverage_search::SwarmAuctionTaskSet* out) {
    if (local_frontier_received_.isZero() || peer_frontier_received_.isZero() ||
        (now - local_frontier_received_).toSec() > 1.0 ||
        (now - peer_frontier_received_).toSec() > 1.0 ||
        peer_task_received_.isZero() || (now - peer_task_received_).toSec() > 5.0) return false;
    std::map<uint64_t, prometheus_two_uav_coverage_search::SwarmFrontier> tasks;
    std::map<uint64_t, uint32_t> task_advertisers;
    const auto merge = [&](const prometheus_two_uav_coverage_search::SwarmFrontierArray& source) {
      for (const auto& frontier : source.frontiers) {
        if (frontier.candidate_viewpoints.empty()) continue;
        const auto found = task_advertisers.find(frontier.task_id);
        if (found == task_advertisers.end() || source.source_uav_id < found->second) {
          tasks[frontier.task_id] = frontier;
          task_advertisers[frontier.task_id] = source.source_uav_id;
        }
      }
    };
    merge(local_frontiers_);
    merge(peer_frontiers_);
    for (auto it = unreachable_tasks_.begin(); it != unreachable_tasks_.end();) {
      const auto current = tasks.find(it->first);
      if (current == tasks.end() ||
          current->second.cluster_version != it->second.cluster_version) {
        unreachable_retry_after_.erase(it->first);
        unreachable_retry_counts_.erase(it->first);
        it = unreachable_tasks_.erase(it);
      } else {
        ++it;
      }
    }

    out->header.stamp = now;
    out->source_uav_id = uav_id_;
    out->map_epoch = map_epoch_;
    out->uav1_state_sequence = uav_id_ == 1 ? state_sequence_ : peer_.sequence;
    out->uav2_state_sequence = uav_id_ == 1 ? peer_.sequence : state_sequence_;
    out->uav1_position = uav_id_ == 1 ? ownPoint() : peer_.pose.position;
    out->uav2_position = uav_id_ == 1 ? peer_.pose.position : ownPoint();
    out->frontiers.clear();
    for (auto it = recently_assigned_frontiers_.begin();
         it != recently_assigned_frontiers_.end();) {
      if (it->second.second <= now) it = recently_assigned_frontiers_.erase(it);
      else ++it;
    }
    for (const auto& entry : tasks) {
      const auto& frontier = entry.second;
      if (active_fresh && frontier.task_id == active_task_id) {
        continue;
      }
      const bool equivalent_live_lease = std::any_of(
          own_leases_.begin(), own_leases_.end(), [&](const auto& lease) {
            return lease.second.lease_expire_time > now &&
                two_uav_auction::samePhysicalTask(frontier, lease.second);
          }) || std::any_of(peer_tasks_.tasks.begin(), peer_tasks_.tasks.end(),
              [&](const auto& lease) {
                return lease.lease_expire_time > now &&
                    two_uav_auction::samePhysicalTask(frontier, lease);
              }) || std::any_of(recently_assigned_frontiers_.begin(),
                  recently_assigned_frontiers_.end(), [&](const auto& recent) {
                    return recent.second.second > now &&
                        two_uav_auction::samePhysicalTask(frontier, recent.second.first);
                  });
      if (equivalent_live_lease) {
        continue;
      }
      if (frontier.reservation_owner_uav_id != 0 &&
          frontier.reservation_expire_time > now) {
        continue;
      }
      bool peer_owns = false;
      for (const auto& lease : peer_tasks_.tasks) {
        if (lease.task_id == frontier.task_id && lease.lease_expire_time > now) {
          peer_owns = true;
          break;
        }
      }
      if (peer_owns) {
        continue;
      }
      const auto retry = unreachable_retry_after_.find(frontier.task_id);
      if (retry != unreachable_retry_after_.end() && retry->second > now) {
        continue;
      }
      const auto suppression = recently_assigned_.find(frontier.task_id);
      if (suppression != recently_assigned_.end()) {
        if (suppression->second.first == frontier.cluster_version &&
            suppression->second.second > now) continue;
        recently_assigned_.erase(suppression);
      }
      auto descriptor = frontier;
      descriptor.frontier_cells.clear();
      descriptor.candidate_viewpoints.resize(1);  // best representative only
      const auto previous_owner = last_committed_owners_.find(frontier.task_id);
      descriptor.previous_owner_uav_id = previous_owner == last_committed_owners_.end()
          ? 0 : previous_owner->second;
      out->frontiers.push_back(descriptor);
    }
    std::sort(out->frontiers.begin(), out->frontiers.end(), two_uav_auction::taskLess);
    out->task_set_hash = two_uav_auction::taskSetHash(out->frontiers);
    return true;
  }

  void clearAuctionOffers() {
    local_auction_offer_ = prometheus_two_uav_coverage_search::SwarmAuctionTaskSet();
    peer_auction_offer_ = prometheus_two_uav_coverage_search::SwarmAuctionTaskSet();
    local_auction_offer_created_ = ros::Time(0);
    peer_auction_offer_received_ = ros::Time(0);
    last_offer_pair_log_hash_ = 0;
  }

  void logAuctionOfferDifference(
      const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& confirmed) {
    const uint64_t low = std::min(local_auction_offer_.task_set_hash,
                                  peer_auction_offer_.task_set_hash);
    const uint64_t high = std::max(local_auction_offer_.task_set_hash,
                                   peer_auction_offer_.task_set_hash);
    const uint64_t pair_hash = two_uav_auction::mix(two_uav_auction::mix(
        1469598103934665603ULL, low), high);
    if (pair_hash == last_offer_pair_log_hash_) return;
    last_offer_pair_log_hash_ = pair_hash;
    const auto only_in = [](const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& lhs,
                            const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& rhs) {
      std::vector<prometheus_two_uav_coverage_search::SwarmFrontier> difference;
      for (const auto& task : lhs.frontiers) {
        const bool found = std::any_of(rhs.frontiers.begin(), rhs.frontiers.end(),
            [&](const prometheus_two_uav_coverage_search::SwarmFrontier& peer) {
              return two_uav_auction::sameTaskKey(task, peer);
            });
        if (!found) difference.push_back(task);
      }
      return difference;
    };
    const auto local_only = only_in(local_auction_offer_, peer_auction_offer_);
    const auto peer_only = only_in(peer_auction_offer_, local_auction_offer_);
    if (local_only.empty() && peer_only.empty()) return;
    ROS_INFO("[AuctionTaskSet] UAV %d local=%zu peer=%zu agreed=%zu "
             "local_only=%s peer_only=%s agreed_uids=%s.",
             uav_id_, local_auction_offer_.frontiers.size(),
             peer_auction_offer_.frontiers.size(), confirmed.frontiers.size(),
             taskIds(local_only).c_str(), taskIds(peer_only).c_str(),
             taskIds(confirmed.frontiers).c_str());
  }

  bool auctionInputsFresh(const ros::Time& now) const {
    return !local_frontier_received_.isZero() && !peer_frontier_received_.isZero() &&
        !peer_task_received_.isZero() &&
        (now - local_frontier_received_).toSec() <= 1.0 &&
        (now - peer_frontier_received_).toSec() <= 1.0 &&
        (now - peer_task_received_).toSec() <= 5.0;
  }

  bool auctionSnapshotStillEligible(const ros::Time& now) const {
    if (!auctionInputsFresh(now)) return true;

    std::vector<prometheus_two_uav_coverage_search::SwarmFrontier> offered;
    offered.reserve(local_frontiers_.frontiers.size() + peer_frontiers_.frontiers.size());
    const auto append_present = [&](const prometheus_two_uav_coverage_search::SwarmFrontierArray& source) {
      for (const auto& frontier : source.frontiers) {
        if (!frontier.candidate_viewpoints.empty()) offered.push_back(frontier);
      }
    };
    // Once frozen, a peer committing this same round makes the task leased;
    // that is progress, not invalidation. Only source retirement/removal may
    // cancel the snapshot before the slower peer commits its part.
    append_present(local_frontiers_);
    append_present(peer_frontiers_);
    return two_uav_auction::snapshotStillOffered(current_auction_task_set_.frontiers, offered);
  }

  enum AuctionTimingStage : uint32_t {
    TIMING_FROZEN = 1u << 0,
    TIMING_TASK_SET = 1u << 1,
    TIMING_LOCAL_PROPOSAL = 1u << 2,
    TIMING_PEER_PROPOSAL = 1u << 3,
    TIMING_LOCAL_FINAL = 1u << 4,
    TIMING_PEER_FINAL = 1u << 5,
    TIMING_COMMITTED = 1u << 6,
  };

  void logAuctionStage(uint32_t stage, const ros::Time& stamp) {
    if ((auction_timing_stages_ & stage) != 0 || auction_round_started_.isZero()) return;
    const ros::Time event_time = stamp.isZero() ? ros::Time::now() : stamp;
    const ros::Time previous = auction_last_stage_.isZero() ? auction_round_started_ : auction_last_stage_;
    size_t stage_index = 0;
    while ((stage >> stage_index) != 1u) ++stage_index;
    auction_stage_elapsed_s_[stage_index] =
        std::max(0.0, (event_time - previous).toSec());
    auction_timing_stages_ |= stage;
    if (event_time > auction_last_stage_) auction_last_stage_ = event_time;
    if (stage == TIMING_COMMITTED) {
      ROS_INFO("[AuctionTiming] UAV %d round=%llu tasks=%zu completed "
               "frozen=%.3fs consensus=%.3fs local=%.3fs peer=%.3fs "
               "final=%.3fs peer_final=%.3fs commit=%.3fs total=%.3fs.",
               uav_id_, static_cast<unsigned long long>(current_auction_task_set_hash_),
               current_auction_task_set_.frontiers.size(), auction_stage_elapsed_s_[0],
               auction_stage_elapsed_s_[1], auction_stage_elapsed_s_[2],
               auction_stage_elapsed_s_[3], auction_stage_elapsed_s_[4],
               auction_stage_elapsed_s_[5], auction_stage_elapsed_s_[6],
               std::max(0.0, (event_time - auction_round_started_).toSec()));
    }
  }

  void logAuctionWait(const char* waiting_for, const ros::Time& now) const {
    if ((now - auction_round_started_).toSec() < 2.0) return;
    ROS_WARN_THROTTLE(2.0,
        "[AuctionTiming] UAV %d round=%llu stalled_at=%s total=%.3fs.",
        uav_id_, static_cast<unsigned long long>(current_auction_task_set_hash_),
        waiting_for, std::max(0.0, (now - auction_round_started_).toSec()));
  }

  void logAuctionAbort(const char* reason, const ros::Time& now) const {
    ROS_WARN("[AuctionTiming] UAV %d round=%llu tasks=%zu outcome=aborted "
             "reason=%s total=%.3fs.",
             uav_id_, static_cast<unsigned long long>(current_auction_task_set_hash_),
             current_auction_task_set_.frontiers.size(), reason,
             std::max(0.0, (now - auction_round_started_).toSec()));
  }

  void clearAuctionRound(bool clear_offers = true) {
    current_auction_task_set_ = prometheus_two_uav_coverage_search::SwarmAuctionTaskSet();
    current_auction_task_set_hash_ = 0;
    auction_round_started_ = ros::Time(0);
    local_auction_proposal_ = prometheus_two_uav_coverage_search::SwarmAuctionAssignment();
    peer_auction_proposal_ = prometheus_two_uav_coverage_search::SwarmAuctionAssignment();
    local_auction_final_ = prometheus_two_uav_coverage_search::SwarmAuctionAssignment();
    peer_auction_final_ = prometheus_two_uav_coverage_search::SwarmAuctionAssignment();
    auction_last_stage_ = ros::Time(0);
    auction_timing_stages_ = 0;
    auction_stage_elapsed_s_.fill(0.0);
    if (clear_offers) clearAuctionOffers();
  }

  void resetAuctionRound(
      const prometheus_two_uav_coverage_search::SwarmAuctionTaskSet& task_set) {
    current_auction_task_set_ = task_set;
    current_auction_task_set_hash_ = task_set.task_set_hash;
    local_auction_proposal_ = prometheus_two_uav_coverage_search::SwarmAuctionAssignment();
    peer_auction_proposal_ = prometheus_two_uav_coverage_search::SwarmAuctionAssignment();
    local_auction_final_ = prometheus_two_uav_coverage_search::SwarmAuctionAssignment();
    peer_auction_final_ = prometheus_two_uav_coverage_search::SwarmAuctionAssignment();
    local_auction_proposal_received_ = ros::Time(0);
    peer_auction_proposal_received_ = ros::Time(0);
    peer_auction_final_received_ = ros::Time(0);
  }

  bool makeFinalAssignment(
      prometheus_two_uav_coverage_search::SwarmAuctionAssignment* final) const {
    if (!assignmentMatchesTaskSet(local_auction_proposal_, current_auction_task_set_) ||
        !assignmentMatchesTaskSet(peer_auction_proposal_, current_auction_task_set_)) return false;
    *final = local_auction_proposal_;
    final->header.stamp = ros::Time::now();
    final->source_uav_id = uav_id_;
    final->final_result = true;
    for (size_t i = 0; i < final->owner_uav_ids.size(); ++i) {
      const uint32_t local_owner = local_auction_proposal_.owner_uav_ids[i];
      const uint32_t peer_owner = peer_auction_proposal_.owner_uav_ids[i];
      const uint32_t owner = two_uav_auction::resolveConsensusOwner(
          local_owner, local_auction_proposal_.source_uav_id,
          peer_owner, peer_auction_proposal_.source_uav_id,
          std::min<uint32_t>(uav_id_, peer_uav_id_),
          current_auction_task_set_.frontiers[i].previous_owner_uav_id);
      final->owner_uav_ids[i] = owner;
      if (owner == 0) {
        final->assignment_modes[i] =
            prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_UNREACHABLE;
        if (local_owner != peer_owner) {
          ROS_WARN("[AuctionResolution] task=%llu explicit UNREACHABLE: "
                   "UAV%d proposed owner=%u, UAV%d proposed owner=%u; "
                   "candidate owner did not confirm its own endpoint.",
                   static_cast<unsigned long long>(final->task_ids[i]),
                   local_auction_proposal_.source_uav_id, local_owner,
                   peer_auction_proposal_.source_uav_id, peer_owner);
        }
        continue;
      }
      const auto *owner_proposal =
          local_auction_proposal_.source_uav_id == owner && local_owner == owner
              ? &local_auction_proposal_
              : peer_auction_proposal_.source_uav_id == owner && peer_owner == owner
              ? &peer_auction_proposal_ : nullptr;
      final->viewpoint_indices[i] = owner_proposal ? owner_proposal->viewpoint_indices[i] : 0;
      final->assignment_modes[i] = owner_proposal
          ? owner_proposal->assignment_modes[i]
          : static_cast<uint8_t>(
              prometheus_two_uav_coverage_search::SwarmAuctionAssignment::MODE_ASTAR);
    }
    final->assignment_hash = two_uav_auction::assignmentHash(*final);
    return true;
  }

  void commitAuctionAssignment(
      const prometheus_two_uav_coverage_search::SwarmAuctionAssignment& final,
      const ros::Time& now) {
    if (!assignmentMatchesTaskSet(final, current_auction_task_set_)) return;
    int assigned = 0, unreachable = 0;
    for (size_t i = 0; i < final.owner_uav_ids.size(); ++i) {
      const auto& frontier = current_auction_task_set_.frontiers[i];
      const uint32_t owner = final.owner_uav_ids[i];
      if (owner == 0) {
        last_committed_owners_.erase(frontier.task_id);
        recently_assigned_.erase(frontier.task_id);
        recently_assigned_frontiers_.erase(frontier.task_id);
        auto unresolved = frontier;
        const uint32_t view = final.viewpoint_indices[i];
        if (view < unresolved.candidate_viewpoints.size() && view != 0) {
          std::rotate(unresolved.candidate_viewpoints.begin(),
                      unresolved.candidate_viewpoints.begin() + view,
                      unresolved.candidate_viewpoints.begin() + view + 1);
        }
        unreachable_tasks_[frontier.task_id] = unresolved;
        const uint32_t failures = ++unreachable_retry_counts_[frontier.task_id];
        unreachable_retry_after_[frontier.task_id] =
            now + ros::Duration(two_uav_auction::unreachableRetryDelay(
                unreachable_retry_period_, unreachable_retry_max_period_, failures));
        ++unreachable;
      } else {
        last_committed_owners_[frontier.task_id] = owner;
        unreachable_tasks_.erase(frontier.task_id);
        unreachable_retry_after_.erase(frontier.task_id);
        unreachable_retry_counts_.erase(frontier.task_id);
        // Both coordinators suppress this committed UID until its advertised
        // lease expires. This closes the bridge-delay window where the losing
        // UAV has not received the owner's task heartbeat yet and would
        // otherwise immediately auction the same task again.
        recently_assigned_[frontier.task_id] = std::make_pair(
            frontier.cluster_version, now + ros::Duration(task_lease_duration_));
        recently_assigned_frontiers_[frontier.task_id] = std::make_pair(
            frontier, now + ros::Duration(task_lease_duration_));
      }
      ROS_INFO("[AuctionResult] UAV %d round=%llu task=%llu owner=%u view=%u mode=%s.",
               uav_id_, static_cast<unsigned long long>(final.task_set_hash),
               static_cast<unsigned long long>(frontier.task_id), owner,
               final.viewpoint_indices[i], assignmentModeName(final.assignment_modes[i]));
      if (owner != static_cast<uint32_t>(uav_id_)) continue;
      auto task = makeLease(frontier, now, final.viewpoint_indices[i]);
      own_leases_[task.task_id] = task;
      ++assigned;
    }
    publishOwnLeases(now);
    ROS_INFO("[two_uav_coordinator] UAV %d completed distributed round %llu: "
             "%d new lease(s), %d explicitly unreachable task(s).",
             uav_id_, static_cast<unsigned long long>(final.task_set_hash),
             assigned, unreachable);
  }

  void runAuction() {
    const ros::Time now = ros::Time::now();
    const double period = current_auction_task_set_hash_ == 0 ? auction_period_ : 0.05;
    if ((now - last_auction_).toSec() < period) return;
    last_auction_ = now;

    const uint64_t active_task_id = local_trajectory_.active_task_id;
    const bool active_fresh = active_task_id != 0 && !local_trajectory_received_.isZero() &&
        (now - local_trajectory_received_).toSec() <= 2.5;
    const auto source_still_offers = [&](const prometheus_two_uav_coverage_search::SwarmTask& task) {
      const auto contains = [&](const prometheus_two_uav_coverage_search::SwarmFrontierArray& frontiers) {
        return std::any_of(frontiers.frontiers.begin(), frontiers.frontiers.end(),
            [&](const prometheus_two_uav_coverage_search::SwarmFrontier& frontier) {
              return frontier.task_id == task.task_id &&
                  frontier.frontier_source_uav_id == task.frontier_source_uav_id;
            });
      };
      const bool local_unknown = local_frontier_received_.isZero() ||
          (now - local_frontier_received_).toSec() > 5.0;
      const bool peer_unknown = peer_frontier_received_.isZero() ||
          (now - peer_frontier_received_).toSec() > 5.0;
      return local_unknown || peer_unknown || contains(local_frontiers_) || contains(peer_frontiers_);
    };
    for (auto it = own_leases_.begin(); it != own_leases_.end();) {
      if (!source_still_offers(it->second) || it->second.lease_expire_time <= now) {
        it = own_leases_.erase(it);
      } else {
        // Only the task actually being executed renews its lease. Queued
        // distributed tasks return to the shared pool after 8 seconds so one
        // UAV cannot hold an entire bundle indefinitely.
        if (it->first == active_task_id && active_fresh)
          it->second.lease_expire_time = now + ros::Duration(task_lease_duration_);
        ++it;
      }
    }

    prometheus_two_uav_coverage_search::SwarmAuctionTaskSet task_set;
    if (current_auction_task_set_hash_ == 0) {
      if (local_auction_offer_created_.isZero()) {
        if (!buildAuctionTaskSet(now, active_task_id, active_fresh,
                                 &local_auction_offer_)) {
          publishOwnLeases(now);
          return;
        }
        local_auction_offer_.source_uav_id = uav_id_;
        local_auction_offer_.offer_sequence = ++auction_offer_sequence_;
        local_auction_offer_.peer_offer_sequence_ack = 0;
        local_auction_offer_created_ = now;
      }
      const bool peer_offer_fresh = !peer_auction_offer_received_.isZero() &&
          (now - peer_auction_offer_received_).toSec() <= 1.0;
      if (peer_offer_fresh)
        local_auction_offer_.peer_offer_sequence_ack = peer_auction_offer_.offer_sequence;
      auction_task_set_sync_pub_.publish(local_auction_offer_);
      if (!peer_offer_fresh) {
        if (!local_auction_offer_.frontiers.empty()) {
          ROS_WARN_THROTTLE(2.0,
              "[AuctionTaskSet] UAV %d peer offer missing; local_uids=%s.",
              uav_id_, taskIds(local_auction_offer_.frontiers).c_str());
        }
        publishOwnLeases(now);
        return;
      }
      if (!two_uav_auction::offersMutuallyAcknowledged(
              local_auction_offer_, peer_auction_offer_)) {
        publishOwnLeases(now);
        return;
      }
      task_set = two_uav_auction::confirmedTaskSet(local_auction_offer_, peer_auction_offer_);
      std::set<uint64_t> offered_uids;
      for (const auto& task : local_auction_offer_.frontiers) offered_uids.insert(task.task_id);
      for (const auto& task : peer_auction_offer_.frontiers) offered_uids.insert(task.task_id);
      if (task_set.frontiers.size() < offered_uids.size()) {
        ROS_INFO("[AuctionDedup] UAV %d suppressed %zu geometrically equivalent public task(s).",
                 uav_id_, offered_uids.size() - task_set.frontiers.size());
      }
      task_set.header.stamp = now;
      task_set.source_uav_id = uav_id_;
      logAuctionOfferDifference(task_set);
      if (task_set.frontiers.empty()) {
        if ((now - local_auction_offer_created_).toSec() >=
            std::max(0.6, 3.0 * auction_period_)) clearAuctionOffers();
        publishOwnLeases(now);
        return;
      }
      auction_round_started_ = now;
      resetAuctionRound(task_set);
      logAuctionStage(TIMING_FROZEN, now);
    } else {
      // Keep the frozen offer visible until both coordinators finish this
      // round. This lets the slower peer complete the mutual acknowledgement.
      auction_task_set_sync_pub_.publish(local_auction_offer_);
      task_set = current_auction_task_set_;
      if (!auctionSnapshotStillEligible(now)) {
        logAuctionAbort("member_disappeared", now);
        clearAuctionRound();
        publishOwnLeases(now);
        return;
      }
    }

    prometheus_two_uav_coverage_search::SwarmAuctionManifest manifest;
    manifest.header.stamp = now;
    manifest.source_uav_id = uav_id_;
    manifest.map_epoch = map_epoch_;
    manifest.task_set_hash = task_set.task_set_hash;
    manifest.task_count = task_set.frontiers.size();
    manifest.uav1_state_sequence = task_set.uav1_state_sequence;
    manifest.uav2_state_sequence = task_set.uav2_state_sequence;
    auction_manifest_pub_.publish(manifest);

    const bool peer_manifest_is_current = !peer_auction_manifest_received_.isZero() &&
        peer_auction_manifest_received_ >= auction_round_started_;
    const bool task_set_confirmed = peer_manifest_is_current &&
        (now - peer_auction_manifest_received_).toSec() <= 5.0 &&
        peer_auction_manifest_.task_set_hash == task_set.task_set_hash &&
        peer_auction_manifest_.task_count == task_set.frontiers.size() &&
        peer_auction_manifest_.uav1_state_sequence == task_set.uav1_state_sequence &&
        peer_auction_manifest_.uav2_state_sequence == task_set.uav2_state_sequence;
    if (!task_set_confirmed) {
      if (peer_manifest_is_current &&
          (peer_auction_manifest_.task_set_hash != task_set.task_set_hash ||
           peer_auction_manifest_.task_count != task_set.frontiers.size() ||
           peer_auction_manifest_.uav1_state_sequence != task_set.uav1_state_sequence ||
           peer_auction_manifest_.uav2_state_sequence != task_set.uav2_state_sequence)) {
        logAuctionAbort("peer_common_set_differs", now);
        ROS_INFO("[two_uav_coordinator] UAV %d abandoned distributed round %llu: peer task-set snapshot differs.",
                 uav_id_, static_cast<unsigned long long>(task_set.task_set_hash));
        // The peer may still have emitted a delayed manifest from its previous
        // offer pair. Preserve our frozen offer so the explicit offer ACK can
        // converge without generating yet another candidate generation.
        clearAuctionRound(false);
      } else if ((now - auction_round_started_).toSec() > task_commit_timeout_) {
        logAuctionAbort("peer_common_set_timeout", now);
        ROS_WARN("[two_uav_coordinator] UAV %d abandoned distributed round %llu: peer confirmation timed out.",
                 uav_id_, static_cast<unsigned long long>(task_set.task_set_hash));
        clearAuctionRound();
      } else {
        logAuctionWait("peer_manifest", now);
      }
      return;
    }
    logAuctionStage(TIMING_TASK_SET, now);

    if ((now - auction_round_started_).toSec() > task_commit_timeout_) {
      logAuctionAbort("proposal_or_final_timeout", now);
      ROS_WARN("[two_uav_coordinator] UAV %d abandoned distributed round %llu: "
               "proposal/final consensus timed out.",
               uav_id_, static_cast<unsigned long long>(task_set.task_set_hash));
      clearAuctionRound();
      return;
    }

    if (!assignmentMatchesTaskSet(local_auction_proposal_, task_set)) {
      auction_task_set_pub_.publish(task_set);
      logAuctionWait("local_proposal", now);
      return;
    }
    logAuctionStage(TIMING_LOCAL_PROPOSAL, local_auction_proposal_received_);
    auction_assignment_pub_.publish(local_auction_proposal_);
    if (!assignmentMatchesTaskSet(peer_auction_proposal_, task_set)) {
      logAuctionWait("peer_proposal", now);
      return;
    }
    logAuctionStage(TIMING_PEER_PROPOSAL, peer_auction_proposal_received_);

    if (!assignmentMatchesTaskSet(local_auction_final_, task_set)) {
      if (!makeFinalAssignment(&local_auction_final_)) return;
      logAuctionStage(TIMING_LOCAL_FINAL, now);
    }
    auction_assignment_pub_.publish(local_auction_final_);
    if (!assignmentMatchesTaskSet(peer_auction_final_, task_set) ||
        !peer_auction_final_.final_result ||
        peer_auction_final_.assignment_hash != local_auction_final_.assignment_hash) {
      logAuctionWait("peer_final", now);
      return;
    }
    logAuctionStage(TIMING_PEER_FINAL, peer_auction_final_received_);

    logAuctionStage(TIMING_COMMITTED, now);
    commitAuctionAssignment(local_auction_final_, now);
    clearAuctionRound();
  }

  void timerCb(const ros::TimerEvent&) {
    publishState();
    // Task ownership remains visible independently of flight readiness and
    // planning state. Otherwise an early hold return hides valid leases.
    publishTaskLabels();
    if (!ownReady()) return;
    if (!peerFresh()) {
      if (phase_ == ACTIVE) ROS_ERROR_THROTTLE(1.0, "[two_uav_coordinator] peer unavailable: hold");
      phase_ = WAIT_PEER;
      publishHold();
      return;
    }
    if (!peer_.odom_valid || !peer_.command_ready) {
      phase_ = WAIT_TAKEOFF;
      publishHold();
      return;
    }
    if ((phase_ == WAIT_PEER || phase_ == WAIT_TAKEOFF) && !startSeparationOk()) {
      ROS_WARN_THROTTLE(1.0, "[two_uav_coordinator] UAV %d waits: start separation < %.2fm",
                        uav_id_, min_start_separation_);
      phase_ = WAIT_TAKEOFF;
      publishHold();
      return;
    }
    // Low speed is a takeoff-readiness condition, not a flight-speed limit.
    // Applying it while ACTIVE repeatedly changed a valid trajectory command
    // into a position hold whenever the aircraft accelerated past 0.35 m/s.
    if (!altitudeAtHeight(state_.position[2]) ||
        !altitudeAtHeight(peer_.pose.position.z)) {
      phase_ = WAIT_TAKEOFF;
      publishHold();
      return;
    }
    if ((phase_ == WAIT_PEER || phase_ == WAIT_TAKEOFF) && !stableAtHeight(state_)) {
      phase_ = WAIT_TAKEOFF;
      publishHold();
      return;
    }
    phase_ = ACTIVE;
    frontier_pub_.publish(peerVisibleFrontiers(ros::Time::now()));
    runAuction();
    if (!rawCommandFresh()) {
      ROS_WARN_THROTTLE(1.0,
                        "[two_uav_coordinator] UAV %d raw coverage command %s: hold",
                        uav_id_, have_raw_command_ ? "is stale" : "is missing");
      phase_ = HOLD;
      publishHold();
      return;
    }
    std::string safety_reason;
    if (!peerTrajectorySafe(&safety_reason)) {
      ROS_WARN_THROTTLE(1.0, "[two_uav_coordinator] UAV %d holds: %s", uav_id_, safety_reason.c_str());
      phase_ = HOLD;
      publishHold();
      if (uav_id_ < peer_uav_id_) publishHoldTrajectory();
      else trajectory_pub_.publish(local_trajectory_);
      return;
    }
    trajectory_pub_.publish(local_trajectory_);
  }

  void publishTaskLabels() {
    visualization_msgs::Marker label;
    label.header.frame_id = "world";
    label.header.stamp = ros::Time::now();
    label.ns = "task_owner";
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::Marker::ADD;
    label.scale.z = 0.30;
    label.lifetime = ros::Duration(2.5);

    const ros::Time now = ros::Time::now();

    std::set<int> current_ids;
    const auto markerId = [&](uint64_t task_id) {
      const auto found = task_label_ids_.find(task_id);
      if (found != task_label_ids_.end()) return found->second;
      const int id = next_task_label_id_++;
      task_label_ids_[task_id] = id;
      return id;
    };
    // Each coordinator publishes only the tasks it owns. RViz subscribes to
    // both UAV topics, so every assigned viewpoint appears exactly once.
    for (const auto& task : local_tasks_.tasks) {
      if (task.lease_expire_time <= now ||
          task.winner_uav_id != static_cast<uint32_t>(uav_id_)) continue;
      label.id = markerId(task.task_id);
      current_ids.insert(label.id);
      label.pose = labelPose(task);
      label.pose.position.z = fly_height_ + 0.55;
      label.text = "UAV" + std::to_string(task.winner_uav_id) + "\n" +
          std::to_string(std::max(0, (int)std::ceil((task.lease_expire_time - now).toSec()))) + "s";
      label.color.r = uav_id_ == 1 ? 0.0f : 0.2f;
      label.color.g = 1.0f;
      label.color.b = uav_id_ == 1 ? 0.0f : 1.0f;
      label.color.a = 0.9f;
      task_label_pub_.publish(label);
    }

    // Unreachable is a task state, not an ownership label. Publish it once
    // from the lower-ID coordinator so it cannot silently disappear or double.
    if (uav_id_ < peer_uav_id_) {
      for (const auto& entry : unreachable_tasks_) {
        label.id = markerId(entry.first);
        current_ids.insert(label.id);
        label.pose = labelPose(entry.second);
        label.pose.position.z = fly_height_ + 0.55;
        label.text = "UNREACHABLE";
        label.color.r = 1.0f;
        label.color.g = 0.1f;
        label.color.b = 0.1f;
        label.color.a = 0.95f;
        task_label_pub_.publish(label);
      }
    }

    // Marker IDs follow task UIDs, not vector order. Removing one task can no
    // longer leave an old UAV1/UAV2 label attached to another viewpoint.
    for (const int id : published_task_label_ids_) {
      if (current_ids.count(id) != 0) continue;
      label.id = id;
      label.action = visualization_msgs::Marker::DELETE;
      task_label_pub_.publish(label);
    }
    published_task_label_ids_ = std::move(current_ids);
  }

  ros::NodeHandle nh_;
  int uav_id_ = 1, peer_uav_id_ = 2, map_epoch_ = 1, task_bundle_size_ = 16;
  double fly_height_ = 1.5, min_start_separation_ = 1.0, safe_separation_ = 1.2;
  double peer_timeout_ = 0.6, trajectory_timeout_ = 0.6, max_vel_ = 1.0;
  double yaw_max_rate_ = 1.3962634, yaw_max_acc_ = 1.5707963;
  double auction_period_ = 0.2, task_reach_dist_ = 0.35, raw_command_timeout_ = 0.6;
  double task_commit_timeout_ = 15.0, task_retry_cooldown_ = 12.0, task_goal_separation_ = 3.0;
  double task_lease_duration_ = 8.0, unreachable_retry_period_ = 1.0;
  double unreachable_retry_max_period_ = 8.0;
  std::string tx_prefix_, rx_prefix_;
  Phase phase_ = WAIT_PEER;
  bool have_state_ = false, have_peer_ = false, have_raw_command_ = false, completion_ready_ = false;
  bool relay_yaw_inited_ = false;
  double relay_yaw_ = 0.0, relay_yaw_rate_ = 0.0;
  prometheus_msgs::UAVState state_;
  prometheus_msgs::UAVControlState control_;
  prometheus_msgs::UAVCommand raw_command_;
  prometheus_two_uav_coverage_search::SwarmState peer_;
  prometheus_two_uav_coverage_search::SwarmFrontierArray local_frontiers_, peer_frontiers_;
  prometheus_two_uav_coverage_search::SwarmTaskArray local_tasks_, peer_tasks_;
  prometheus_two_uav_coverage_search::SwarmTrajectory local_trajectory_, peer_trajectory_;
  prometheus_two_uav_coverage_search::SwarmAuctionManifest peer_auction_manifest_;
  prometheus_two_uav_coverage_search::SwarmAuctionTaskSet local_auction_offer_, peer_auction_offer_;
  prometheus_two_uav_coverage_search::SwarmAuctionTaskSet current_auction_task_set_;
  prometheus_two_uav_coverage_search::SwarmAuctionAssignment local_auction_proposal_, peer_auction_proposal_;
  prometheus_two_uav_coverage_search::SwarmAuctionAssignment local_auction_final_, peer_auction_final_;
  ros::Time peer_received_, local_frontier_received_, peer_frontier_received_, local_task_published_, peer_task_received_;
  ros::Time last_raw_command_received_;
  ros::Time local_trajectory_received_, peer_trajectory_received_, last_auction_;
  ros::Time peer_auction_manifest_received_, local_auction_proposal_received_,
      peer_auction_proposal_received_, peer_auction_final_received_;
  ros::Time local_auction_offer_created_, peer_auction_offer_received_;
  ros::Time last_collision_replan_request_;
  uint64_t state_sequence_ = 0;
  uint64_t current_auction_task_set_hash_ = 0;
  uint64_t auction_offer_sequence_ = 0;
  uint64_t last_offer_pair_log_hash_ = 0;
  ros::Time auction_round_started_, auction_last_stage_;
  uint32_t auction_timing_stages_ = 0;
  std::array<double, 7> auction_stage_elapsed_s_{};
  uint32_t task_revision_ = 0, command_id_ = 0;
  std::map<uint64_t, prometheus_two_uav_coverage_search::SwarmTask> own_leases_;
  std::map<uint64_t, uint32_t> last_committed_owners_;
  std::map<uint64_t, prometheus_two_uav_coverage_search::SwarmFrontier> unreachable_tasks_;
  std::map<uint64_t, ros::Time> unreachable_retry_after_;
  std::map<uint64_t, uint32_t> unreachable_retry_counts_;
  std::map<uint64_t, std::pair<uint32_t, ros::Time>> recently_assigned_;
  std::map<uint64_t, std::pair<
      prometheus_two_uav_coverage_search::SwarmFrontier, ros::Time>>
      recently_assigned_frontiers_;
  ros::Subscriber state_sub_, control_sub_, raw_command_sub_, local_frontier_sub_, local_trajectory_sub_, completion_ready_sub_;
  ros::Subscriber local_auction_assignment_sub_;
  ros::Subscriber peer_state_sub_, peer_frontier_sub_, peer_task_sub_, peer_trajectory_sub_;
  ros::Subscriber peer_auction_manifest_sub_, peer_auction_task_set_sub_, peer_auction_assignment_sub_;
  ros::Publisher state_pub_, frontier_pub_, task_pub_, trajectory_pub_, peer_collision_replan_pub_, command_pub_, task_label_pub_;
  ros::Publisher auction_manifest_pub_, auction_assignment_pub_, auction_task_set_pub_,
      auction_task_set_sync_pub_;
  ros::Publisher local_task_pub_;
  std::map<uint64_t, int> task_label_ids_;
  std::set<int> published_task_label_ids_;
  int next_task_label_id_ = 0;
  ros::Timer timer_, command_timer_;
};

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--self-test") {
    return yawRelaySelfTest() ? 0 : 1;
  }
  ros::init(argc, argv, "two_uav_coordinator");
  ros::NodeHandle nh("~");
  TwoUavCoordinator coordinator;
  coordinator.init(nh);
  ros::spin();
  return 0;
}
