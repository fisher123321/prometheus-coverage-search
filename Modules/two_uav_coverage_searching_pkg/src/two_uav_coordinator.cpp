#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
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
#include <prometheus_two_uav_coverage_search/SwarmBidArray.h>
#include <prometheus_two_uav_coverage_search/SwarmState.h>
#include <prometheus_two_uav_coverage_search/SwarmTaskArray.h>
#include <prometheus_two_uav_coverage_search/SwarmTrajectory.h>

namespace {
enum Phase : uint8_t { WAIT_PEER = 0, WAIT_TAKEOFF = 1, ACTIVE = 2, HOLD = 3 };

double distance2d(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
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
    nh_.param("auction_period", auction_period_, 2.0);
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
    local_bid_sub_ = nh_.subscribe(uav + "/prometheus/coverage_search/swarm_bids", 2,
                                    &TwoUavCoordinator::localBidCb, this);
    local_trajectory_sub_ = nh_.subscribe(uav + "/prometheus/coverage_search/swarm_trajectory", 2,
                                           &TwoUavCoordinator::localTrajectoryCb, this);
    completion_ready_sub_ = nh_.subscribe(uav + "/prometheus/coverage_search/completion_ready", 1,
                                           &TwoUavCoordinator::completionReadyCb, this);
    peer_state_sub_ = nh_.subscribe(rx_prefix_ + "/state", 10, &TwoUavCoordinator::peerStateCb, this);
    peer_frontier_sub_ = nh_.subscribe(rx_prefix_ + "/frontier", 2,
                                        &TwoUavCoordinator::peerFrontierCb, this);
    peer_bid_sub_ = nh_.subscribe(rx_prefix_ + "/bid", 2,
                                   &TwoUavCoordinator::peerBidCb, this);
    peer_task_sub_ = nh_.subscribe(rx_prefix_ + "/task", 2, &TwoUavCoordinator::peerTaskCb, this);
    peer_trajectory_sub_ = nh_.subscribe(rx_prefix_ + "/trajectory", 2,
                                          &TwoUavCoordinator::peerTrajectoryCb, this);

    state_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmState>(tx_prefix_ + "/state", 10);
    frontier_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmFrontierArray>(tx_prefix_ + "/frontier", 2);
    bid_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmBidArray>(tx_prefix_ + "/bid", 2);
    task_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmTaskArray>(tx_prefix_ + "/task", 2);
    trajectory_pub_ = nh_.advertise<prometheus_two_uav_coverage_search::SwarmTrajectory>(tx_prefix_ + "/trajectory", 2);
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
  }

  void commandTimerCb(const ros::TimerEvent& event) {
    if (!ownReady() || phase_ != ACTIVE || !have_raw_command_) return;
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
    local_frontiers_ = *msg;
    local_frontier_received_ = ros::Time::now();
  }
  void localBidCb(const prometheus_two_uav_coverage_search::SwarmBidArray::ConstPtr& msg) {
    if (static_cast<int>(msg->source_uav_id) != uav_id_) return;
    local_bids_ = *msg;
    local_bid_received_ = ros::Time::now();
    // Match the already-working frontier relay: the coverage node publishes
    // locally, and the coordinator hands the bid to the independent-master bridge.
    bid_pub_.publish(local_bids_);
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
    if (static_cast<int>(msg->source_uav_id) != peer_uav_id_) return;
    peer_frontiers_ = *msg;
    peer_frontier_received_ = ros::Time::now();
  }
  void peerBidCb(const prometheus_two_uav_coverage_search::SwarmBidArray::ConstPtr& msg) {
    if (static_cast<int>(msg->source_uav_id) != peer_uav_id_) return;
    peer_bids_ = *msg;
    peer_bid_received_ = ros::Time::now();
  }
  void peerTaskCb(const prometheus_two_uav_coverage_search::SwarmTaskArray::ConstPtr& msg) {
    if (static_cast<int>(msg->source_uav_id) != peer_uav_id_) return;
    peer_tasks_ = *msg;
    peer_task_received_ = ros::Time::now();
  }
  void peerTrajectoryCb(const prometheus_two_uav_coverage_search::SwarmTrajectory::ConstPtr& msg) {
    if (static_cast<int>(msg->source_uav_id) != peer_uav_id_) return;
    peer_trajectory_ = *msg;
    peer_trajectory_received_ = ros::Time::now();
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
    if (first >= static_cast<int>(trajectory.points.size())) return false;
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
    const bool trajectories_fresh = !local_trajectory_received_.isZero() &&
        !peer_trajectory_received_.isZero() &&
        (now - local_trajectory_received_).toSec() <= trajectory_timeout_ &&
        (now - peer_trajectory_received_).toSec() <= trajectory_timeout_;
    if (!peerFresh() || !trajectories_fresh) return true;

    geometry_msgs::Point self, peer;
    bool high_id_escaping = false;
    double previous_separation = 0.0;
    for (double t = 0.0; t <= 2.0; t += 0.05) {
      const ros::Time when = now + ros::Duration(t);
      if (!sampleTrajectory(local_trajectory_, when, &self) ||
          !sampleTrajectory(peer_trajectory_, when, &peer)) break;
      const double separation = std::sqrt(
          (self.x - peer.x) * (self.x - peer.x) +
          (self.y - peer.y) * (self.y - peer.y) +
          (self.z - peer.z) * (self.z - peer.z));
      if (separation >= safe_separation_) {
        if (high_id_escaping) return true;
        continue;
      }

      if (uav_id_ > peer_uav_id_ &&
          (last_collision_replan_request_.isZero() ||
           (now - last_collision_replan_request_).toSec() >= 0.5)) {
        std_msgs::Bool request;
        request.data = true;
        peer_collision_replan_pub_.publish(request);
        last_collision_replan_request_ = now;
      }
      // 两机已经进入安全间距内时，高 ID 机不能因 t=0 的既有重叠
      // 永远被拦截；只允许它沿间距单调增加的轨迹退出，低 ID 保持悬停。
      if (uav_id_ > peer_uav_id_) {
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
            (uav_id_ < peer_uav_id_ ? "lower ID holds" : "higher ID replans");
      }
      return false;
    }
    return !high_id_escaping;
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

  void runAuction() {
    const ros::Time now = ros::Time::now();
    if ((now - last_auction_).toSec() < auction_period_) return;
    last_auction_ = now;

    const uint64_t active_task_id = local_trajectory_.active_task_id;
    const bool active_fresh = active_task_id != 0 && !local_trajectory_received_.isZero() &&
        (now - local_trajectory_received_).toSec() <= 2.5;
    std::vector<prometheus_two_uav_coverage_search::SwarmTask> own;
    for (auto it = own_leases_.begin(); it != own_leases_.end();) {
      if (it->first == active_task_id && active_fresh) {
        it->second.lease_expire_time = now + ros::Duration(task_lease_duration_);
      }
      if (it->second.lease_expire_time <= now) {
        it = own_leases_.erase(it);
      } else {
        own.push_back(it->second);
        ++it;
      }
    }

    const bool peer_lease_fresh = !peer_task_received_.isZero() &&
        (now - peer_task_received_).toSec() <= 5.0;
    std::map<uint64_t, prometheus_two_uav_coverage_search::SwarmFrontier> tasks;
    std::map<uint64_t, uint32_t> task_sources;
    auto merge_tasks = [&](const prometheus_two_uav_coverage_search::SwarmFrontierArray& source) {
      for (const auto& frontier : source.frontiers) {
        const auto found = task_sources.find(frontier.task_id);
        // Both coordinators must choose the same representation for a task ID.
        // Lower ID is a deterministic tie-breaker while map chunks converge.
        if (found == task_sources.end() || source.source_uav_id < found->second) {
          auto selected = frontier;
          if (found != task_sources.end() &&
              tasks[frontier.task_id].reservation_expire_time >
                  selected.reservation_expire_time) {
            selected.reservation_owner_uav_id =
                tasks[frontier.task_id].reservation_owner_uav_id;
            selected.reservation_expire_time =
                tasks[frontier.task_id].reservation_expire_time;
          }
          tasks[frontier.task_id] = selected;
          task_sources[frontier.task_id] = source.source_uav_id;
        } else if (frontier.reservation_expire_time >
                   tasks[frontier.task_id].reservation_expire_time) {
          tasks[frontier.task_id].reservation_owner_uav_id =
              frontier.reservation_owner_uav_id;
          tasks[frontier.task_id].reservation_expire_time =
              frontier.reservation_expire_time;
        }
      }
    };
    merge_tasks(local_frontiers_);
    merge_tasks(peer_frontiers_);
    const double bid_stale_limit = std::max(2.5, 2.0 * auction_period_ + 0.5);
    const bool bids_fresh = !local_bid_received_.isZero() && !peer_bid_received_.isZero() &&
        (now - local_bid_received_).toSec() <= bid_stale_limit &&
        (now - peer_bid_received_).toSec() <= bid_stale_limit;
    if (!peer_lease_fresh || !bids_fresh) {
      ROS_INFO_THROTTLE(2.0, "[two_uav_coordinator] UAV %d renews existing leases only.", uav_id_);
    } else {
      std::map<uint64_t, double> self_route_cost, peer_route_cost;
      for (const auto& bid : local_bids_.bids) {
        if (static_cast<int>(bid.bidder_uav_id) == uav_id_ && bid.reachable &&
            std::isfinite(bid.cost)) self_route_cost[bid.task_id] = bid.cost;
      }
      for (const auto& bid : peer_bids_.bids) {
        if (static_cast<int>(bid.bidder_uav_id) == peer_uav_id_ && bid.reachable &&
            std::isfinite(bid.cost)) peer_route_cost[bid.task_id] = bid.cost;
      }
      if (active_fresh && own_leases_.count(active_task_id) == 0) {
        const auto active = tasks.find(active_task_id);
        if (active != tasks.end() &&
            active->second.reservation_expire_time <= now) {
          const auto self_bid = self_route_cost.find(active_task_id);
          prometheus_two_uav_coverage_search::SwarmTask held;
          held.task_id = active_task_id;
          held.task_version = std::max(local_bids_.frontier_revision,
                                       peer_bids_.frontier_revision);
          held.cluster_version = active->second.cluster_version;
          held.frontier_cell_count = active->second.frontier_cell_count;
          held.winner_uav_id = uav_id_;
          held.cost = self_bid == self_route_cost.end()
              ? std::numeric_limits<float>::infinity() : self_bid->second;
          held.centroid = active->second.centroid;
          held.box_min = active->second.box_min;
          held.box_max = active->second.box_max;
          held.goal = active->second.viewpoint;
          own.push_back(held);
        }
      }
      struct Candidate {
        prometheus_two_uav_coverage_search::SwarmTask task;
        double peer_cost;
      };
      std::vector<Candidate> candidates;
      for (const auto& entry : tasks) {
        const auto& frontier = entry.second;
        if (own_leases_.count(frontier.task_id) != 0) continue;
        if (active_fresh && frontier.task_id == active_task_id) continue;
        if (frontier.reservation_owner_uav_id != 0 &&
            frontier.reservation_expire_time > now) continue;
        bool peer_already_leases = false;
        for (const auto& task : peer_tasks_.tasks) {
          // A fresh unexpired peer entry is authoritative even if its owner
          // field is inconsistent.  Waiting for it to expire is safer than
          // turning a temporary disagreement into a takeover.
          if (task.task_id == frontier.task_id && task.lease_expire_time > now) {
            peer_already_leases = true;
            break;
          }
        }
        if (peer_already_leases ||
            distance2d(frontier.viewpoint.position, ownPoint()) < task_reach_dist_ ||
            distance2d(frontier.viewpoint.position, peer_.pose.position) < task_reach_dist_) continue;
        const auto self_bid = self_route_cost.find(frontier.task_id);
        const auto peer_bid = peer_route_cost.find(frontier.task_id);
        const double self_cost = self_bid == self_route_cost.end()
            ? std::numeric_limits<double>::infinity() : self_bid->second;
        const double peer_cost = peer_bid == peer_route_cost.end()
            ? std::numeric_limits<double>::infinity() : peer_bid->second;
        if (!std::isfinite(self_cost) && !std::isfinite(peer_cost)) continue;
        prometheus_two_uav_coverage_search::SwarmTask bid;
        bid.task_id = frontier.task_id;
        bid.task_version = std::max(local_bids_.frontier_revision, peer_bids_.frontier_revision);
        bid.cluster_version = frontier.cluster_version;
        bid.frontier_cell_count = frontier.frontier_cell_count;
        bid.winner_uav_id = uav_id_;
        bid.cost = self_cost;
        bid.centroid = frontier.centroid;
        bid.box_min = frontier.box_min;
        bid.box_max = frontier.box_max;
        bid.goal = frontier.viewpoint;
        candidates.push_back({bid, peer_cost});
      }

      int own_seed = -1;
      int peer_seed = -1;
      if (candidates.size() >= 2) {
        for (int pass = 0; pass < 2 && own_seed < 0; ++pass) {
          double best_pair_cost = std::numeric_limits<double>::infinity();
          for (size_t self = 0; self < candidates.size(); ++self) {
            for (size_t peer = 0; peer < candidates.size(); ++peer) {
              if (self == peer) continue;
              if (pass == 0 && distance2d(candidates[self].task.goal.position,
                                          candidates[peer].task.goal.position) < task_goal_separation_) continue;
              const double pair_cost = candidates[self].task.cost + candidates[peer].peer_cost;
              if (pair_cost < best_pair_cost) {
                best_pair_cost = pair_cost;
                own_seed = static_cast<int>(self);
                peer_seed = static_cast<int>(peer);
              }
            }
          }
        }
      } else if (candidates.size() == 1) {
        const bool self_wins = candidates.front().task.cost < candidates.front().peer_cost - 1e-3 ||
            (std::fabs(candidates.front().task.cost - candidates.front().peer_cost) <= 1e-3 &&
             uav_id_ < peer_uav_id_);
        if (self_wins) own_seed = 0;
      }
      if (own_seed < 0) {
        double best_self_cost = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < candidates.size(); ++i) {
          const bool self_wins = candidates[i].task.cost < candidates[i].peer_cost - 1e-3 ||
              (std::fabs(candidates[i].task.cost - candidates[i].peer_cost) <= 1e-3 &&
               uav_id_ < peer_uav_id_);
          if (self_wins && candidates[i].task.cost < best_self_cost) {
            best_self_cost = candidates[i].task.cost;
            own_seed = static_cast<int>(i);
          }
        }
      }
      if (own_seed >= 0 && static_cast<int>(own.size()) < task_bundle_size_)
        own.push_back(candidates[own_seed].task);
      for (size_t i = 0; i < candidates.size() && static_cast<int>(own.size()) < task_bundle_size_; ++i) {
        if (static_cast<int>(i) == own_seed || static_cast<int>(i) == peer_seed) continue;
        const bool self_wins = candidates[i].task.cost < candidates[i].peer_cost - 1e-3 ||
            (std::fabs(candidates[i].task.cost - candidates[i].peer_cost) <= 1e-3 &&
             uav_id_ < peer_uav_id_);
        if (self_wins) own.push_back(candidates[i].task);
      }
    }

    for (auto& task : own) {
      assert(task.winner_uav_id == static_cast<uint32_t>(uav_id_));
      const auto found = own_leases_.find(task.task_id);
      if (found == own_leases_.end()) {
        task.lease_expire_time = now + ros::Duration(task_lease_duration_);
      } else {
        task.lease_expire_time = found->second.lease_expire_time;
      }
      own_leases_[task.task_id] = task;
    }

    prometheus_two_uav_coverage_search::SwarmTaskArray out;
    out.header.stamp = now;
    out.source_uav_id = uav_id_;
    out.map_epoch = map_epoch_;
    out.revision = ++task_revision_;
    out.tasks = own;
    local_tasks_ = out;
    local_task_published_ = now;
    task_pub_.publish(out);
    ROS_INFO_THROTTLE(2.0, "[two_uav_coordinator] UAV %d holds %zu leases.",
                      uav_id_, own.size());
  }

  void timerCb(const ros::TimerEvent&) {
    publishState();
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
    frontier_pub_.publish(local_frontiers_);
    runAuction();
    if (!have_raw_command_) {
      ROS_WARN_THROTTLE(1.0, "[two_uav_coordinator] UAV %d has no raw coverage command: hold", uav_id_);
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
    publishTaskLabels();
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

    struct LabelTask {
      prometheus_two_uav_coverage_search::SwarmTask task;
      bool conflict = false;
    };
    const ros::Time now = ros::Time::now();
    std::map<uint64_t, LabelTask> tasks;
    const auto merge = [&](const prometheus_two_uav_coverage_search::SwarmTaskArray& source,
                           bool fresh) {
      if (!fresh) return;
      for (const auto& task : source.tasks) {
        if (task.lease_expire_time <= now) continue;
        const auto found = tasks.find(task.task_id);
        if (found == tasks.end()) {
          tasks.emplace(task.task_id, LabelTask{task, false});
        } else if (found->second.task.winner_uav_id != task.winner_uav_id) {
          found->second.conflict = true;
        } else if (task.lease_expire_time > found->second.task.lease_expire_time) {
          found->second.task = task;
        }
      }
    };
    merge(local_tasks_, !local_task_published_.isZero() &&
          (now - local_task_published_).toSec() <= 5.0);
    merge(peer_tasks_, !peer_task_received_.isZero() &&
          (now - peer_task_received_).toSec() <= 5.0);

    int id = 0;
    for (const auto& entry : tasks) {
      const auto& item = entry.second;
      const auto& task = item.task;
      label.id = id++;
      label.pose = task.goal;
      label.pose.position.z = fly_height_ + 0.55;
      label.text = item.conflict ? "CONFLICT" : "UAV" + std::to_string(task.winner_uav_id) + "_" +
          std::to_string(std::max(0, (int)std::ceil((task.lease_expire_time - now).toSec()))) + "s";
      label.color.r = item.conflict ? 1.0f : (task.winner_uav_id == static_cast<uint32_t>(uav_id_) ? 0.0f : 0.2f);
      label.color.g = item.conflict ? 0.0f : 1.0f;
      label.color.b = item.conflict ? 0.0f : (task.winner_uav_id == static_cast<uint32_t>(uav_id_) ? 0.0f : 1.0f);
      label.color.a = 0.9f;
      task_label_pub_.publish(label);
    }

    // clear stale labels
    for (int i = id; i < last_task_label_count_; ++i) {
      label.id = i;
      label.action = visualization_msgs::Marker::DELETE;
      task_label_pub_.publish(label);
    }
    last_task_label_count_ = id;
  }

  ros::NodeHandle nh_;
  int uav_id_ = 1, peer_uav_id_ = 2, map_epoch_ = 1, task_bundle_size_ = 16;
  double fly_height_ = 1.5, min_start_separation_ = 1.0, safe_separation_ = 1.2;
  double peer_timeout_ = 0.6, trajectory_timeout_ = 0.6, max_vel_ = 1.0;
  double yaw_max_rate_ = 1.3962634, yaw_max_acc_ = 1.5707963;
  double auction_period_ = 2.0, task_reach_dist_ = 0.35;
  double task_commit_timeout_ = 15.0, task_retry_cooldown_ = 12.0, task_goal_separation_ = 3.0;
  double task_lease_duration_ = 8.0;
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
  prometheus_two_uav_coverage_search::SwarmBidArray local_bids_, peer_bids_;
  prometheus_two_uav_coverage_search::SwarmTaskArray local_tasks_, peer_tasks_;
  prometheus_two_uav_coverage_search::SwarmTrajectory local_trajectory_, peer_trajectory_;
  ros::Time peer_received_, local_frontier_received_, peer_frontier_received_, local_task_published_, peer_task_received_;
  ros::Time local_bid_received_, peer_bid_received_;
  ros::Time local_trajectory_received_, peer_trajectory_received_, last_auction_;
  ros::Time last_collision_replan_request_;
  uint64_t state_sequence_ = 0;
  uint32_t task_revision_ = 0, command_id_ = 0;
  std::map<uint64_t, prometheus_two_uav_coverage_search::SwarmTask> own_leases_;
  ros::Subscriber state_sub_, control_sub_, raw_command_sub_, local_frontier_sub_, local_bid_sub_, local_trajectory_sub_, completion_ready_sub_;
  ros::Subscriber peer_state_sub_, peer_frontier_sub_, peer_bid_sub_, peer_task_sub_, peer_trajectory_sub_;
  ros::Publisher state_pub_, frontier_pub_, bid_pub_, task_pub_, trajectory_pub_, peer_collision_replan_pub_, command_pub_, task_label_pub_;
  int last_task_label_count_ = 0;
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
