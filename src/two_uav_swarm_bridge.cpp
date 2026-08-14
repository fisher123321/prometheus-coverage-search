// Adapted from swarm_ros_bridge (BSD-3-Clause, Peixuan Shu, 2023).
// This version deliberately bridges only the messages used by this package.
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <ros/serialization.h>
#include <zmqpp/zmqpp.hpp>

#include <prometheus_two_uav_coverage_search/SwarmFrontierArray.h>
#include <prometheus_two_uav_coverage_search/SwarmAuctionManifest.h>
#include <prometheus_two_uav_coverage_search/SwarmAuctionTaskSet.h>
#include <prometheus_two_uav_coverage_search/SwarmAuctionAssignment.h>
#include <prometheus_two_uav_coverage_search/SwarmFrontierTransferAck.h>
#include <prometheus_two_uav_coverage_search/SwarmMapChunk.h>
#include <prometheus_two_uav_coverage_search/SwarmMapRequest.h>
#include <prometheus_two_uav_coverage_search/SwarmState.h>
#include <prometheus_two_uav_coverage_search/SwarmTaskArray.h>
#include <prometheus_two_uav_coverage_search/SwarmTrajectory.h>

namespace {

template <typename Message>
class BridgeChannel {
 public:
  BridgeChannel(ros::NodeHandle& nh, zmqpp::context_t& context,
                const std::string& tx_topic, const std::string& rx_topic,
                const std::string& local_ip, const std::string& peer_ip,
                int local_port, int peer_port, int max_hz, uint32_t max_bytes,
                int queue_size = 10)
      : max_hz_(std::max(0, max_hz)), max_bytes_(max_bytes),
        sender_(new zmqpp::socket(context, zmqpp::socket_type::pub)),
        receiver_(new zmqpp::socket(context, zmqpp::socket_type::sub)) {
    const std::string self_endpoint = "tcp://" + local_ip + ":" + std::to_string(local_port);
    const std::string peer_endpoint = "tcp://" + peer_ip + ":" + std::to_string(peer_port);
    sender_->bind(self_endpoint);
    receiver_->subscribe("");
    receiver_->connect(peer_endpoint);
    queue_size = std::max(1, queue_size);
    publisher_ = nh.advertise<Message>(rx_topic, queue_size);
    subscriber_ = nh.subscribe<Message>(tx_topic, queue_size, &BridgeChannel::send, this,
                                        ros::TransportHints().tcpNoDelay());
    receive_thread_ = std::thread(&BridgeChannel::receiveLoop, this);
    ROS_INFO("[two_uav_bridge] %s -> %s on port %d", tx_topic.c_str(),
             rx_topic.c_str(), local_port);
  }

  ~BridgeChannel() {
    running_ = false;
    subscriber_.shutdown();
    sender_->close();
    receiver_->close();
    if (receive_thread_.joinable()) receive_thread_.join();
  }

 private:
  void send(const typename Message::ConstPtr& msg) {
    const ros::Time now = ros::Time::now();
    if (max_hz_ > 0.0 && !last_send_.isZero() &&
        (now - last_send_).toSec() < 1.0 / max_hz_) return;
    const uint32_t length = static_cast<uint32_t>(ros::serialization::serializationLength(*msg));
    if (length == 0 || length > max_bytes_) {
      ROS_ERROR_THROTTLE(1.0, "[two_uav_bridge] drop %u-byte message (limit %u)",
                         length, max_bytes_);
      return;
    }
    std::vector<uint8_t> bytes(length);
    ros::serialization::OStream stream(bytes.data(), bytes.size());
    ros::serialization::serialize(stream, *msg);
    zmqpp::message wire;
    wire << length;
    wire.add_raw(bytes.data(), bytes.size());
    try {
      sender_->send(wire, true);
      last_send_ = now;
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "[two_uav_bridge] send failed: %s", error.what());
    }
  }

  void receiveLoop() {
    while (running_ && ros::ok()) {
      zmqpp::message wire;
      try {
        if (!receiver_->receive(wire, true)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }
        uint32_t length = 0;
        wire >> length;
        if (length == 0 || length > max_bytes_ || wire.parts() < 2 ||
            wire.size(1) != length) {
          ROS_WARN_THROTTLE(1.0, "[two_uav_bridge] reject malformed packet");
          continue;
        }
        const uint8_t* bytes = static_cast<const uint8_t*>(wire.raw_data(1));
        Message msg;
        ros::serialization::IStream stream(const_cast<uint8_t*>(bytes), length);
        ros::serialization::deserialize(stream, msg);
        publisher_.publish(msg);
      } catch (const std::exception& error) {
        if (running_) ROS_WARN_THROTTLE(1.0, "[two_uav_bridge] receive failed: %s", error.what());
      }
    }
  }

  const double max_hz_;
  const uint32_t max_bytes_;
  std::atomic<bool> running_{true};
  ros::Time last_send_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::unique_ptr<zmqpp::socket> sender_;
  std::unique_ptr<zmqpp::socket> receiver_;
  std::thread receive_thread_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "two_uav_swarm_bridge");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  std::string local_ip;
  std::string peer_ip;
  std::string tx_prefix;
  std::string rx_prefix;
  int port_base;
  int peer_port_base;
  int state_hz;
  int data_hz;
  int max_packet_kib;
  private_nh.param<std::string>("local_ip", local_ip, "0.0.0.0");
  private_nh.param<std::string>("peer_ip", peer_ip, "");
  private_nh.param<std::string>("tx_prefix", tx_prefix, "/two_uav/tx");
  private_nh.param<std::string>("rx_prefix", rx_prefix, "/two_uav/rx");
  private_nh.param("port_base", port_base, 31000);
  private_nh.param("peer_port_base", peer_port_base, port_base);
  private_nh.param("state_hz", state_hz, 20);
  private_nh.param("data_hz", data_hz, 5);
  private_nh.param("max_packet_kib", max_packet_kib, 512);
  if (peer_ip.empty()) {
    ROS_FATAL("[two_uav_bridge] peer_ip is required");
    return 2;
  }
  const uint32_t max_bytes = static_cast<uint32_t>(std::max(16, max_packet_kib) * 1024);
  zmqpp::context_t context;

  BridgeChannel<prometheus_two_uav_coverage_search::SwarmState> state(
      nh, context, tx_prefix + "/state", rx_prefix + "/state", local_ip, peer_ip,
      port_base, peer_port_base, state_hz, max_bytes);
  BridgeChannel<prometheus_two_uav_coverage_search::SwarmTrajectory> trajectory(
      nh, context, tx_prefix + "/trajectory", rx_prefix + "/trajectory", local_ip, peer_ip,
      port_base + 1, peer_port_base + 1, state_hz, max_bytes);
  BridgeChannel<prometheus_two_uav_coverage_search::SwarmFrontierArray> frontier(
      nh, context, tx_prefix + "/frontier", rx_prefix + "/frontier", local_ip, peer_ip,
      port_base + 2, peer_port_base + 2, data_hz, max_bytes);
  BridgeChannel<prometheus_two_uav_coverage_search::SwarmAuctionManifest> auction_manifest(
      nh, context, tx_prefix + "/auction_manifest", rx_prefix + "/auction_manifest",
      local_ip, peer_ip, port_base + 7, peer_port_base + 7, data_hz, max_bytes);
  BridgeChannel<prometheus_two_uav_coverage_search::SwarmAuctionAssignment> auction_assignment(
      nh, context, tx_prefix + "/auction_assignment", rx_prefix + "/auction_assignment",
      // Proposal and final are consecutive protocol events. Dropping the
      // second one deadlocks both coordinators while they wait for peer_final.
      local_ip, peer_ip, port_base + 8, peer_port_base + 8, 0, max_bytes);
  BridgeChannel<prometheus_two_uav_coverage_search::SwarmAuctionTaskSet> auction_task_set(
      nh, context, tx_prefix + "/auction_task_set", rx_prefix + "/auction_task_set",
      local_ip, peer_ip, port_base + 10, peer_port_base + 10, data_hz, max_bytes);
  BridgeChannel<prometheus_two_uav_coverage_search::SwarmFrontierTransferAck> frontier_transfer_ack(
      nh, context, tx_prefix + "/frontier_transfer_ack", rx_prefix + "/frontier_transfer_ack",
      local_ip, peer_ip, port_base + 9, peer_port_base + 9, data_hz, max_bytes);
  BridgeChannel<prometheus_two_uav_coverage_search::SwarmTaskArray> task(
      nh, context, tx_prefix + "/task", rx_prefix + "/task", local_ip, peer_ip,
      port_base + 3, peer_port_base + 3, data_hz, max_bytes);
  BridgeChannel<prometheus_two_uav_coverage_search::SwarmMapChunk> map_chunk(
      nh, context, tx_prefix + "/map_chunk", rx_prefix + "/map_chunk", local_ip, peer_ip,
      // A 4 Hz update is a burst of independent chunks, not one message.
      // Buffer the full burst and never rate-limit individual chunks.
      port_base + 4, peer_port_base + 4, 0, max_bytes, 512);
  BridgeChannel<prometheus_two_uav_coverage_search::SwarmMapRequest> map_request(
      nh, context, tx_prefix + "/map_request", rx_prefix + "/map_request", local_ip, peer_ip,
      port_base + 5, peer_port_base + 5, 0, max_bytes, 128);
  ros::spin();
  return 0;
}
