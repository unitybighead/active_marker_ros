#include "active_marker/illuminance_sub.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace active_marker::illuminance_sub {
void IlluminanceSubNode::init() {
  const auto qos = rclcpp::QoS(1).best_effort();
  illuminance_subscription_ = this->create_subscription<IlluminanceMsg>(
      "illuminance", qos,
      std::bind(&IlluminanceSubNode::set_illuminance, this,
                std::placeholders::_1));
  timer_ = this->create_wall_timer(
      1000ms / update_hz_, std::bind(&IlluminanceSubNode::update, this));
}

void IlluminanceSubNode::set_illuminance(
    IlluminanceMsg::SharedPtr illuminance_msg) {
  illuminance_ = (uint16_t)illuminance_msg->data;
  no_recv_count_ = 0;
}

void IlluminanceSubNode::update() {
  no_recv_count_++;
  RCLCPP_INFO(this->get_logger(), "%d", illuminance_);
}

}  // namespace active_marker::illuminance_sub