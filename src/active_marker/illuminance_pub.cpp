#include "active_marker/illuminance_pub.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace active_marker::illuminance_pub {

void IlluminancePubNode::init() {
  const auto qos = rclcpp::QoS(1).best_effort();
  illuminance_publisher_ =
      this->create_publisher<IlluminanceMsg>("illuminance", qos);
  timer_ = this->create_wall_timer(
      1000ms / update_hz_, std::bind(&IlluminancePubNode::update, this));
}

void IlluminancePubNode::update() {}
}  // namespace active_marker::illuminance_pub