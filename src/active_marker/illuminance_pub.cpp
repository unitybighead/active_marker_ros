#include "active_marker/illuminance_pub.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "../../active_marker_lib/include/udp.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace active_marker::illuminance_pub {

void IlluminancePubNode::init() {
  const auto qos = rclcpp::QoS(1).best_effort();
  illuminance_publisher_ =
      this->create_publisher<IlluminanceMsg>("illuminance", qos);
  timer_ = this->create_wall_timer(
      1000ms / update_hz_, std::bind(&IlluminancePubNode::update, this));
  illuminance_ = 0;
}

void IlluminancePubNode::pub_illuminance(uint16_t illuminance_value) {
  auto msg = IlluminanceMsg();
  msg.data = illuminance_value;
  illuminance_ = illuminance_value;
  RCLCPP_INFO(this->get_logger(), "Publishing: %d", msg.data);
  illuminance_publisher_->publish(msg);
}

void IlluminancePubNode::update() {}
}  // namespace active_marker::illuminance_pub