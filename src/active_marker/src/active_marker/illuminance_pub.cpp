#include "active_marker/illuminance_pub.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "../../active_marker_lib/include/udp.hpp"
#include "active_marker/uart_proto.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace active_marker {

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

uint16_t IlluminancePubNode::serialize_uint16(uint8_t* data) {
  return data[1] << 8 + data[0];
}

void IlluminancePubNode::update() {
  if (!uart_.is_open()) {
    return;
  }
  uint8_t data[3];
  if (!data[0] == static_cast<uint8_t>(UartCommand::ILLUMINANCE)) {
    return;
  }
  pub_illuminance(serialize_uint16(&data[1]));
}
}  // namespace active_marker