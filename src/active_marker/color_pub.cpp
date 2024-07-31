#include "active_marker/color_pub.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace active_marker::color_pub {

void ColorPubNode::init() {
  const auto qos = rclcpp::QoS(1).best_effort();
  p_publisher_ = this->create_publisher<ColorMsg>("pink", qos);
  g_publisher_ = this->create_publisher<ColorMsg>("green", qos);
  b_publisher_ = this->create_publisher<ColorMsg>("blue", qos);
  y_publisher_ = this->create_publisher<ColorMsg>("yellow", qos);
  timer_ = this->create_wall_timer(1000ms / update_hz_,
                                   std::bind(&ColorPubNode::update, this));
}

void ColorPubNode::update() {}
}  // namespace active_marker::color_pub