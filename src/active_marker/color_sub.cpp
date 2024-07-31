#include "active_marker/color_sub.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace active_marker::color_sub {

void ColorSubNode::init() {
  const auto qos = rclcpp::QoS(1).best_effort();
  p_subscription_ = this->create_subscription<ColorMsg>(
      "pink", qos,
      std::bind(&ColorSubNode::set_pink, this, std::placeholders::_1));
  g_subscription_ = this->create_subscription<ColorMsg>(
      "green", qos,
      std::bind(&ColorSubNode::set_green, this, std::placeholders::_1));
  b_subscription_ = this->create_subscription<ColorMsg>(
      "blue", qos,
      std::bind(&ColorSubNode::set_blue, this, std::placeholders::_1));
  y_subscription_ = this->create_subscription<ColorMsg>(
      "yellow", qos,
      std::bind(&ColorSubNode::set_yellow, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(1000ms / update_hz_,
                                   std::bind(&ColorSubNode::update, this));
  no_recv_count_ = 0;
  pink_.a = 0;
  green_.a = 0;
  blue_.a = 0;
  yellow_.a = 0;
}

void ColorSubNode::set_pink(ColorMsg::SharedPtr color_msg) {
  pink_.r = color_msg->r;
  pink_.g = color_msg->g;
  pink_.b = color_msg->b;
  no_recv_count_ = 0;
}

void ColorSubNode::set_green(ColorMsg::SharedPtr color_msg) {
  green_.r = color_msg->r;
  green_.g = color_msg->g;
  green_.b = color_msg->b;
  no_recv_count_ = 0;
}

void ColorSubNode::set_blue(ColorMsg::SharedPtr color_msg) {
  blue_.r = color_msg->r;
  blue_.g = color_msg->g;
  blue_.b = color_msg->b;
  no_recv_count_ = 0;
}

void ColorSubNode::set_yellow(ColorMsg::SharedPtr color_msg) {
  yellow_.r = color_msg->r;
  yellow_.g = color_msg->g;
  yellow_.b = color_msg->b;
  no_recv_count_ = 0;
}

void ColorSubNode::update() {
  no_recv_count_++;
  if (no_recv_count_ > update_hz_ * 3) {
    RCLCPP_ERROR(this->get_logger(), "no receive");
  } else {
    RCLCPP_INFO(this->get_logger(), "runnning");
  }
  RCLCPP_INFO(this->get_logger(), "Pink: %d %d %d", pink_.r, pink_.g, pink_.b);
  RCLCPP_INFO(this->get_logger(), "Green: %d %d %d", green_.r, green_.g,
              green_.b);
  RCLCPP_INFO(this->get_logger(), "Blue: %d %d %d", blue_.r, blue_.g, blue_.b);
  RCLCPP_INFO(this->get_logger(), "Yellow: %d %d %d", yellow_.r, yellow_.g,
              yellow_.b);
}

}  // namespace active_marker::color_sub