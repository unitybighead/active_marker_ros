#include "active_marker/color_sub.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "../../active_marker_lib/include/uart.hpp"
#include "active_marker/uart_proto.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace active_marker {

void ColorSubNode::init() {
  const auto qos = rclcpp::QoS(1).reliable();
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
  color_is_setting_subscription_ = this->create_subscription<BoolMsg>(
      "color_is_setting", qos,
      std::bind(&ColorSubNode::set_color_is_setting, this,
                std::placeholders::_1));
  timer_ = this->create_wall_timer(1000ms / update_hz_,
                                   std::bind(&ColorSubNode::update, this));
  no_recv_count_ = 0;
  color_init();

  uart_.start_receive();
}

void ColorSubNode::color_init() {
  if (!(this->get_parameter("pink_.r", pink_.r) &&
        this->get_parameter("pink_.g", pink_.g) &&
        this->get_parameter("pink_.b", pink_.b))) {
    pink_ = {255, 0, 50};
  }

  if (!(this->get_parameter("green_.r", green_.r) &&
        this->get_parameter("green_.g", green_.g) &&
        this->get_parameter("green_.b", green_.b))) {
    green_ = {0, 255, 0};
  }

  if (!(this->get_parameter("blue_.r", blue_.r) &&
        this->get_parameter("blue_.g", blue_.g) &&
        this->get_parameter("blue_.b", blue_.b))) {
    blue_ = {0, 0, 255};
  }

  if (!(this->get_parameter("yellow_.r", yellow_.r) &&
        this->get_parameter("yellow_.g", yellow_.g) &&
        this->get_parameter("yellow_.b", yellow_.b))) {
    yellow_ = {255, 255, 0};
  }
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

void ColorSubNode::set_color_is_setting(BoolMsg::SharedPtr bool_msg) {
  color_is_setting_ = bool_msg->data;
  no_recv_count_ = 0;
}

void ColorSubNode::update() {
  no_recv_count_++;
  RCLCPP_INFO(this->get_logger(), "Pink: %d %d %d", pink_.r, pink_.g, pink_.b);
  RCLCPP_INFO(this->get_logger(), "Green: %d %d %d", green_.r, green_.g,
              green_.b);
  RCLCPP_INFO(this->get_logger(), "Blue: %d %d %d", blue_.r, blue_.g, blue_.b);
  RCLCPP_INFO(this->get_logger(), "Yellow: %d %d %d", yellow_.r, yellow_.g,
              yellow_.b);
  uint8_t b_data[] = {static_cast<uint8_t>(UartCommand::BLUE),
                      static_cast<uint8_t>(blue_.r),
                      static_cast<uint8_t>(blue_.g),
                      static_cast<uint8_t>(blue_.b),
                      0,
                      0,
                      0,
                      0};
  uint8_t y_data[] = {static_cast<uint8_t>(UartCommand::YELLLOW),
                      static_cast<uint8_t>(yellow_.r),
                      static_cast<uint8_t>(yellow_.g),
                      static_cast<uint8_t>(yellow_.b),
                      0,
                      0,
                      0,
                      0};
  uint8_t p_data[] = {static_cast<uint8_t>(UartCommand::PINK),
                      static_cast<uint8_t>(pink_.r),
                      static_cast<uint8_t>(pink_.g),
                      static_cast<uint8_t>(pink_.b),
                      0,
                      0,
                      0,
                      0};
  uint8_t g_data[] = {static_cast<uint8_t>(UartCommand::GREEN),
                      static_cast<uint8_t>(green_.r),
                      static_cast<uint8_t>(green_.g),
                      static_cast<uint8_t>(green_.b),
                      0,
                      0,
                      0,
                      0};
  std::vector<std::uint8_t> b_trans(
      b_data, b_data + sizeof(b_data) / sizeof(b_data[0]));
  std::vector<std::uint8_t> y_trans(
      y_data, y_data + sizeof(y_data) / sizeof(y_data[0]));
  std::vector<std::uint8_t> p_trans(
      p_data, p_data + sizeof(p_data) / sizeof(p_data[0]));
  std::vector<std::uint8_t> g_trans(
      g_data, g_data + sizeof(g_data) / sizeof(g_data[0]));
  uart_.start_transmit(b_trans);
  uart_.start_transmit(y_trans);
  uart_.start_transmit(p_trans);
  uart_.start_transmit(g_trans);

  // uart_.transmit(b_data, sizeof(b_data));
  // uart_.transmit(y_data, sizeof(y_data));
  // uart_.transmit(p_data, sizeof(p_data));
  // uart_.transmit(g_data, sizeof(g_data));
}

}  // namespace active_marker
