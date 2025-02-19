#ifndef COLOR_SUB_HPP_
#define COLOR_SUB_HPP_

#include "../../active_marker_lib/include/uart.hpp"
#include "active_marker_msgs/msg/rgb.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace active_marker {
class ColorSubNode : public rclcpp::Node {
 public:
  template <class... Args>
  ColorSubNode(Args... args)
      : Node("color_sub", "/am", args...),
        update_hz_(this->declare_parameter<int>("update_hz", 60)),
        uart_("/dev/ttyTHS2", B115200) {
    init();
  }

  typedef struct {
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
  } RGB;

 private:
  using ColorMsg = active_marker_msgs::msg::RGB;
  using BoolMsg = std_msgs::msg::Bool;

  const std::size_t update_hz_;
  rclcpp::Subscription<ColorMsg>::SharedPtr p_subscription_;
  rclcpp::Subscription<ColorMsg>::SharedPtr g_subscription_;
  rclcpp::Subscription<ColorMsg>::SharedPtr b_subscription_;
  rclcpp::Subscription<ColorMsg>::SharedPtr y_subscription_;
  rclcpp::Subscription<BoolMsg>::SharedPtr color_is_setting_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  lib::Uart uart_;

  RGB pink_, green_, blue_, yellow_;
  bool color_is_setting_ = false;

  std::size_t no_recv_count_;

  void init();
  void color_init();
  void set_pink(ColorMsg::SharedPtr color_msg);
  void set_green(ColorMsg::SharedPtr color_msg);
  void set_blue(ColorMsg::SharedPtr color_msg);
  void set_yellow(ColorMsg::SharedPtr color_msg);
  void set_color_is_setting(BoolMsg::SharedPtr bool_msg);
  void update();
};
}  // namespace active_marker

#endif  // COLOR_SUB_HPP_