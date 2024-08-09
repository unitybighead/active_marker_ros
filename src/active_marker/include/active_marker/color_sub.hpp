#ifndef COLOR_SUB_HPP_
#define COLOR_SUB_HPP_

#include "../../active_marker_lib/include/uart.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace active_marker {
class ColorSubNode : public rclcpp::Node {
 public:
  template <class... Args>
  ColorSubNode(Args... args)
      : Node("color_sub", "/am", args...),
        update_hz_(this->declare_parameter<int>("update_hz", 10)),
        uart_("/dev/ttyTHS2", B38400) {
    init();
  }

  typedef struct {
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
    std::uint8_t a;
  } RGBA;

 private:
  using ColorMsg = std_msgs::msg::ColorRGBA;

  const std::size_t update_hz_;
  rclcpp::Subscription<ColorMsg>::SharedPtr p_subscription_;
  rclcpp::Subscription<ColorMsg>::SharedPtr g_subscription_;
  rclcpp::Subscription<ColorMsg>::SharedPtr b_subscription_;
  rclcpp::Subscription<ColorMsg>::SharedPtr y_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  lib::Uart uart_;

  RGBA pink_, green_, blue_, yellow_;

  std::size_t no_recv_count_;

  void init();
  void color_init();
  void set_pink(ColorMsg::SharedPtr color_msg);
  void set_green(ColorMsg::SharedPtr color_msg);
  void set_blue(ColorMsg::SharedPtr color_msg);
  void set_yellow(ColorMsg::SharedPtr color_msg);
  void update();
};
}  // namespace active_marker

#endif  // COLOR_SUB_HPP_