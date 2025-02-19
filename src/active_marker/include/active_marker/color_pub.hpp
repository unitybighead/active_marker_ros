#ifndef COLOR_PUB_HPP_
#define COLOR_PUB_HPP_

#include "active_marker_msgs/msg/rgb.hpp"
#include "rclcpp/rclcpp.hpp"

namespace active_marker {

class ColorPubNode : public rclcpp::Node {
 public:
  ColorPubNode()
      : Node("color_pub"),
        update_hz_(this->declare_parameter<int>("update_hz", 60)) {
    init();
  }

 private:
  using ColorMsg = active_marker_msgs::msg::RGB;

  const std::size_t update_hz_;
  rclcpp::Publisher<ColorMsg>::SharedPtr p_publisher_;
  rclcpp::Publisher<ColorMsg>::SharedPtr g_publisher_;
  rclcpp::Publisher<ColorMsg>::SharedPtr b_publisher_;
  rclcpp::Publisher<ColorMsg>::SharedPtr y_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void init();
  void update();
};
}  // namespace active_marker

#endif  // COLOR_PUB_HPP_