#ifndef ROBOT_ID_HPP_
#define ROBOT_ID_HPP_

#include <rclcpp/rclcpp.hpp>

#include "../active_marker_lib/include/uart.hpp"
#include "std_msgs/msg/int16.hpp"

namespace active_marker {
class RobotIDNode : public rclcpp::Node {
 public:
  RobotIDNode();

 private:
  using Int16Msg = std_msgs::msg::Int16;

  rclcpp::Subscription<Int16Msg>::SharedPtr last_key_subsctiption_;
  rclcpp::TimerBase::SharedPtr timer_;

  lib::Uart uart_;
  int ID_;
  std::string team_color_;

  void set_team_color(Int16Msg::SharedPtr);
  void update();
};
}  // namespace active_marker

#endif