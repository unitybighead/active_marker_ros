#ifndef ILLUMINANCE_SUB_HPP_
#define ILLUMINANCE_SUB_HPP_

#include "../../active_marker_lib/include/uart.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace active_marker {
class IlluminanceSubNode : public rclcpp::Node {
 public:
  template <class... Args>
  IlluminanceSubNode(Args... args)
      : Node("illuminance_sub", "/am", args...),
        update_hz_(this->declare_parameter<int>("update_hz", 60)),
        uart_("/dev/ttyTHS2", B115200) {
    init();
  }

 private:
  using IlluminanceMsg = std_msgs::msg::UInt16;

  const std::size_t update_hz_;
  rclcpp::Subscription<IlluminanceMsg>::SharedPtr illuminance_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  uint16_t illuminance_;
  std::size_t no_recv_count_;
  lib::Uart uart_;

  void init();
  void set_illuminance(IlluminanceMsg::SharedPtr illuminance_msg);
  void update();
};
}  // namespace active_marker

#endif