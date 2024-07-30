#ifndef ILLUMINANCE_SUB_HPP_
#define ILLUMINANCE_SUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace illuminance_sub {
class IlluminanceSubNode : public rclcpp::Node {
 public:
  IlluminanceSubNode()
      : Node("illuminance_sub"),
        update_hz_(this->declare_parameter<int>("update_hz", 60)) {
    init();
  }

 private:
  using IlluminanceMsg = std_msgs::msg::Float32;

  const std::size_t update_hz_;
  rclcpp::Subscription<IlluminanceMsg>::SharedPtr illuminance_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  float illuminance_;
  std::size_t no_recv_count_;

  void init();
  void set_illuminance(IlluminanceMsg::SharedPtr illuminance_msg);
  void update();
};
}  // namespace illuminance_sub

#endif