#ifndef ILLUMINANCE_PUB_HPP_
#define ILLUMINANCE_PUB_HPP_

#include "../active_marker_lib/include/udp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace active_marker {
class IlluminancePubNode : public rclcpp::Node {
 public:
  template <class... Args>
  IlluminancePubNode(Args... args)
      : Node("illuminance_pub", "/am", args...),
        update_hz_(this->declare_parameter<int>("update_hz", 60)),
        udp_receiver_(50007, std::bind(&IlluminancePubNode::pub_illuminance,
                                       this, std::placeholders::_1)) {
    init();
  }

 private:
  using IlluminanceMsg = std_msgs::msg::UInt16;

  const std::size_t update_hz_;
  rclcpp::Publisher<IlluminanceMsg>::SharedPtr illuminance_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  uint16_t illuminance_;
  lib::UdpReceiver udp_receiver_;

  void init();
  void pub_illuminance(uint16_t illuminance_value);
  void update();
};
}  // namespace active_marker

#endif  // ILLUMINANCE_PUB_HPP_