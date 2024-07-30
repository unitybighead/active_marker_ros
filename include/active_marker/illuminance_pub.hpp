#ifndef ILLUMINANCE_PUB_HPP_
#define ILLUMINANCE_PUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace illuminance_pub {
class IlluminancePubNode : public rclcpp::Node {
 public:
  IlluminancePubNode()
      : Node("illuminance_pub"),
        update_hz_(this->declare_parameter<int>("update_hz", 60)) {
    init();
  }

 private:
  using IlluminanceMsg = std_msgs::msg::Float32;

  const std::size_t update_hz_;
  rclcpp::Publisher<IlluminanceMsg>::SharedPtr illuminance_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void init();
  void update();
};
}  //  namespace illuminance_pub

#endif  // ILLUMINANCE_PUB_HPP_