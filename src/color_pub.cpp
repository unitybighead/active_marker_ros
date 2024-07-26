#include "active_marker/color_pub.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<color_pub::ColorPubNode>());
  rclcpp::shutdown();
  return 0;
}
