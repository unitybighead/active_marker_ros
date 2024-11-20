#include "active_marker/robot_ID.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<active_marker::RobotIDNode>());
  rclcpp::shutdown();
  return 0;
}