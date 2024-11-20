#include "active_marker/robot_ID.hpp"

#include "active_marker/uart_proto.hpp"

using namespace std::chrono_literals;

namespace active_marker {
RobotIDNode::RobotIDNode()
    : Node("robot_ID", "/am"),
      ID_(this->declare_parameter<int>("ID", 10)),
      team_color_(this->declare_parameter<std::string>("team_color", "yellow")),
      uart_("/dev/tthTHS2", B115200) {
  last_key_subsctiption_ = this->create_subscription<Int16Msg>(
      "last_key", rclcpp::QoS(1).reliable(),
      std::bind(&RobotIDNode::set_team_color, this, std::placeholders::_1));
  timer_ =
      this->create_wall_timer(3000ms, std::bind(&RobotIDNode::update, this));
}

void RobotIDNode::set_team_color(Int16Msg::SharedPtr msg) {
  if (ID_ == 16) {  // debug only
    char key = (char)msg->data;
    switch (key) {
      case 'b':
      case 'B':
        team_color_ = "blue";
        break;
      case 'y':
      case 'Y':
        team_color_ = "yellow";
        break;
      default:
        break;
    }
  }
}

void RobotIDNode::update() {
  uint8_t data[8] = {0};
  data[0] = static_cast<uint8_t>(UartCommand::ID);
  if (ID_ < 0 || ID_ > 16) {
    RCLCPP_ERROR(this->get_logger(), "Invalid ID");
    return;
  }
  data[1] = static_cast<uint8_t>(ID_);
  if (team_color_ == "blue") {
    data[2] = static_cast<uint8_t>(TeamColor::BLUE);
  } else if (team_color_ == "yellow") {
    data[2] = static_cast<uint8_t>(TeamColor::YELLOW);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid team color");
    return;
  }

  uart_.transmit(data, sizeof(data));
}

}  // namespace active_marker