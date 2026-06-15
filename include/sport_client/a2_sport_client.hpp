#ifndef A2_BRIDGE_SPORT_CLIENT_H_
#define A2_BRIDGE_SPORT_CLIENT_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <unitree/robot/a2/sport/sport_client.hpp>

#include "mode_fsm.hpp"
#include "a2_interfaces/msg/operating_mode.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace a2 {
namespace bridge {

/*
 * Custom Sport Client Wrapper on the A2 Sport Client
 * Ingests ROS messages and sends unitree Sport Client SDK commands
 *
 * This keeps the feature set low and incorporates safety modes
 * to ensure unwanted behavior from being executed on the robot
 */
class A2SportClientManager {
public:
  explicit A2SportClientManager();

  void init(rclcpp::Node *node);

private:
  // Initialization
  void setupSubscribers();
  void setupTimers();

  // Callback Functions
  void modeCallback(const a2_interfaces::msg::OperatingMode::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Control
  void controlLoop();

private:
  // Node for loggers
  rclcpp::Node *node_;

  // Sport Client Specific
  unitree::robot::a2::SportClient sport_client_;

  // ROS
  rclcpp::Subscription<a2_interfaces::msg::OperatingMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State - Need to be accessed via mutex
  ModeFsm mode_fsm_;
  std::mutex state_mutex_;
};

}
}


#endif /* A2_BRIDGE_SPORT_CLIENT_H_ */
