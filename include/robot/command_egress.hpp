#ifndef A2_BRIDGE_ROBOT_CMD_EGRESS_H_
#define A2_BRIDGE_ROBOT_CMD_EGRESS_H_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <unitree/robot/a2/sport/sport_client.hpp>
#include "a2_interfaces/msg/operating_mode.hpp"
#include "common/egress.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "robot/mode_fsm.hpp"

namespace a2 {
namespace bridge {

/*
 * Wraps the Unitree SportClient SDK. Subscribes to /a2/mode and /cmd_vel,
 * runs a 50 Hz control loop that translates the current OpMode into SDK calls.
 *
 * This keeps the set of possible SDK calls low and incorporates safety modes
 * to ensure unwanted behavior from being executed on the robot
 */
class A2CommandPublisher : public EgressType {
public:
  A2CommandPublisher() : mode_fsm_(kMaxVelX, kMaxVelY, kMaxYawRate) {}

  void init(rclcpp::Node* node);

private:
  // TODO: expose via ROS params
  static constexpr float kMaxVelX{0.15f};
  static constexpr float kMaxVelY{0.1f};
  static constexpr float kMaxYawRate{0.1f};
  static constexpr int64_t kCmdVelMaxAgeNs{500'000'000LL};  // 500 ms

  // Initialization
  void setupSubscribers();
  void setupTimers();

  // Callback Functions
  void modeCallback(const a2_interfaces::msg::OperatingMode::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // Control
  void controlLoop();

private:
  // Node for loggers
  rclcpp::Node* node_;

  // Sport Client Specific
  unitree::robot::a2::SportClient sport_client_;

  // ROS
  rclcpp::Subscription<a2_interfaces::msg::OperatingMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State - Need to be accessed via mutex
  ModeFsm mode_fsm_;
  std::mutex state_mutex_;
};

}  // namespace bridge
}  // namespace a2

#endif /* A2_BRIDGE_ROBOT_CMD_EGRESS_H_ */
