#ifndef UNITREE_BRIDGE_H_
#define UNITREE_BRIDGE_H_

#include <rclcpp/node.hpp>
#include <unitree/robot/a2/sport/sport_client.hpp>
#include "interfaces.hpp"

#ifdef A2_MODE_SIM
#include <builtin_interfaces/msg/time.hpp>
#include <mutex>
#endif

namespace a2 {
namespace bridge {

class A2BridgeBase {
public:
  virtual ~A2BridgeBase() = default;
};

// ─── Robot bridge ─────────────────────────────────────────────────────────────
// DDS → ROS: joint states + IMU + sport mode.
// Odometry / localisation from DLIO/SEAM pipeline
#ifndef A2_MODE_SIM
class A2RobotBridge : public A2BridgeBase {
public:
  explicit A2RobotBridge(rclcpp::Node* node);

private:
  unitree::robot::a2::SportClient sport_client_;
  LowStateTopic low_state_topic_;
  SportStateTopic sport_state_topic_;
};
#endif  // !A2_MODE_SIM

// ─── Sim bridge ───────────────────────────────────────────────────────────────
// DDS → ROS: joint states + IMU + sport mode + ground-truth odometry + TF.
// ROS → ROS: camera image + registered lidar scan.
#ifdef A2_MODE_SIM
class A2SimBridge : public A2BridgeBase {
public:
  explicit A2SimBridge(rclcpp::Node* node);

private:
  // Publishers
  LowCmdTopic low_cmd_pub_topic_;

  // Subscribers
  LowStateTopic low_state_topic_;
  SportStateTopic sport_state_topic_;
  SimCameraTopic camera_topic_;
  FrontLidarTopic front_lidar_topic_;
  RearLidarTopic rear_lidar_topic_;

  // Written by LowStateTopic callback, read by SportStateTopic callback.
  // This is to maintain consistency between the state estimation and raw sensor timestamps
  // Both run on separate Unitree DDS threads — protected by cache_mutex_.
  std::mutex cache_mutex_;
  float cached_quat_[4] = {1, 0, 0, 0};
  float cached_ang_vel_[3] = {0, 0, 0};
  builtin_interfaces::msg::Time cached_stamp_{};
};

#endif  // A2_MODE_SIM

}  // namespace bridge
}  // namespace a2

#endif /* UNITREE_BRIDGE_H_ */
