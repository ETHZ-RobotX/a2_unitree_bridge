/*
 * Shared converters — work with any LowState type (unitree_hg or unitree_go)
 * that exposes motor_state() / imu_state() with the same field names.
 * sport_mode() uses unitree_go::SportModeState_ for both sim and robot.
 */
#ifndef A2_BRIDGE_COMMON_CONVERTERS_H_
#define A2_BRIDGE_COMMON_CONVERTERS_H_

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

namespace a2 {
namespace bridge {
namespace converters {

inline const std::vector<std::string> kJointNames = {
  "FR_hip_joint",   "FR_thigh_joint", "FR_calf_joint",  "FL_hip_joint",
  "FL_thigh_joint", "FL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint",
  "RR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint", "RL_calf_joint"};

template <typename LowState_t>
inline sensor_msgs::msg::JointState joint_state(const LowState_t& msg,
                                                const builtin_interfaces::msg::Time& stamp) {
  sensor_msgs::msg::JointState ros_msg;
  ros_msg.header.stamp = stamp;
  ros_msg.name = kJointNames;
  for (size_t i = 0; i < kJointNames.size(); ++i) {
    ros_msg.position.push_back(msg.motor_state()[i].q());
    ros_msg.velocity.push_back(msg.motor_state()[i].dq());
    ros_msg.effort.push_back(msg.motor_state()[i].tau_est());
  }
  return ros_msg;
}

template <typename LowState_t>
inline sensor_msgs::msg::Imu imu(const LowState_t& msg,
                                 const builtin_interfaces::msg::Time& stamp) {
  sensor_msgs::msg::Imu ros_msg;
  ros_msg.header.stamp = stamp;
  ros_msg.header.frame_id = "imu_link";
  ros_msg.orientation.w = msg.imu_state().quaternion()[0];
  ros_msg.orientation.x = msg.imu_state().quaternion()[1];
  ros_msg.orientation.y = msg.imu_state().quaternion()[2];
  ros_msg.orientation.z = msg.imu_state().quaternion()[3];
  ros_msg.angular_velocity.x = msg.imu_state().gyroscope()[0];
  ros_msg.angular_velocity.y = msg.imu_state().gyroscope()[1];
  ros_msg.angular_velocity.z = msg.imu_state().gyroscope()[2];
  ros_msg.linear_acceleration.x = msg.imu_state().accelerometer()[0];
  ros_msg.linear_acceleration.y = msg.imu_state().accelerometer()[1];
  ros_msg.linear_acceleration.z = msg.imu_state().accelerometer()[2];
  return ros_msg;
}

inline std_msgs::msg::UInt8 sport_mode(const unitree_go::msg::dds_::SportModeState_& msg) {
  std_msgs::msg::UInt8 ros_msg;
  ros_msg.data = static_cast<uint8_t>(msg.mode());
  return ros_msg;
}

}  // namespace converters
}  // namespace bridge
}  // namespace a2

#endif /* A2_BRIDGE_COMMON_CONVERTERS_H_ */
