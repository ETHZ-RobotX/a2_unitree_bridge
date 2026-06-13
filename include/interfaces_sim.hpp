/*
 * Sim topic structs (MuJoCo + Unitree DDS).
 * LowState uses unitree_go (Go2 format from MuJoCo). Publishes joints + IMU +
 * ground-truth odometry + TF + camera image + registered lidar scan.
 */
#ifndef INTERFACES_SIM_H_
#define INTERFACES_SIM_H_

#include <unitree/dds_wrapper/common/Publisher.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/common/thread/thread_decl.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree_go/msg/low_cmd.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "converters_sim.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace a2 {
namespace bridge {

static const rclcpp::QoS kDefaultRosQoS{10};
static const rclcpp::QoS kLowCmdRosQoS{50};
static const int64_t kDefaultDdsQueueLen{1};

// ─── LowCmdTopic Publisher ───────────────────────────────────────────────────
// ROS publisher sends lowcmd ros messages, convert and publish as lowcmd DDS
struct LowCmdTopic {
  using LowCmdDds_t = unitree_go::msg::dds_::LowCmd_;
  using LowCmdRos_t = unitree_go::msg::LowCmd;
  static constexpr const char* dds_topic = "rt/lowcmd";
  static constexpr const char* ros_topic = "/lowcmd";

  unitree::robot::ChannelPublisherPtr<LowCmdDds_t> pub;
  rclcpp::Subscription<LowCmdRos_t>::SharedPtr sub;

  void init(rclcpp::Node* node, const rclcpp::SubscriptionOptions& sub_options) {
    pub.reset(new unitree::robot::ChannelPublisher<LowCmdDds_t>(dds_topic));
    pub->InitChannel();

    sub = node->create_subscription<LowCmdRos_t>(
      ros_topic, kLowCmdRosQoS,
      [this](const LowCmdRos_t::SharedPtr msg) { pub->Write(converters::lowcmd_ros_to_dds(*msg)); },
      sub_options);
  }
};

// ─── LowStateTopic ───────────────────────────────────────────────────────────
// Publishes sim_clock from mujoco
// Publishes /joint_states and /imu/data. Throttled to 200 Hz.
// Writes to the bridge's shared cache (quat, ang_vel, stamp) so SportStateTopic
// can use an authoritative timestamp and orientation from the same LowState
// tick.

struct LowStateTopic {
  using LowStateDds_t = unitree_go::msg::dds_::LowState_;
  using JointState_t = sensor_msgs::msg::JointState;
  using ImuState_t = sensor_msgs::msg::Imu;
  using ClockState_t = rosgraph_msgs::msg::Clock;

  static constexpr const char* dds_topic = "rt/lowstate";
  static constexpr const char* joint_states_topic = "/joint_states";
  static constexpr const char* imu_topic = "/imu/data";
  static constexpr const char* clock_topic = "/clock";

  LowStateDds_t state;
  unitree::robot::ChannelSubscriberPtr<LowStateDds_t> sub;
  rclcpp::Publisher<JointState_t>::SharedPtr joint_pub;
  rclcpp::Publisher<ImuState_t>::SharedPtr imu_pub;
  rclcpp::Publisher<ClockState_t>::SharedPtr clock_pub;
  rclcpp::Time last_pub{0, 0, RCL_ROS_TIME};

  void init(rclcpp::Node* node, float* cached_quat, float* cached_ang_vel,
            builtin_interfaces::msg::Time* cached_stamp, std::mutex* cache_mutex) {
    joint_pub = node->create_publisher<JointState_t>(joint_states_topic, kDefaultRosQoS);
    imu_pub = node->create_publisher<ImuState_t>(imu_topic, kDefaultRosQoS);
    clock_pub = node->create_publisher<ClockState_t>(clock_topic, kDefaultRosQoS);

    sub.reset(new unitree::robot::ChannelSubscriber<LowStateDds_t>(dds_topic));
    sub->InitChannel(
      [this, cached_quat, cached_ang_vel, cached_stamp, cache_mutex](const void* msg) {
        state = *static_cast<const LowStateDds_t*>(msg);
        builtin_interfaces::msg::Time stamp = converters::tick_to_stamp(state.tick());

        ClockState_t clock_msg;
        clock_msg.clock = stamp;
        clock_pub->publish(clock_msg);

        {
          std::lock_guard<std::mutex> lock(*cache_mutex);
          *cached_stamp = stamp;
          cached_quat[0] = state.imu_state().quaternion()[0];
          cached_quat[1] = state.imu_state().quaternion()[1];
          cached_quat[2] = state.imu_state().quaternion()[2];
          cached_quat[3] = state.imu_state().quaternion()[3];
          cached_ang_vel[0] = state.imu_state().gyroscope()[0];
          cached_ang_vel[1] = state.imu_state().gyroscope()[1];
          cached_ang_vel[2] = state.imu_state().gyroscope()[2];
        }

        rclcpp::Time t(stamp);
        if ((t - last_pub).seconds() < 0.005)
          return;
        last_pub = t;

        joint_pub->publish(converters::joint_state(state, stamp));
        imu_pub->publish(converters::imu(state, stamp));
      },
      kDefaultDdsQueueLen);
  }
};

// ─── SportStateTopic ─────────────────────────────────────────────────────────
// Throttled to 50 Hz. Reads cached quat/ang_vel/stamp from LowStateTopic.
// Publishes /odom, /state_estimation, /a2/sport_mode. Broadcasts map→base_link
// TF.

struct SportStateTopic {
  using DdsTopic_t = unitree_go::msg::dds_::SportModeState_;
  static constexpr const char* dds_topic = "rt/sportmodestate";
  static constexpr const char* sport_mode_topic = "/a2/sport_mode";
  static constexpr const char* odom_topic = "/odom";
  static constexpr const char* state_est_topic = "/state_estimation";

  DdsTopic_t state;
  unitree::robot::ChannelSubscriberPtr<DdsTopic_t> sub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_est_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Time last_pub{0, 0, RCL_ROS_TIME};

  void init(rclcpp::Node* node, const float* cached_quat, const float* cached_ang_vel,
            const builtin_interfaces::msg::Time* cached_stamp, std::mutex* cache_mutex) {
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    mode_pub = node->create_publisher<std_msgs::msg::UInt8>(sport_mode_topic, kDefaultRosQoS);
    odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(odom_topic, kDefaultRosQoS);
    state_est_pub =
      node->create_publisher<nav_msgs::msg::Odometry>(state_est_topic, kDefaultRosQoS);

    sub.reset(new unitree::robot::ChannelSubscriber<DdsTopic_t>(dds_topic));
    sub->InitChannel(
      [this, cached_quat, cached_ang_vel, cached_stamp, cache_mutex](const void* msg) {
        state = *static_cast<const DdsTopic_t*>(msg);

        // Copy cache under lock to avoid data race with LowStateTopic's DDS
        // thread.
        builtin_interfaces::msg::Time stamp;
        float quat[4];
        float ang_vel[3];
        {
          std::lock_guard<std::mutex> lock(*cache_mutex);
          stamp = *cached_stamp;
          quat[0] = cached_quat[0];
          quat[1] = cached_quat[1];
          quat[2] = cached_quat[2];
          quat[3] = cached_quat[3];
          ang_vel[0] = cached_ang_vel[0];
          ang_vel[1] = cached_ang_vel[1];
          ang_vel[2] = cached_ang_vel[2];
        }

        rclcpp::Time t(stamp);
        if ((t - last_pub).seconds() < 0.02)
          return;
        last_pub = t;

        auto odom = converters::odometry(state, quat, ang_vel, stamp);
        odom_pub->publish(odom);
        state_est_pub->publish(odom);
        mode_pub->publish(converters::sport_mode(state));

        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = state.position()[0];
        tf.transform.translation.y = state.position()[1];
        tf.transform.translation.z = state.position()[2];
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster->sendTransform(tf);
      },
      kDefaultDdsQueueLen);
  }
};

// ─── SimCameraTopic ───────────────────────────────────────────────────────────
// MuJoCo encodes the front camera RGB image inside a PointCloud2 DDS message.
// Converts to sensor_msgs::Image + publishes hardcoded CameraInfo intrinsics.
struct SimCameraTopic {
  using PointCloudDds_t = sensor_msgs::msg::dds_::PointCloud2_;
  static constexpr const char* dds_topic = "rt/mujoco/front_camera_pointcloud";
  static constexpr const char* image_pub_topic = "/camera/image_raw";
  static constexpr const char* info_pub_topic = "/camera/camera_info";

  unitree::robot::ChannelSubscriberPtr<PointCloudDds_t> sub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;

  void init(rclcpp::Node* node) {
    image_pub = node->create_publisher<sensor_msgs::msg::Image>(image_pub_topic, kDefaultRosQoS);
    info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(info_pub_topic, kDefaultRosQoS);
    sub.reset(new unitree::robot::ChannelSubscriber<PointCloudDds_t>(dds_topic));
    sub->InitChannel(
      [this](const void* msg) {
        auto state = *static_cast<const PointCloudDds_t*>(msg);
        image_pub->publish(converters::camera_image(state));
        info_pub->publish(converters::camera_info(state.header().stamp()));
      },
      kDefaultDdsQueueLen);
  }
};

// ─── SimLidarTopic ────────────────────────────────────────────────────────────
// Templated to allow multiple lidar scans raw topic published as a ROS topic
// Registered scan gets published from a2_utils/registered_scan_pub

template <typename Type>
struct SimLidarTopic {
  using PointCloudDds_t = sensor_msgs::msg::dds_::PointCloud2_;
  using PointCloudRos_t = sensor_msgs::msg::PointCloud2;
  static constexpr double kTfLagSec = 0.025;

  unitree::robot::ChannelSubscriberPtr<PointCloudDds_t> sub;
  rclcpp::Publisher<PointCloudRos_t>::SharedPtr raw_pub;

  void init(rclcpp::Node* node) {
    raw_pub = node->create_publisher<PointCloudRos_t>(Type::ros_topic, kDefaultRosQoS);
    sub.reset(new unitree::robot::ChannelSubscriber<PointCloudDds_t>(Type::dds_topic));
    sub->InitChannel(
      [this, node](const void* msg) {
        auto state = *static_cast<const PointCloudDds_t*>(msg);
        PointCloudRos_t raw_cloud = converters::pointcloud(state);
        raw_pub->publish(raw_cloud);
      },
      kDefaultDdsQueueLen);
  }
};

struct FrontLidarTraits {
  static constexpr const char* dds_topic = "rt/mujoco/front_lidar";
  static constexpr const char* ros_topic = "/mujoco/front_lidar";
};
using FrontLidarTopic = SimLidarTopic<FrontLidarTraits>;

struct RearLidarTraits {
  static constexpr const char* dds_topic = "rt/mujoco/rear_lidar";
  static constexpr const char* ros_topic = "/mujoco/rear_lidar";
};
using RearLidarTopic = SimLidarTopic<RearLidarTraits>;

}  // namespace bridge
}  // namespace a2

#endif /* INTERFACES_SIM_H_ */
