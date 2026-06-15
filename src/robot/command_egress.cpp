#include "robot/command_egress.hpp"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace {
constexpr std::chrono::milliseconds kControlPeriod{20};  // 50 Hz
}  // namespace

namespace a2 {
namespace bridge {

void A2CommandPublisher::init(rclcpp::Node* node) {
  node_ = node;
  sport_client_.SetTimeout(25.0f);
  sport_client_.Init();
  setupSubscribers();
  setupTimers();
}

void A2CommandPublisher::setupSubscribers() {
  mode_sub_ = node_->create_subscription<a2_interfaces::msg::OperatingMode>(
    "/a2/mode", rclcpp::QoS(10),
    [this](const a2_interfaces::msg::OperatingMode::SharedPtr msg) { modeCallback(msg); });

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmdVelCallback(msg); });
}

void A2CommandPublisher::setupTimers() {
  timer_ = node_->create_timer(kControlPeriod, [this]() { controlLoop(); });
}

void A2CommandPublisher::modeCallback(
  const a2_interfaces::msg::OperatingMode::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!mode_fsm_.mode_transition(static_cast<OpMode>(msg->mode))) {
    RCLCPP_WARN(node_->get_logger(), "Invalid mode transition to %d", msg->mode);
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Mode Requested: %d", msg->mode);
}

void A2CommandPublisher::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!mode_fsm_.set_cmd_vel(msg->linear.x, msg->linear.y, msg->angular.z)) {
    RCLCPP_WARN(node_->get_logger(), "Incompatible op mode, zeroing velocity");
  }
}

void A2CommandPublisher::controlLoop() {
  OpMode req_mode;
  std::array<float, 3> req_vel;
  bool mode_changed = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::tie(req_mode, mode_changed) = mode_fsm_.get_mode();
    req_vel = mode_fsm_.get_cmd_vel();
  }

  if (req_mode != OpMode::VELOCITY_MOVE && !mode_changed) {
    // Nothing changed when not in velocity move, so noop
    return;
  }

  switch (req_mode) {
    case OpMode::ESTOP:
      RCLCPP_DEBUG(node_->get_logger(), "Mode: ESTOP/DAMP");
      sport_client_.Damp();
      break;
    case OpMode::STAND_DOWN:
      RCLCPP_DEBUG(node_->get_logger(), "Mode: STAND_DOWN");
      sport_client_.StandDown();
      break;
    case OpMode::STAND_UP:
      RCLCPP_DEBUG(node_->get_logger(), "Mode: STAND_UP");
      sport_client_.StandUp();
      break;
    case OpMode::BALANCE_STAND:
      RCLCPP_DEBUG(node_->get_logger(), "Mode: BALANCE_STAND");
      sport_client_.BalanceStand();
      break;
    case OpMode::VELOCITY_MOVE:
      RCLCPP_DEBUG(node_->get_logger(), "Mode: VELOCITY_MOVE");
      sport_client_.Move(req_vel[0], req_vel[1], req_vel[2]);
      break;
    case OpMode::FREE:
      RCLCPP_DEBUG(node_->get_logger(), "Mode: FREE");
      sport_client_.StopMove();
      break;
    default:
      RCLCPP_WARN(node_->get_logger(), "Unknown mode requested, ignoring");
      break;
  }
}

}  // namespace bridge
}  // namespace a2
