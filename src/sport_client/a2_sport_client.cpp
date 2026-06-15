#include "sport_client/a2_sport_client.hpp"
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include "a2_interfaces/msg/operating_mode.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace {
constexpr std::chrono::milliseconds kControlPeriod{20}; // 50 Hz
// TODO: Move these to ros params?
constexpr float kMaxVelX{0.15};  // m/s
constexpr float kMaxVelY{0.1};   // m/s
constexpr float kMaxYawRate{0.1}; // rad/s
} // namespace

using namespace a2::bridge;

A2SportClientManager::A2SportClientManager() : node_(nullptr), mode_fsm_(kMaxVelX, kMaxVelY, kMaxYawRate) {}

void A2SportClientManager::init(rclcpp::Node *node) {
  node_ = node;
  sport_client_.SetTimeout(5.0f);
  sport_client_.Init();

  setupSubscribers();
  setupTimers();
}

void A2SportClientManager::setupSubscribers() {
  mode_sub_ = node_->create_subscription<a2_interfaces::msg::OperatingMode>("/mode", rclcpp::QoS(10), [this](a2_interfaces::msg::OperatingMode::SharedPtr msg){
    this->modeCallback(msg);
  });

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10), [this](geometry_msgs::msg::Twist::SharedPtr msg){
    this->cmdVelCallback(msg);
  });
}

void A2SportClientManager::setupTimers() {
  timer_ = node_->create_timer(kControlPeriod, [this]() {
      this->controlLoop();
  });
}

void A2SportClientManager::modeCallback(const a2_interfaces::msg::OperatingMode::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!mode_fsm_.mode_transition(static_cast<OpMode>(msg->mode))) {
    RCLCPP_WARN(node_->get_logger(), "Invalid Transition to %d", msg->mode);
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Mode Requested: %d", msg->mode);
}

void A2SportClientManager::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!mode_fsm_.set_cmd_vel(msg->linear.x, msg->linear.y, msg->angular.z)) {
    RCLCPP_WARN(node_->get_logger(), "Incompatible Op Mode, Zero Velocity");
  }
}

void A2SportClientManager::controlLoop() {
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
      RCLCPP_DEBUG(node_->get_logger(), "Mode: STAND DOWN");
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
      RCLCPP_WARN(node_->get_logger(), "Invalid Mode Requested. Ignoring");
      break;
  }
}
