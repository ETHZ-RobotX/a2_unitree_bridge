#include "robot/command_egress.hpp"

#include <chrono>
#include <rclcpp/logging.hpp>

namespace {
// TODO: expose via ROS params
constexpr float kMaxVelX{0.15f};
constexpr float kMaxVelY{0.1f};
constexpr float kMaxYawRate{0.1f};
constexpr std::chrono::milliseconds kControlPeriod{20};
constexpr int64_t kCmdVelMaxAgeNs{500'000'000LL};  // 500 ms

}  // namespace

namespace a2 {
namespace bridge {

A2CommandPublisher::A2CommandPublisher()
    : SafeVelocityRosInterface(kMaxVelX, kMaxVelY, kMaxYawRate, kControlPeriod, kCmdVelMaxAgeNs) {}

void A2CommandPublisher::init(rclcpp::Node* node) {
  sport_client_.SetTimeout(5.0f);
  sport_client_.Init();
  SafeVelocityRosInterface::init(node);
}

void A2CommandPublisher::onControl(utils::OpMode mode, bool mode_changed,
                                   std::array<float, 3> vel) {
  switch (mode) {
    case utils::OpMode::ESTOP:
      RCLCPP_DEBUG(node_->get_logger(), "Mode: ESTOP/DAMP");
      sport_client_.Damp();
      break;
    case utils::OpMode::STAND_DOWN:
      RCLCPP_DEBUG(node_->get_logger(), "Mode: STAND_DOWN");
      sport_client_.StandDown();
      break;
    case utils::OpMode::STAND_UP:
      RCLCPP_DEBUG(node_->get_logger(), "Mode: STAND_UP");
      sport_client_.StandUp();
      break;
    case utils::OpMode::BALANCE_STAND:
      RCLCPP_DEBUG(node_->get_logger(), "Mode: BALANCE_STAND");
      sport_client_.BalanceStand();
      break;
    case utils::OpMode::VELOCITY_MOVE:
      RCLCPP_DEBUG(node_->get_logger(), "Mode: VELOCITY_MOVE");
      sport_client_.Move(vel[0], vel[1], vel[2]);
      break;
    case utils::OpMode::FREE:
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
