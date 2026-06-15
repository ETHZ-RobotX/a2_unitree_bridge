#include "robot/mode_fsm.hpp"

#include <algorithm>
#include <stdexcept>

namespace a2 {
namespace bridge {

static constexpr std::array<std::pair<OpMode, OpMode>, 9> kValidTransitions = {{
  // Reset behaviors
  {OpMode::ESTOP, OpMode::STAND_DOWN},
  {OpMode::FREE, OpMode::STAND_DOWN},
  {OpMode::FREE, OpMode::VELOCITY_MOVE},
  // Normal behaviors
  {OpMode::STAND_DOWN, OpMode::STAND_UP},
  {OpMode::STAND_UP, OpMode::BALANCE_STAND},
  {OpMode::BALANCE_STAND, OpMode::VELOCITY_MOVE},
  {OpMode::VELOCITY_MOVE, OpMode::BALANCE_STAND},
  {OpMode::BALANCE_STAND, OpMode::STAND_UP},
  {OpMode::STAND_UP, OpMode::STAND_DOWN},
}};

ModeFsm::ModeFsm(float max_vel_x, float max_vel_y, float max_yaw_rate)
    : mode_(OpMode::STAND_DOWN),
      mode_changed_(true),  // ensures initial fire of the control loop
      cmd_vel_({0.0f, 0.0f, 0.0f}),
      max_vel_x_(max_vel_x),
      max_vel_y_(max_vel_y),
      max_yaw_rate_(max_yaw_rate) {
  if (max_vel_x_ < 0 || max_vel_y_ < 0 || max_yaw_rate_ < 0) {
    throw std::invalid_argument("Max velocities and yaw rate must be non-negative");
  }
}

bool ModeFsm::mode_transition(OpMode next) {
  if (mode_ == next)
    return true;
  bool allowed = (next == OpMode::ESTOP) || (next == OpMode::FREE) ||
                 std::find(kValidTransitions.begin(), kValidTransitions.end(),
                           std::pair{mode_, next}) != kValidTransitions.end();
  if (!allowed)
    return false;
  mode_ = next;
  mode_changed_ = true;
  reset_cmd_vel();
  return true;
}

inline void ModeFsm::reset_cmd_vel() {
  cmd_vel_.fill(0.0f);
}

bool ModeFsm::set_cmd_vel(float x, float y, float yaw) {
  if (mode_ == OpMode::VELOCITY_MOVE) {
    cmd_vel_[0] = std::clamp(x, -max_vel_x_, max_vel_x_);
    cmd_vel_[1] = std::clamp(y, -max_vel_y_, max_vel_y_);
    cmd_vel_[2] = std::clamp(yaw, -max_yaw_rate_, max_yaw_rate_);
    return true;
  }
  reset_cmd_vel();
  return false;
}

std::pair<OpMode, bool> ModeFsm::get_mode() {
  auto rval = std::make_pair(mode_, mode_changed_);
  mode_changed_ = false;
  return rval;
}

std::array<float, 3> ModeFsm::get_cmd_vel() {
  auto rval = cmd_vel_;
  reset_cmd_vel();
  return rval;
}

}  // namespace bridge
}  // namespace a2
