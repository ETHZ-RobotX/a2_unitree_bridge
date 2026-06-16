#ifndef A2_BRIDGE_MODE_FSM_H_
#define A2_BRIDGE_MODE_FSM_H_

#include <array>
#include <cstdint>
#include <utility>

namespace a2 {
namespace bridge {

enum class OpMode : uint8_t {
  ESTOP = 0,          // Joints disabled (most emergency)
  STAND_DOWN = 1,     // Stand down, joints locked
  STAND_UP = 2,       // Stands up, joints locked
  BALANCE_STAND = 3,  // Stands up, joints unlocked
  VELOCITY_MOVE = 4,  // Move with velocity
  FREE = 5,           // Stops the robot but does not disable anything
};

// thread unsafe — caller must manage locks
class ModeFsm {
public:
  explicit ModeFsm(float max_vel_x, float max_vel_y, float max_yaw_rate);

  bool mode_transition(OpMode next);
  void reset_cmd_vel();

  // TODO: clipping here or rejection?
  bool set_cmd_vel(float x, float y, float yaw);

  // Read by control loop. Once read, changed flag resets.
  std::pair<OpMode, bool> get_mode();
  std::array<float, 3> get_cmd_vel();

private:
  bool free_reset_allowed(OpMode next);

private:
  OpMode mode_;
  // Used when robot gets into free to understand which state it's allowed to return to
  OpMode prev_mode_;
  bool mode_changed_;

  std::array<float, 3> cmd_vel_;
  const float max_vel_x_;
  const float max_vel_y_;
  const float max_yaw_rate_;
};

}  // namespace bridge
}  // namespace a2

#endif /* A2_BRIDGE_MODE_FSM_H_ */
