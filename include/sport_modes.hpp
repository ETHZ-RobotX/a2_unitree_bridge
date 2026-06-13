/* Separate HPP for sport mode enum to operation mode conversion for consistency between higher level FSMs */

#ifndef SPORT_MODE_ENUMS_H_
#define SPORT_MODE_ENUMS_H_

#include <cstdint>
namespace a2 {
namespace bridge {

// This is from the unitree SDK, serves to define as types for mapping
// No msg interfaces are intentionally created to prevent wide access
enum class UnitreeSportMode : uint8_t {
  PASSIVE = 0,
  STAND_DOWN = 1,
  STAND_UP = 2,
  DEFAULT_MODE = 3,
  RUNNING_MODE = 4,
  CLIMB_MODE = 5,
  LEFT_SIDE_GAIT = 6,
  RIGHT_SIDE_GAIT = 7,
  HANDSTAND = 8,
  BIPED_STAND = 9,
  FRONT_FLIP = 10,
  BACK_FLIP = 11,
  RECOVERY = 12,
  BASE_HEIGHT_CTRL = 13,
};

}  // namespace bridge
}  // namespace a2

#endif /* SPORT_MODE_ENUMS_H_ */
