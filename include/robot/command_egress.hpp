#ifndef A2_BRIDGE_ROBOT_CMD_EGRESS_H_
#define A2_BRIDGE_ROBOT_CMD_EGRESS_H_

#include <unitree/robot/a2/sport/sport_client.hpp>
#include "a2_utils/safe_velocity_ros_interface.hpp"
#include "common/egress.hpp"

namespace a2 {
namespace bridge {

/*
 * Wraps the Unitree SportClient SDK. Inherits ROS subscriptions and FSM from
 * SafeVelocityRosInterface and implements onControl() to translate mode into
 * SDK calls.
 */
class A2CommandPublisher : public utils::SafeVelocityRosInterface, public EgressType {
public:
  A2CommandPublisher();

  void init(rclcpp::Node* node);

protected:
  void onControl(utils::OpMode mode, bool mode_changed, std::array<float, 3> vel) override;

private:
private:
  unitree::robot::a2::SportClient sport_client_;
};

}  // namespace bridge
}  // namespace a2

#endif /* A2_BRIDGE_ROBOT_CMD_EGRESS_H_ */
