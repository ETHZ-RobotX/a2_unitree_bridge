# a2_unitree_bridge

ROS 2 bridge between the Unitree DDS layer and ROS 2 topics for the A2 robot. Two build targets share the same source — one for the real robot (HG firmware) and one for MuJoCo simulation.

## Build

```bash
colcon build --packages-select a2_unitree_bridge
```

## Run

**Simulation (MuJoCo + Unitree DDS on loopback):**
```bash
ros2 launch a2_unitree_bridge sim.launch.py
```

**Real robot (ethernet, HG firmware):**
```bash
ros2 launch a2_unitree_bridge robot.launch.py
```

## Topics

### Sim (`a2_bridge_sim`)

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | DDS rate | Sim time derived from MuJoCo `tick` |
| `/joint_states` | `sensor_msgs/JointState` | 200 Hz | Throttled from LowState DDS |
| `/imu/data` | `sensor_msgs/Imu` | 200 Hz | Throttled from LowState DDS |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | Ground-truth pose from SportModeState |
| `/state_estimation` | `nav_msgs/Odometry` | 50 Hz | Same as `/odom` (for downstream compatibility) |
| `/a2/sport_mode` | `std_msgs/UInt8` | 50 Hz | Current sport mode enum value |
| `/camera/image_raw` | `sensor_msgs/Image` | DDS rate | Front camera RGB decoded from PointCloud2 DDS |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | DDS rate | Hardcoded intrinsics, published with each image |
| `/mujoco/front_lidar` | `sensor_msgs/PointCloud2` | DDS rate | Raw front lidar scan |
| `/mujoco/rear_lidar` | `sensor_msgs/PointCloud2` | DDS rate | Raw rear lidar scan |
| `/lowcmd` (subscribed) | `unitree_go/LowCmd` | — | Receives low-level joint commands and forwards to DDS |

**TF:** `map → base_link` broadcast at 50 Hz from SportModeState ground-truth position.

### Robot (`a2_bridge_robot`)

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 200 Hz | Throttled from LowState DDS (HG firmware) |
| `/imu/data` | `sensor_msgs/Imu` | 200 Hz | Throttled from LowState DDS |
| `/a2/sport_mode` | `std_msgs/UInt8` | 50 Hz | Current sport mode enum value |

Odometry on the real robot comes from DLIO, not this bridge.
