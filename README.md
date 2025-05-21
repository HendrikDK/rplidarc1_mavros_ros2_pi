# rplidarc1_mavros_ros2_pi

This project integrates a Slamtec RPLIDAR C1 with MAVROS using ROS 2 Humble on a Raspberry Pi 4 running Ubuntu 22.04.3 LTS. It enables MAVLink `OBSTACLE_DISTANCE` messages to be published for use with ArduPilot-based obstacle avoidance.

## ✅ Features

- ROS 2 Humble on Raspberry Pi 4
- RPLIDAR C1 via UART at 460800 baud
- MAVROS integration via serial and UDP
- Custom ROS 2 Python node for scan-to-MAVLink conversion
- Auto-start option using systemd
- GitHub-ready with SSH key authentication

## 📦 Packages

- [`rplidar_ros`](https://github.com/Slamtec/rplidar_ros)
- [`mavros`](https://github.com/mavlink/mavros/tree/ros2)
- `scan_to_mavlink` (custom ROS 2 Python package)
- `scan_to_mavlink_interfaces` (custom message definition for `ObstacleDistance`)

## ⚙️ System Setup

- **Serial Ports:**
  - `/dev/ttyAMA1` - RPLIDAR C1 (460800 baud)
  - `/dev/ttyAMA0` - Flight Controller (115200 baud)

- **ROS 2 Nodes:**
  - `rplidar_node`: Publishes `/scan`
  - `scan_to_mavlink_node`: Converts `/scan` to `/uas1/obstacle_distance`
  - `mavros_node`: Communicates with FCU and GCS

## 🚀 Run Manually

```bash
# Source the workspace
cd ~/rplidar_ws
source install/setup.bash

# Launch RPLIDAR
ros2 launch rplidar_ros rplidar_c1_launch.py \
    serial_port:=/dev/ttyAMA1 \
    serial_baudrate:=460800

# In a new terminal (source workspace again)
ros2 run scan_to_mavlink scan_to_mavlink_node

# In a third terminal (source workspace again)
ros2 launch mavros node.launch \
    fcu_url:=serial:///dev/ttyAMA0:115200 \
    gcs_url:=udp://@ \
    tgt_system:=1 \
    tgt_component:=1 \
    pluginlists_yaml:=${HOME}/rplidar_ws/src/mavros/mavros/launch/apm_pluginlists.yaml \
    config_yaml:=${HOME}/rplidar_ws/src/mavros/mavros/launch/apm_config.yaml
🛠️ systemd Autostart
Create a file called drone_service_stack.sh that launches all 3 components.

Then use systemd to autostart it at boot.

📂 Directory Structure
rplidar_ws/
├── src/
│   ├── mavros/
│   ├── rplidar_ros/
│   ├── scan_to_mavlink/
│   └── scan_to_mavlink_interfaces/
├── install/
├── build/
└── log/
📡 Obstacle Avoidance
To use the published /uas1/obstacle_distance in ArduPilot:

Enable AVOID_ENABLE = 7

Set PRX1_TYPE = 5 (MAVLink)

Ensure PRX1_ORIENT matches sensor mount

Verify in Mission Planner under proximity view

📖 License
MIT or other SPDX-compatible license.
