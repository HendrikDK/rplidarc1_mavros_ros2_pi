# rplidarc1_mavros_ros2_pi

This project integrates a **Slamtec RPLIDAR C1** with **MAVROS** using **ROS 2 Humble** on a **Raspberry Pi 4 running Ubuntu 22.04.3 LTS**. It enables MAVLink `OBSTACLE_DISTANCE` messages to be published for use with **ArduPilot**'s obstacle avoidance system.

---

## ✅ Features

- ROS 2 Humble on Raspberry Pi 4
- RPLIDAR C1 connected via UART at 460800 baud
- MAVROS integration using serial and UDP links
- Custom ROS 2 Python node that converts `/scan` to `OBSTACLE_DISTANCE`
- Mission Planner-compatible obstacle avoidance data
- Optional systemd auto-start

---

## 📦 ROS 2 Packages Used

- [`rplidar_ros`](https://github.com/Slamtec/rplidar_ros)
- [`mavros`](https://github.com/mavlink/mavros/tree/ros2)
- `scan_to_mavlink` – custom ROS 2 Python package
- `scan_to_mavlink_interfaces` – includes `ObstacleDistance.msg`

---

## ⚙️ Hardware Setup

| Device            | Port            | Baudrate  |
|------------------|------------------|-----------|
| RPLIDAR C1       | `/dev/ttyAMA1`   | 460800    |
| Flight Controller| `/dev/ttyAMA0`   | 115200    |

---

## 🚀 Full Runtime Sequence (Manual Start)

### 1. Source the workspace:

```bash
cd ~/rplidar_ws
source install/setup.bash
2. Start the RPLIDAR C1 driver:
bash
Kopiëren
Bewerken
ros2 launch rplidar_ros rplidar_c1_launch.py \
    serial_port:=/dev/ttyAMA1 \
    serial_baudrate:=460800
3. In a second terminal (source again):
bash
Kopiëren
Bewerken
cd ~/rplidar_ws
source install/setup.bash

ros2 run scan_to_mavlink scan_to_mavlink_node
4. In a third terminal (source again):
bash
Kopiëren
Bewerken
cd ~/rplidar_ws
source install/setup.bash

ros2 launch mavros node.launch \
    fcu_url:=serial:///dev/ttyAMA0:115200 \
    gcs_url:=udp://@ \
    tgt_system:=1 \
    tgt_component:=1 \
    pluginlists_yaml:=${HOME}/rplidar_ws/src/mavros/mavros/launch/apm_pluginlists.yaml \
    config_yaml:=${HOME}/rplidar_ws/src/mavros/mavros/launch/apm_config.yaml
🔁 Auto-start with systemd
A drone_service_stack.sh launch script and .service unit can be created to auto-start all components at boot.

📂 Directory Structure
bash
Kopiëren
Bewerken
rplidar_ws/
├── src/
│   ├── mavros/
│   ├── rplidar_ros/
│   ├── scan_to_mavlink/
│   └── scan_to_mavlink_interfaces/
├── build/
├── install/
└── log/
🧭 ArduPilot Obstacle Avoidance Settings
Ensure these parameters are set in Mission Planner:

Parameter	Value	Description
AVOID_ENABLE	7	Enable all avoidance features
PRX1_TYPE	5	MAVLink proximity sensor
PRX1_ORIENT	0	Front-facing (adjust as needed)

📖 License
MIT or similar open license.
