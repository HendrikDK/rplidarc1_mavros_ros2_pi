# RPLIDAR C1 + MAVROS + ROS 2 Integration on Raspberry Pi

[![ROS 2 Humble](https://img.shields.io/badge/ROS--2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()
[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%204-orange)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview

This project integrates a Slamtec RPLIDAR C1 with ROS 2 Humble and MAVROS on a Raspberry Pi 4. It enables publishing LIDAR scan data as `OBSTACLE_DISTANCE` MAVLink messages for obstacle avoidance in an ArduPilot-based UAV system.

## Features

* 🔄 Real-time LIDAR scan processing
* 📡 MAVLink OBSTACLE\_DISTANCE publishing
* 🛰️ MAVROS namespace `/uas1` for clean multi-UAS integration
* 🚀 Auto-launch ready via systemd

## Repository Structure

```
rplidarc1_mavros_ros2_pi/
├── src/
│   ├── scan_to_mavlink/            # Python node to convert /scan to OBSTACLE_DISTANCE
│   ├── scan_to_mavlink_interfaces/ # Custom ROS 2 interface package
│   ├── rplidar_ros/                # RPLIDAR ROS 2 driver
│   ├── mavros_launch/              # Custom MAVROS launch files
│   └── mavlink/                    # MAVLink headers (for completeness)
├── build/                          # colcon build artifacts
├── install/                        # colcon install artifacts
├── .gitignore
└── README.md
```

## Dependencies

* Ubuntu 22.04.3 LTS (Server ARM64)
* ROS 2 Humble
* MAVROS (`ros-humble-mavros`, `mavros_extras` partially from source)
* Python 3.10+

## Quick Start

```bash
# Clone the repository
cd ~
git clone https://github.com/HendrikDK/rplidarc1_mavros_ros2_pi.git
cd rplidarc1_mavros_ros2_pi

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --symlink-install
source install/setup.bash

# Launch MAVROS
ros2 launch mavros_launch mavros.launch.py \
  fcu_url:=/dev/ttyAMA0:115200 \
  gcs_url:=udp://@127.0.0.1 \
  fcu_protocol:=v2.0 \
  namespace:=uas1

# Launch RPLIDAR + OBSTACLE_DISTANCE converter
ros2 launch scan_to_mavlink scan_to_mavlink.launch.py
```

## Topics Published

* `/scan`: `sensor_msgs/LaserScan` (from RPLIDAR)
* `/uas1/obstacle_distance`: `scan_to_mavlink_interfaces/msg/ObstacleDistance`

## Systemd Autostart (Optional)

To enable auto-start of MAVROS and scan\_to\_mavlink on boot, see the `systemd` folder (TODO: publish config).

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Author

**HendrikDK**
[https://github.com/HendrikDK](https://github.com/HendrikDK)

## Contributions

Pull requests are welcome! For major changes, please open an issue first to discuss what you would like to change.
