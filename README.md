# sim_localization

A collection of lightweight ROS 2 nodes that emulate simple localization-related sensors and utilities for a rectangular “basketball-court” world. The package is completely **simulation-only**. No external simulators (Gazebo / Webots) are required – everything is computed analytically and visualised in RViz.

---
## Features / Nodes

| Node | Executable | Purpose |
|------|------------|---------|
| Laser Simulator | `laser_simulator` | Simulates a 360° laser around a robot moving inside a rectangle court (default 15 × 7 m). Publishes <br/>• `laser_scan` – 360° scan in `base_link` frame <br/>• `laser_scan_fake` – 360° scan in `fake_base_link` frame <br/>Broadcasts TF: `map→odom` (static, identity) &amp; dynamic `odom→base_link`, `map→fake_base_link`. |
| Scan Listener | `scan_listener` | Example consumer of `laser_scan`. Additionally listens to RViz `initialpose` and (re-)publishes **static** TF `map→odom` computed from the pose, enabling **unlimited relocation**. |
| 5-Beam Sensor → LaserScan | `sensor_data_to_scan` | Converts 5-element `std_msgs/Float32MultiArray` on topic **`sensor_data`** into a regular `sensor_msgs/LaserScan`. The five beams are assumed to start at 60° (w.r.t. robot +x) with 72° increments. Also supports `initialpose` relocation exactly like `scan_listener`. |
| Fake Laser Generator | `fake_laser_generator` | The inverse of the above: listens to TF `map→fake_base_link`, analytically intersects 5 fixed-angle beams with court boundaries, and publishes the resulting ranges on **`sensor_data`**.  This is useful when you want to visualise the simple 5-beam sensor in RViz via the previous node. |

---
## Quick Start

```bash
# 1. build (from workspace root!)
cd ~/ros2_ws
colcon build --packages-select sim_localization --symlink-install
source install/setup.bash

# 2. run the ready-made launch file (full 360° laser simulation)
ros2 launch sim_localization laser_simulator.launch.py
```

This launch file starts RViz with a preset configuration, the map server (basketball-court map) and `laser_simulator`.

### Minimal 5-beam demo

In a second terminal:
```bash
# Terminal 1 – generate 5-beam ranges from robot ground-truth pose
ros2 run sim_localization fake_laser_generator \
  --ros-args -p court_length:=15 -p court_width:=7 -p update_rate:=10

# Terminal 2 – convert to LaserScan for RViz
ros2 run sim_localization sensor_data_to_scan
```
Add a **LaserScan Display** in RViz, topic `scan`, to see the 5 virtual rays.

You can relocate the robot anytime from RViz with the **2D Pose Estimate** tool – both `scan_listener` and `sensor_data_to_scan` will recompute and broadcast a fresh `map→odom` transform.

---
## Parameters summary

| Parameter | Default | Affected nodes | Description |
|-----------|---------|----------------|-------------|
| `court_length` | `15.0` | `laser_simulator`, `fake_laser_generator` | Court X dimension in metres |
| `court_width`  | `7.0`  | same as above | Court Y dimension in metres |
| `laser_count`  | `5` (only in `laser_simulator`) | Number of beams for 360° scan (affects angular resolution) |
| `sensor_count` | `5` | `sensor_data_to_scan`, `fake_laser_generator` | Number of fixed beams |
| `first_angle_deg` | `60.0` |  `sensor_data_to_scan`, `fake_laser_generator` | Angle of the first fixed beam wrt robot +x |
| `angle_increment_deg` | `72.0` | same as above | Angular separation between adjacent fixed beams |
| `update_rate` | `10.0` | all simulator / generator nodes | Publishing frequency in Hz |

All parameters can be overridden with standard `--ros-args -p param:=value` syntax or by passing a YAML file.

---
## Dependency Installation

```bash
sudo apt update && rosdep install --from-paths src --ignore-src -r -y
```
All runtime dependencies are ROS 2 core packages only (`rclcpp`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, etc.).

---
## Development tips

1. Always build from the **workspace root** (`~/ros2_ws`), _never_ inside the package directory.
2. After every build: `source install/setup.bash` (or add it to your shell RC).
3. Use `--log-level debug` for verbose output, e.g. `ros2 run sim_localization laser_simulator --ros-args --log-level debug`.

Enjoy your lightweight localisation playground! :) 