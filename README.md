# sim_localization Package

## Purpose
This package is designed for simulating localization tasks in a ROS 2 environment. It provides tools and configurations to simulate and test localization algorithms.

## Compilation Steps
1. **Clone the package to your workspace's `src` directory**:
   ```bash
   git clone [repository_url] /home/wufy/ros2_ws/src/sim_localization
   ```

2. **Install dependencies**:
   ```bash
   cd /home/wufy/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   colcon build --symlink-install
   ```

4. **Source the setup file**:
   ```bash
   source install/setup.bash
   ```

5. **Launch the simulation**:
   ```bash
   ros2 launch sim_localization [launch_file_name].launch.py
   ```

Replace `[launch_file_name]` with the appropriate launch file from the `launch` directory. 