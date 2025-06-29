cmake_minimum_required(VERSION 3.8)
project(sim_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(laser_simulator src/laser_simulator.cpp)
ament_target_dependencies(
  laser_simulator
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_ros"
  "visualization_msgs"
)

add_executable(scan_listener src/scan_listener.cpp)
ament_target_dependencies(
  scan_listener
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# Sensor data to LaserScan converter
add_executable(sensor_data_to_scan src/sensor_data_to_scan.cpp)
ament_target_dependencies(
  sensor_data_to_scan
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# Fake laser generator node
add_executable(fake_laser_generator src/fake_laser_generator.cpp)
ament_target_dependencies(
  fake_laser_generator
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
)

install(
  DIRECTORY include
  DESTINATION include
)
install(TARGETS
  laser_simulator
  scan_listener
  sensor_data_to_scan
  fake_laser_generator
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config rviz maps
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  ${PROJECT_NAME}/relocalization.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
