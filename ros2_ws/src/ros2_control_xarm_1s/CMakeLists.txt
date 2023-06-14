cmake_minimum_required(VERSION 3.8)
project(ros2_control_xarm_1s)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# COMPILE
add_library(
  ros2_control_xarm_1s
  SHARED
  src/xarm_1s.cpp
)
target_compile_features(ros2_control_xarm_1s PUBLIC cxx_std_17)
target_link_libraries(ros2_control_xarm_1s PUBLIC serial)
target_include_directories(ros2_control_xarm_1s PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/ros2_control_xarm_1s>
)
ament_target_dependencies(
  ros2_control_xarm_1s PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface ros2_control_xarm_1s.xml)

# INSTALL
install(
  DIRECTORY include/ros2_control_xarm_1s
  DESTINATION include/ros2_control_xarm_1s
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_xarm_1s
)
install(TARGETS ros2_control_xarm_1s
  EXPORT export_ros2_control_xarm_1s
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# TESTING
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# EXPORTS
ament_export_targets(export_ros2_control_xarm_1s HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
