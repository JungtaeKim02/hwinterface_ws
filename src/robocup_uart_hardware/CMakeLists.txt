cmake_minimum_required(VERSION 3.16)
project(robocup_uart_hardware)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(robocup_uart_hardware SHARED src/uart_link.cpp src/uart_system.cpp)
target_compile_features(robocup_uart_hardware PUBLIC cxx_std_17)
target_compile_options(robocup_uart_hardware PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(robocup_uart_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robocup_uart_hardware
  diagnostic_msgs hardware_interface pluginlib rclcpp sensor_msgs std_msgs std_srvs)

pluginlib_export_plugin_description_file(hardware_interface uart_hardware.xml)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS robocup_uart_hardware
  EXPORT export_robocup_uart_hardware
  LIBRARY DESTINATION lib)
ament_export_targets(export_robocup_uart_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(diagnostic_msgs hardware_interface pluginlib rclcpp sensor_msgs std_msgs std_srvs)
ament_package()
