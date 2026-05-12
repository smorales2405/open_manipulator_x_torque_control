cmake_minimum_required(VERSION 3.8)
project(open_manipulator_x_torque_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(torque_test_node src/torque_test_node.cpp)
ament_target_dependencies(torque_test_node rclcpp std_msgs sensor_msgs)
target_compile_features(torque_test_node PUBLIC cxx_std_17)

install(TARGETS torque_test_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY xacro config launch scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
