cmake_minimum_required(VERSION 3.8)
project(open_manipulator_x_torque_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Pinocchio 3.x is installed from robotpkg under /opt/openrobots
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)

# ── torque_test_node ─────────────────────────────────────────────────────────
add_executable(torque_test_node src/torque_test_node.cpp)
ament_target_dependencies(torque_test_node rclcpp std_msgs sensor_msgs)
target_compile_features(torque_test_node PUBLIC cxx_std_17)
target_compile_definitions(torque_test_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\"")

# ── fl_control_node ──────────────────────────────────────────────────────────
add_executable(fl_control_node src/fl_control_node.cpp)
ament_target_dependencies(fl_control_node rclcpp std_msgs sensor_msgs)
target_link_libraries(fl_control_node
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(fl_control_node PUBLIC cxx_std_17)
target_compile_definitions(fl_control_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── io_control_node ──────────────────────────────────────────────────────────
add_executable(io_control_node src/io_control_node.cpp)
ament_target_dependencies(io_control_node rclcpp std_msgs sensor_msgs)
target_link_libraries(io_control_node
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(io_control_node PUBLIC cxx_std_17)
target_compile_definitions(io_control_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS torque_test_node fl_control_node io_control_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY xacro config launch scripts data urdf meshes worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
