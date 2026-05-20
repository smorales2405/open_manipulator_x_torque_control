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
find_package(dynamixel_sdk REQUIRED)

# ── gz_fl_control_node ───────────────────────────────────────────────────────
add_executable(gz_fl_control_node src/gz_fl_control_node.cpp)
ament_target_dependencies(gz_fl_control_node rclcpp std_msgs sensor_msgs)
target_link_libraries(gz_fl_control_node
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(gz_fl_control_node PUBLIC cxx_std_17)
target_compile_definitions(gz_fl_control_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── gz_io_control_node ───────────────────────────────────────────────────────
add_executable(gz_io_control_node src/gz_io_control_node.cpp)
ament_target_dependencies(gz_io_control_node rclcpp std_msgs sensor_msgs)
target_link_libraries(gz_io_control_node
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(gz_io_control_node PUBLIC cxx_std_17)
target_compile_definitions(gz_io_control_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── hw_fl_control_node ───────────────────────────────────────────────────────
add_executable(hw_fl_control_node src/hw_fl_control_node.cpp)
ament_target_dependencies(hw_fl_control_node rclcpp sensor_msgs)
target_include_directories(hw_fl_control_node PRIVATE ${dynamixel_sdk_INCLUDE_DIRS})
target_link_libraries(hw_fl_control_node
  ${dynamixel_sdk_LIBRARIES}
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(hw_fl_control_node PUBLIC cxx_std_17)
target_compile_definitions(hw_fl_control_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── hw_io_control_node ───────────────────────────────────────────────────────
add_executable(hw_io_control_node src/hw_io_control_node.cpp)
ament_target_dependencies(hw_io_control_node rclcpp sensor_msgs)
target_include_directories(hw_io_control_node PRIVATE ${dynamixel_sdk_INCLUDE_DIRS})
target_link_libraries(hw_io_control_node
  ${dynamixel_sdk_LIBRARIES}
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(hw_io_control_node PUBLIC cxx_std_17)
target_compile_definitions(hw_io_control_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── gz_fl_control_node_base ──────────────────────────────────────────────────
add_executable(gz_fl_control_node_base src/gz_fl_control_node_base.cpp)
ament_target_dependencies(gz_fl_control_node_base rclcpp std_msgs sensor_msgs)
target_link_libraries(gz_fl_control_node_base
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(gz_fl_control_node_base PUBLIC cxx_std_17)
target_compile_definitions(gz_fl_control_node_base PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── gz_io_control_node_base ──────────────────────────────────────────────────
add_executable(gz_io_control_node_base src/gz_io_control_node_base.cpp)
ament_target_dependencies(gz_io_control_node_base rclcpp std_msgs sensor_msgs)
target_link_libraries(gz_io_control_node_base
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(gz_io_control_node_base PUBLIC cxx_std_17)
target_compile_definitions(gz_io_control_node_base PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── hw_fl_control_node_base ──────────────────────────────────────────────────
add_executable(hw_fl_control_node_base src/hw_fl_control_node_base.cpp)
ament_target_dependencies(hw_fl_control_node_base rclcpp sensor_msgs)
target_include_directories(hw_fl_control_node_base PRIVATE ${dynamixel_sdk_INCLUDE_DIRS})
target_link_libraries(hw_fl_control_node_base
  ${dynamixel_sdk_LIBRARIES}
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(hw_fl_control_node_base PUBLIC cxx_std_17)
target_compile_definitions(hw_fl_control_node_base PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── hw_io_control_node_base ──────────────────────────────────────────────────
add_executable(hw_io_control_node_base src/hw_io_control_node_base.cpp)
ament_target_dependencies(hw_io_control_node_base rclcpp sensor_msgs)
target_include_directories(hw_io_control_node_base PRIVATE ${dynamixel_sdk_INCLUDE_DIRS})
target_link_libraries(hw_io_control_node_base
  ${dynamixel_sdk_LIBRARIES}
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(hw_io_control_node_base PUBLIC cxx_std_17)
target_compile_definitions(hw_io_control_node_base PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS
    gz_fl_control_node gz_io_control_node
    hw_fl_control_node hw_io_control_node
    gz_fl_control_node_base gz_io_control_node_base
    hw_fl_control_node_base hw_io_control_node_base
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY xacro config launch scripts data urdf meshes worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
