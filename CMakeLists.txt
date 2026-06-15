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

set(FL_INSTR   "src/Feedback Linearization/instructor")
set(FL_STUD    "src/Feedback Linearization/student")
set(TO_INSTR   "src/Trajectory Optimization TV-LQR/instructor")
set(GC_SRC     "src/Gravity Compensation")
set(IDENT_SRC  "src/Identification")
set(SMC_INSTR  "src/Sliding Mode/instructor")

# ── gz_fl_control_node ───────────────────────────────────────────────────────
add_executable(gz_fl_control_node "${FL_INSTR}/gz_fl_control_node.cpp")
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
add_executable(gz_io_control_node "${FL_INSTR}/gz_io_control_node.cpp")
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
add_executable(hw_fl_control_node "${FL_INSTR}/hw_fl_control_node.cpp")
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
  "PACKAGE_CONFIG_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/config\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── hw_io_control_node ───────────────────────────────────────────────────────
add_executable(hw_io_control_node "${FL_INSTR}/hw_io_control_node.cpp")
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
  "PACKAGE_CONFIG_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/config\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── gz_fl_control_node_base ──────────────────────────────────────────────────
add_executable(gz_fl_control_node_base "${FL_STUD}/gz_fl_control_node_base.cpp")
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
add_executable(gz_io_control_node_base "${FL_STUD}/gz_io_control_node_base.cpp")
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
add_executable(hw_fl_control_node_base "${FL_STUD}/hw_fl_control_node_base.cpp")
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
  "PACKAGE_CONFIG_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/config\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── hw_io_control_node_base ──────────────────────────────────────────────────
add_executable(hw_io_control_node_base "${FL_STUD}/hw_io_control_node_base.cpp")
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
  "PACKAGE_CONFIG_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/config\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── gz_gravity_comp_node ─────────────────────────────────────────────────────
add_executable(gz_gravity_comp_node "${GC_SRC}/gz_gravity_comp_node.cpp")
ament_target_dependencies(gz_gravity_comp_node rclcpp std_msgs sensor_msgs)
target_link_libraries(gz_gravity_comp_node
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(gz_gravity_comp_node PUBLIC cxx_std_17)
target_compile_definitions(gz_gravity_comp_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── hw_gravity_comp_node ─────────────────────────────────────────────────────
add_executable(hw_gravity_comp_node "${GC_SRC}/hw_gravity_comp_node.cpp")
ament_target_dependencies(hw_gravity_comp_node rclcpp sensor_msgs)
target_include_directories(hw_gravity_comp_node PRIVATE ${dynamixel_sdk_INCLUDE_DIRS})
target_link_libraries(hw_gravity_comp_node
  ${dynamixel_sdk_LIBRARIES}
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(hw_gravity_comp_node PUBLIC cxx_std_17)
target_compile_definitions(hw_gravity_comp_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── gz_tvlqr_node ────────────────────────────────────────────────────────────
add_executable(gz_tvlqr_node "${TO_INSTR}/gz_tvlqr_node.cpp")
ament_target_dependencies(gz_tvlqr_node rclcpp std_msgs sensor_msgs)
target_link_libraries(gz_tvlqr_node Eigen3::Eigen)
target_compile_features(gz_tvlqr_node PUBLIC cxx_std_17)
target_compile_definitions(gz_tvlqr_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_SHARE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

# ── hw_tvlqr_node ────────────────────────────────────────────────────────────
add_executable(hw_tvlqr_node "${TO_INSTR}/hw_tvlqr_node.cpp")
ament_target_dependencies(hw_tvlqr_node rclcpp sensor_msgs)
target_include_directories(hw_tvlqr_node PRIVATE ${dynamixel_sdk_INCLUDE_DIRS})
target_link_libraries(hw_tvlqr_node
  ${dynamixel_sdk_LIBRARIES}
  Eigen3::Eigen)
target_compile_features(hw_tvlqr_node PUBLIC cxx_std_17)
target_compile_definitions(hw_tvlqr_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_SHARE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

# ── hw_identify_node ─────────────────────────────────────────────────────────
add_executable(hw_identify_node "${IDENT_SRC}/hw_identify_node.cpp")
ament_target_dependencies(hw_identify_node rclcpp sensor_msgs)
target_include_directories(hw_identify_node PRIVATE ${dynamixel_sdk_INCLUDE_DIRS})
target_link_libraries(hw_identify_node
  ${dynamixel_sdk_LIBRARIES}
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(hw_identify_node PUBLIC cxx_std_17)
target_compile_definitions(hw_identify_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)


# ── hw_sinusoidal_torque_node (diagnostico: torques sinusoidales por joint) ──
add_executable(hw_sinusoidal_torque_node "src/Diagnostics/hw_sinusoidal_torque_node.cpp")
ament_target_dependencies(hw_sinusoidal_torque_node rclcpp sensor_msgs)
target_include_directories(hw_sinusoidal_torque_node PRIVATE ${dynamixel_sdk_INCLUDE_DIRS})
target_link_libraries(hw_sinusoidal_torque_node
  ${dynamixel_sdk_LIBRARIES}
  Eigen3::Eigen)
target_compile_features(hw_sinusoidal_torque_node PUBLIC cxx_std_17)
target_compile_definitions(hw_sinusoidal_torque_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_CONFIG_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/config\"")

# ── gz_smc_art_node ──────────────────────────────────────────────────────────
add_executable(gz_smc_art_node "${SMC_INSTR}/gz_SMC_art_node.cpp")
ament_target_dependencies(gz_smc_art_node rclcpp std_msgs sensor_msgs)
target_link_libraries(gz_smc_art_node
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(gz_smc_art_node PUBLIC cxx_std_17)
target_compile_definitions(gz_smc_art_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── gz_smc_cart_node ─────────────────────────────────────────────────────────
add_executable(gz_smc_cart_node "${SMC_INSTR}/gz_SMC_cart_node.cpp")
ament_target_dependencies(gz_smc_cart_node rclcpp std_msgs sensor_msgs)
target_link_libraries(gz_smc_cart_node
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(gz_smc_cart_node PUBLIC cxx_std_17)
target_compile_definitions(gz_smc_cart_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── gz_SMCI_cart_node ────────────────────────────────────────────────────────
add_executable(gz_SMCI_cart_node "${SMC_INSTR}/gz_SMCI_cart_node.cpp")
ament_target_dependencies(gz_SMCI_cart_node rclcpp std_msgs sensor_msgs)
target_link_libraries(gz_SMCI_cart_node
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  Eigen3::Eigen)
target_compile_features(gz_SMCI_cart_node PUBLIC cxx_std_17)
target_compile_definitions(gz_SMCI_cart_node PRIVATE
  "PACKAGE_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/data\""
  "PACKAGE_URDF_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/urdf\""
  COAL_DISABLE_HPP_FCL_WARNINGS)

# ── hw_torque_probe_node (diagnostico de polaridad torque->corriente) ────────
add_executable(hw_torque_probe_node "src/Diagnostics/hw_torque_probe_node.cpp")
ament_target_dependencies(hw_torque_probe_node rclcpp)
target_include_directories(hw_torque_probe_node PRIVATE ${dynamixel_sdk_INCLUDE_DIRS})
target_link_libraries(hw_torque_probe_node ${dynamixel_sdk_LIBRARIES})
target_compile_features(hw_torque_probe_node PUBLIC cxx_std_17)

# ── Forward Kinematics ───────────────────────────────────────────────────────
set(FK_SRC "src/Forward Kinematics")

# hw_fkin_node: lee posiciones del robot real (torque cero) e imprime la
# cinematica directa del efector final. Usa fkin analitica (sin Pinocchio).
add_executable(hw_fkin_node "${FK_SRC}/hw_fkin_node.cpp")
ament_target_dependencies(hw_fkin_node rclcpp sensor_msgs)
target_include_directories(hw_fkin_node PRIVATE ${dynamixel_sdk_INCLUDE_DIRS})
target_link_libraries(hw_fkin_node
  ${dynamixel_sdk_LIBRARIES}
  Eigen3::Eigen)
target_compile_features(hw_fkin_node PUBLIC cxx_std_17)

# fkin_display_node (Python)
install(PROGRAMS
  "${FK_SRC}/fkin_display_node.py"
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS
    gz_fl_control_node gz_io_control_node
    hw_fl_control_node hw_io_control_node
    gz_fl_control_node_base gz_io_control_node_base
    hw_fl_control_node_base hw_io_control_node_base
    gz_tvlqr_node
    hw_tvlqr_node
    gz_gravity_comp_node
    hw_gravity_comp_node
    hw_fkin_node
    hw_sinusoidal_torque_node
    hw_torque_probe_node
    hw_identify_node
    gz_smc_art_node gz_smc_cart_node gz_SMCI_cart_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY xacro config launch scripts data urdf meshes worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
