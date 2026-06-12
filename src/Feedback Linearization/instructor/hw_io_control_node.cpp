/*
 * hw_io_control_node.cpp
 * Input-Output Linearization cartesiana — OpenMANIPULATOR-X hardware real
 * vía Dynamixel SDK directo, sin ros2_control.
 *
 * Salida de tarea:  y = [x, y, z, phi]^T
 *   (x,y,z) posición cartesiana del efector final (frame end_effector_link).
 *   phi = q2 + q3 + q4  ángulo de inclinación analítico.
 *
 * Ley de control:
 *   v     = yddot_des + Kp*(y_des - y) + Kd*(ydot_des - ydot)
 *   qddot = J4^T (J4 J4^T + lambda^2 I)^-1 (v - Jdot*qdot)   [DLS]
 *   tau   = M(q)*qddot + b(q,qdot)
 *
 * Trayectoria:
 *   Fase [0, T_TRANS):  transición quintica cartesiana y0_capturada → Y_START
 *   Fase [T_TRANS, ∞):  x=0.17+0.05*sin(t'), y=0.05*cos(t'), z=0.27, phi=0.22 rad
 *
 * Parámetros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name              [string]          "/dev/ttyUSB0"
 *   gain_scale             [double]          1.0
 *   t_imp                  [double]          20.0  (tiempo de implementacion)
 *   log_id                 [int]             1     (CSV: hw_io_data_<log_id>.csv)
 *   vel_cutoff_hz          [double]          2.0   (filtro EMA velocidad; 0 = desactivado)
 *
 * Parámetros del modelo identificado (cargar desde config/motor_params.yaml):
 *   motor_alpha            [double[4]]   ticks/N·m       — ganancia de torque por joint
 *   motor_Fv               [double[4]]   ticks/(rad/s)   — fricción viscosa
 *   motor_Fc               [double[4]]   ticks           — fricción de Coulomb
 *   motor_I_offset         [double[4]]   ticks           — offset de corriente
 *   motor_epsilon_friction [double]      rad/s           — suavizado tanh
 *
 * Publisher: /hw/joint_states (sensor_msgs/JointState) — monitoreo
 *
 * ADVERTENCIA: No ejecutar junto a hardware.launch.py ni ningún proceso
 * que acceda a /dev/ttyUSB0 (ros2_control_node, dynamixel_hardware_interface).
 */

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <Eigen/Dense>
#include <Eigen/LU>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "dynamixel_sdk/dynamixel_sdk.h"

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif
#ifndef PACKAGE_CONFIG_DIR
#define PACKAGE_CONFIG_DIR "."
#endif

using namespace std::chrono_literals;

// ============================================================
// Constantes Dynamixel / conversión de unidades
// ============================================================

static constexpr double PI = 3.14159265358979323846;
static constexpr int    NUM_JOINTS = 4;

static const std::array<uint8_t, NUM_JOINTS> DXL_ID = {11, 12, 13, 14};
static constexpr int    BAUDRATE         = 1000000;
static constexpr double PROTOCOL_VERSION = 2.0;

static constexpr uint16_t ADDR_OPERATING_MODE   = 11;
static constexpr uint16_t ADDR_CURRENT_LIMIT    = 38;
static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
static constexpr uint16_t ADDR_GOAL_CURRENT     = 102;
static constexpr uint16_t ADDR_PRESENT_CURRENT  = 126;
static constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
static constexpr uint16_t ADDR_PRESENT_POSITION = 132;

static constexpr uint16_t LEN_GOAL_CURRENT     = 2;
static constexpr uint16_t LEN_PRESENT_CURRENT  = 2;
static constexpr uint16_t LEN_PRESENT_VELOCITY = 4;
static constexpr uint16_t LEN_PRESENT_POSITION = 4;

static constexpr uint8_t CURRENT_CONTROL_MODE = 0;
static constexpr uint8_t TORQUE_ENABLE_VAL    = 1;
static constexpr uint8_t TORQUE_DISABLE_VAL   = 0;

static constexpr double POS_UNIT_RAD           = 2.0 * PI / 4096.0;
static constexpr double VEL_UNIT_RAD_S         = 0.229 * 2.0 * PI / 60.0;
static constexpr double CURRENT_UNIT_A         = 0.00269;
static constexpr double TORQUE_CONSTANT_NM_A   = 1.654;
static constexpr double TORQUE_PER_CURRENT_TICK = TORQUE_CONSTANT_NM_A * CURRENT_UNIT_A;

// JOINT_ZERO_TICK, ENCODER_SIGN, CURRENT_SIGN, JOINT_LOWER/UPPER,
// CURRENT_LIMIT_REGISTER, CURRENT_CMD_LIMIT, CURRENT_MEASURED_PEAK, TAU_MAX
// → cargados desde config/motor_params.yaml como parámetros ROS 2.

// ── Parámetros del controlador IO (idénticos a act_2) ─────────────────────
static const Eigen::Vector4d KP_Y = {200.0, 80.0, 250.0, 250.0};
static const Eigen::Vector4d KD_Y = {0.0, 0.0, 0.0, 0.0};
static constexpr double LAMBDA    = 0.01;
static constexpr double LAMBDA_SQ = LAMBDA * LAMBDA;

// Punto de inicio de la trayectoria cartesiana [x, y, z, phi] en t'=0
static const Eigen::Vector4d Y_START{0.17, 0.05, 0.27, 0.22};
static constexpr double T_TRANS = 3.0;

static constexpr char EFF_FRAME_NAME[] = "end_effector_link";

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Utilidades de conversión (idénticas a act_2)
// ============================================================

static int32_t toSigned32(uint32_t v)
{
  if (v > 0x7FFFFFFFu) return -static_cast<int32_t>(0xFFFFFFFFu - v + 1u);
  return static_cast<int32_t>(v);
}

static int16_t toSigned16(uint32_t v)
{
  const uint16_t w = static_cast<uint16_t>(v & 0xFFFFu);
  if (w > 0x7FFFu) return -static_cast<int16_t>(0xFFFFu - w + 1u);
  return static_cast<int16_t>(w);
}

static int32_t wrappedTickDiff(int32_t raw, int32_t zero)
{
  int32_t d = raw - zero;
  while (d >  2048) d -= 4096;
  while (d < -2048) d += 4096;
  return d;
}

static void currentToBytes(int16_t cur, uint8_t p[2])
{
  const uint16_t w = static_cast<uint16_t>(cur);
  p[0] = DXL_LOBYTE(w);
  p[1] = DXL_HIBYTE(w);
}

static int16_t clampCurrent(double x, int16_t lim)
{
  return static_cast<int16_t>(std::lround(
    std::min(std::max(x, -static_cast<double>(lim)), static_cast<double>(lim))));
}

// ============================================================
// Trayectoria cartesiana (idéntica a act_2)
// ============================================================

struct CartRef {
  Eigen::Vector4d y, ydot, yddot;
};

static CartRef cartesianTrajectory(double t)
{
  const double w = 1.0;
  CartRef r;
  r.y    << 0.17 + 0.05*std::sin(w*t),   0.05*std::cos(w*t),  0.27,  0.22;
  r.ydot <<  0.05*w*std::cos(w*t),       -0.05*w*std::sin(w*t), 0.0,  0.0;
  r.yddot<< -0.05*w*w*std::sin(w*t),     -0.05*w*w*std::cos(w*t), 0.0, 0.0;
  return r;
}

static CartRef cartesianTransition(double t,
                                    const Eigen::Vector4d& y0,
                                    const Eigen::Vector4d& yf,
                                    double T)
{
  const double tau  = std::min(1.0, t / T);
  const double tau2 = tau*tau, tau3=tau2*tau, tau4=tau3*tau, tau5=tau4*tau;
  const double s    =  10*tau3 - 15*tau4 +  6*tau5;
  const double sd   = (30*tau2 - 60*tau3 + 30*tau4) / T;
  const double sdd  = (60*tau  - 180*tau2 + 120*tau3) / (T*T);
  const Eigen::Vector4d delta = yf - y0;
  CartRef r;
  r.y     = y0 + s   * delta;
  r.ydot  =      sd  * delta;
  r.yddot =      sdd * delta;
  return r;
}

// ============================================================
// Nodo ROS 2
// ============================================================

class HWIOControlNode : public rclcpp::Node
{
public:
  explicit HWIOControlNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("hw_io_control_node", opts),
    hw_active_(false), y0_initialized_(false)
  {
    // ── Parámetros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",  "/dev/ttyUSB0");
    this->declare_parameter<double>     ("gain_scale",  1.0);
    this->declare_parameter<double>     ("t_imp",       20.0);
    this->declare_parameter<int>        ("log_id",      1);

    using dvec = std::vector<double>;
    this->declare_parameter<dvec>("motor_alpha",            dvec{208.5, 208.5, 208.5, 208.5});
    this->declare_parameter<dvec>("motor_Fv",               dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<dvec>("motor_Fc",               dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<dvec>("motor_I_offset",         dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<double>("motor_epsilon_friction", 0.05);
    this->declare_parameter<double>("vel_cutoff_hz",          2.0);

    using ivec = std::vector<int64_t>;
    this->declare_parameter<ivec>  ("joint_zero_tick",        ivec{2048, 2048, 2048, 2048});
    this->declare_parameter<dvec>  ("encoder_sign",           dvec{+1.0, +1.0, +1.0, +1.0});
    this->declare_parameter<dvec>  ("current_sign",           dvec{+1.0, +1.0, +1.0, +1.0});
    this->declare_parameter<dvec>  ("joint_lower",            dvec{-2.356194, -1.919862, -1.919862, -1.8});
    this->declare_parameter<dvec>  ("joint_upper",            dvec{+2.356194, +1.745329, +1.570796, +2.1});
    this->declare_parameter<int>   ("current_limit_register", 350);
    this->declare_parameter<ivec>  ("current_cmd_limit",      ivec{257, 257, 257, 257});
    this->declare_parameter<int>   ("current_measured_peak",  313);
    this->declare_parameter<double>("tau_max",                1.2);

    port_name_  = this->get_parameter("port_name").as_string();
    gain_scale_ = this->get_parameter("gain_scale").as_double();
    t_imp_      = this->get_parameter("t_imp").as_double();
    const int log_id = this->get_parameter("log_id").as_int();

    auto load_vec4 = [this](const std::string& name) {
      auto v = get_parameter(name).as_double_array();
      return Vec4(v[0], v[1], v[2], v[3]);
    };
    motor_alpha_    = load_vec4("motor_alpha");
    motor_Fv_       = load_vec4("motor_Fv");
    motor_Fc_       = load_vec4("motor_Fc");
    motor_I_offset_ = load_vec4("motor_I_offset");
    motor_epsilon_  = get_parameter("motor_epsilon_friction").as_double();

    vel_cutoff_hz_ = get_parameter("vel_cutoff_hz").as_double();
    vel_filter_alpha_ = (vel_cutoff_hz_ > 0.0)
        ? std::exp(-2.0 * PI * vel_cutoff_hz_ * 0.01)
        : 0.0;

    {
      const auto zt = get_parameter("joint_zero_tick").as_integer_array();
      for (int i = 0; i < NUM_JOINTS; ++i) joint_zero_tick_[i] = static_cast<int32_t>(zt[i]);
    }
    encoder_sign_ = load_vec4("encoder_sign");
    current_sign_ = load_vec4("current_sign");
    joint_lower_  = load_vec4("joint_lower");
    joint_upper_  = load_vec4("joint_upper");
    current_limit_register_ = static_cast<uint16_t>(get_parameter("current_limit_register").as_int());
    {
      const auto cl = get_parameter("current_cmd_limit").as_integer_array();
      for (int i = 0; i < NUM_JOINTS; ++i) current_cmd_limit_[i] = static_cast<int16_t>(cl[i]);
    }
    current_measured_peak_ = static_cast<int16_t>(get_parameter("current_measured_peak").as_int());
    tau_max_ = get_parameter("tau_max").as_double();

    RCLCPP_INFO(get_logger(), "puerto=%s  gain=%.2f  dur=%.1fs  id=%d",
      port_name_.c_str(), gain_scale_, t_imp_, log_id);
    RCLCPP_INFO(get_logger(),
      "Kp_y=[%.1f %.1f %.1f %.1f]  Kd_y=[%.1f %.1f %.1f %.1f]  lambda=%.3f  tau_max=%.2f",
      KP_Y[0], KP_Y[1], KP_Y[2], KP_Y[3],
      KD_Y[0], KD_Y[1], KD_Y[2], KD_Y[3],
      LAMBDA, tau_max_);
    RCLCPP_INFO(get_logger(),
      "motor α=[%.1f %.1f %.1f %.1f]  Fv=[%.2f %.2f %.2f %.2f]  ε=%.3f",
      motor_alpha_(0), motor_alpha_(1), motor_alpha_(2), motor_alpha_(3),
      motor_Fv_(0), motor_Fv_(1), motor_Fv_(2), motor_Fv_(3), motor_epsilon_);
    if (vel_cutoff_hz_ > 0.0)
      RCLCPP_INFO(get_logger(),
        "Filtro velocidad: fc=%.1f Hz  α=%.4f  (lag@0.2Hz≈%.1f°)",
        vel_cutoff_hz_, vel_filter_alpha_,
        std::atan(0.2 / vel_cutoff_hz_) * 180.0 / PI);
    else
      RCLCPP_WARN(get_logger(), "Filtro velocidad DESACTIVADO (vel_cutoff_hz=0) — riesgo de chattering.");

    // ── Pinocchio ────────────────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    if (!model_.existFrame(EFF_FRAME_NAME)) {
      RCLCPP_FATAL(get_logger(), "Frame '%s' no encontrado en URDF", EFF_FRAME_NAME);
      throw std::runtime_error("Frame not found");
    }
    frame_id_ = model_.getFrameId(EFF_FRAME_NAME);
    RCLCPP_INFO(get_logger(), "Pinocchio: nv=%d  frame='%s' (id=%lu)",
      model_.nv, EFF_FRAME_NAME, frame_id_);

    // ── CSV ──────────────────────────────────────────────────────────────────
    std::filesystem::create_directories(std::string(PACKAGE_DATA_DIR) + "/lab4/real/act2");
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab4/real/act2/hw_io_data_"
                + std::to_string(log_id) + ".csv";
    csv_.open(csv_path_);
    if (csv_.is_open()) {
      csv_ << "t,q1,q2,q3,q4,"
              "dq1,dq2,dq3,dq4,"
              "dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
              "x,y,z,phi,x_des,y_des,z_des,phi_des,"
              "xdot,ydot,zdot,phidot,xdot_des,ydot_des,zdot_des,phidot_des,"
              "tau1,tau2,tau3,tau4,"
              "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
              "curr_meas1,curr_meas2,curr_meas3,curr_meas4,"
              "det_J4\n";
      RCLCPP_INFO(get_logger(), "CSV: %s", csv_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "No se pudo crear CSV: %s", csv_path_.c_str());
    }

    // ── Publisher de monitoreo ────────────────────────────────────────────────
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── SDK ──────────────────────────────────────────────────────────────────
    if (!init_hardware()) {
      RCLCPP_FATAL(get_logger(), "Fallo hardware init. Abortando.");
      throw std::runtime_error("Hardware init failed");
    }

    // ── Timer 100 Hz ─────────────────────────────────────────────────────────
    start_time_ = std::chrono::high_resolution_clock::now();
    timer_ = this->create_wall_timer(10ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Control IO activo a 100 Hz. Ctrl+C para detener.");
  }

  ~HWIOControlNode()
  {
    if (timer_) timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Hardware
  // ─────────────────────────────────────────────────────────────────────────

  bool init_hardware()
  {
    port_handler_   = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!port_handler_->openPort()) {
      RCLCPP_ERROR(get_logger(), "No se pudo abrir %s", port_name_.c_str());
      return false;
    }
    if (!port_handler_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(get_logger(), "No se pudo configurar baudrate");
      port_handler_->closePort();
      return false;
    }

    for (const auto id : DXL_ID)
      dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "pre-disable");

    for (const auto id : DXL_ID) {
      if (!dxl_write1(id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE, "set mode") ||
          !dxl_write2(id, ADDR_CURRENT_LIMIT, current_limit_register_, "set limit") ||
          !dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE_VAL, "enable torque")) {
        port_handler_->closePort();
        return false;
      }
      RCLCPP_INFO(get_logger(), "DXL ID %d listo", static_cast<int>(id));
    }

    grp_pos_  = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    grp_vel_  = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
    grp_cur_  = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT);
    grp_wcur_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    for (const auto id : DXL_ID) {
      grp_pos_->addParam(id);
      grp_vel_->addParam(id);
      grp_cur_->addParam(id);
    }

    hw_active_ = true;
    RCLCPP_INFO(get_logger(), "Hardware inicializado en %s", port_name_.c_str());
    return true;
  }

  void shutdown_hardware()
  {
    if (!hw_active_) return;
    hw_active_ = false;

    if (grp_wcur_) {
      const std::array<int16_t, NUM_JOINTS> zero = {0, 0, 0, 0};
      send_currents(zero);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    for (const auto id : DXL_ID)
      dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "shutdown disable");

    if (port_handler_) {
      port_handler_->closePort();
      RCLCPP_INFO(get_logger(), "Puerto cerrado.");
    }
  }

  bool dxl_write1(uint8_t id, uint16_t addr, uint8_t val, const char* lbl)
  {
    uint8_t err = 0;
    const int r = packet_handler_->write1ByteTxRx(port_handler_, id, addr, val, &err);
    if (r != COMM_SUCCESS || err != 0) {
      RCLCPP_WARN(get_logger(), "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
      return false;
    }
    return true;
  }

  bool dxl_write2(uint8_t id, uint16_t addr, uint16_t val, const char* lbl)
  {
    uint8_t err = 0;
    const int r = packet_handler_->write2ByteTxRx(port_handler_, id, addr, val, &err);
    if (r != COMM_SUCCESS || err != 0) {
      RCLCPP_WARN(get_logger(), "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
      return false;
    }
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Lectura / escritura SDK
  // ─────────────────────────────────────────────────────────────────────────

  bool read_state(Vec4& q, Vec4& dq, std::array<int16_t, NUM_JOINTS>& cur)
  {
    if (grp_pos_->txRxPacket() != COMM_SUCCESS ||
        grp_vel_->txRxPacket() != COMM_SUCCESS ||
        grp_cur_->txRxPacket() != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncRead fallo");
      return false;
    }
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const uint8_t id = DXL_ID[i];
      if (!grp_pos_->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) ||
          !grp_vel_->isAvailable(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY) ||
          !grp_cur_->isAvailable(id, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT)) {
        RCLCPP_ERROR(get_logger(), "[ID %d] dato no disponible", id);
        return false;
      }
      const int32_t rp = toSigned32(static_cast<uint32_t>(
        grp_pos_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)));
      const int32_t rv = toSigned32(static_cast<uint32_t>(
        grp_vel_->getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)));
      const int16_t rc = toSigned16(static_cast<uint32_t>(
        grp_cur_->getData(id, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT)));

      q(i)   = encoder_sign_(i) * static_cast<double>(wrappedTickDiff(rp, joint_zero_tick_[i])) * POS_UNIT_RAD;
      dq(i)  = encoder_sign_(i) * static_cast<double>(rv) * VEL_UNIT_RAD_S;
      cur[i] = rc;
    }
    return true;
  }

  bool send_currents(const std::array<int16_t, NUM_JOINTS>& cmd)
  {
    uint8_t p[NUM_JOINTS][2];
    for (int i = 0; i < NUM_JOINTS; ++i) {
      currentToBytes(cmd[i], p[i]);
      grp_wcur_->addParam(DXL_ID[i], p[i]);
    }
    const int r = grp_wcur_->txPacket();
    grp_wcur_->clearParam();
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncWrite fallo");
      return false;
    }
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Conversión torque → corriente (idéntica a act_2)
  // ─────────────────────────────────────────────────────────────────────────

  std::array<int16_t, NUM_JOINTS> torque_to_current(const Vec4& tau, const Vec4& dq)
  {
    std::array<int16_t, NUM_JOINTS> cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double I_model = motor_alpha_(i) * tau(i)
                           + motor_Fv_(i)    * dq(i)
                           + motor_Fc_(i)    * std::tanh(dq(i) / motor_epsilon_)
                           + motor_I_offset_(i);
      cmd[i] = clampCurrent(current_sign_(i) * encoder_sign_(i) * I_model, current_cmd_limit_[i]);
    }
    return cmd;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Ley de control IO (idéntica a act_2)
  // ─────────────────────────────────────────────────────────────────────────

  struct IOOut {
    Vec4            tau;
    Eigen::Vector4d y_actual, ydot_actual, ydot_des;
    double          det_J4;
  };

  IOOut compute_io_control(const Vec4& q, const Vec4& dq, const CartRef& ref)
  {
    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head(NUM_JOINTS)  = q;
    dq_pin.head(NUM_JOINTS) = dq;

    // FK + Jacobiano 6×nv
    Eigen::MatrixXd J6 = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::computeFrameJacobian(model_, data_, q_pin, frame_id_,
                                     pinocchio::LOCAL_WORLD_ALIGNED, J6);

    const Eigen::Vector3d p   = data_.oMf[frame_id_].translation();
    const double phi          = q[1] + q[2] + q[3];

    // Jacobiano de tarea 4×4
    Eigen::Matrix4d J4;
    J4.row(0) = J6.row(0).head<NUM_JOINTS>();
    J4.row(1) = J6.row(1).head<NUM_JOINTS>();
    J4.row(2) = J6.row(2).head<NUM_JOINTS>();
    J4.row(3) << 0.0, 1.0, 1.0, 1.0;

    const Eigen::Vector4d ydot = J4 * dq;

    // Término de bias  Jdot*qdot  (FK con qddot=0)
    pinocchio::forwardKinematics(model_, data_, q_pin, dq_pin,
                                  Eigen::VectorXd::Zero(model_.nv));
    const pinocchio::Motion bias =
      pinocchio::getFrameClassicalAcceleration(model_, data_, frame_id_,
                                                pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::Vector4d jdqd;
    jdqd << bias.linear()[0], bias.linear()[1], bias.linear()[2], 0.0;

    // Dinámica M(q) y b(q,qdot)
    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();
    pinocchio::nonLinearEffects(model_, data_, q_pin, dq_pin);

    const Eigen::Matrix4d M4   = data_.M.topLeftCorner<NUM_JOINTS, NUM_JOINTS>();
    const Eigen::Vector4d nle4 = data_.nle.head<NUM_JOINTS>();

    // Errores
    const Eigen::Vector4d y_actual{p[0], p[1], p[2], phi};
    const Eigen::Vector4d ey    = ref.y    - y_actual;
    const Eigen::Vector4d eydot = ref.ydot - ydot;

    const Eigen::Vector4d kp_y = KP_Y * gain_scale_;
    const Eigen::Vector4d kd_y = KD_Y * std::sqrt(std::max(gain_scale_, 0.0));

    // Entrada auxiliar
    const Eigen::Vector4d v = ref.yddot + kp_y.asDiagonal()*ey + kd_y.asDiagonal()*eydot;

    // Pseudo-inversa DLS
    const Eigen::Matrix4d A = J4*J4.transpose() + LAMBDA_SQ*Eigen::Matrix4d::Identity();
    const Eigen::Vector4d qddot = J4.transpose() * A.ldlt().solve(v - jdqd);

    // Torque con saturación
    const Eigen::Vector4d tau_unsat = M4*qddot + nle4;
    const Eigen::Vector4d tau_sat   = tau_unsat.cwiseMin(tau_max_).cwiseMax(-tau_max_);

    IOOut out;
    out.tau        = tau_sat;
    out.y_actual   = y_actual;
    out.ydot_actual = ydot;
    out.ydot_des   = ref.ydot;
    out.det_J4     = J4.determinant();
    return out;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Parada de emergencia
  // ─────────────────────────────────────────────────────────────────────────

  void emergency_stop(const std::string& reason)
  {
    RCLCPP_ERROR(get_logger(), "PARADA: %s", reason.c_str());
    timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    rclcpp::shutdown();
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Callback del timer (100 Hz)
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    if (!hw_active_) return;

    const auto tp  = std::chrono::high_resolution_clock::now();
    const double t = std::chrono::duration<double>(tp - start_time_).count();

    if (t_imp_ > 0.0 && t >= t_imp_) {
      RCLCPP_INFO(get_logger(), "Duración completada (%.1f s).", t_imp_);
      timer_->cancel();
      shutdown_hardware();
      if (csv_.is_open()) { csv_.flush(); csv_.close(); }
      rclcpp::shutdown();
      return;
    }

    // 1. Lectura de estado
    Vec4 q, dq;
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    if (!read_state(q, dq, cur_meas)) {
      emergency_stop("SyncRead fallido");
      return;
    }

    // 2. Filtro EMA de velocidad
    if (!dq_filter_initialized_) {
      dq_filtered_          = dq;
      dq_filter_initialized_ = true;
    } else {
      dq_filtered_ = vel_filter_alpha_ * dq_filtered_ + (1.0 - vel_filter_alpha_) * dq;
    }

    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head(NUM_JOINTS)  = q;
    dq_pin.head(NUM_JOINTS) = dq;

    // 3. Capturar pose cartesiana inicial (primer tick)
    if (!y0_initialized_) {
      pinocchio::forwardKinematics(model_, data_, q_pin);
      pinocchio::updateFramePlacement(model_, data_, frame_id_);
      const Eigen::Vector3d p0 = data_.oMf[frame_id_].translation();
      const double phi0        = q[1] + q[2] + q[3];
      y0_ = Eigen::Vector4d{p0[0], p0[1], p0[2], phi0};
      y0_initialized_ = true;
      RCLCPP_INFO(get_logger(),
        "y_inicial=[%.3f %.3f %.3f %.3f]  Y_START=[%.3f %.3f %.3f %.3f]",
        y0_[0], y0_[1], y0_[2], y0_[3],
        Y_START[0], Y_START[1], Y_START[2], Y_START[3]);
    }

    // 4. Verificar límites articulares del estado real
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (q(i) < joint_lower_(i) || q(i) > joint_upper_(i)) {
        emergency_stop("Articulacion " + std::to_string(i+1) + " fuera de limites: "
                       + std::to_string(q(i)) + " rad");
        return;
      }
    }

    // 5. Referencia cartesiana
    const CartRef ref = (t < T_TRANS)
      ? cartesianTransition(t, y0_, Y_START, T_TRANS)
      : cartesianTrajectory(t - T_TRANS);

    // 6. Ley de control IO — usa dq_filtered_ para reducir chattering
    const IOOut ctrl = compute_io_control(q, dq_filtered_, ref);

    // 7. Conversión torque → corriente
    const auto cur_cmd = torque_to_current(ctrl.tau, dq_filtered_);

    // 8. Escribir corriente
    if (!send_currents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 9. Verificar corriente medida
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > current_measured_peak_) {
        emergency_stop("Corriente insegura J" + std::to_string(i+1)
                       + ": " + std::to_string(cur_meas[i]) + " ticks");
        return;
      }
    }

    // 10. Publicar JointState de monitoreo
    {
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();
      js.name         = {"joint1", "joint2", "joint3", "joint4"};
      js.position     = {q(0), q(1), q(2), q(3)};
      js.velocity     = {dq(0), dq(1), dq(2), dq(3)};
      js.effort = {
        static_cast<double>(cur_meas[0]) * TORQUE_PER_CURRENT_TICK,
        static_cast<double>(cur_meas[1]) * TORQUE_PER_CURRENT_TICK,
        static_cast<double>(cur_meas[2]) * TORQUE_PER_CURRENT_TICK,
        static_cast<double>(cur_meas[3]) * TORQUE_PER_CURRENT_TICK
      };
      js_pub_->publish(js);
    }

    // 11. CSV
    if (csv_.is_open()) {
      const char* phase = (t < T_TRANS) ? "TRANS" : "TRAJ";
      (void)phase;
      csv_ << std::fixed << std::setprecision(6)
           << t
           << ',' << q(0) << ',' << q(1) << ',' << q(2) << ',' << q(3)
           << ',' << dq(0) << ',' << dq(1) << ',' << dq(2) << ',' << dq(3)
           << ',' << dq_filtered_(0) << ',' << dq_filtered_(1) << ',' << dq_filtered_(2) << ',' << dq_filtered_(3)
           << ',' << ctrl.y_actual[0] << ',' << ctrl.y_actual[1]
           << ',' << ctrl.y_actual[2] << ',' << ctrl.y_actual[3]
           << ',' << ref.y[0] << ',' << ref.y[1] << ',' << ref.y[2] << ',' << ref.y[3]
           << ',' << ctrl.ydot_actual[0] << ',' << ctrl.ydot_actual[1]
           << ',' << ctrl.ydot_actual[2] << ',' << ctrl.ydot_actual[3]
           << ',' << ctrl.ydot_des[0] << ',' << ctrl.ydot_des[1]
           << ',' << ctrl.ydot_des[2] << ',' << ctrl.ydot_des[3]
           << ',' << ctrl.tau[0] << ',' << ctrl.tau[1]
           << ',' << ctrl.tau[2] << ',' << ctrl.tau[3]
           << ',' << cur_cmd[0] << ',' << cur_cmd[1]
           << ',' << cur_cmd[2] << ',' << cur_cmd[3]
           << ',' << cur_meas[0] << ',' << cur_meas[1]
           << ',' << cur_meas[2] << ',' << cur_meas[3]
           << ',' << ctrl.det_J4
           << '\n';
    }

    // 12. Log periódico
    if (++log_cnt_ % 100 == 0) {
      if (csv_.is_open()) csv_.flush();
      const char* phase = (t < T_TRANS) ? "TRANS" : "TRAJ ";
      RCLCPP_INFO(get_logger(),
        "[%s] t=%.2fs  y=[%.3f %.3f %.3f %.3f]  |ey|=%.4f  detJ=%.4f",
        phase, t,
        ctrl.y_actual[0], ctrl.y_actual[1], ctrl.y_actual[2], ctrl.y_actual[3],
        (ref.y - ctrl.y_actual).norm(), ctrl.det_J4);
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  pinocchio::Model      model_;
  pinocchio::Data       data_;
  pinocchio::FrameIndex frame_id_;

  std::string port_name_;
  double gain_scale_, t_imp_;
  Vec4   motor_alpha_, motor_Fv_, motor_Fc_, motor_I_offset_;
  double motor_epsilon_;

  std::array<int32_t, NUM_JOINTS> joint_zero_tick_;
  Vec4     encoder_sign_, current_sign_;
  Vec4     joint_lower_, joint_upper_;
  uint16_t current_limit_register_;
  std::array<int16_t, NUM_JOINTS> current_cmd_limit_;
  int16_t  current_measured_peak_;
  double   tau_max_;

  double vel_cutoff_hz_;
  double vel_filter_alpha_;
  Vec4   dq_filtered_;
  bool   dq_filter_initialized_{false};

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead>  grp_pos_, grp_vel_, grp_cur_;
  std::unique_ptr<dynamixel::GroupSyncWrite> grp_wcur_;

  bool hw_active_;
  bool y0_initialized_;
  Eigen::Vector4d y0_;

  std::chrono::high_resolution_clock::time_point start_time_;
  int log_cnt_{0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::ofstream csv_;
  std::string   csv_path_;
};

// ============================================================
// main
// ============================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  const std::string cfg = std::string(PACKAGE_CONFIG_DIR) + "/motor_params.yaml";
  if (std::filesystem::exists(cfg)) {
    std::vector<std::string> args = {"--ros-args", "--params-file", cfg};
    bool in_ros_args = false;
    for (int i = 1; i < argc; ++i) {
      const std::string a(argv[i]);
      if (a == "--ros-args") { in_ros_args = true; continue; }
      if (in_ros_args) args.push_back(a);
    }
    opts.arguments(args);
    opts.use_global_arguments(false);
    RCLCPP_INFO(rclcpp::get_logger("hw_io_control_node"),
      "motor_params auto-cargado: %s", cfg.c_str());
  }

  try {
    rclcpp::spin(std::make_shared<HWIOControlNode>(opts));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_io_control_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
