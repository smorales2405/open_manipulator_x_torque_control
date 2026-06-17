/*
 * hw_sinusoidal_torque_node.cpp
 * Diagnóstico: torques sinusoidales y/o control de posición por articulación.
 *
 * Cada articulación puede configurarse independientemente en:
 *   "current"  — envía τ_i(t') = A_i + B_i·sin(w_i·t')  [N·m → corriente via modelo OLS]
 *   "position" — sigue q_ref_i(t') = A_pos_i + B_pos_i·sin(w_pos_i·t') [rad] via Position Control Mode
 *
 * Secuencia de fases:
 *   SETTLING (t < t_settle):
 *     → TODOS los joints en Position Control Mode, van a pos_rad_i.
 *     → Permite que el robot alcance la configuración inicial antes de aplicar torques.
 *     → pos_rad: [0.0, 0.0, 0.0, 0.0]  lleva el robot a la posición cero de cada joint.
 *   RAMP (t_settle ≤ t_ctrl < t_ramp):
 *     → Joints en mode="current" cambian a Current Control Mode.
 *     → Rampa lineal del offset DC: τ_i = (t_ctrl/t_ramp)·A_i
 *   RUN (t_ctrl ≥ t_ramp):
 *     → Señal completa: τ_i = A_i + B_i·sin(w_i·t')
 *
 * Ejecución:
 *   ros2 run open_manipulator_x_torque_control hw_sinusoidal_torque_node \
 *       --ros-args -p log_id:=3
 *
 * CSV de salida: data/diagnostics/sinusoidal/hw_sin_torque_<log_id>.csv
 *   t, q1..4, dq1..4, tau_ref1..4, pos_ref1..4, curr_cmd1..4, curr_meas1..4
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <Eigen/Dense>

#include "dynamixel_sdk/dynamixel_sdk.h"

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_CONFIG_DIR
#define PACKAGE_CONFIG_DIR "."
#endif

using namespace std::chrono_literals;

// ============================================================
// Constantes Dynamixel / conversión de unidades
// ============================================================

static constexpr double PI         = 3.14159265358979323846;
static constexpr int    NUM_JOINTS = 4;

static const std::array<uint8_t, NUM_JOINTS> DXL_ID = {11, 12, 13, 14};
static constexpr int    BAUDRATE         = 1000000;
static constexpr double PROTOCOL_VERSION = 2.0;

// Control Table addresses (XM430-W350)
static constexpr uint16_t ADDR_PROFILE_ACC      = 108;
static constexpr uint16_t ADDR_PROFILE_VEL      = 112;
static constexpr uint16_t ADDR_OPERATING_MODE   = 11;
static constexpr uint16_t ADDR_CURRENT_LIMIT    = 38;
static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
static constexpr uint16_t ADDR_GOAL_CURRENT     = 102;
static constexpr uint16_t ADDR_GOAL_POSITION    = 116;
static constexpr uint16_t ADDR_PRESENT_CURRENT  = 126;
static constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
static constexpr uint16_t ADDR_PRESENT_POSITION = 132;

static constexpr uint16_t LEN_GOAL_CURRENT      = 2;
static constexpr uint16_t LEN_GOAL_POSITION      = 4;
static constexpr uint16_t LEN_PRESENT_CURRENT   = 2;
static constexpr uint16_t LEN_PRESENT_VELOCITY  = 4;
static constexpr uint16_t LEN_PRESENT_POSITION  = 4;

static constexpr uint8_t CURRENT_CONTROL_MODE   = 0;
static constexpr uint8_t POSITION_CONTROL_MODE  = 3;
static constexpr uint8_t TORQUE_ENABLE_VAL      = 1;
static constexpr uint8_t TORQUE_DISABLE_VAL     = 0;

static constexpr double POS_UNIT_RAD            = 2.0 * PI / 4096.0;
static constexpr double VEL_UNIT_RAD_S          = 0.229 * 2.0 * PI / 60.0;

static const std::array<int32_t, NUM_JOINTS> JOINT_ZERO_TICK = {2048, 2048, 2048, 2048};
static const std::array<double,  NUM_JOINTS> ENCODER_SIGN    = {+1.0, +1.0, +1.0, +1.0};
static const std::array<double,  NUM_JOINTS> CURRENT_SIGN    = {+1.0, +1.0, +1.0, +1.0};

static constexpr uint16_t CURRENT_LIMIT_REGISTER  = 350;
static constexpr int16_t  CURRENT_CMD_LIMIT_J123  = 257;
static constexpr int16_t  CURRENT_CMD_LIMIT_J4    = 257;
static constexpr int16_t  CURRENT_MEASURED_PEAK   = 313;
static constexpr double   TAU_MAX                 = 1.2;   // [N·m]

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Fases de operación
// ============================================================

enum class Phase { SETTLING, RAMP, RUN };

// ============================================================
// Utilidades de conversión
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

// Convierte pos_rad [rad] al tick absoluto del servo
static int32_t radToTicks(int joint_idx, double pos_rad)
{
  const int32_t t = static_cast<int32_t>(std::round(
    JOINT_ZERO_TICK[joint_idx]
    + ENCODER_SIGN[joint_idx] * pos_rad / POS_UNIT_RAD));
  return std::clamp(t, 0, 4095);
}

static void currentToBytes(int16_t cur, uint8_t p[2])
{
  const uint16_t w = static_cast<uint16_t>(cur);
  p[0] = DXL_LOBYTE(w);
  p[1] = DXL_HIBYTE(w);
}

static void positionToBytes(int32_t ticks, uint8_t p[4])
{
  const uint32_t u = static_cast<uint32_t>(std::clamp(ticks, 0, 4095));
  p[0] = DXL_LOBYTE(DXL_LOWORD(u));
  p[1] = DXL_HIBYTE(DXL_LOWORD(u));
  p[2] = DXL_LOBYTE(DXL_HIWORD(u));
  p[3] = DXL_HIBYTE(DXL_HIWORD(u));
}

static int16_t clampCurrent(double x, int16_t lim)
{
  return static_cast<int16_t>(std::lround(
    std::min(std::max(x, -static_cast<double>(lim)), static_cast<double>(lim))));
}

// ============================================================
// Nodo ROS 2
// ============================================================

class HWSinusoidalTorqueNode : public rclcpp::Node
{
public:
  explicit HWSinusoidalTorqueNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("hw_sinusoidal_torque_node", opts), hw_active_(false), phase_(Phase::SETTLING)
  {
    // ── Parámetros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",               "/dev/ttyUSB0");
    this->declare_parameter<bool>       ("open_port",               false);
    this->declare_parameter<bool>       ("enable_torque",           false);
    this->declare_parameter<bool>       ("enable_current_commands", false);
    this->declare_parameter<int>        ("log_id",                  1);
    this->declare_parameter<double>     ("t_settle",                3.0);
    this->declare_parameter<double>     ("t_ramp",                  3.0);
    this->declare_parameter<double>     ("duration_s",              30.0);

    using svec = std::vector<std::string>;
    using dvec = std::vector<double>;
    this->declare_parameter<svec>("mode",    svec{"current","current","current","current"});
    this->declare_parameter<dvec>("A",       dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("B",       dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("w",       dvec{1.0, 1.0, 1.0, 1.0});
    this->declare_parameter<dvec>("A_pos",   dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("B_pos",   dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("w_pos",   dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("pos_rad", dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<int> ("pos_profile_vel", 50);
    this->declare_parameter<int> ("pos_profile_acc", 20);

    this->declare_parameter<dvec>  ("motor_alpha",            dvec{208.5,208.5,208.5,208.5});
    this->declare_parameter<dvec>  ("motor_Fv",               dvec{0.0,  0.0,  0.0,  0.0 });
    this->declare_parameter<dvec>  ("motor_Fc",               dvec{0.0,  0.0,  0.0,  0.0 });
    this->declare_parameter<dvec>  ("motor_I_offset",         dvec{0.0,  0.0,  0.0,  0.0 });
    this->declare_parameter<double>("motor_epsilon_friction", 0.05);

    // ── Lectura ──────────────────────────────────────────────────────────────
    port_name_               = get_parameter("port_name").as_string();
    open_port_               = get_parameter("open_port").as_bool();
    enable_torque_           = get_parameter("enable_torque").as_bool();
    enable_current_commands_ = get_parameter("enable_current_commands").as_bool();
    const int log_id         = get_parameter("log_id").as_int();
    t_settle_                = get_parameter("t_settle").as_double();
    t_ramp_                  = get_parameter("t_ramp").as_double();
    duration_s_              = get_parameter("duration_s").as_double();

    joint_mode_ = get_parameter("mode").as_string_array();
    if ((int)joint_mode_.size() != NUM_JOINTS)
      throw std::runtime_error("Parámetro 'mode' debe tener 4 elementos");
    for (const auto& m : joint_mode_) {
      if (m != "current" && m != "position")
        throw std::runtime_error("Modo inválido '" + m + "'; usar 'current' o 'position'");
    }

    auto load_vec4 = [this](const std::string& name) {
      auto v = get_parameter(name).as_double_array();
      return Vec4(v[0], v[1], v[2], v[3]);
    };
    A_dc_           = load_vec4("A");
    B_amp_          = load_vec4("B");
    w_freq_         = load_vec4("w");
    A_pos_          = load_vec4("A_pos");
    B_pos_          = load_vec4("B_pos");
    w_pos_          = load_vec4("w_pos");
    pos_rad_        = load_vec4("pos_rad");
    pos_profile_vel_ = get_parameter("pos_profile_vel").as_int();
    pos_profile_acc_ = get_parameter("pos_profile_acc").as_int();

    motor_alpha_    = load_vec4("motor_alpha");
    motor_Fv_       = load_vec4("motor_Fv");
    motor_Fc_       = load_vec4("motor_Fc");
    motor_I_offset_ = load_vec4("motor_I_offset");
    motor_epsilon_  = get_parameter("motor_epsilon_friction").as_double();

    // ── Log de configuración ─────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(),
      "hw=[port=%d torq=%d cmd=%d]  settle=%.1fs  ramp=%.1fs  dur=%.1fs  id=%d",
      (int)open_port_, (int)enable_torque_, (int)enable_current_commands_,
      t_settle_, t_ramp_, duration_s_, log_id);
    RCLCPP_INFO(get_logger(),
      "FASE SETTLING: todos los joints → pos_rad [%.3f, %.3f, %.3f, %.3f] rad (%.1f s)",
      pos_rad_(0), pos_rad_(1), pos_rad_(2), pos_rad_(3), t_settle_);
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] == "current") {
        RCLCPP_INFO(get_logger(),
          "J%d [current]  A=%.3f  B=%.3f  w=%.3f rad/s  (inicio tras settling)",
          i+1, A_dc_(i), B_amp_(i), w_freq_(i));
        const double tau_pk = std::abs(A_dc_(i)) + std::abs(B_amp_(i));
        if (tau_pk > TAU_MAX)
          RCLCPP_WARN(get_logger(),
            "J%d: |A|+|B|=%.3f N·m > TAU_MAX=%.3f N·m → se saturará", i+1, tau_pk, TAU_MAX);
      } else {
        if (std::abs(B_pos_(i)) > 1e-9) {
          RCLCPP_INFO(get_logger(),
            "J%d [position] A=%.4f rad  B=%.4f rad  w=%.4f rad/s  (settling→%.4f rad)",
            i+1, A_pos_(i), B_pos_(i), w_pos_(i), pos_rad_(i));
        } else {
          RCLCPP_INFO(get_logger(),
            "J%d [position] setpoint=%.4f rad (%.2f°)  profile_vel=%d ticks",
            i+1, A_pos_(i), A_pos_(i)*180.0/PI, pos_profile_vel_);
        }
        const double abs_ticks = JOINT_ZERO_TICK[i]
          + ENCODER_SIGN[i] * A_pos_(i) / POS_UNIT_RAD;
        if (abs_ticks < 0.0 || abs_ticks > 4095.0)
          RCLCPP_WARN(get_logger(),
            "J%d: A_pos=%.4f rad → %.1f ticks FUERA del rango [0,4095]",
            i+1, A_pos_(i), abs_ticks);
      }
    }

    // ── CSV de salida ────────────────────────────────────────────────────────
    std::filesystem::create_directories(
      std::string(PACKAGE_DATA_DIR) + "/diagnostics/sinusoidal");
    csv_path_ = std::string(PACKAGE_DATA_DIR)
      + "/diagnostics/sinusoidal/hw_sin_torque_" + std::to_string(log_id) + ".csv";
    csv_.open(csv_path_);
    if (csv_.is_open()) {
      csv_ << "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,"
              "tau_ref1,tau_ref2,tau_ref3,tau_ref4,"
              "pos_ref1,pos_ref2,pos_ref3,pos_ref4,"
              "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
              "curr_meas1,curr_meas2,curr_meas3,curr_meas4\n";
      RCLCPP_INFO(get_logger(), "CSV salida: %s", csv_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "No se pudo crear CSV: %s", csv_path_.c_str());
    }

    // ── Publisher ────────────────────────────────────────────────────────────
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── Hardware ─────────────────────────────────────────────────────────────
    if (open_port_) {
      if (!init_hardware()) {
        RCLCPP_FATAL(get_logger(), "Fallo hardware init.");
        throw std::runtime_error("Hardware init failed");
      }
    } else {
      RCLCPP_WARN(get_logger(), "MODO SECO (open_port=false) — sin hardware real.");
    }

    // ── Timer 100 Hz ─────────────────────────────────────────────────────────
    start_time_ = std::chrono::high_resolution_clock::now();
    ctrl_start_time_ = start_time_;  // se actualiza al terminar settling
    timer_       = this->create_wall_timer(10ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(),
      "Settling activo: %.1f s en posición inicial → luego control sinusoidal a 100 Hz.",
      t_settle_);
  }

  ~HWSinusoidalTorqueNode()
  {
    if (timer_) timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Generación de torque (current mode)
  //  t_ctrl: tiempo relativo al inicio del control (después del settling)
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_tau_ref(double t_ctrl) const
  {
    Vec4 tau = Vec4::Zero();
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "current") continue;
      if (t_ctrl < t_ramp_) {
        tau(i) = (t_ctrl / t_ramp_) * A_dc_(i);
      } else {
        const double tp = t_ctrl - t_ramp_;
        tau(i) = A_dc_(i) + B_amp_(i) * std::sin(w_freq_(i) * tp);
      }
      tau(i) = std::clamp(tau(i), -TAU_MAX, TAU_MAX);
    }
    return tau;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Generación de referencia de posición (position mode)
  //  q_ref_i(t_ctrl) = A_pos_i + B_pos_i * sin(w_pos_i * t_ctrl)
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_pos_ref(double t_ctrl) const
  {
    Vec4 ref = Vec4::Zero();
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "position") continue;
      ref(i) = A_pos_(i) + B_pos_(i) * std::sin(w_pos_(i) * t_ctrl);
    }
    return ref;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Conversión torque → corriente (modelo OLS identificado)
  // ─────────────────────────────────────────────────────────────────────────

  std::array<int16_t, NUM_JOINTS> torque_to_current(const Vec4& tau, const Vec4& dq) const
  {
    static const std::array<int16_t, NUM_JOINTS> cur_lim = {
      CURRENT_CMD_LIMIT_J123, CURRENT_CMD_LIMIT_J123,
      CURRENT_CMD_LIMIT_J123, CURRENT_CMD_LIMIT_J4
    };
    std::array<int16_t, NUM_JOINTS> cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "current") { cmd[i] = 0; continue; }
      const double I = motor_alpha_(i) * tau(i)
                     + motor_Fv_(i)    * dq(i)
                     + motor_Fc_(i)    * std::tanh(dq(i) / motor_epsilon_)
                     + motor_I_offset_(i);
      cmd[i] = clampCurrent(CURRENT_SIGN[i] * ENCODER_SIGN[i] * I, cur_lim[i]);
    }
    return cmd;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Hardware — inicialización
  //  Todos los joints arrancan en POSITION_CONTROL_MODE para la fase settling.
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

    // Deshabilitar todos antes de cambiar modo
    for (const auto id : DXL_ID)
      dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "pre-disable");

    if (enable_torque_) {
      // TODOS los joints arrancan en Position Control Mode (para settling)
      for (int i = 0; i < NUM_JOINTS; ++i) {
        const uint8_t id = DXL_ID[i];
        if (!dxl_write1(id, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE, "set pos mode") ||
            !dxl_write2(id, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_REGISTER, "set limit"))
          { port_handler_->closePort(); return false; }
        if (!dxl_write4(id, ADDR_PROFILE_ACC, static_cast<uint32_t>(pos_profile_acc_),
                        "set profile acc") ||
            !dxl_write4(id, ADDR_PROFILE_VEL, static_cast<uint32_t>(pos_profile_vel_),
                        "set profile vel"))
          { port_handler_->closePort(); return false; }
        if (!dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE_VAL, "enable"))
          { port_handler_->closePort(); return false; }
        RCLCPP_INFO(get_logger(),
          "DXL ID %d → POSITION mode (settling)  target=%.4f rad",
          static_cast<int>(id), pos_rad_(i));
      }
    } else {
      RCLCPP_WARN(get_logger(), "enable_torque=false — motores en modo lectura.");
    }

    // SyncRead (todos los joints)
    grp_pos_  = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    grp_vel_  = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
    grp_cur_  = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT);
    for (const auto id : DXL_ID) {
      grp_pos_->addParam(id);
      grp_vel_->addParam(id);
      grp_cur_->addParam(id);
    }

    // SyncWrite — grupos separados
    grp_wcur_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, ADDR_GOAL_CURRENT,  LEN_GOAL_CURRENT);
    grp_wpos_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

    hw_active_ = true;
    RCLCPP_INFO(get_logger(), "Hardware inicializado en %s", port_name_.c_str());
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Cambio de modo: joints "current" → CURRENT_CONTROL_MODE
  //  Se llama una sola vez al terminar la fase SETTLING.
  // ─────────────────────────────────────────────────────────────────────────

  void switch_to_run_modes()
  {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "current") continue;
      const uint8_t id = DXL_ID[i];
      dxl_write1(id, ADDR_TORQUE_ENABLE,  TORQUE_DISABLE_VAL,     "mode-switch disable");
      dxl_write1(id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE,   "set current mode");
      dxl_write2(id, ADDR_CURRENT_LIMIT,  CURRENT_LIMIT_REGISTER, "set limit");
      dxl_write1(id, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE_VAL,      "mode-switch enable");
      RCLCPP_INFO(get_logger(), "DXL ID %d → CURRENT mode", static_cast<int>(id));
    }
    RCLCPP_INFO(get_logger(), "Settling completado → iniciando control sinusoidal.");
  }

  void shutdown_hardware()
  {
    if (!hw_active_) return;
    hw_active_ = false;

    if (grp_wcur_ && phase_ != Phase::SETTLING) {
      const std::array<int16_t, NUM_JOINTS> zero{};
      send_currents(zero);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    for (const auto id : DXL_ID)
      dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "shutdown");
    if (port_handler_) {
      port_handler_->closePort();
      RCLCPP_INFO(get_logger(), "Puerto cerrado.");
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Primitivas Dynamixel
  // ─────────────────────────────────────────────────────────────────────────

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

  bool dxl_write4(uint8_t id, uint16_t addr, uint32_t val, const char* lbl)
  {
    uint8_t err = 0;
    const int r = packet_handler_->write4ByteTxRx(port_handler_, id, addr, val, &err);
    if (r != COMM_SUCCESS || err != 0) {
      RCLCPP_WARN(get_logger(), "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
      return false;
    }
    return true;
  }

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

      q(i)   = ENCODER_SIGN[i] * static_cast<double>(wrappedTickDiff(rp, JOINT_ZERO_TICK[i])) * POS_UNIT_RAD;
      dq(i)  = ENCODER_SIGN[i] * static_cast<double>(rv) * VEL_UNIT_RAD_S;
      cur[i] = rc;
    }
    return true;
  }

  // Envía goal position a TODOS los joints (fase settling)
  bool send_positions_all()
  {
    uint8_t p[NUM_JOINTS][4];
    for (int i = 0; i < NUM_JOINTS; ++i) {
      positionToBytes(radToTicks(i, pos_rad_(i)), p[i]);
      grp_wpos_->addParam(DXL_ID[i], p[i]);
    }
    const int r = grp_wpos_->txPacket();
    grp_wpos_->clearParam();
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncWrite (position all) fallo");
      return false;
    }
    return true;
  }

  // Envía goal position a joints en modo "position" (fase control)
  // pos_ref: referencia sinusoidal calculada por compute_pos_ref()
  bool send_positions(const Vec4& pos_ref)
  {
    uint8_t p[NUM_JOINTS][4];
    bool any = false;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "position") continue;
      positionToBytes(radToTicks(i, pos_ref(i)), p[i]);
      grp_wpos_->addParam(DXL_ID[i], p[i]);
      any = true;
    }
    if (!any) return true;
    const int r = grp_wpos_->txPacket();
    grp_wpos_->clearParam();
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncWrite (position) fallo");
      return false;
    }
    return true;
  }

  // Envía corriente solo a joints en modo "current"
  bool send_currents(const std::array<int16_t, NUM_JOINTS>& cmd)
  {
    uint8_t p[NUM_JOINTS][2];
    bool any = false;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (joint_mode_[i] != "current") continue;
      currentToBytes(cmd[i], p[i]);
      grp_wcur_->addParam(DXL_ID[i], p[i]);
      any = true;
    }
    if (!any) return true;
    const int r = grp_wcur_->txPacket();
    grp_wcur_->clearParam();
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncWrite (current) fallo");
      return false;
    }
    return true;
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
    const auto   tp = std::chrono::high_resolution_clock::now();
    const double t  = std::chrono::duration<double>(tp - start_time_).count();

    if (duration_s_ > 0.0 && t >= duration_s_) {
      RCLCPP_INFO(get_logger(), "Duración completada (%.1f s).", t);
      timer_->cancel();
      shutdown_hardware();
      if (csv_.is_open()) { csv_.flush(); csv_.close(); }
      rclcpp::shutdown();
      return;
    }

    // ── 1. Lectura de estado ──────────────────────────────────────────────
    Vec4 q = Vec4::Zero(), dq = Vec4::Zero();
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    if (hw_active_) {
      if (!read_state(q, dq, cur_meas)) {
        emergency_stop("SyncRead fallido"); return;
      }
    }

    // ── 2. Lógica de fase ─────────────────────────────────────────────────
    Vec4 tau_ref = Vec4::Zero();
    std::array<int16_t, NUM_JOINTS> cur_cmd{};
    Vec4 pos_cmd_rad = Vec4::Zero();

    if (phase_ == Phase::SETTLING) {
      // Enviar posición a TODOS los joints
      pos_cmd_rad = pos_rad_;
      if (hw_active_ && enable_current_commands_) {
        if (!send_positions_all()) { emergency_stop("SyncWrite pos (settling) fallo"); return; }
      }

      // Transición al control cuando se cumple t_settle
      if (t >= t_settle_) {
        if (hw_active_) switch_to_run_modes();
        ctrl_start_time_ = tp;
        phase_ = Phase::RAMP;
      }

    } else {
      // Fases RAMP y RUN
      const double t_ctrl = std::chrono::duration<double>(tp - ctrl_start_time_).count();
      phase_ = (t_ctrl < t_ramp_) ? Phase::RAMP : Phase::RUN;

      tau_ref     = compute_tau_ref(t_ctrl);
      pos_cmd_rad = compute_pos_ref(t_ctrl);
      cur_cmd     = torque_to_current(tau_ref, dq);

      if (hw_active_ && enable_current_commands_) {
        if (!send_currents(cur_cmd))        { emergency_stop("SyncWrite current fallo"); return; }
        if (!send_positions(pos_cmd_rad))   { emergency_stop("SyncWrite position fallo"); return; }
      }
    }

    // ── 3. Verificación de corriente medida ───────────────────────────────
    if (hw_active_) {
      for (int i = 0; i < NUM_JOINTS; ++i) {
        if (std::abs(cur_meas[i]) > CURRENT_MEASURED_PEAK) {
          emergency_stop("Corriente insegura J" + std::to_string(i + 1)
                         + ": " + std::to_string(cur_meas[i]) + " ticks");
          return;
        }
      }
    }

    // ── 4. Publicar JointState ────────────────────────────────────────────
    {
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();
      js.name         = {"joint1","joint2","joint3","joint4"};
      js.position     = {q(0),  q(1),  q(2),  q(3)};
      js.velocity     = {dq(0), dq(1), dq(2), dq(3)};
      js.effort       = {tau_ref(0), tau_ref(1), tau_ref(2), tau_ref(3)};
      js_pub_->publish(js);
    }

    // ── 5. CSV ────────────────────────────────────────────────────────────
    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6) << t
           << ',' << q(0)           << ',' << q(1)           << ',' << q(2)           << ',' << q(3)
           << ',' << dq(0)          << ',' << dq(1)          << ',' << dq(2)          << ',' << dq(3)
           << ',' << tau_ref(0)     << ',' << tau_ref(1)     << ',' << tau_ref(2)     << ',' << tau_ref(3)
           << ',' << pos_cmd_rad(0) << ',' << pos_cmd_rad(1) << ',' << pos_cmd_rad(2) << ',' << pos_cmd_rad(3)  // pos_ref
           << ',' << cur_cmd[0]     << ',' << cur_cmd[1]     << ',' << cur_cmd[2]     << ',' << cur_cmd[3]
           << ',' << cur_meas[0]    << ',' << cur_meas[1]    << ',' << cur_meas[2]    << ',' << cur_meas[3]
           << '\n';
    }

    // ── 6. Log periódico ──────────────────────────────────────────────────
    if (++log_cnt_ % 100 == 0) {
      if (csv_.is_open()) csv_.flush();
      const char* fase = (phase_ == Phase::SETTLING) ? "SETTL"
                       : (phase_ == Phase::RAMP)     ? "RAMP "
                                                     : "RUN  ";
      RCLCPP_INFO(get_logger(),
        "[%s] t=%.2fs  q=[%.3f %.3f %.3f %.3f] rad  τ=[%.3f %.3f %.3f %.3f] N·m",
        fase, t, q(0), q(1), q(2), q(3),
        tau_ref(0), tau_ref(1), tau_ref(2), tau_ref(3));
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  std::string port_name_;
  bool open_port_, enable_torque_, enable_current_commands_;
  double t_settle_, t_ramp_, duration_s_;

  std::vector<std::string> joint_mode_;
  Vec4 A_dc_, B_amp_, w_freq_;             // parámetros señal de torque (current mode)
  Vec4 A_pos_, B_pos_, w_pos_;             // parámetros trayectoria de posición (position mode)
  Vec4 pos_rad_;                           // posición inicial de settling (todos los joints)
  int  pos_profile_vel_, pos_profile_acc_;

  Vec4   motor_alpha_, motor_Fv_, motor_Fc_, motor_I_offset_;
  double motor_epsilon_;

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead>  grp_pos_, grp_vel_, grp_cur_;
  std::unique_ptr<dynamixel::GroupSyncWrite> grp_wcur_, grp_wpos_;

  bool  hw_active_;
  Phase phase_;

  std::string   csv_path_;
  std::ofstream csv_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::high_resolution_clock::time_point start_time_;
  std::chrono::high_resolution_clock::time_point ctrl_start_time_;
  int log_cnt_{0};
};

// ============================================================
// main — carga automática del YAML de configuración
// ============================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  const std::string cfg = std::string(PACKAGE_CONFIG_DIR) + "/hw_sinusoidal_torque_params.yaml";

  if (std::filesystem::exists(cfg)) {
    // Construimos la lista de args combinando: YAML primero, luego los --ros-args
    // del usuario. Así los -p pasados por línea de comandos sobreescriben el YAML.
    std::vector<std::string> args = {"--ros-args", "--params-file", cfg};
    bool in_ros_args = false;
    for (int i = 1; i < argc; ++i) {
      const std::string a(argv[i]);
      if (a == "--ros-args") { in_ros_args = true; continue; }
      if (in_ros_args) args.push_back(a);
    }
    opts.arguments(args);
    opts.use_global_arguments(false);   // evitar doble procesamiento
    RCLCPP_INFO(rclcpp::get_logger("main"), "Config auto-cargado: %s", cfg.c_str());
  } else {
    RCLCPP_WARN(rclcpp::get_logger("main"),
      "Config no encontrado: %s — usando defaults del código.", cfg.c_str());
  }

  try {
    rclcpp::spin(std::make_shared<HWSinusoidalTorqueNode>(opts));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Excepcion: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
