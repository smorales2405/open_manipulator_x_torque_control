/*
 * hw_fl_control_node.cpp
 * Feedback Linearization / Computed Torque — OpenMANIPULATOR-X hardware real
 * vía Dynamixel SDK directo, sin ros2_control.
 *
 * Ley de control:
 *   tau = M(q)*v + h(q,dq),  v = ddq_des - Kd*(dq-dq_des) - Kp*(q-q_des)
 *
 * Parámetros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name       [string]  "/dev/ttyUSB0"
 *   gain_scale      [double]  10.0   (escala lineal de Kp; Kd escala con sqrt)
 *   deadzone_ticks  [double]  30.0   (compensación zona muerta en ticks de corriente)
 *   viscous_comp    [double]  5.0   (compensación viscosa en unidades de corriente/rad·s)
 *   duration_s      [double]  20.0  (0 = sin límite)
 *   log_id          [int]     1     (nombre del CSV: hw_fl_data_<log_id>.csv)
 *
 * Publisher: /hw/joint_states (sensor_msgs/JointState) — solo monitoreo
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

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "dynamixel_sdk/dynamixel_sdk.h"

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
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
static constexpr double TORQUE_CONSTANT_NM_A   = 1.666;
static constexpr double TORQUE_PER_CURRENT_TICK = TORQUE_CONSTANT_NM_A * CURRENT_UNIT_A;

static const std::array<int32_t, NUM_JOINTS> JOINT_ZERO_TICK = {2048, 2048, 2048, 2048};
static const std::array<double,  NUM_JOINTS> ENCODER_SIGN    = {+1.0, -1.0, -1.0, -1.0};
static const std::array<double,  NUM_JOINTS> CURRENT_SIGN    = {+1.0, +1.0, +1.0, +1.0};

static const std::array<double, NUM_JOINTS> JOINT_LOWER = {
  -1.570796, -1.570796, -1.570796, -1.790707812546182
};
static const std::array<double, NUM_JOINTS> JOINT_UPPER = {
  +1.570796, +1.570796, +1.570796, +2.0420352248333655
};

static constexpr uint16_t CURRENT_LIMIT_REGISTER = 300;
static constexpr int16_t  CURRENT_CMD_LIMIT_J123 = 190;
static constexpr int16_t  CURRENT_CMD_LIMIT_J4   = 152;
static constexpr int16_t  CURRENT_MEASURED_PEAK  = 220;

static constexpr double RAMP_TIME_S = 3.0;

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Utilidades de conversión (idénticas a act_1)
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
// Trayectoria deseada (articular, idéntica a act_1)
// ============================================================

struct Reference { Vec4 q, dq, ddq; };

static Reference desiredTrajectory(double t)
{
  const double w = 1.0;
  Reference r;
  r.q   << (PI/4.0)*std::sin(w*t),      -0.5 + 0.5*std::sin(w*t),     0.3 - 0.5*std::sin(w*t),    (PI/4.0);
  r.dq  << (PI/4.0)*w*std::cos(w*t),     0.5*w*std::cos(w*t),         -0.5*w*std::cos(w*t),          0.0;
  r.ddq << -(PI/4.0)*w*w*std::sin(w*t), -0.5*w*w*std::sin(w*t),       0.5*w*w*std::sin(w*t),         0.0;
  return r;
}

static Reference quinticTransition(double t, double T, const Vec4& q0)
{
  const Reference target = desiredTrajectory(T);
  if (T <= 0.0) return target;

  const Vec4 v0 = Vec4::Zero();
  const Vec4 a0 = Vec4::Zero();
  const Vec4 qf = target.q, vf = target.dq, af = target.ddq;

  const double T2=T*T, T3=T2*T, T4=T3*T, T5=T4*T;
  const Vec4 c0 = q0;
  const Vec4 c1 = v0;
  const Vec4 c2 = 0.5 * a0;
  const Vec4 c3 = (20.0*(qf-q0) - (8.0*vf+12.0*v0)*T - (3.0*a0-af)*T2) / (2.0*T3);
  const Vec4 c4 = (30.0*(q0-qf) + (14.0*vf+16.0*v0)*T + (3.0*a0-2.0*af)*T2) / (2.0*T4);
  const Vec4 c5 = (12.0*(qf-q0) - (6.0*vf+6.0*v0)*T - (a0-af)*T2) / (2.0*T5);

  const double t2=t*t, t3=t2*t, t4=t3*t, t5=t4*t;
  Reference r;
  r.q   = c0 + c1*t  + c2*t2  + c3*t3  + c4*t4  + c5*t5;
  r.dq  = c1 + 2.0*c2*t + 3.0*c3*t2 + 4.0*c4*t3 + 5.0*c5*t4;
  r.ddq = 2.0*c2 + 6.0*c3*t + 12.0*c4*t2 + 20.0*c5*t3;
  return r;
}

// ============================================================
// Nodo ROS 2
// ============================================================

class HWFLControlNode : public rclcpp::Node
{
public:
  HWFLControlNode()
  : Node("hw_fl_control_node"),
    hw_active_(false), q_initial_captured_(false)
  {
    // ── Parámetros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",     "/dev/ttyUSB0");
    this->declare_parameter<double>     ("gain_scale",     10.0);
    this->declare_parameter<double>     ("deadzone_ticks", 30.0);
    this->declare_parameter<double>     ("viscous_comp",   5.0);
    this->declare_parameter<double>     ("duration_s",     20.0);
    this->declare_parameter<int>        ("log_id",         1);

    port_name_      = this->get_parameter("port_name").as_string();
    gain_scale_     = this->get_parameter("gain_scale").as_double();
    deadzone_ticks_ = this->get_parameter("deadzone_ticks").as_double();
    viscous_comp_   = this->get_parameter("viscous_comp").as_double();
    duration_s_     = this->get_parameter("duration_s").as_double();
    const int log_id = this->get_parameter("log_id").as_int();

    RCLCPP_INFO(get_logger(), "puerto=%s  gain=%.2f  dz=%.1f  visc=%.2f  dur=%.1fs  id=%d",
      port_name_.c_str(), gain_scale_, deadzone_ticks_, viscous_comp_, duration_s_, log_id);

    // ── Pinocchio ────────────────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/openmani.urdf";
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(get_logger(), "Pinocchio: nv=%d", model_.nv);

    // ── CSV ──────────────────────────────────────────────────────────────────
    std::filesystem::create_directories(std::string(PACKAGE_DATA_DIR) + "/real/act1");
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/real/act1/hw_fl_data_"
                + std::to_string(log_id) + ".csv";
    csv_.open(csv_path_);
    if (csv_.is_open()) {
      csv_ << "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,"
              "q1_des,q2_des,q3_des,q4_des,dq1_des,dq2_des,dq3_des,dq4_des,"
              "tau1,tau2,tau3,tau4,"
              "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
              "curr_meas1,curr_meas2,curr_meas3,curr_meas4\n";
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
    RCLCPP_INFO(get_logger(), "Control activo a 100 Hz. Ctrl+C para detener.");
  }

  ~HWFLControlNode()
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
          !dxl_write2(id, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_REGISTER, "set limit") ||
          !dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE_VAL, "enable torque")) {
        port_handler_->closePort();
        return false;
      }
      RCLCPP_INFO(get_logger(), "DXL ID %d listo", static_cast<int>(id));
    }

    grp_pos_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    grp_vel_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
    grp_cur_ = std::make_unique<dynamixel::GroupSyncRead>(
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
  //  Lectura de estado
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

      q(i)   = ENCODER_SIGN[i] * static_cast<double>(wrappedTickDiff(rp, JOINT_ZERO_TICK[i])) * POS_UNIT_RAD;
      dq(i)  = ENCODER_SIGN[i] * static_cast<double>(rv) * VEL_UNIT_RAD_S;
      cur[i] = rc;
    }
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Escritura de corriente
  // ─────────────────────────────────────────────────────────────────────────

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
  //  Ley de control (idéntica a act_1, sin estáticos: usa miembros del nodo)
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_torque(const Vec4& q, const Vec4& dq, const Reference& ref)
  {
    Vec4 kp, kd;
    kp << 100.0, 100.0, 100.0, 100.0;
    kd <<  20.0,  20.0,  20.0,  20.0;
    kp *= gain_scale_;
    kd *= std::sqrt(std::max(gain_scale_, 0.0));

    const Vec4 eq  = q - ref.q;
    const Vec4 deq = dq - ref.dq;
    const Vec4 v   = ref.ddq - kd.cwiseProduct(deq) - kp.cwiseProduct(eq);

    Eigen::VectorXd q_pin   = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd ddq_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head(NUM_JOINTS)   = q;
    dq_pin.head(NUM_JOINTS)  = dq;
    ddq_pin.head(NUM_JOINTS) = v;

    const Eigen::VectorXd tau_full = pinocchio::rnea(model_, data_, q_pin, dq_pin, ddq_pin);
    return tau_full.head(NUM_JOINTS);
  }

  std::array<int16_t, NUM_JOINTS> torque_to_current(const Vec4& tau, const Vec4& dq)
  {
    static const std::array<double,  NUM_JOINTS> dz_gain  = {0.8, 0.9, 0.9, 1.0};
    static const std::array<int16_t, NUM_JOINTS> cur_lim  = {
      CURRENT_CMD_LIMIT_J123, CURRENT_CMD_LIMIT_J123, CURRENT_CMD_LIMIT_J123, CURRENT_CMD_LIMIT_J4
    };
    std::array<int16_t, NUM_JOINTS> cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
      double c = CURRENT_SIGN[i] * ENCODER_SIGN[i] * tau(i) / TORQUE_PER_CURRENT_TICK;
      c += CURRENT_SIGN[i] * ENCODER_SIGN[i] * viscous_comp_ * dq(i);
      if (deadzone_ticks_ > 0.0)
        c += dz_gain[i] * deadzone_ticks_ * std::tanh(30.0 * c * CURRENT_UNIT_A);
      cmd[i] = clampCurrent(c, cur_lim[i]);
    }
    return cmd;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Parada de emergencia centralizada
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

    if (duration_s_ > 0.0 && t >= duration_s_) {
      RCLCPP_INFO(get_logger(), "Duración completada (%.1f s).", duration_s_);
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

    // 2. Captura posición inicial (primer tick)
    if (!q_initial_captured_) {
      q_initial_ = q;
      q_initial_captured_ = true;
      RCLCPP_INFO(get_logger(), "q_inicial=[%.3f %.3f %.3f %.3f] rad",
        q(0), q(1), q(2), q(3));
    }

    // 3. Referencia (rampa quintica + trayectoria sinusoidal)
    const Reference ref = (t < RAMP_TIME_S)
      ? quinticTransition(t, RAMP_TIME_S, q_initial_)
      : desiredTrajectory(t);

    // 4. Verificar límites de la referencia
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (ref.q(i) < JOINT_LOWER[i] + 0.02 || ref.q(i) > JOINT_UPPER[i] - 0.02) {
        emergency_stop("Referencia fuera de límites articulares");
        return;
      }
    }

    // 5. Ley de control
    const Vec4 tau = compute_torque(q, dq, ref);
    const auto cur_cmd = torque_to_current(tau, dq);

    // 6. Escribir corriente
    if (!send_currents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 7. Verificar corriente medida (seguridad)
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > CURRENT_MEASURED_PEAK) {
        emergency_stop("Corriente medida insegura en J" + std::to_string(i + 1)
                       + ": " + std::to_string(cur_meas[i]) + " ticks");
        return;
      }
    }

    // 8. Publicar JointState de monitoreo
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

    // 9. CSV
    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6) << t
           << ',' << q(0)       << ',' << q(1)       << ',' << q(2)       << ',' << q(3)
           << ',' << dq(0)      << ',' << dq(1)      << ',' << dq(2)      << ',' << dq(3)
           << ',' << ref.q(0)   << ',' << ref.q(1)   << ',' << ref.q(2)   << ',' << ref.q(3)
           << ',' << ref.dq(0)  << ',' << ref.dq(1)  << ',' << ref.dq(2)  << ',' << ref.dq(3)
           << ',' << tau(0)     << ',' << tau(1)      << ',' << tau(2)     << ',' << tau(3)
           << ',' << cur_cmd[0] << ',' << cur_cmd[1]  << ',' << cur_cmd[2] << ',' << cur_cmd[3]
           << ',' << cur_meas[0]<< ',' << cur_meas[1] << ',' << cur_meas[2]<< ',' << cur_meas[3]
           << '\n';
    }

    // 10. Log periódico por consola
    if (++log_cnt_ % 100 == 0) {
      if (csv_.is_open()) csv_.flush();
      RCLCPP_INFO(get_logger(),
        "t=%.2fs  q=[%.3f %.3f %.3f %.3f]  |e|=%.4f  i=[%d %d %d %d]",
        t, q(0), q(1), q(2), q(3), (q - ref.q).norm(),
        cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  pinocchio::Model model_;
  pinocchio::Data  data_;

  std::string port_name_;
  double gain_scale_, deadzone_ticks_, viscous_comp_, duration_s_;

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead>  grp_pos_, grp_vel_, grp_cur_;
  std::unique_ptr<dynamixel::GroupSyncWrite> grp_wcur_;

  bool hw_active_;
  bool q_initial_captured_;
  Vec4 q_initial_;
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
  try {
    rclcpp::spin(std::make_shared<HWFLControlNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_fl_control_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
