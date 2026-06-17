/*
 * hw_identify_node.cpp
 * Identificación de la conversión torque→corriente — OpenMANIPULATOR-X hardware real.
 * Via Dynamixel SDK directo (Current Control Mode), sin ros2_control.
 *
 * Modos de operación (parámetro mode):
 *   dry_run           — sin hardware; genera trayectoria, calcula tau_model, estima corriente
 *   read_only         — abre puerto, lee q/dq/I, no activa torque, no escribe corriente
 *   zero_current      — abre puerto, habilita torque, envía corriente cero
 *   smooth_excitation — ejecuta trayectoria sinusoidal; comanda corriente si enable_current_commands=true
 *   friction_test     — sinusoidal muy lenta en un joint; los demás con compensación gravitacional
 *
 * Parámetros de seguridad (todos off por defecto):
 *   open_port                [bool]    false
 *   enable_torque            [bool]    false
 *   enable_current_commands  [bool]    false
 *   current_scale            [double]  0.0   (0..1, escala adicional de seguridad)
 *   trajectory_scale         [double]  0.0   (0..1, escala de amplitudes de trayectoria)
 *
 * CSV generado: data/identification/hw_ident_<mode>_<log_id>.csv
 *
 * ADVERTENCIA: No ejecutar junto a hardware.launch.py ni ningún proceso
 * que acceda a /dev/ttyUSB0.
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
#include <vector>

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

static constexpr double PI         = 3.14159265358979323846;
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

// Conversión de unidades
static constexpr double POS_UNIT_RAD            = 2.0 * PI / 4096.0;
static constexpr double VEL_UNIT_RAD_S          = 0.229 * 2.0 * PI / 60.0;
static constexpr double CURRENT_UNIT_A          = 0.00269;    // A/tick
static constexpr double TORQUE_CONSTANT_NM_A    = 1.7826;     // N·m/A (datasheet XM430-W350)
static constexpr double TORQUE_PER_CURRENT_TICK = TORQUE_CONSTANT_NM_A * CURRENT_UNIT_A;

// Signos verificados con hw_torque_probe: +ticks → +dirección física en todos los joints
static const std::array<double, NUM_JOINTS> ENCODER_SIGN = {+1.0, +1.0, +1.0, +1.0};
static const std::array<double, NUM_JOINTS> CURRENT_SIGN = {+1.0, +1.0, +1.0, +1.0};

static const std::array<int32_t, NUM_JOINTS> JOINT_ZERO_TICK = {2048, 2048, 2048, 2048};

// Límites articulares [rad] — coinciden con open_manipulator_x.urdf
static const std::array<double, NUM_JOINTS> JOINT_LOWER = {
  -2.827433388230814, -1.790707812546182, -0.9424777960769379, -1.790707812546182
};
static const std::array<double, NUM_JOINTS> JOINT_UPPER = {
  +2.827433388230814, +1.5707963267948966, +1.382300767579509, +2.0420352248333655
};

// Límites de corriente — 30% del stall torque XM430-W350 (4.1 N·m @ 12V)
static constexpr uint16_t CURRENT_LIMIT_REGISTER = 280;   // tope HW: 280×0.004795=1.34 N·m
static constexpr int16_t  CURRENT_CMD_LIMIT       = 257;   // tope SW: 257×0.004795=1.23 N·m = TAU_MAX
static constexpr int16_t  CURRENT_MEASURED_PEAK   = 320;   // parada de emergencia (debe ser > CMD_LIMIT)

static constexpr double TAU_MAX = 1.23;  // [N·m] 30% stall torque XM430-W350

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Utilidades de conversión (idénticas al resto del paquete)
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

// ============================================================
// Nodo ROS 2
// ============================================================

class HWIdentifyNode : public rclcpp::Node
{
public:
  enum class Mode {
    DRY_RUN, READ_ONLY, ZERO_CURRENT, SMOOTH_EXCITATION, FRICTION_TEST
  };

  struct TrajPoint { Vec4 q, dq, ddq; };

  struct CurrentResult {
    std::array<int16_t, NUM_JOINTS> ticks{};
    bool saturated{false};
  };

  HWIdentifyNode()
  : Node("hw_identify_node"),
    hw_active_(false), q_init_captured_(false),
    dq_prev_(Vec4::Zero()), ddq_filt_(Vec4::Zero()),
    t_prev_(-1.0)
  {
    // ── Parámetros de modo y seguridad ──────────────────────────────────────
    declare_parameter<std::string>("mode",                    "dry_run");
    declare_parameter<bool>       ("open_port",               false);
    declare_parameter<bool>       ("enable_torque",           false);
    declare_parameter<bool>       ("enable_current_commands", false);
    declare_parameter<double>     ("current_scale",           0.0);
    declare_parameter<double>     ("trajectory_scale",        0.0);

    // ── Parámetros de hardware y sesión ─────────────────────────────────────
    declare_parameter<std::string>("port_name",  "/dev/ttyUSB0");
    declare_parameter<int>        ("log_id",     1);
    declare_parameter<double>     ("duration_s", 30.0);
    declare_parameter<double>     ("t_ramp",     3.0);

    // ── Parámetros de trayectoria (4 joints) ────────────────────────────────
    declare_parameter<std::vector<double>>("q0", {0.0,  0.0,  0.5236, 0.5236});
    declare_parameter<std::vector<double>>("A1", {0.3,  0.2,  0.2,    0.15});
    declare_parameter<std::vector<double>>("w1", {0.5,  0.5,  0.5,    0.5});
    declare_parameter<std::vector<double>>("A2", {0.1,  0.1,  0.1,    0.05});
    declare_parameter<std::vector<double>>("w2", {1.2,  1.2,  1.2,    1.2});

    // ── Filtro diferenciador de ddq ──────────────────────────────────────────
    declare_parameter<double>("f_cut_hz", 5.0);

    // ── Compensación de zona muerta y viscosa ────────────────────────────────
    declare_parameter<double>("deadzone_ticks", 0.0);
    declare_parameter<double>("viscous_comp",   0.0);

    // ── Parámetros del modo friction_test ───────────────────────────────────
    declare_parameter<int>   ("friction_joint", 1);
    declare_parameter<double>("friction_w",     0.1);
    declare_parameter<double>("friction_A",     0.3);
    declare_parameter<double>("friction_kp",    3.0);

    declare_parameter<double>("kp_pd", 0.0);   // [N·m/rad]   proporcional posición
    declare_parameter<double>("kd_pd", 0.0);   // [N·m·s/rad] derivativa velocidad

    // ── Leer parámetros ──────────────────────────────────────────────────────
    const std::string mode_str    = get_parameter("mode").as_string();
    open_port_                    = get_parameter("open_port").as_bool();
    enable_torque_                = get_parameter("enable_torque").as_bool();
    enable_current_commands_      = get_parameter("enable_current_commands").as_bool();
    current_scale_    = std::clamp(get_parameter("current_scale").as_double(),    0.0, 1.0);
    trajectory_scale_ = std::clamp(get_parameter("trajectory_scale").as_double(), 0.0, 1.0);
    port_name_        = get_parameter("port_name").as_string();
    const int log_id  = get_parameter("log_id").as_int();
    duration_s_       = get_parameter("duration_s").as_double();
    t_ramp_           = get_parameter("t_ramp").as_double();
    f_cut_hz_         = get_parameter("f_cut_hz").as_double();
    deadzone_ticks_   = get_parameter("deadzone_ticks").as_double();
    viscous_comp_     = get_parameter("viscous_comp").as_double();
    friction_joint_   = static_cast<int>(
                          std::clamp(get_parameter("friction_joint").as_int(),
                                     static_cast<int64_t>(1),
                                     static_cast<int64_t>(NUM_JOINTS))) - 1;
    friction_w_       = get_parameter("friction_w").as_double();
    friction_A_       = get_parameter("friction_A").as_double();
    friction_kp_      = get_parameter("friction_kp").as_double();
    kp_pd_            = get_parameter("kp_pd").as_double();
    kd_pd_            = get_parameter("kd_pd").as_double();

    // alpha = exp(-2π * f_cut * dt),  dt = 0.01 s (100 Hz)
    filter_alpha_ = std::exp(-2.0 * PI * f_cut_hz_ * 0.01);

    // Parsear modo
    if      (mode_str == "read_only")         mode_ = Mode::READ_ONLY;
    else if (mode_str == "zero_current")      mode_ = Mode::ZERO_CURRENT;
    else if (mode_str == "smooth_excitation") mode_ = Mode::SMOOTH_EXCITATION;
    else if (mode_str == "friction_test")     mode_ = Mode::FRICTION_TEST;
    else                                      mode_ = Mode::DRY_RUN;

    // Validaciones de seguridad
    if (!open_port_ && mode_ != Mode::DRY_RUN) {
      RCLCPP_WARN(get_logger(), "open_port=false → forzando dry_run");
      mode_ = Mode::DRY_RUN;
    }
    if (enable_current_commands_ && !enable_torque_) {
      RCLCPP_WARN(get_logger(),
        "enable_current_commands=true requiere enable_torque=true — forzando false");
      enable_current_commands_ = false;
    }

    // Cargar y validar parámetros de trayectoria
    auto load_vec4 = [&](const std::string & name, Vec4 & v) {
      const auto raw = get_parameter(name).as_double_array();
      if (static_cast<int>(raw.size()) < NUM_JOINTS)
        throw std::runtime_error(name + " necesita 4 elementos");
      for (int i = 0; i < NUM_JOINTS; ++i) v(i) = raw[i];
    };
    load_vec4("q0", q0_);
    load_vec4("A1", A1_);
    load_vec4("w1", w1_);
    load_vec4("A2", A2_);
    load_vec4("w2", w2_);

    // Verificar que la trayectoria cabe dentro de los límites articulares
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double amp    = trajectory_scale_ * (A1_(i) + A2_(i));
      const double q_min  = q0_(i) - amp;
      const double q_max  = q0_(i) + amp;
      const double margin = 0.05;  // [rad]
      if (q_min < JOINT_LOWER[i] + margin || q_max > JOINT_UPPER[i] - margin) {
        RCLCPP_FATAL(get_logger(),
          "Trayectoria J%d excede límites: [%.3f, %.3f] vs límites [%.3f, %.3f]",
          i+1, q_min, q_max, JOINT_LOWER[i], JOINT_UPPER[i]);
        throw std::runtime_error("Trayectoria fuera de límites articulares");
      }
    }

    RCLCPP_INFO(get_logger(),
      "modo=%s  open_port=%s  enable_torque=%s  enable_cmd=%s  "
      "current_scale=%.2f  traj_scale=%.2f  dur=%.1fs  id=%d",
      mode_str.c_str(),
      open_port_               ? "true" : "false",
      enable_torque_           ? "true" : "false",
      enable_current_commands_ ? "true" : "false",
      current_scale_, trajectory_scale_, duration_s_, log_id);
    RCLCPP_INFO(get_logger(),
      "kt=%.4f N·m/A  Iu=%.5f A/tick  TORQUE_PER_TICK=%.6f N·m/tick",
      TORQUE_CONSTANT_NM_A, CURRENT_UNIT_A, TORQUE_PER_CURRENT_TICK);

    // ── Pinocchio (todos los modos) ──────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(get_logger(), "Pinocchio cargado: nv=%d", model_.nv);

    // ── CSV ──────────────────────────────────────────────────────────────────
    const std::string csv_dir = std::string(PACKAGE_DATA_DIR) + "/identification";
    std::filesystem::create_directories(csv_dir);
    csv_path_ = csv_dir + "/hw_ident_" + mode_str + "_" + std::to_string(log_id) + ".csv";
    csv_.open(csv_path_);
    if (!csv_.is_open()) {
      RCLCPP_WARN(get_logger(), "No se pudo crear CSV: %s", csv_path_.c_str());
    } else {
      write_csv_header();
      RCLCPP_INFO(get_logger(), "CSV: %s", csv_path_.c_str());
    }

    // ── Hardware (si aplica) ─────────────────────────────────────────────────
    if (open_port_) {
      if (!init_hardware()) {
        RCLCPP_FATAL(get_logger(), "Fallo hardware init. Abortando.");
        throw std::runtime_error("Hardware init failed");
      }
    } else {
      // dry_run: estado inicial = centro de la trayectoria
      q_init_          = q0_;
      q_init_captured_ = true;
      RCLCPP_INFO(get_logger(), "dry_run activo: sin hardware. q_init=q0.");
    }

    // ── Publisher de monitoreo ────────────────────────────────────────────────
    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── Timer 100 Hz ─────────────────────────────────────────────────────────
    start_time_ = std::chrono::high_resolution_clock::now();
    timer_ = create_wall_timer(10ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Nodo activo a 100 Hz. Ctrl+C para detener.");
  }

  ~HWIdentifyNode()
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

    // Deshabilitar torque antes de cambiar modo (requerimiento Dynamixel)
    for (const auto id : DXL_ID)
      dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "pre-disable");

    if (enable_torque_) {
      for (const auto id : DXL_ID) {
        if (!dxl_write1(id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE,   "set mode")  ||
            !dxl_write2(id, ADDR_CURRENT_LIMIT,  CURRENT_LIMIT_REGISTER, "set limit") ||
            !dxl_write1(id, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE_VAL,      "enable")) {
          port_handler_->closePort();
          return false;
        }
        RCLCPP_INFO(get_logger(),
          "DXL ID %d listo (Current Mode, HW limit=%d ticks, SW limit=%d ticks)",
          static_cast<int>(id), CURRENT_LIMIT_REGISTER, CURRENT_CMD_LIMIT);
      }
    } else {
      RCLCPP_INFO(get_logger(), "enable_torque=false: motores backdrivable (lectura únicamente).");
    }

    // GroupSyncRead: posición, velocidad, corriente
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

    if (grp_wcur_ && enable_torque_) {
      const std::array<int16_t, NUM_JOINTS> zero = {0, 0, 0, 0};
      send_currents(zero);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    if (enable_torque_) {
      for (const auto id : DXL_ID)
        dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "shutdown disable");
    }
    if (port_handler_) {
      port_handler_->closePort();
      RCLCPP_INFO(get_logger(), "Puerto cerrado.");
    }
  }

  bool dxl_write1(uint8_t id, uint16_t addr, uint8_t val, const char * lbl)
  {
    uint8_t err = 0;
    const int r = packet_handler_->write1ByteTxRx(port_handler_, id, addr, val, &err);
    if (r != COMM_SUCCESS || err != 0) {
      RCLCPP_WARN(get_logger(), "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
      return false;
    }
    return true;
  }

  bool dxl_write2(uint8_t id, uint16_t addr, uint16_t val, const char * lbl)
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
  //  Lectura de estado (q, dq, corriente + ticks crudos)
  // ─────────────────────────────────────────────────────────────────────────

  bool read_state(Vec4 & q, Vec4 & dq,
                  std::array<int16_t, NUM_JOINTS> & cur,
                  std::array<int32_t, NUM_JOINTS> & raw_ticks)
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

      raw_ticks[i] = rp;
      q(i)         = ENCODER_SIGN[i] * static_cast<double>(
                       wrappedTickDiff(rp, JOINT_ZERO_TICK[i])) * POS_UNIT_RAD;
      dq(i)        = ENCODER_SIGN[i] * static_cast<double>(rv) * VEL_UNIT_RAD_S;
      cur[i]       = rc;
    }
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Escritura de corriente (GroupSyncWrite)
  // ─────────────────────────────────────────────────────────────────────────

  bool send_currents(const std::array<int16_t, NUM_JOINTS> & cmd)
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
  //  Dinámica via Pinocchio RNEA
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_gravity(const Vec4 & q)
  {
    Eigen::VectorXd q_p  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(model_.nv);
    q_p.head(NUM_JOINTS) = q;
    return pinocchio::rnea(model_, data_, q_p, zero, zero).head(NUM_JOINTS);
  }

  // C(q,dq)*dq + G(q)  — aceleración cero
  Vec4 compute_nle(const Vec4 & q, const Vec4 & dq)
  {
    Eigen::VectorXd q_p   = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_p  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd zero  = Eigen::VectorXd::Zero(model_.nv);
    q_p.head(NUM_JOINTS)  = q;
    dq_p.head(NUM_JOINTS) = dq;
    return pinocchio::rnea(model_, data_, q_p, dq_p, zero).head(NUM_JOINTS);
  }

  // M(q)*ddq + C(q,dq)*dq + G(q)
  Vec4 compute_tau_model(const Vec4 & q, const Vec4 & dq, const Vec4 & ddq)
  {
    Eigen::VectorXd q_p    = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_p   = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd ddq_p  = Eigen::VectorXd::Zero(model_.nv);
    q_p.head(NUM_JOINTS)   = q;
    dq_p.head(NUM_JOINTS)  = dq;
    ddq_p.head(NUM_JOINTS) = ddq;
    return pinocchio::rnea(model_, data_, q_p, dq_p, ddq_p).head(NUM_JOINTS);
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Diferenciador de velocidad con filtro IIR de primer orden.
  //  Usa dt medido del timer real (no asume 100 Hz) y recalcula alpha en cada tick
  //  para que el filtro sea invariante a la frecuencia real de muestreo.
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 differentiate_velocity(const Vec4 & dq, double t_now)
  {
    if (t_prev_ < 0.0) {
      // Primer llamado: inicializar sin derivar
      t_prev_   = t_now;
      dq_prev_  = dq;
      ddq_filt_ = Vec4::Zero();
      return ddq_filt_;
    }
    const double dt = t_now - t_prev_;
    if (dt < 1e-6) return ddq_filt_;   // evitar división por cero

    const double alpha     = std::exp(-2.0 * PI * f_cut_hz_ * dt);
    const Vec4   ddq_raw   = (dq - dq_prev_) / dt;
    ddq_filt_ = alpha * ddq_filt_ + (1.0 - alpha) * ddq_raw;
    dq_prev_  = dq;
    t_prev_   = t_now;
    return ddq_filt_;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Trayectoria de referencia
  // ─────────────────────────────────────────────────────────────────────────

  // Sinusoide pura (t es tiempo relativo desde fin de rampa)
  TrajPoint compute_excitation(double t_exc) const
  {
    TrajPoint r;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      r.q(i)   = q0_(i) + trajectory_scale_ * (
                   A1_(i) * std::sin(w1_(i) * t_exc) +
                   A2_(i) * std::sin(w2_(i) * t_exc));
      r.dq(i)  = trajectory_scale_ * (
                   A1_(i) * w1_(i) * std::cos(w1_(i) * t_exc) +
                   A2_(i) * w2_(i) * std::cos(w2_(i) * t_exc));
      r.ddq(i) = trajectory_scale_ * (
                  -A1_(i) * w1_(i)*w1_(i) * std::sin(w1_(i) * t_exc)
                  -A2_(i) * w2_(i)*w2_(i) * std::sin(w2_(i) * t_exc));
    }
    return r;
  }

  // Rampa quintica de q_from → q0_ en T segundos (v=0, a=0 en ambos extremos)
  TrajPoint compute_ramp(double t, const Vec4 & q_from, double T) const
  {
    if (T <= 0.0 || t >= T) {
      TrajPoint r; r.q = q0_; r.dq = Vec4::Zero(); r.ddq = Vec4::Zero(); return r;
    }
    const Vec4   dq_q = q0_ - q_from;
    const double T3   = T*T*T, T4 = T3*T, T5 = T4*T;
    const Vec4   c3   =  10.0 * dq_q / T3;
    const Vec4   c4   = -15.0 * dq_q / T4;
    const Vec4   c5   =   6.0 * dq_q / T5;
    const double t2   = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
    TrajPoint r;
    r.q   = q_from +       c3*t3 +       c4*t4 +       c5*t5;
    r.dq  =          3.0 * c3*t2 + 4.0 * c4*t3 + 5.0 * c5*t4;
    r.ddq =          6.0 * c3*t  +12.0 * c4*t2 +20.0 * c5*t3;
    return r;
  }

  // Devuelve referencia (ramp o excitación) y phase (0=ramp, 1=excitación)
  std::pair<TrajPoint, int> get_reference(double t) const
  {
    if (t < t_ramp_) return {compute_ramp(t, q_init_, t_ramp_), 0};
    return {compute_excitation(t - t_ramp_), 1};
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Conversión torque → corriente
  // ─────────────────────────────────────────────────────────────────────────

  CurrentResult torque_to_current(const Vec4 & tau, const Vec4 & dq) const
  {
    CurrentResult res;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      double c = CURRENT_SIGN[i] * ENCODER_SIGN[i] * tau(i) / TORQUE_PER_CURRENT_TICK;
      c += CURRENT_SIGN[i] * ENCODER_SIGN[i] * viscous_comp_ * dq(i);
      if (deadzone_ticks_ > 0.0)
        c += deadzone_ticks_ * std::tanh(30.0 * c * CURRENT_UNIT_A);
      const double c_lim = static_cast<double>(CURRENT_CMD_LIMIT);
      if (std::abs(c) > c_lim) res.saturated = true;
      res.ticks[i] = static_cast<int16_t>(
        std::lround(std::clamp(c, -c_lim, c_lim)));
    }
    return res;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  CSV
  // ─────────────────────────────────────────────────────────────────────────

  void write_csv_header()
  {
    csv_ << "t,"
         // estado medido
         << "q1_ticks,q2_ticks,q3_ticks,q4_ticks,"
         << "q1,q2,q3,q4,"
         << "dq1,dq2,dq3,dq4,"
         << "ddq1,ddq2,ddq3,ddq4,"
         // referencia
         << "q1_ref,q2_ref,q3_ref,q4_ref,"
         << "dq1_ref,dq2_ref,dq3_ref,dq4_ref,"
         << "ddq1_ref,ddq2_ref,ddq3_ref,ddq4_ref,"
         // dinámica
         << "tau_g1,tau_g2,tau_g3,tau_g4,"
         << "tau_nle1,tau_nle2,tau_nle3,tau_nle4,"
         << "tau_meas1,tau_meas2,tau_meas3,tau_meas4,"
         << "tau_ref1,tau_ref2,tau_ref3,tau_ref4,"
         << "tau_cmd1,tau_cmd2,tau_cmd3,tau_cmd4,"
         // corriente comandada
         << "curr_cmd_ticks1,curr_cmd_ticks2,curr_cmd_ticks3,curr_cmd_ticks4,"
         << "curr_cmd_A1,curr_cmd_A2,curr_cmd_A3,curr_cmd_A4,"
         // corriente medida
         << "curr_meas_ticks1,curr_meas_ticks2,curr_meas_ticks3,curr_meas_ticks4,"
         << "curr_meas_A1,curr_meas_A2,curr_meas_A3,curr_meas_A4,"
         // flags
         << "phase,saturated\n";
  }

  void write_csv_row(
    double t,
    const std::array<int32_t, NUM_JOINTS> & raw_ticks,
    const Vec4 & q_meas, const Vec4 & dq_meas, const Vec4 & ddq,
    const TrajPoint & ref,
    const Vec4 & tau_g, const Vec4 & tau_nle,
    const Vec4 & tau_meas, const Vec4 & tau_ref, const Vec4 & tau_cmd,
    const CurrentResult & curr_cmd,
    const std::array<int16_t, NUM_JOINTS> & curr_meas_ticks,
    int phase)
  {
    if (!csv_.is_open()) return;
    csv_ << std::fixed << std::setprecision(6) << t;
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << raw_ticks[i];
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << q_meas(i);
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << dq_meas(i);
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << ddq(i);
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << ref.q(i);
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << ref.dq(i);
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << ref.ddq(i);
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << tau_g(i);
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << tau_nle(i);
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << tau_meas(i);
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << tau_ref(i);
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << tau_cmd(i);
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << curr_cmd.ticks[i];
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << curr_cmd.ticks[i] * CURRENT_UNIT_A;
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << curr_meas_ticks[i];
    for (int i = 0; i < NUM_JOINTS; ++i) csv_ << ',' << curr_meas_ticks[i] * CURRENT_UNIT_A;
    csv_ << ',' << phase << ',' << (curr_cmd.saturated ? 1 : 0) << '\n';
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Seguridad centralizada
  // ─────────────────────────────────────────────────────────────────────────

  bool check_joint_limits(const Vec4 & q)
  {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (q(i) < JOINT_LOWER[i] || q(i) > JOINT_UPPER[i]) {
        emergency_stop("J" + std::to_string(i+1) + " fuera de límites: " +
                       std::to_string(q(i)) + " rad");
        return false;
      }
    }
    return true;
  }

  bool check_current_safety(const std::array<int16_t, NUM_JOINTS> & cur)
  {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur[i]) > CURRENT_MEASURED_PEAK) {
        emergency_stop("Corriente insegura J" + std::to_string(i+1) +
                       ": " + std::to_string(cur[i]) + " ticks");
        return false;
      }
    }
    return true;
  }

  void emergency_stop(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "PARADA: %s", reason.c_str());
    if (timer_) timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    rclcpp::shutdown();
  }

  void finish_normal()
  {
    RCLCPP_INFO(get_logger(), "Duración completada (%.1f s).", duration_s_);
    if (timer_) timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    rclcpp::shutdown();
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Publisher de monitoreo
  // ─────────────────────────────────────────────────────────────────────────

  void publish_joint_state(const Vec4 & q, const Vec4 & dq,
                           const std::array<int16_t, NUM_JOINTS> & cur)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name         = {"joint1", "joint2", "joint3", "joint4"};
    js.position     = {q(0), q(1), q(2), q(3)};
    js.velocity     = {dq(0), dq(1), dq(2), dq(3)};
    js.effort = {
      static_cast<double>(cur[0]) * TORQUE_PER_CURRENT_TICK,
      static_cast<double>(cur[1]) * TORQUE_PER_CURRENT_TICK,
      static_cast<double>(cur[2]) * TORQUE_PER_CURRENT_TICK,
      static_cast<double>(cur[3]) * TORQUE_PER_CURRENT_TICK
    };
    js_pub_->publish(js);
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Tick principal (100 Hz)
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    const auto tp = std::chrono::high_resolution_clock::now();
    const double t = std::chrono::duration<double>(tp - start_time_).count();

    if (duration_s_ > 0.0 && t >= duration_s_) { finish_normal(); return; }

    switch (mode_) {
      case Mode::DRY_RUN:           tick_dry_run(t);           break;
      case Mode::READ_ONLY:         tick_read_only(t);         break;
      case Mode::ZERO_CURRENT:      tick_zero_current(t);      break;
      case Mode::SMOOTH_EXCITATION: tick_smooth_excitation(t); break;
      case Mode::FRICTION_TEST:     tick_friction_test(t);     break;
    }

    if (++log_cnt_ % 100 == 0 && csv_.is_open()) csv_.flush();
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Modo dry_run
  // ─────────────────────────────────────────────────────────────────────────

  void tick_dry_run(double t)
  {
    const auto [ref, phase] = get_reference(t);

    // En dry_run: q_meas = q_ref (sin hardware)
    const Vec4 & q_meas  = ref.q;
    const Vec4 & dq_meas = ref.dq;
    const Vec4 & ddq     = ref.ddq;  // analítico, sin ruido

    const Vec4 tau_g    = compute_gravity(q_meas);
    const Vec4 tau_nle  = compute_nle(q_meas, dq_meas);
    const Vec4 tau_ref  = compute_tau_model(ref.q, ref.dq, ref.ddq);
    const Vec4 tau_cmd  = current_scale_ * tau_ref.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);
    const auto curr_cmd = torque_to_current(tau_cmd, dq_meas);

    const std::array<int32_t, NUM_JOINTS> raw_zero  = {0, 0, 0, 0};
    const std::array<int16_t, NUM_JOINTS> meas_zero = {0, 0, 0, 0};

    write_csv_row(t, raw_zero, q_meas, dq_meas, ddq, ref,
                  tau_g, tau_nle, tau_ref, tau_ref, tau_cmd,
                  curr_cmd, meas_zero, phase);

    if (log_cnt_ % 100 == 0) {
      RCLCPP_INFO(get_logger(),
        "dry_run t=%.1fs phase=%d  tau_ref=[%.3f %.3f %.3f %.3f] N·m  "
        "curr_cmd=[%d %d %d %d] ticks",
        t, phase,
        tau_ref(0), tau_ref(1), tau_ref(2), tau_ref(3),
        curr_cmd.ticks[0], curr_cmd.ticks[1], curr_cmd.ticks[2], curr_cmd.ticks[3]);
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Modo read_only
  // ─────────────────────────────────────────────────────────────────────────

  void tick_read_only(double t)
  {
    Vec4 q_meas, dq_meas;
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    std::array<int32_t, NUM_JOINTS> raw_ticks{};
    if (!read_state(q_meas, dq_meas, cur_meas, raw_ticks)) {
      emergency_stop("SyncRead fallido"); return;
    }

    if (!q_init_captured_) {
      q_init_ = q_meas; q_init_captured_ = true; dq_prev_ = dq_meas;
      RCLCPP_INFO(get_logger(), "q_init=[%.3f %.3f %.3f %.3f] rad",
        q_meas(0), q_meas(1), q_meas(2), q_meas(3));
    }

    const auto [ref, phase] = get_reference(t);
    const Vec4 ddq = differentiate_velocity(dq_meas, t);

    const Vec4 tau_g    = compute_gravity(q_meas);
    const Vec4 tau_nle  = compute_nle(q_meas, dq_meas);
    const Vec4 tau_meas = compute_tau_model(q_meas, dq_meas, ddq);
    const Vec4 tau_ref  = compute_tau_model(ref.q, ref.dq, ref.ddq);
    const Vec4 tau_cmd  = Vec4::Zero();
    const CurrentResult curr_cmd;  // zeros

    publish_joint_state(q_meas, dq_meas, cur_meas);
    write_csv_row(t, raw_ticks, q_meas, dq_meas, ddq, ref,
                  tau_g, tau_nle, tau_meas, tau_ref, tau_cmd,
                  curr_cmd, cur_meas, phase);
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Modo zero_current
  // ─────────────────────────────────────────────────────────────────────────

  void tick_zero_current(double t)
  {
    Vec4 q_meas, dq_meas;
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    std::array<int32_t, NUM_JOINTS> raw_ticks{};
    if (!read_state(q_meas, dq_meas, cur_meas, raw_ticks)) {
      emergency_stop("SyncRead fallido"); return;
    }
    if (!q_init_captured_) {
      q_init_ = q_meas; q_init_captured_ = true; dq_prev_ = dq_meas;
    }

    if (enable_current_commands_) {
      const std::array<int16_t, NUM_JOINTS> zero = {0, 0, 0, 0};
      if (!send_currents(zero)) { emergency_stop("SyncWrite fallido"); return; }
    }

    if (!check_current_safety(cur_meas)) return;

    const Vec4 ddq   = differentiate_velocity(dq_meas, t);
    const Vec4 tau_g = compute_gravity(q_meas);
    const Vec4 tau_nle = compute_nle(q_meas, dq_meas);
    const Vec4 tau_m = compute_tau_model(q_meas, dq_meas, ddq);
    const TrajPoint ref{q0_, Vec4::Zero(), Vec4::Zero()};
    const CurrentResult curr_cmd;

    publish_joint_state(q_meas, dq_meas, cur_meas);
    write_csv_row(t, raw_ticks, q_meas, dq_meas, ddq, ref,
                  tau_g, tau_nle, tau_m, tau_m, Vec4::Zero(),
                  curr_cmd, cur_meas, 0);

    if (log_cnt_ % 100 == 0) {
      RCLCPP_INFO(get_logger(),
        "zero_current t=%.1fs  curr_meas=[%d %d %d %d] ticks  q=[%.3f %.3f %.3f %.3f]",
        t, cur_meas[0], cur_meas[1], cur_meas[2], cur_meas[3],
        q_meas(0), q_meas(1), q_meas(2), q_meas(3));
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Modo smooth_excitation
  // ─────────────────────────────────────────────────────────────────────────

  void tick_smooth_excitation(double t)
  {
    Vec4 q_meas, dq_meas;
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    std::array<int32_t, NUM_JOINTS> raw_ticks{};
    if (!read_state(q_meas, dq_meas, cur_meas, raw_ticks)) {
      emergency_stop("SyncRead fallido"); return;
    }
    if (!q_init_captured_) {
      q_init_ = q_meas; q_init_captured_ = true; dq_prev_ = dq_meas;
      RCLCPP_INFO(get_logger(), "q_init=[%.3f %.3f %.3f %.3f] rad",
        q_meas(0), q_meas(1), q_meas(2), q_meas(3));
    }

    if (!check_joint_limits(q_meas))    return;
    if (!check_current_safety(cur_meas)) return;

    const auto [ref, phase] = get_reference(t);
    const Vec4 ddq = differentiate_velocity(dq_meas, t);

    const Vec4 tau_g    = compute_gravity(q_meas);
    const Vec4 tau_nle  = compute_nle(q_meas, dq_meas);
    const Vec4 tau_meas = compute_tau_model(q_meas, dq_meas, ddq);

    // Feed-forward RNEA + corrección PD para compensar error del modelo
    const Vec4 tau_ref  = compute_tau_model(ref.q, ref.dq, ref.ddq);
    const Vec4 tau_pd   = kp_pd_ * (ref.q - q_meas) + kd_pd_ * (ref.dq - dq_meas);
    const Vec4 tau_cmd  = (current_scale_ * tau_ref + tau_pd).cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);
    const auto curr_cmd = torque_to_current(tau_cmd, dq_meas);

    if (enable_current_commands_) {
      if (!send_currents(curr_cmd.ticks)) { emergency_stop("SyncWrite fallido"); return; }
    }

    publish_joint_state(q_meas, dq_meas, cur_meas);
    write_csv_row(t, raw_ticks, q_meas, dq_meas, ddq, ref,
                  tau_g, tau_nle, tau_meas, tau_ref, tau_cmd,
                  curr_cmd, cur_meas, phase);

    if (log_cnt_ % 100 == 0) {
      RCLCPP_INFO(get_logger(),
        "t=%.1fs phase=%d  q=[%.3f %.3f %.3f %.3f]  "
        "tau_cmd=[%.3f %.3f %.3f %.3f]  i_meas=[%d %d %d %d] sat=%s",
        t, phase, q_meas(0), q_meas(1), q_meas(2), q_meas(3),
        tau_cmd(0), tau_cmd(1), tau_cmd(2), tau_cmd(3),
        cur_meas[0], cur_meas[1], cur_meas[2], cur_meas[3],
        curr_cmd.saturated ? "SI" : "no");
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Modo friction_test
  //  Excita lentamente el joint seleccionado con gravedad + P suave.
  //  Los demás joints reciben únicamente compensación gravitacional.
  // ─────────────────────────────────────────────────────────────────────────

  void tick_friction_test(double t)
  {
    Vec4 q_meas, dq_meas;
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    std::array<int32_t, NUM_JOINTS> raw_ticks{};
    if (!read_state(q_meas, dq_meas, cur_meas, raw_ticks)) {
      emergency_stop("SyncRead fallido"); return;
    }
    if (!q_init_captured_) {
      q_init_ = q_meas; q_init_captured_ = true; dq_prev_ = dq_meas;
    }

    if (!check_joint_limits(q_meas))    return;
    if (!check_current_safety(cur_meas)) return;

    const Vec4 ddq   = differentiate_velocity(dq_meas, t);
    const Vec4 tau_g = compute_gravity(q_meas);

    // Referencia del joint de fricción
    const double t_exc      = std::max(0.0, t - t_ramp_);
    const double q_fric_ref = q0_(friction_joint_) + friction_A_ * std::sin(friction_w_ * t_exc);
    const double dq_fric_ref= friction_A_ * friction_w_ * std::cos(friction_w_ * t_exc);

    // Torque: gravedad en todos + P suave solo en el joint seleccionado
    Vec4 tau_cmd = tau_g;
    tau_cmd(friction_joint_) += friction_kp_ * (q_fric_ref - q_meas(friction_joint_));
    tau_cmd = current_scale_ * tau_cmd.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    const auto curr_cmd = torque_to_current(tau_cmd, dq_meas);

    if (enable_current_commands_) {
      if (!send_currents(curr_cmd.ticks)) { emergency_stop("SyncWrite fallido"); return; }
    }

    // Referencia para el CSV (solo el joint activo tiene trayectoria)
    TrajPoint ref;
    ref.q   = q0_;
    ref.dq  = Vec4::Zero();
    ref.ddq = Vec4::Zero();
    ref.q(friction_joint_)  = q_fric_ref;
    ref.dq(friction_joint_) = dq_fric_ref;

    const Vec4 tau_nle  = compute_nle(q_meas, dq_meas);
    const Vec4 tau_meas = compute_tau_model(q_meas, dq_meas, ddq);

    publish_joint_state(q_meas, dq_meas, cur_meas);
    write_csv_row(t, raw_ticks, q_meas, dq_meas, ddq, ref,
                  tau_g, tau_nle, tau_meas, tau_cmd, tau_cmd,
                  curr_cmd, cur_meas, (t < t_ramp_) ? 0 : 1);

    if (log_cnt_ % 50 == 0) {
      RCLCPP_INFO(get_logger(),
        "friction J%d  t=%.1fs  q=%.4f (ref=%.4f)  dq=%.4f  "
        "i_cmd=%d  i_meas=%d ticks",
        friction_joint_+1, t,
        q_meas(friction_joint_), q_fric_ref, dq_meas(friction_joint_),
        curr_cmd.ticks[friction_joint_], cur_meas[friction_joint_]);
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  Mode   mode_;
  bool   open_port_, enable_torque_, enable_current_commands_;
  double current_scale_, trajectory_scale_;
  std::string port_name_;
  double duration_s_, t_ramp_, f_cut_hz_;
  double deadzone_ticks_, viscous_comp_;
  int    friction_joint_;
  double friction_w_, friction_A_, friction_kp_;
  double kp_pd_, kd_pd_;
  double filter_alpha_;

  Vec4 q0_, A1_, w1_, A2_, w2_;

  pinocchio::Model model_;
  pinocchio::Data  data_;

  dynamixel::PortHandler  * port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead>  grp_pos_, grp_vel_, grp_cur_;
  std::unique_ptr<dynamixel::GroupSyncWrite> grp_wcur_;

  bool hw_active_;
  bool q_init_captured_;
  Vec4 q_init_;
  Vec4 dq_prev_, ddq_filt_;
  double t_prev_;

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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HWIdentifyNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_identify_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
