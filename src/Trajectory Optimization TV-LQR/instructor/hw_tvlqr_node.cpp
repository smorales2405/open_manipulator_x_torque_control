/*
 * hw_tvlqr_node.cpp
 * Seguimiento TV-LQR — OpenMANIPULATOR-X hardware real
 * via Dynamixel SDK directo, sin ros2_control.
 *
 * Ley de control TV-LQR:
 *   x(t)    = [q; dq]                                  (8 x 1)
 *   x_ref,k = [q_ref_k; dq_ref_k]                     (8 x 1)
 *   tau     = u_ref_k - K_k * (x(t) - x_ref,k)        (4 x 1)
 *   tau_sat = torque_scale * clamp(tau, -TAU_MAX, TAU_MAX)
 *
 * Infraestructura proporcionada (NO modificar):
 *   - Inicializacion Dynamixel SDK (init_hardware, shutdown_hardware)
 *   - Lectura de estado SyncRead: q, dq, corriente medida (read_state)
 *   - Escritura de corriente SyncWrite (send_currents)
 *   - Conversion torque -> corriente via modelo OLS identificado (motor_params.yaml)
 *   - Verificacion de corriente medida (parada de emergencia)
 *   - Publicador /hw/joint_states para monitoreo
 *
 * Parametros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name              [string]    "/dev/ttyUSB0"
 *   t_warmup               [double]    2.0   (warmup con tau_gravity antes de TV-LQR, 0 = sin warmup)
 *   torque_scale           [double]    1.0   (escala de seguridad, rango 0..1)
 *   t_run                  [double]    1.5   (duracion en segundos, 0 = sin limite)
 *   test_num               [int]       1     (identificador del CSV)
 *   reference_dir          [string]    "src/Trajectory Optimization TV-LQR/references"
 *   vel_cutoff_hz          [double]    2.0   (filtro EMA velocidad; 0 = desactivado)
 *   motor_alpha            [double[4]] ticks/N·m
 *   motor_Fv               [double[4]] ticks/(rad/s)
 *   motor_Fc               [double[4]] ticks
 *   motor_I_offset         [double[4]] ticks
 *   motor_epsilon_friction [double]    rad/s
 *
 * Publisher: /hw/joint_states (sensor_msgs/JointState) — monitoreo
 *
 * ADVERTENCIA: No ejecutar junto a hardware.launch.py ni ningun proceso
 * que acceda al puerto USB (ros2_control_node, dynamixel_hardware_interface).
 *
 * CSV generado: data/real/lab5/data_log_real_lab5_<test_num>.csv
 */

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include "dynamixel_sdk/dynamixel_sdk.h"

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_SHARE_DIR
#define PACKAGE_SHARE_DIR "."
#endif

using namespace std::chrono_literals;

// ============================================================
// Constantes Dynamixel / conversion de unidades
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

static constexpr double POS_UNIT_RAD            = 2.0 * PI / 4096.0;
static constexpr double VEL_UNIT_RAD_S          = 0.229 * 2.0 * PI / 60.0;
static constexpr double CURRENT_UNIT_A          = 0.00269;
static constexpr double TORQUE_CONSTANT_NM_A    = 1.7826;
static constexpr double TORQUE_PER_CURRENT_TICK = TORQUE_CONSTANT_NM_A * CURRENT_UNIT_A;

static const std::array<int32_t, NUM_JOINTS> JOINT_ZERO_TICK = {2048, 2048, 2048, 2048};
static const std::array<double,  NUM_JOINTS> ENCODER_SIGN    = {+1.0, +1.0, +1.0, +1.0};
static const std::array<double,  NUM_JOINTS> CURRENT_SIGN    = {+1.0, +1.0, +1.0, +1.0};

static const std::array<double, NUM_JOINTS> JOINT_LOWER = {
  -3.0/4.0*PI, -11.0/18.0*PI, -11.0/18.0*PI, -1.8
};
static const std::array<double, NUM_JOINTS> JOINT_UPPER = {
  +3.0/4.0*PI, +5.0/9.0*PI,   +PI/2.0,       2.1
};

static constexpr uint16_t CURRENT_LIMIT_REGISTER = 350;
static constexpr int16_t  CURRENT_CMD_LIMIT_J123 = 257;
static constexpr int16_t  CURRENT_CMD_LIMIT_J4   = 257;
static constexpr int16_t  CURRENT_MEASURED_PEAK  = 313;

static constexpr double TAU_MAX = 1.2;   // [N·m]

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ── Utilidades de conversion ──────────────────────────────────────────────────
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

// ── Utilidad: leer una matriz de texto con 'cols' columnas ───────────────────
static bool load_matrix(const std::string & path, int cols,
                         std::vector<std::vector<double>> & rows)
{
  std::ifstream f(path);
  if (!f.is_open()) { return false; }
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) { continue; }
    std::istringstream ss(line);
    std::vector<double> row;
    double v;
    while (ss >> v) { row.push_back(v); }
    if (static_cast<int>(row.size()) != cols) { return false; }
    rows.push_back(row);
  }
  return !rows.empty();
}

// ── Nodo ROS 2 ───────────────────────────────────────────────────────────────
class HWTVLQRNode : public rclcpp::Node
{
public:
  HWTVLQRNode()
  : Node("hw_tvlqr_node"),
    hw_active_(false), refs_loaded_(false), N_(0), Ts_(0.05),
    tau_gravity_(Vec4::Zero()), warmup_ticks_(0), warmup_active_(false)
  {
    // ── Parametros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",     "/dev/ttyUSB0");
    this->declare_parameter<double>     ("t_warmup",       2.0);
    this->declare_parameter<double>     ("torque_scale",   1.0);
    this->declare_parameter<double>     ("t_run",          1.5);
    this->declare_parameter<int>        ("test_num",        1);
    this->declare_parameter<std::string>("reference_dir",
      "src/Trajectory Optimization TV-LQR/references");

    using dvec = std::vector<double>;
    this->declare_parameter<dvec>("motor_alpha",            dvec{208.5, 208.5, 208.5, 208.5});
    this->declare_parameter<dvec>("motor_Fv",               dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<dvec>("motor_Fc",               dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<dvec>("motor_I_offset",         dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<double>("motor_epsilon_friction", 0.05);
    this->declare_parameter<double>("vel_cutoff_hz",          2.0);

    port_name_        = this->get_parameter("port_name").as_string();
    const double t_warmup_sec = this->get_parameter("t_warmup").as_double();
    torque_scale_     = std::min(std::max(this->get_parameter("torque_scale").as_double(), 0.0), 1.0);
    t_run_            = this->get_parameter("t_run").as_double();
    const int test_num = this->get_parameter("test_num").as_int();
    const std::string ref_name = this->get_parameter("reference_dir").as_string();
    ref_dir_ = std::string(PACKAGE_SHARE_DIR) + "/" + ref_name;

    auto load_vec4 = [this](const std::string & name) {
      auto v = get_parameter(name).as_double_array();
      return Vec4(v[0], v[1], v[2], v[3]);
    };
    motor_alpha_    = load_vec4("motor_alpha");
    motor_Fv_       = load_vec4("motor_Fv");
    motor_Fc_       = load_vec4("motor_Fc");
    motor_I_offset_ = load_vec4("motor_I_offset");
    motor_epsilon_  = get_parameter("motor_epsilon_friction").as_double();

    vel_cutoff_hz_    = get_parameter("vel_cutoff_hz").as_double();
    vel_filter_alpha_ = (vel_cutoff_hz_ > 0.0)
        ? std::exp(-2.0 * PI * vel_cutoff_hz_ * 0.01)
        : 0.0;

    RCLCPP_INFO(get_logger(),
      "puerto=%s  scale=%.2f  t_run=%.1fs  id=%d",
      port_name_.c_str(), torque_scale_, t_run_, test_num);
    RCLCPP_INFO(get_logger(),
      "motor α=[%.1f %.1f %.1f %.1f]  Fv=[%.2f %.2f %.2f %.2f]  ε=%.3f",
      motor_alpha_(0), motor_alpha_(1), motor_alpha_(2), motor_alpha_(3),
      motor_Fv_(0), motor_Fv_(1), motor_Fv_(2), motor_Fv_(3), motor_epsilon_);
    if (vel_cutoff_hz_ > 0.0)
      RCLCPP_INFO(get_logger(),
        "Filtro velocidad: fc=%.1f Hz  α=%.4f", vel_cutoff_hz_, vel_filter_alpha_);
    else
      RCLCPP_WARN(get_logger(), "Filtro velocidad DESACTIVADO (vel_cutoff_hz=0).");
    RCLCPP_INFO(get_logger(), "reference_dir: %s", ref_dir_.c_str());

    // ── Seccion 1: Carga de referencias ─────────────────────────────────────
    auto fatal_load = [&](const std::string & path) {
      RCLCPP_FATAL(get_logger(), "No se pudo cargar: %s", path.c_str());
      throw std::runtime_error("Archivo de referencia no encontrado: " + path);
    };

    // time_ref.txt — 1 columna
    {
      std::vector<std::vector<double>> rows;
      const std::string path = ref_dir_ + "/time_ref.txt";
      if (!load_matrix(path, 1, rows)) { fatal_load(path); }
      N_  = static_cast<int>(rows.size());
      Ts_ = rows[0][0];
      t_ref_.reserve(N_);
      for (const auto & r : rows) { t_ref_.push_back(r[0]); }
    }

    // q_ref.txt — 4 columnas
    {
      std::vector<std::vector<double>> rows;
      const std::string path = ref_dir_ + "/q_ref.txt";
      if (!load_matrix(path, 4, rows)) { fatal_load(path); }
      q_ref_.reserve(N_);
      for (const auto & r : rows)
        q_ref_.emplace_back(r[0], r[1], r[2], r[3]);
    }

    // dq_ref.txt — 4 columnas
    {
      std::vector<std::vector<double>> rows;
      const std::string path = ref_dir_ + "/dq_ref.txt";
      if (!load_matrix(path, 4, rows)) { fatal_load(path); }
      dq_ref_.reserve(N_);
      for (const auto & r : rows)
        dq_ref_.emplace_back(r[0], r[1], r[2], r[3]);
    }

    // u_ref.txt — 4 columnas
    {
      std::vector<std::vector<double>> rows;
      const std::string path = ref_dir_ + "/u_ref.txt";
      if (!load_matrix(path, 4, rows)) { fatal_load(path); }
      u_ref_.reserve(N_);
      for (const auto & r : rows)
        u_ref_.emplace_back(r[0], r[1], r[2], r[3]);
    }

    // K_TV.txt — 32 columnas (col-major, reshape de K_k 4x8)
    {
      std::vector<std::vector<double>> rows;
      const std::string path = ref_dir_ + "/K_TV.txt";
      if (!load_matrix(path, 32, rows)) { fatal_load(path); }
      K_TV_.reserve(N_);
      for (const auto & r : rows) {
        Eigen::Map<const Eigen::Matrix<double,4,8,Eigen::ColMajor>> K(r.data());
        K_TV_.push_back(K);
      }
    }

    refs_loaded_ = true;
    RCLCPP_INFO(get_logger(), "Referencias cargadas: N=%d  Ts=%.3f s", N_, Ts_);

    // tau_gravity.txt — 1 fila x 4 columnas (opcional, fallback a u_ref_[0])
    {
      std::vector<std::vector<double>> rows;
      const std::string grav_path = ref_dir_ + "/tau_gravity.txt";
      if (load_matrix(grav_path, 4, rows) && !rows.empty()) {
        tau_gravity_ = Vec4(rows[0][0], rows[0][1], rows[0][2], rows[0][3]);
        RCLCPP_INFO(get_logger(),
          "tau_gravity.txt cargado: [%.3f %.3f %.3f %.3f] Nm",
          tau_gravity_[0], tau_gravity_[1], tau_gravity_[2], tau_gravity_[3]);
      } else {
        tau_gravity_ = u_ref_[0];
        RCLCPP_WARN(get_logger(),
          "tau_gravity.txt no encontrado — usando u_ref[0]: [%.3f %.3f %.3f %.3f] Nm",
          tau_gravity_[0], tau_gravity_[1], tau_gravity_[2], tau_gravity_[3]);
      }
    }

    warmup_ticks_ = (t_warmup_sec > 0.0) ? static_cast<int>(std::round(t_warmup_sec / 0.01)) : 0;
    warmup_active_ = (warmup_ticks_ > 0);
    if (warmup_active_)
      RCLCPP_INFO(get_logger(), "Warmup: %.1f s  (tau_gravity sin torque_scale)", t_warmup_sec);

    // ── CSV ──────────────────────────────────────────────────────────────────
    std::filesystem::create_directories(std::string(PACKAGE_DATA_DIR) + "/lab5/real");
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab5/real/data_log_real_lab5_"
                + std::to_string(test_num) + ".csv";
    csv_.open(csv_path_);
    if (csv_.is_open()) {
      csv_ << "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
              "q1_ref,q2_ref,q3_ref,q4_ref,dq1_ref,dq2_ref,dq3_ref,dq4_ref,"
              "tau1,tau2,tau3,tau4,"
              "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
              "curr_meas1,curr_meas2,curr_meas3,curr_meas4\n";
      RCLCPP_INFO(get_logger(), "CSV: %s", csv_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "No se pudo crear CSV: %s", csv_path_.c_str());
    }

    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    if (!init_hardware()) {
      RCLCPP_FATAL(get_logger(), "Fallo hardware init. Abortando.");
      throw std::runtime_error("Hardware init failed");
    }

    start_time_ = std::chrono::high_resolution_clock::now();
    timer_ = this->create_wall_timer(10ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Control TV-LQR activo a 100 Hz. Ctrl+C para detener.");
  }

  ~HWTVLQRNode()
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

  bool read_state(Vec4 & q, Vec4 & dq, std::array<int16_t, NUM_JOINTS> & cur)
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

  std::array<int16_t, NUM_JOINTS> torque_to_current(const Vec4 & tau, const Vec4 & dq)
  {
    static const std::array<int16_t, NUM_JOINTS> cur_lim = {
      CURRENT_CMD_LIMIT_J123, CURRENT_CMD_LIMIT_J123,
      CURRENT_CMD_LIMIT_J123, CURRENT_CMD_LIMIT_J4
    };
    std::array<int16_t, NUM_JOINTS> cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double I_model = motor_alpha_(i) * tau(i)
                           + motor_Fv_(i)    * dq(i)
                           + motor_Fc_(i)    * std::tanh(dq(i) / motor_epsilon_)
                           + motor_I_offset_(i);
      cmd[i] = clampCurrent(CURRENT_SIGN[i] * ENCODER_SIGN[i] * I_model, cur_lim[i]);
    }
    return cmd;
  }

  void emergency_stop(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "PARADA: %s", reason.c_str());
    if (timer_) timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    rclcpp::shutdown();
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Callback del timer (100 Hz)
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    if (!hw_active_ || !refs_loaded_) return;

    // ── Fase warmup: compensacion gravitatoria ────────────────────────────
    if (warmup_active_) {
      Vec4 q, dq;
      std::array<int16_t, NUM_JOINTS> cur_meas{};
      if (!read_state(q, dq, cur_meas)) {
        emergency_stop("SyncRead fallido en warmup"); return;
      }
      if (!dq_filter_initialized_) {
        dq_filtered_           = dq;
        dq_filter_initialized_ = true;
      } else {
        dq_filtered_ = vel_filter_alpha_ * dq_filtered_ + (1.0 - vel_filter_alpha_) * dq;
      }
      for (int i = 0; i < NUM_JOINTS; ++i) {
        if (q(i) < JOINT_LOWER[i] || q(i) > JOINT_UPPER[i]) {
          emergency_stop("Articulacion " + std::to_string(i+1)
                         + " fuera de limites en warmup: " + std::to_string(q(i)) + " rad");
          return;
        }
      }
      auto cur_cmd = torque_to_current(tau_gravity_, dq_filtered_);
      if (!send_currents(cur_cmd)) {
        emergency_stop("SyncWrite fallido en warmup"); return;
      }
      if (--warmup_ticks_ == 0) {
        warmup_active_ = false;
        start_time_ = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO(get_logger(), "Warmup completado. Iniciando TV-LQR.");
      }
      return;
    }

    const auto tp = std::chrono::high_resolution_clock::now();
    const double t = std::chrono::duration<double>(tp - start_time_).count();

    if (t_run_ > 0.0 && t >= t_run_) {
      RCLCPP_INFO(get_logger(), "Duracion completada (%.2f s).", t_run_);
      if (timer_) timer_->cancel();
      shutdown_hardware();
      if (csv_.is_open()) { csv_.flush(); csv_.close(); }
      rclcpp::shutdown();
      return;
    }

    // ── 1. Lectura de estado ──────────────────────────────────────────────
    Vec4 q, dq;
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    if (!read_state(q, dq, cur_meas)) {
      emergency_stop("SyncRead fallido");
      return;
    }

    // ── 1b. Filtro EMA sobre velocidad medida ─────────────────────────────
    if (!dq_filter_initialized_) {
      dq_filtered_           = dq;
      dq_filter_initialized_ = true;
    } else {
      dq_filtered_ = vel_filter_alpha_ * dq_filtered_ + (1.0 - vel_filter_alpha_) * dq;
    }

    // ── 2. Verificar limites articulares ──────────────────────────────────
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (q(i) < JOINT_LOWER[i] || q(i) > JOINT_UPPER[i]) {
        emergency_stop("Articulacion " + std::to_string(i+1)
                       + " fuera de limites: " + std::to_string(q(i)) + " rad");
        return;
      }
    }

    // ── Seccion 2: Construccion del vector de estado ──────────────────────
    int k = static_cast<int>(std::floor(t / Ts_));
    k = std::max(0, std::min(k, N_ - 1));

    Eigen::Matrix<double,8,1> x_state, x_ref_k;
    x_state << q, dq;
    x_ref_k << q_ref_[k], dq_ref_[k];

    // ── Seccion 3: Ley de control TV-LQR ─────────────────────────────────
    const Vec4 tau = u_ref_[k] - K_TV_[k] * (x_state - x_ref_k);

    // ── Seccion 4: Saturacion y escala de seguridad ───────────────────────
    const Vec4 tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);
    const Vec4 tau_cmd = torque_scale_ * tau_sat;
    auto cur_cmd = torque_to_current(tau_cmd, dq_filtered_);

    // ── 5. Enviar corriente ────────────────────────────────────────────────
    if (!send_currents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // ── 6. Verificar corriente medida ─────────────────────────────────────
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > CURRENT_MEASURED_PEAK) {
        emergency_stop("Corriente insegura en J" + std::to_string(i+1)
                       + ": " + std::to_string(cur_meas[i]) + " ticks");
        return;
      }
    }

    // ── 7. Publicar JointState de monitoreo ───────────────────────────────
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

    // ── Seccion 5: Exportacion de datos al CSV ────────────────────────────
    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t                       << ","
           << q(0)  << "," << q(1)  << "," << q(2)  << "," << q(3)  << ","
           << dq(0) << "," << dq(1) << "," << dq(2) << "," << dq(3) << ","
           << dq_filtered_(0) << "," << dq_filtered_(1) << "," << dq_filtered_(2) << "," << dq_filtered_(3) << ","
           << q_ref_[k](0)  << "," << q_ref_[k](1)  << "," << q_ref_[k](2)  << "," << q_ref_[k](3)  << ","
           << dq_ref_[k](0) << "," << dq_ref_[k](1) << "," << dq_ref_[k](2) << "," << dq_ref_[k](3) << ","
           << tau_sat(0) << "," << tau_sat(1) << "," << tau_sat(2) << "," << tau_sat(3) << ","
           << cur_cmd[0] << "," << cur_cmd[1] << "," << cur_cmd[2] << "," << cur_cmd[3] << ","
           << cur_meas[0] << "," << cur_meas[1] << "," << cur_meas[2] << "," << cur_meas[3]
           << "\n";
    }

    if (++log_cnt_ % 100 == 0) {
      if (csv_.is_open()) csv_.flush();
      RCLCPP_INFO(get_logger(),
        "t=%.2fs  k=%d/%d  q=[%.3f %.3f %.3f %.3f]  |e|=%.4f  i=[%d %d %d %d]",
        t, k, N_-1, q(0), q(1), q(2), q(3),
        (q - q_ref_[k]).norm(),
        cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  std::string ref_dir_;
  std::string port_name_;
  double      torque_scale_, t_run_;
  Vec4        motor_alpha_, motor_Fv_, motor_Fc_, motor_I_offset_;
  double      motor_epsilon_;
  double      vel_cutoff_hz_;
  double      vel_filter_alpha_;
  Vec4        dq_filtered_;
  bool        dq_filter_initialized_{false};

  bool hw_active_;
  bool refs_loaded_;
  int    N_;
  double Ts_;

  Vec4 tau_gravity_;
  int  warmup_ticks_;
  bool warmup_active_;

  std::vector<double>                                    t_ref_;
  std::vector<Vec4>                                      q_ref_;
  std::vector<Vec4>                                      dq_ref_;
  std::vector<Vec4>                                      u_ref_;
  std::vector<Eigen::Matrix<double,4,8,Eigen::ColMajor>> K_TV_;

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead>  grp_pos_, grp_vel_, grp_cur_;
  std::unique_ptr<dynamixel::GroupSyncWrite> grp_wcur_;

  std::chrono::high_resolution_clock::time_point start_time_;
  int log_cnt_{0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::ofstream csv_;
  std::string   csv_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HWTVLQRNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_tvlqr_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
