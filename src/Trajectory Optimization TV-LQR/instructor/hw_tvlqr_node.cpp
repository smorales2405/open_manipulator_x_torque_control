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
 *   - Lectura de estado en UN SOLO SyncRead del bloque contiguo 126-135
 *     (corriente + velocidad + posicion): una transaccion USB por tick,
 *     habilita el lazo a 200 Hz (read_state)
 *   - Filtro α-β sobre la posicion medida: estima dq_hat sin la cuantizacion
 *     ni el retardo del registro de velocidad Dynamixel (defaults validados
 *     en el Lab 4: α=0.2, β=0.02 @ 200 Hz)
 *   - Escritura de corriente SyncWrite (send_currents)
 *   - Conversion torque -> corriente via modelo OLS identificado (motor_params.yaml);
 *     la compensacion de Coulomb usa la velocidad DESEADA dq_ref (leccion del
 *     Lab 4: sin chattering por ruido de dq y empuje de despegue al arrancar)
 *   - Verificacion de corriente medida (parada de emergencia)
 *   - Deteccion de overrun del lazo
 *   - Publicador /hw/joint_states para monitoreo
 *
 * Arranque: el nodo inicia TV-LQR directamente, SIN warmup de compensacion
 * gravitatoria. La posicion inicial q_init = [pi/2, 0, pi/6, pi/3] es
 * autoestable (la reductora la sostiene sin torque) y aplicar tau_gravity en
 * escalon solo producia un impulso inicial indeseado.
 *
 * Parametros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name              [string]    "/dev/ttyUSB0"
 *   torque_scale           [double]    1.0   (escala de seguridad, rango 0..1)
 *   test_num               [int]       1     (identificador del CSV)
 *   reference_dir          [string]    "src/Trajectory Optimization TV-LQR/references"
 *   ab_alpha               [double]    0.2   (filtro α-β: ganancia de posicion)
 *   ab_beta                [double]    0.02  (filtro α-β: ganancia de velocidad)
 *   loop_rate_hz           [double]    200.0 (frecuencia del lazo [Hz], rango 50..400;
 *                                             requiere latency_timer=1 en el FTDI)
 *   friction_fc_scale      [double]    0.95  (fraccion de Fc compensada)
 *
 * Duracion: automatica, igual al ultimo instante de time_ref.txt (tf = N*Ts).
 * Al completar la trayectoria el nodo apaga los motores y termina solo.
 *
 * USO TIPICO:
 *   ros2 run open_manipulator_x_torque_control hw_tvlqr_node --ros-args -p test_num:=1
 *
 * Publisher: /hw/joint_states (sensor_msgs/JointState) — monitoreo
 *
 * ADVERTENCIA: No ejecutar junto a hardware.launch.py ni ningun proceso
 * que acceda al puerto USB (ros2_control_node, dynamixel_hardware_interface).
 *
 * CSV generado: data/lab5/real/data_log_real_lab5_<test_num>.csv
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

// Bloque contiguo 126-135: Present Current (2B) + Velocity (4B) + Position (4B).
// Un solo SyncRead del bloque reemplaza 3 transacciones USB por tick.
static constexpr uint16_t ADDR_STATE_BLOCK = ADDR_PRESENT_CURRENT;
static constexpr uint16_t LEN_STATE_BLOCK  = 10;

static constexpr uint8_t CURRENT_CONTROL_MODE = 0;
static constexpr uint8_t TORQUE_ENABLE_VAL    = 1;
static constexpr uint8_t TORQUE_DISABLE_VAL   = 0;

static constexpr double POS_UNIT_RAD            = 2.0 * PI / 4096.0;
static constexpr double VEL_UNIT_RAD_S          = 0.229 * 2.0 * PI / 60.0;
static constexpr double CURRENT_UNIT_A          = 0.00269;
static constexpr double TORQUE_CONSTANT_NM_A    = 1.654;
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
  explicit HWTVLQRNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("hw_tvlqr_node", opts),
    hw_active_(false), refs_loaded_(false), N_(0), Ts_(0.05)
  {
    // ── Parametros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",     "/dev/ttyUSB0");
    this->declare_parameter<double>     ("torque_scale",   1.0);
    this->declare_parameter<int>        ("test_num",        1);
    this->declare_parameter<std::string>("reference_dir",
      "src/Trajectory Optimization TV-LQR/references");

    using dvec = std::vector<double>;
    this->declare_parameter<dvec>("motor_alpha",            dvec{208.5, 208.5, 208.5, 208.5});
    this->declare_parameter<dvec>("motor_Fv",               dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<dvec>("motor_Fc",               dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<dvec>("motor_I_offset",         dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<double>("motor_epsilon_friction", 0.05);
    this->declare_parameter<double>("friction_fc_scale",      0.95);
    this->declare_parameter<double>("ab_alpha",     0.2);
    this->declare_parameter<double>("ab_beta",      0.02);
    this->declare_parameter<double>("loop_rate_hz", 200.0);

    port_name_        = this->get_parameter("port_name").as_string();
    torque_scale_     = std::min(std::max(this->get_parameter("torque_scale").as_double(), 0.0), 1.0);
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
    fc_scale_       = get_parameter("friction_fc_scale").as_double();

    ab_alpha_     = get_parameter("ab_alpha").as_double();
    ab_beta_      = get_parameter("ab_beta").as_double();
    loop_rate_hz_ = std::min(std::max(get_parameter("loop_rate_hz").as_double(), 50.0), 400.0);
    Ts_loop_      = 1.0 / loop_rate_hz_;

    RCLCPP_INFO(get_logger(),
      "puerto=%s  scale=%.2f  id=%d",
      port_name_.c_str(), torque_scale_, test_num);
    RCLCPP_INFO(get_logger(),
      "motor α=[%.1f %.1f %.1f %.1f]  Fv=[%.2f %.2f %.2f %.2f]  ε=%.3f  fc_scale=%.2f",
      motor_alpha_(0), motor_alpha_(1), motor_alpha_(2), motor_alpha_(3),
      motor_Fv_(0), motor_Fv_(1), motor_Fv_(2), motor_Fv_(3), motor_epsilon_, fc_scale_);
    RCLCPP_INFO(get_logger(),
      "Filtro α-β: α=%.3f  β=%.3f  |  lazo: %.0f Hz (Ts=%.1f ms)",
      ab_alpha_, ab_beta_, loop_rate_hz_, 1e3 * Ts_loop_);
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
    t_end_ = t_ref_.back();   // duracion = ultimo instante de time_ref.txt
    RCLCPP_INFO(get_logger(),
      "Referencias cargadas: N=%d  Ts=%.3f s  (duracion: %.2f s)", N_, Ts_, t_end_);

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
    const auto period = std::chrono::microseconds(
      static_cast<int64_t>(std::lround(1e6 / loop_rate_hz_)));
    timer_ = this->create_wall_timer(period, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Control TV-LQR activo a %.0f Hz (Ts=%.1f ms). Ctrl+C para detener.",
      loop_rate_hz_, 1e3 * Ts_loop_);
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

    // SyncRead unico del bloque contiguo corriente+velocidad+posicion
    grp_read_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_STATE_BLOCK, LEN_STATE_BLOCK);
    grp_wcur_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    for (const auto id : DXL_ID) {
      grp_read_->addParam(id);
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
    if (grp_read_->txRxPacket() != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncRead fallo");
      return false;
    }
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const uint8_t id = DXL_ID[i];
      if (!grp_read_->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) ||
          !grp_read_->isAvailable(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY) ||
          !grp_read_->isAvailable(id, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT)) {
        RCLCPP_ERROR(get_logger(), "[ID %d] dato no disponible", id);
        return false;
      }
      const int32_t rp = toSigned32(static_cast<uint32_t>(
        grp_read_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)));
      const int32_t rv = toSigned32(static_cast<uint32_t>(
        grp_read_->getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)));
      const int16_t rc = toSigned16(static_cast<uint32_t>(
        grp_read_->getData(id, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT)));

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

  // Fv usa la velocidad medida (término suave); Fc usa la velocidad DESEADA:
  // señal sin ruido → sin chattering, y aporta el empuje de despegue justo
  // cuando la referencia arranca (mismo criterio que hw_fl_control_node).
  std::array<int16_t, NUM_JOINTS> torque_to_current(const Vec4 & tau, const Vec4 & dq,
                                                    const Vec4 & dq_des)
  {
    static const std::array<int16_t, NUM_JOINTS> cur_lim = {
      CURRENT_CMD_LIMIT_J123, CURRENT_CMD_LIMIT_J123,
      CURRENT_CMD_LIMIT_J123, CURRENT_CMD_LIMIT_J4
    };
    std::array<int16_t, NUM_JOINTS> cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double I_model = motor_alpha_(i) * tau(i)
                           + motor_Fv_(i)    * dq(i)
                           + fc_scale_ * motor_Fc_(i) * std::tanh(dq_des(i) / motor_epsilon_)
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
  //  Callback del timer (loop_rate_hz)
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    if (!hw_active_ || !refs_loaded_) return;

    const auto tick_t0 = std::chrono::steady_clock::now();
    const auto tp = std::chrono::high_resolution_clock::now();
    const double t = std::chrono::duration<double>(tp - start_time_).count();

    if (t >= t_end_) {
      RCLCPP_INFO(get_logger(), "Trayectoria completada (%.2f s).", t_end_);
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

    // ── 1b. Filtro α-β: estimacion conjunta de posicion y velocidad ───────
    //   prediccion: q_pred = q_hat + Ts·dq_hat,  dq_pred = dq_hat
    //   residuo:    r      = q_meas - q_pred
    //   correccion: q_hat  = q_pred + α·r,        dq_hat = dq_pred + (β/Ts)·r
    //   dq_hat evita la cuantizacion (~0.024 rad/s) y el retardo del registro
    //   de velocidad Dynamixel (defaults validados en el Lab 4).
    if (!ab_initialized_) {
      q_hat_          = q;
      dq_hat_         = dq;
      ab_initialized_ = true;
    } else {
      const Vec4 q_pred  = q_hat_ + Ts_loop_ * dq_hat_;
      const Vec4 dq_pred = dq_hat_;
      const Vec4 r       = q - q_pred;
      q_hat_  = q_pred  + ab_alpha_ * r;
      dq_hat_ = dq_pred + (ab_beta_ / Ts_loop_) * r;
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

    // Estado para el feedback: posicion medida (cuantizacion fina, 0.088°)
    // y velocidad estimada por el filtro α-β.
    Eigen::Matrix<double,8,1> x_state, x_ref_k;
    x_state << q, dq_hat_;
    x_ref_k << q_ref_[k], dq_ref_[k];

    // ── Seccion 3: Ley de control TV-LQR ─────────────────────────────────
    const Vec4 tau = u_ref_[k] - K_TV_[k] * (x_state - x_ref_k);

    // ── Seccion 4: Saturacion y escala de seguridad ───────────────────────
    const Vec4 tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);
    const Vec4 tau_cmd = torque_scale_ * tau_sat;
    auto cur_cmd = torque_to_current(tau_cmd, dq_hat_, dq_ref_[k]);

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
           << dq_hat_(0) << "," << dq_hat_(1) << "," << dq_hat_(2) << "," << dq_hat_(3) << ","
           << q_ref_[k](0)  << "," << q_ref_[k](1)  << "," << q_ref_[k](2)  << "," << q_ref_[k](3)  << ","
           << dq_ref_[k](0) << "," << dq_ref_[k](1) << "," << dq_ref_[k](2) << "," << dq_ref_[k](3) << ","
           << tau_sat(0) << "," << tau_sat(1) << "," << tau_sat(2) << "," << tau_sat(3) << ","
           << cur_cmd[0] << "," << cur_cmd[1] << "," << cur_cmd[2] << "," << cur_cmd[3] << ","
           << cur_meas[0] << "," << cur_meas[1] << "," << cur_meas[2] << "," << cur_meas[3]
           << "\n";
    }

    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      RCLCPP_INFO(get_logger(),
        "t=%.2fs  k=%d/%d  q=[%.3f %.3f %.3f %.3f]  |e|=%.4f  i=[%d %d %d %d]",
        t, k, N_-1, q(0), q(1), q(2), q(3),
        (q - q_ref_[k]).norm(),
        cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }

    // ── 8. Deteccion de overrun: si el ciclo (bus + control) no cabe en el
    //    periodo, el lazo real corre mas lento de lo configurado → bajar
    //    loop_rate_hz o revisar el latency_timer del adaptador USB-serial.
    const double tick_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - tick_t0).count();
    if (tick_ms > 1e3 * Ts_loop_) {
      if (++overrun_cnt_ % 50 == 1) {
        RCLCPP_WARN(get_logger(), "Overrun del lazo: tick=%.2f ms > Ts=%.1f ms (n=%d)",
          tick_ms, 1e3 * Ts_loop_, overrun_cnt_);
      }
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  std::string ref_dir_;
  std::string port_name_;
  double      torque_scale_;
  double      t_end_{0.0};   // duracion de la trayectoria (time_ref.txt)
  Vec4        motor_alpha_, motor_Fv_, motor_Fc_, motor_I_offset_;
  double      motor_epsilon_;
  double      fc_scale_;

  // Filtro α-β y lazo de control
  double ab_alpha_{0.2}, ab_beta_{0.02};
  double loop_rate_hz_{200.0};
  double Ts_loop_{0.005};          // periodo del lazo (≠ Ts_ de la referencia)
  Vec4   q_hat_{Vec4::Zero()}, dq_hat_{Vec4::Zero()};
  bool   ab_initialized_{false};
  int    overrun_cnt_{0};

  bool hw_active_;
  bool refs_loaded_;
  int    N_;
  double Ts_;

  std::vector<double>                                    t_ref_;
  std::vector<Vec4>                                      q_ref_;
  std::vector<Vec4>                                      dq_ref_;
  std::vector<Vec4>                                      u_ref_;
  std::vector<Eigen::Matrix<double,4,8,Eigen::ColMajor>> K_TV_;

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead>  grp_read_;
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

  // Auto-carga config/motorXM430W350T_params.yaml; los -p de la línea de comandos
  // (test_num, torque_scale, etc.) lo sobrescriben → no requiere launch.
  rclcpp::NodeOptions opts;
  const std::string cfg = std::string(PACKAGE_CONFIG_DIR) + "/motorXM430W350T_params.yaml";
  if (std::filesystem::exists(cfg)) {
    std::vector<std::string> args = {"--ros-args"};
    bool in_ros_args = false;
    for (int i = 1; i < argc; ++i) {
      const std::string a(argv[i]);
      if (a == "--ros-args") { in_ros_args = true; continue; }
      if (in_ros_args) args.push_back(a);
    }
    // params-file AL FINAL → el YAML del motor tiene prioridad sobre los -p del CLI:
    // los parámetros del motor NO se pueden sobreescribir; los params propios del
    // nodo (no presentes en el YAML) sí se ajustan con -p.
    args.push_back("--params-file");
    args.push_back(cfg);
    opts.arguments(args);
    opts.use_global_arguments(false);
    RCLCPP_INFO(rclcpp::get_logger("hw_tvlqr_node"),
      "motorXM430W350T_params auto-cargado: %s", cfg.c_str());
  } else {
    RCLCPP_WARN(rclcpp::get_logger("hw_tvlqr_node"),
      "motorXM430W350T_params no encontrado: %s — usando defaults.", cfg.c_str());
  }

  try {
    rclcpp::spin(std::make_shared<HWTVLQRNode>(opts));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_tvlqr_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
