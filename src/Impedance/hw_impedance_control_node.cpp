/*
 * hw_impedance_control_node.cpp
 * Control de impedancia articular — OpenMANIPULATOR-X hardware real.
 * Via Dynamixel SDK directo, sin ros2_control.
 *
 * Ley de control (por articulacion):
 *   tau = K.*(q_ref - q) + D.*(dq_ref - dq_hat) + gravity_scale.*G(q)   [N·m]
 *   I   = alpha*tau + fc_scale*Fc*tanh(dq_ref/eps)                      [ticks]
 *
 * El robot se comporta como un resorte-amortiguador virtual anclado en q_des:
 * al alejarlo manualmente, regresa automaticamente; el torque de retorno esta
 * acotado por tau_max (1.2 N·m) → compliant e intrinsecamente seguro.
 *
 * Secuencia de arranque (todo en Current Mode, sin cambio de modo del motor):
 *   1) t < soft_start_s          : rampa quintica 0→1 del torque con la
 *      referencia CONGELADA en la pose inicial (el brazo no se mueve; evita
 *      el escalon de corriente que rompe la stiction via backlash — leccion
 *      del nodo de gravity comp).
 *   2) t < soft_start_s + t_trans: transicion quintica q_inicial → q_des.
 *      La PROPIA ley de impedancia rastrea la referencia movil; el torque
 *      queda acotado por tau_max durante todo el trayecto (a diferencia del
 *      Position Mode del motor, cuyo PID interno empuja hasta Current Limit
 *      y ademas exigiria torque-off para conmutar de modo, dejando caer el
 *      brazo un instante).
 *   3) t >= soft_start_s+t_trans : impedancia pura en q_des (dq_ref = 0).
 *
 * NOTA friccion: Fc se alimenta SOLO con la velocidad DESEADA (limpia, y nula
 * al mantener la pose) → ayuda al despegue durante la transicion y desaparece
 * en regimen. Fv e I_offset NO se aplican: con velocidad medida son
 * realimentacion positiva / sesgo en semi-lazo-abierto (diagnostico
 * hw_gc_data_1/2 de gravity comp). La friccion residual del reductor suma
 * amortiguamiento natural; la zona muerta de llegada es ≈ Fc/K por junta.
 *
 * Parametros del MOTOR (alpha, Fc, eps) y config de hardware (zero_tick,
 * signos, limites): auto-cargados de config/motorXM430W350T_params.yaml y
 * FIJOS (el YAML tiene prioridad sobre el CLI).
 *
 * Parametros propios del nodo (ros2 run ... --ros-args -p nombre:=valor):
 *   q_des             [double[4]] [0.0, 0.0, 0.0, 0.0]  pose a mantener [rad]
 *   stiffness         [double[4]] [4.0, 5.0, 5.0, 2.5]  K [N·m/rad]
 *   damping           [double[4]] [0.35, 0.40, 0.28, 0.07] D [N·m·s/rad]
 *                      (≈ 2·0.7·sqrt(K·M_ii) en la pose home; la friccion
 *                       real agrega amortiguamiento extra)
 *   gravity_scale       [double]   1.0
 *   gravity_scale_joint [double[4]] [1.0, 0.85, 0.85, 1.0] (correccion del
 *                        G(q) del URDF identificada en gravity comp)
 *   soft_start_s      [double]   1.5    rampa inicial de torque
 *   t_trans           [double]   3.0    transicion quintica hacia q_des
 *   dq_max            [double]   3.0    parada de seguridad [rad/s]
 *   friction_fc_scale [double]   0.95   fraccion de Fc en feedforward
 *   ab_alpha          [double]   0.2    filtro α-β (posicion)
 *   ab_beta           [double]   0.02   filtro α-β (velocidad)
 *   loop_rate_hz      [double]   200.0  acotado a [50, 400]
 *   duration_s        [double]   0.0    (0 = sin limite de tiempo)
 *   log_id            [int]      1      CSV: hw_imp_data_<log_id>.csv
 *   port_name         [string]   "/dev/ttyUSB0"
 *
 * Ejemplos de uso (no requiere launch; el YAML se auto-carga):
 *   ros2 run open_manipulator_x_torque_control hw_impedance_control_node \
 *     --ros-args -p log_id:=1
 *   ros2 run open_manipulator_x_torque_control hw_impedance_control_node \
 *     --ros-args -p q_des:="[0.0, -0.3, 0.4, 0.5]" \
 *     -p stiffness:="[6.0, 8.0, 8.0, 3.0]" -p damping:="[0.4, 0.5, 0.35, 0.1]"
 *
 * Publisher:  /hw/joint_states (sensor_msgs/JointState) — monitoreo
 * CSV output: data/impedance/hw_imp_data_<log_id>.csv
 *
 * ADVERTENCIA: No ejecutar junto a otro proceso que acceda a /dev/ttyUSB0
 * (ros2_control_node, dynamixel_hardware_interface, etc.).
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
#ifndef PACKAGE_CONFIG_DIR
#define PACKAGE_CONFIG_DIR "."
#endif

using namespace std::chrono_literals;

// ============================================================
// Constantes Dynamixel / conversion de unidades
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

// Bloque contiguo current(126,2)+velocity(128,4)+position(132,4): una sola
// GroupSyncRead por ciclo en vez de tres → latencia de bus ≈ 1/3.
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

// Margen tolerado mas alla del limite articular antes de la parada de seguridad
static constexpr double LIMIT_MARGIN_RAD = 0.05;
// Margen minimo de q_des respecto a los limites articulares
static constexpr double QDES_MARGIN_RAD = 0.02;

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Utilidades de conversion (identicas a hw_fl_control_node)
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

// Polinomio quintico 0→1 con velocidad/aceleracion nulas en los extremos
static inline double quinticS(double x)
{
  x = std::min(std::max(x, 0.0), 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}
static inline double quinticDS(double x)  // ds/dx
{
  x = std::min(std::max(x, 0.0), 1.0);
  return x * x * (30.0 + x * (-60.0 + 30.0 * x));
}

// ============================================================
// Nodo ROS 2
// ============================================================

class HWImpedanceControlNode : public rclcpp::Node
{
public:
  explicit HWImpedanceControlNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("hw_impedance_control_node", opts),
    hw_active_(false), q_initial_captured_(false)
  {
    // ── Parametros propios del nodo ──────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",         "/dev/ttyUSB0");
    this->declare_parameter<double>     ("gravity_scale",      1.0);
    this->declare_parameter<double>     ("soft_start_s",       1.5);
    this->declare_parameter<double>     ("t_trans",            3.0);
    this->declare_parameter<double>     ("dq_max",             3.0);
    this->declare_parameter<double>     ("friction_fc_scale",  0.95);
    this->declare_parameter<double>     ("ab_alpha",           0.2);
    this->declare_parameter<double>     ("ab_beta",            0.02);
    this->declare_parameter<double>     ("loop_rate_hz",       200.0);
    this->declare_parameter<double>     ("duration_s",         0.0);
    this->declare_parameter<int>        ("log_id",             1);

    using dvec = std::vector<double>;
    this->declare_parameter<dvec>("q_des",     dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<dvec>("stiffness", dvec{4.0, 5.0, 5.0, 2.5});
    this->declare_parameter<dvec>("damping",   dvec{0.35, 0.40, 0.28, 0.07});
    this->declare_parameter<dvec>("gravity_scale_joint", dvec{1.0, 0.85, 0.85, 1.0});

    // ── Modelo del motor y config de hardware (motorXM430W350T_params.yaml) ──
    // Solo se usan alpha, Fc y eps; Fv/I_offset NO se aplican (ver cabecera).
    const double alpha_def = 1.0 / TORQUE_PER_CURRENT_TICK;
    this->declare_parameter<dvec>("motor_alpha", dvec{alpha_def, alpha_def, alpha_def, alpha_def});
    this->declare_parameter<dvec>("motor_Fc",    dvec{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<double>("motor_epsilon_friction", 0.05);

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

    port_name_     = this->get_parameter("port_name").as_string();
    gravity_scale_ = this->get_parameter("gravity_scale").as_double();
    soft_start_s_  = this->get_parameter("soft_start_s").as_double();
    t_trans_       = this->get_parameter("t_trans").as_double();
    dq_max_        = this->get_parameter("dq_max").as_double();
    fc_scale_      = this->get_parameter("friction_fc_scale").as_double();
    ab_alpha_      = this->get_parameter("ab_alpha").as_double();
    ab_beta_       = this->get_parameter("ab_beta").as_double();
    loop_rate_hz_  = std::min(std::max(this->get_parameter("loop_rate_hz").as_double(), 50.0), 400.0);
    Ts_            = 1.0 / loop_rate_hz_;
    duration_s_    = this->get_parameter("duration_s").as_double();
    const int log_id = this->get_parameter("log_id").as_int();

    auto load_vec4 = [this](const std::string & name) {
      const auto v = this->get_parameter(name).as_double_array();
      Vec4 out; for (int i = 0; i < NUM_JOINTS; ++i) out(i) = v[i]; return out;
    };
    q_des_        = load_vec4("q_des");
    K_            = load_vec4("stiffness");
    D_            = load_vec4("damping");
    gscale_joint_ = load_vec4("gravity_scale_joint");
    motor_alpha_  = load_vec4("motor_alpha");
    motor_Fc_     = load_vec4("motor_Fc");
    motor_epsilon_ = this->get_parameter("motor_epsilon_friction").as_double();
    encoder_sign_ = load_vec4("encoder_sign");
    current_sign_ = load_vec4("current_sign");
    joint_lower_  = load_vec4("joint_lower");
    joint_upper_  = load_vec4("joint_upper");
    {
      const auto zt = this->get_parameter("joint_zero_tick").as_integer_array();
      const auto cl = this->get_parameter("current_cmd_limit").as_integer_array();
      for (int i = 0; i < NUM_JOINTS; ++i) {
        joint_zero_tick_[i]   = static_cast<int32_t>(zt[i]);
        current_cmd_limit_[i] = static_cast<int16_t>(cl[i]);
      }
    }
    current_limit_register_ = static_cast<uint16_t>(this->get_parameter("current_limit_register").as_int());
    current_measured_peak_  = static_cast<int16_t>(this->get_parameter("current_measured_peak").as_int());
    tau_max_ = this->get_parameter("tau_max").as_double();

    // Validaciones: q_des dentro de limites, ganancias no negativas
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (q_des_(i) < joint_lower_(i) + QDES_MARGIN_RAD ||
          q_des_(i) > joint_upper_(i) - QDES_MARGIN_RAD) {
        RCLCPP_FATAL(get_logger(),
          "q_des[%d]=%.3f fuera de limites [%.3f, %.3f] (margen %.2f)",
          i, q_des_(i), joint_lower_(i), joint_upper_(i), QDES_MARGIN_RAD);
        throw std::runtime_error("q_des fuera de limites articulares");
      }
      if (K_(i) < 0.0 || D_(i) < 0.0) {
        RCLCPP_FATAL(get_logger(), "stiffness/damping[%d] negativo", i);
        throw std::runtime_error("Ganancias de impedancia invalidas");
      }
    }

    RCLCPP_INFO(get_logger(),
      "q_des=[%.3f %.3f %.3f %.3f]  K=[%.1f %.1f %.1f %.1f]  D=[%.2f %.2f %.2f %.2f]",
      q_des_(0), q_des_(1), q_des_(2), q_des_(3),
      K_(0), K_(1), K_(2), K_(3), D_(0), D_(1), D_(2), D_(3));
    RCLCPP_INFO(get_logger(),
      "g_scale=%.2f  g_joint=[%.2f %.2f %.2f %.2f]  rampa=%.1fs  trans=%.1fs  "
      "dq_max=%.1f  lazo=%.0f Hz  tau_max=%.2f",
      gravity_scale_, gscale_joint_(0), gscale_joint_(1), gscale_joint_(2), gscale_joint_(3),
      soft_start_s_, t_trans_, dq_max_, loop_rate_hz_, tau_max_);

    // ── Pinocchio ────────────────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(get_logger(), "Pinocchio: nv=%d", model_.nv);

    // ── CSV ──────────────────────────────────────────────────────────────────
    const std::string csv_dir = std::string(PACKAGE_DATA_DIR) + "/impedance";
    std::filesystem::create_directories(csv_dir);
    csv_path_ = csv_dir + "/hw_imp_data_" + std::to_string(log_id) + ".csv";
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

    // ── Publisher de monitoreo ────────────────────────────────────────────────
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── SDK ──────────────────────────────────────────────────────────────────
    if (!init_hardware()) {
      RCLCPP_FATAL(get_logger(), "Fallo hardware init. Abortando.");
      throw std::runtime_error("Hardware init failed");
    }

    // ── Timer de control (loop_rate_hz) ──────────────────────────────────────
    start_time_ = std::chrono::high_resolution_clock::now();
    const auto period = std::chrono::microseconds(
      static_cast<int64_t>(std::lround(1e6 / loop_rate_hz_)));
    timer_ = this->create_wall_timer(period, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Control activo a %.0f Hz. Ctrl+C para detener.", loop_rate_hz_);
  }

  ~HWImpedanceControlNode()
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
      RCLCPP_INFO(get_logger(), "DXL ID %d listo (Current Mode)", static_cast<int>(id));
    }

    grp_read_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_STATE_BLOCK, LEN_STATE_BLOCK);
    grp_wcur_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    for (const auto id : DXL_ID)
      grp_read_->addParam(id);

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

  // ─────────────────────────────────────────────────────────────────────────
  //  Lectura de estado (SyncRead fusionado: corriente+velocidad+posicion)
  // ─────────────────────────────────────────────────────────────────────────

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

      q(i)   = encoder_sign_(i) * static_cast<double>(wrappedTickDiff(rp, joint_zero_tick_[i])) * POS_UNIT_RAD;
      dq(i)  = encoder_sign_(i) * static_cast<double>(rv) * VEL_UNIT_RAD_S;
      cur[i] = rc;
    }
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Escritura de corriente (SyncWrite Goal_Current)
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
  //  Gravedad del modelo con correccion por junta:
  //  G_i(q) * gravity_scale * gravity_scale_joint[i]
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_gravity(const Vec4 & q)
  {
    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd zero_v = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head(NUM_JOINTS) = q;

    const Eigen::VectorXd tau_full =
      pinocchio::rnea(model_, data_, q_pin, zero_v, zero_v);

    const Vec4 g4 = tau_full.head(NUM_JOINTS);
    return gravity_scale_ * gscale_joint_.cwiseProduct(g4);
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Conversion torque → corriente:  I = alpha·tau + fc_scale·Fc·tanh(dq_ref/eps)
  //  Fc SOLO con velocidad deseada (nula en regimen → jamas realimenta el
  //  movimiento impuesto por el usuario). Sin Fv ni I_offset (ver cabecera).
  // ─────────────────────────────────────────────────────────────────────────

  std::array<int16_t, NUM_JOINTS> torque_to_current(const Vec4 & tau, const Vec4 & dq_ref)
  {
    std::array<int16_t, NUM_JOINTS> cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double I = motor_alpha_(i) * tau(i)
                     + fc_scale_ * motor_Fc_(i) * std::tanh(dq_ref(i) / motor_epsilon_);
      cmd[i] = clampCurrent(current_sign_(i) * encoder_sign_(i) * I, current_cmd_limit_[i]);
    }
    return cmd;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Parada de emergencia
  // ─────────────────────────────────────────────────────────────────────────

  void emergency_stop(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "PARADA DE EMERGENCIA: %s", reason.c_str());
    timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    rclcpp::shutdown();
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Callback del timer
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    if (!hw_active_) return;

    const auto tick_t0 = std::chrono::steady_clock::now();
    const auto tp = std::chrono::high_resolution_clock::now();
    const double t = std::chrono::duration<double>(tp - start_time_).count();

    if (duration_s_ > 0.0 && t >= duration_s_) {
      RCLCPP_INFO(get_logger(), "Duracion completada (%.1f s).", duration_s_);
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

    // 1b. Filtro α-β: estimacion conjunta de posicion y velocidad
    if (!ab_initialized_) {
      q_hat_  = q;
      dq_hat_ = dq;
      ab_initialized_ = true;
    } else {
      const Vec4 q_pred  = q_hat_ + Ts_ * dq_hat_;
      const Vec4 dq_pred = dq_hat_;
      const Vec4 r       = q - q_pred;
      q_hat_  = q_pred  + ab_alpha_ * r;
      dq_hat_ = dq_pred + (ab_beta_ / Ts_) * r;
    }

    // 2. Guardas de seguridad: limites articulares y velocidad excesiva
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (q(i) < joint_lower_(i) - LIMIT_MARGIN_RAD ||
          q(i) > joint_upper_(i) + LIMIT_MARGIN_RAD) {
        emergency_stop("J" + std::to_string(i + 1) + " fuera de limite articular: q="
          + std::to_string(q(i)) + " rad");
        return;
      }
      if (std::abs(dq(i)) > dq_max_) {
        emergency_stop("Velocidad excesiva en J" + std::to_string(i + 1)
          + ": dq=" + std::to_string(dq(i)) + " rad/s");
        return;
      }
    }

    // 3. Captura de la pose inicial (primer tick)
    if (!q_initial_captured_) {
      q_initial_ = q;
      q_initial_captured_ = true;
      RCLCPP_INFO(get_logger(),
        "q_inicial=[%.3f %.3f %.3f %.3f] rad. Rampa %.1f s + transicion %.1f s hacia q_des.",
        q(0), q(1), q(2), q(3), soft_start_s_, t_trans_);
    }

    // 4. Referencia por fases: rampa (congelada en q_inicial) → quintica → q_des
    Vec4 q_ref, dq_ref;
    double ramp = 1.0;
    if (soft_start_s_ > 0.0 && t < soft_start_s_) {
      // Fase 1: la referencia se mantiene en q_inicial y el torque sube 0→1
      ramp   = quinticS(t / soft_start_s_);
      q_ref  = q_initial_;
      dq_ref = Vec4::Zero();
    } else if (t < soft_start_s_ + t_trans_ && t_trans_ > 0.0) {
      // Fase 2: transicion quintica q_inicial → q_des
      if (!trans_announced_) {
        trans_announced_ = true;
        RCLCPP_INFO(get_logger(), "Rampa completada: transicion hacia q_des...");
      }
      const double x = (t - soft_start_s_) / t_trans_;
      q_ref  = q_initial_ + quinticS(x) * (q_des_ - q_initial_);
      dq_ref = (quinticDS(x) / t_trans_) * (q_des_ - q_initial_);
    } else {
      // Fase 3: impedancia en q_des
      if (!hold_announced_) {
        hold_announced_ = true;
        RCLCPP_INFO(get_logger(),
          "Impedancia activa en q_des=[%.3f %.3f %.3f %.3f]. Puede empujar el robot.",
          q_des_(0), q_des_(1), q_des_(2), q_des_(3));
      }
      q_ref  = q_des_;
      dq_ref = Vec4::Zero();
    }

    // 5. Ley de impedancia: resorte-amortiguador virtual + compensacion de gravedad
    const Vec4 tau_unsat = ramp * (K_.cwiseProduct(q_ref - q)
                                 + D_.cwiseProduct(dq_ref - dq_hat_)
                                 + compute_gravity(q));
    const Vec4 tau = tau_unsat.cwiseMin(tau_max_).cwiseMax(-tau_max_);
    const auto cur_cmd = torque_to_current(tau, dq_ref);

    // 6. Escribir corriente
    if (!send_currents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 7. Verificar corriente medida (seguridad)
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > current_measured_peak_) {
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
           << ',' << dq_hat_(0) << ',' << dq_hat_(1) << ',' << dq_hat_(2) << ',' << dq_hat_(3)
           << ',' << q_ref(0)   << ',' << q_ref(1)   << ',' << q_ref(2)   << ',' << q_ref(3)
           << ',' << dq_ref(0)  << ',' << dq_ref(1)  << ',' << dq_ref(2)  << ',' << dq_ref(3)
           << ',' << tau(0)     << ',' << tau(1)     << ',' << tau(2)     << ',' << tau(3)
           << ',' << cur_cmd[0] << ',' << cur_cmd[1] << ',' << cur_cmd[2] << ',' << cur_cmd[3]
           << ',' << cur_meas[0]<< ',' << cur_meas[1]<< ',' << cur_meas[2]<< ',' << cur_meas[3]
           << '\n';
    }

    // 10. Log periodico por consola (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      RCLCPP_INFO(get_logger(),
        "t=%.1fs  q=[%.3f %.3f %.3f %.3f]  |e|=%.4f  i=[%d %d %d %d]",
        t, q(0), q(1), q(2), q(3), (q - q_ref).norm(),
        cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }

    // 11. Deteccion de overrun del lazo
    const double tick_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - tick_t0).count();
    if (tick_ms > 1e3 * Ts_) {
      if (++overrun_cnt_ % 50 == 1) {
        RCLCPP_WARN(get_logger(), "Overrun del lazo: tick=%.2f ms > Ts=%.1f ms (n=%d)",
          tick_ms, 1e3 * Ts_, overrun_cnt_);
      }
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  pinocchio::Model model_;
  pinocchio::Data  data_;

  std::string port_name_;
  Vec4   q_des_, K_, D_;
  double gravity_scale_;
  Vec4   gscale_joint_;
  double soft_start_s_, t_trans_;
  double dq_max_;
  double fc_scale_;
  double duration_s_;

  // Modelo del motor y config de hardware (config/motorXM430W350T_params.yaml)
  Vec4   motor_alpha_, motor_Fc_;
  double motor_epsilon_;
  std::array<int32_t, NUM_JOINTS> joint_zero_tick_{};
  Vec4     encoder_sign_, current_sign_;
  Vec4     joint_lower_, joint_upper_;
  uint16_t current_limit_register_;
  std::array<int16_t, NUM_JOINTS> current_cmd_limit_{};
  int16_t  current_measured_peak_;
  double   tau_max_;

  double ab_alpha_, ab_beta_;
  double loop_rate_hz_{200.0};
  double Ts_{0.005};
  Vec4   q_hat_, dq_hat_;
  bool   ab_initialized_{false};

  dynamixel::PortHandler *   port_handler_{nullptr};
  dynamixel::PacketHandler * packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead>  grp_read_;
  std::unique_ptr<dynamixel::GroupSyncWrite> grp_wcur_;

  bool hw_active_;
  bool q_initial_captured_;
  bool trans_announced_{false};
  bool hold_announced_{false};
  Vec4 q_initial_;
  std::chrono::high_resolution_clock::time_point start_time_;
  int log_cnt_{0};
  int overrun_cnt_{0};

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

  // Auto-carga config/motorXM430W350T_params.yaml; los -p de la línea de comandos
  // (q_des, stiffness, damping, gravity_scale, gravity_scale_joint, soft_start_s,
  // t_trans, dq_max, log_id, ...) lo sobrescriben → no requiere launch.
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
    RCLCPP_INFO(rclcpp::get_logger("hw_impedance_control_node"),
      "motorXM430W350T_params auto-cargado: %s", cfg.c_str());
  } else {
    RCLCPP_WARN(rclcpp::get_logger("hw_impedance_control_node"),
      "motorXM430W350T_params no encontrado: %s — usando defaults.", cfg.c_str());
  }

  try {
    rclcpp::spin(std::make_shared<HWImpedanceControlNode>(opts));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_impedance_control_node"),
      "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
