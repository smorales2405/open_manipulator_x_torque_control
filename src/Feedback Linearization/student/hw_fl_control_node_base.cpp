/*
 * hw_fl_control_node_base.cpp
 * Plantilla: Controlador Feedback Linearization — OpenMANIPULATOR-X hardware real
 * via Dynamixel SDK directo.
 *
 * Ley de control a implementar:
 *   tau = M(q) * v + nle(q, dq)
 *   v   = ddq_des + Kd*(dq_des - dq) + Kp*(q_des - q)
 *
 * Completar las tres secciones marcadas:
 *   SECCION 1 — Trayectoria deseada articular  (desiredTrajectory)
 *   SECCION 2 — Ganancias del controlador      (KP, KD globales)
 *   SECCION 3 — Ley de control FL              (v, tau en compute_torque)
 *
 * Infraestructura proporcionada (NO modificar):
 *   - Inicializacion Dynamixel SDK: apertura del puerto, modos, SyncRead/SyncWrite
 *   - Lectura de estado (q, dq, corriente medida) en UN solo SyncRead del bloque
 *     contiguo 126-135 (corriente+velocidad+posicion), a loop_rate_hz
 *   - Filtro alpha-beta de posicion/velocidad articular (ab_alpha, ab_beta)
 *   - Calculo de M(q) y nle(q,dq) mediante Pinocchio (en compute_torque)
 *   - Conversion torque -> corriente via modelo OLS identificado
 *     (motorXM430W350T_params.yaml); el termino de Coulomb usa la velocidad
 *     DESEADA (senal sin ruido: sin chattering y con empuje de despegue)
 *   - Saturacion del torque a +/-tau_max antes de enviar corriente
 *   - Transicion quintica de 3 s desde la posicion inicial hasta el inicio de la trayectoria
 *   - Retorno quintico al reposo en q_inicial al completar t_run + pausa en
 *     reposo antes de cortar corriente (evita que el brazo caiga por
 *     gravedad al recibir torque cero de golpe en medio de la trayectoria)
 *   - Verificacion de limites articulares y de corriente medida (parada de emergencia)
 *   - Deteccion de overruns del lazo de control
 *   - Publicador /hw/joint_states y escritura de CSV (el retorno final y la
 *     pausa en reposo no se registran en el CSV)
 *
 * Parametros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name              [string]   "/dev/ttyUSB0"
 *   gain_scale             [double]   1.0    (escala lineal de Kp; Kd escala con sqrt)
 *   t_run                  [double]   20.0   (duracion; 0 = sin limite)
 *   log_id                 [int]      1
 *   ab_alpha               [double]   0.2    (filtro alpha-beta: ganancia de posicion)
 *   ab_beta                [double]   0.02   (filtro alpha-beta: ganancia de velocidad)
 *   friction_fc_scale      [double]   0.95   (fraccion de Fc compensada)
 *   loop_rate_hz           [double]   200.0  (frecuencia del lazo [Hz], acotada a [50,400])
 *
 * Parametros del modelo identificado (config/motorXM430W350T_params.yaml):
 *   motor_alpha            [double[4]]   ticks/N·m
 *   motor_Fv               [double[4]]   ticks/(rad/s)
 *   motor_Fc               [double[4]]   ticks
 *   motor_I_offset         [double[4]]   ticks
 *   motor_epsilon_friction [double]      rad/s
 *
 * Ejemplo de ejecucion:
 *   ros2 run open_manipulator_x_torque_control hw_fl_control_node_base \
 *     --ros-args -p gain_scale:=1.0 -p log_id:=1 -p t_run:=20.0
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
#include <pinocchio/algorithm/crba.hpp>
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

// JOINT_ZERO_TICK, ENCODER_SIGN, CURRENT_SIGN, JOINT_LOWER/UPPER,
// CURRENT_LIMIT_REGISTER, CURRENT_CMD_LIMIT, CURRENT_MEASURED_PEAK
// → cargados desde config/motorXM430W350T_params.yaml como parametros ROS 2.

static constexpr double RAMP_TIME_S   = 3.0;   // duracion de la transicion inicial [s]
static constexpr double RETURN_TIME_S = 3.0;   // duracion del retorno quintico a q_inicial
static constexpr double HOLD_TIME_S   = 0.5;   // pausa en reposo antes de cortar corriente

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Utilidades de conversion
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
// Estructura de referencia articular
// ============================================================

struct Reference { Vec4 q, dq, ddq; };

// ═══════════════════════════════════════════════════════════════════════════
//  SECCION 1 — TRAYECTORIA DESEADA ARTICULAR
//
//  Definir ref.q, ref.dq y ref.ddq como funciones del tiempo t
//
//  Parametros de entrada:
//    t   — tiempo transcurrido desde el inicio de la trayectoria [s]
//
//  Salida esperada:
//    ref.q   << q1_des(t), q2_des(t), q3_des(t), q4_des(t);
//    ref.dq  << dq1_des(t),    dq2_des(t),    dq3_des(t),    dq4_des(t);
//    ref.ddq << ddq1_des(t),   ddq2_des(t),   ddq3_des(t),   ddq4_des(t);
//
//  Nota: esta funcion se llama para t >= RAMP_TIME_S.
//        Para t < RAMP_TIME_S se aplica la transicion quintica
//        automaticamente (ver quinticTransition — no modificar).
// ═══════════════════════════════════════════════════════════════════════════

static Reference desiredTrajectory(double t)
{
  Reference ref;

  // -- completar --
  (void)t;
  const double w = 1.0;
  (void)w;

  ref.q.setZero();
  ref.dq.setZero();
  ref.ddq.setZero();
  // ────────────────

  return ref;
}

// ═══════════════════════════════════════════════════════════════════════════

// ── Transiciones quinticas (NO modificar) ───────────────────────────────────
// Polinomio quintico general: interpola (q,dq,ddq) desde el estado de borde
// (q0,v0,a0) en t=0 hasta (qf,vf,af) en t=T.
static Reference quinticBlend(double t, double T,
                              const Vec4& q0, const Vec4& v0, const Vec4& a0,
                              const Vec4& qf, const Vec4& vf, const Vec4& af)
{
  if (T <= 0.0) { Reference r; r.q = qf; r.dq = vf; r.ddq = af; return r; }

  const double T2=T*T, T3=T2*T, T4=T3*T, T5=T4*T;
  const Vec4 c0 = q0;
  const Vec4 c1 = v0;
  const Vec4 c2 = 0.5 * a0;
  const Vec4 c3 = (20.0*(qf-q0) - (8.0*vf+12.0*v0)*T - (3.0*a0-af)*T2) / (2.0*T3);
  const Vec4 c4 = (30.0*(q0-qf) + (14.0*vf+16.0*v0)*T + (3.0*a0-2.0*af)*T2) / (2.0*T4);
  const Vec4 c5 = (12.0*(qf-q0) - (6.0*vf+6.0*v0)*T - (a0-af)*T2) / (2.0*T5);

  const double t2=t*t, t3=t2*t, t4=t3*t, t5=t4*t;
  Reference ref;
  ref.q   = c0 + c1*t   + c2*t2   + c3*t3   + c4*t4   + c5*t5;
  ref.dq  = c1 + 2.0*c2*t + 3.0*c3*t2 + 4.0*c4*t3 + 5.0*c5*t4;
  ref.ddq = 2.0*c2 + 6.0*c3*t + 12.0*c4*t2 + 20.0*c5*t3;
  return ref;
}

// Rampa inicial: parte en reposo (v0=a0=0) en q0 y llega a la trayectoria
// periodica (q,dq,ddq)(T) — continuidad de posicion, velocidad y aceleracion.
static Reference quinticTransition(double t, double T, const Vec4& q0)
{
  const Reference target = desiredTrajectory(T);
  return quinticBlend(t, T, q0, Vec4::Zero(), Vec4::Zero(), target.q, target.dq, target.ddq);
}

// Retorno final: parte del estado de la trayectoria periodica en el instante
// en que termina t_run (start = desiredTrajectory(t_run)) y llega en reposo
// (vf=af=0) a qf (q_inicial) — evita el frenazo/caida por corte de torque.
static Reference quinticReturn(double t, double T, const Reference& start, const Vec4& qf)
{
  return quinticBlend(t, T, start.q, start.dq, start.ddq, qf, Vec4::Zero(), Vec4::Zero());
}

// ═══════════════════════════════════════════════════════════════════════════
//  SECCION 2 — GANANCIAS DEL CONTROLADOR
//
//  Definir KP y KD para cada articulacion: [j1, j2, j3, j4]
//    KP — ganancias proporcionales  [adim]
//    KD — ganancias derivativas     [adim]
//
//  gain_scale_ escala KP linealmente y KD con su raiz cuadrada
//
//  Sugerencia: con FL exacto, la dinamica del error por joint es
//    e'' + kd*e' + kp*e = 0   →   amortiguamiento zeta = kd/(2*sqrt(kp))
//  Mantener kd ≈ 1.4*sqrt(kp) (zeta ≈ 0.7) evita oscilaciones subamortiguadas.
//  Recordar ademas que el torque de feedback es M(q)*v: articulaciones con
//  inercia pequena (munecas) necesitan kp mayores para vencer la friccion.
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d KP = {0.0 /* kp1 */, 0.0 /* kp2 */, 0.0 /* kp3 */, 0.0 /* kp4 */};   // COMPLETAR
static const Eigen::Vector4d KD = {0.0 /* kd1 */, 0.0 /* kd2 */, 0.0 /* kd3 */, 0.0 /* kd4 */};   // COMPLETAR
// TAU_MAX disponible como tau_max_ (cargado desde motorXM430W350T_params.yaml, default 1.2 N·m)
// ═══════════════════════════════════════════════════════════════════════════

// ============================================================
// Nodo ROS 2
// ============================================================

class HWFLControlNode : public rclcpp::Node
{
public:
  explicit HWFLControlNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("hw_fl_control_node", opts),
    hw_active_(false), q_initial_captured_(false)
  {
    // ── Parametros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",  "/dev/ttyUSB0");
    this->declare_parameter<double>     ("gain_scale",  1.0);
    this->declare_parameter<double>     ("t_run",       20.0);
    this->declare_parameter<int>        ("log_id",      1);

    using dvec = std::vector<double>;
    this->declare_parameter<dvec>("motor_alpha",            dvec{208.5, 208.5, 208.5, 208.5});
    this->declare_parameter<dvec>("motor_Fv",               dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<dvec>("motor_Fc",               dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<dvec>("motor_I_offset",         dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<double>("motor_epsilon_friction", 0.05);
    this->declare_parameter<double>("friction_fc_scale",      0.95);
    this->declare_parameter<double>("ab_alpha", 0.2);
    this->declare_parameter<double>("ab_beta",  0.02);
    this->declare_parameter<double>("loop_rate_hz", 200.0);

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
    t_run_      = this->get_parameter("t_run").as_double();
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
    fc_scale_       = get_parameter("friction_fc_scale").as_double();

    ab_alpha_ = get_parameter("ab_alpha").as_double();
    ab_beta_  = get_parameter("ab_beta").as_double();
    loop_rate_hz_ = std::min(std::max(get_parameter("loop_rate_hz").as_double(), 50.0), 400.0);
    Ts_ = 1.0 / loop_rate_hz_;

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
      port_name_.c_str(), gain_scale_, t_run_, log_id);
    RCLCPP_INFO(get_logger(),
      "motor alpha=[%.1f %.1f %.1f %.1f]  Fv=[%.2f %.2f %.2f %.2f]  epsilon=%.3f",
      motor_alpha_(0), motor_alpha_(1), motor_alpha_(2), motor_alpha_(3),
      motor_Fv_(0), motor_Fv_(1), motor_Fv_(2), motor_Fv_(3), motor_epsilon_);
    RCLCPP_INFO(get_logger(),
      "Filtro alpha-beta: alpha=%.3f  beta=%.4f  |  Fc_scale=%.2f  |  lazo=%.0f Hz",
      ab_alpha_, ab_beta_, fc_scale_, loop_rate_hz_);

    // ── Pinocchio ────────────────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(get_logger(), "Pinocchio: nv=%d", model_.nv);

    // ── CSV ──────────────────────────────────────────────────────────────────
    std::filesystem::create_directories(std::string(PACKAGE_DATA_DIR) + "/lab4/real/act1");
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab4/real/act1/hw_fl_data_"
                + std::to_string(log_id) + ".csv";
    csv_.open(csv_path_);
    if (csv_.is_open()) {
      csv_ << "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
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

    // ── Timer de control (loop_rate_hz) ──────────────────────────────────────
    start_time_ = std::chrono::high_resolution_clock::now();
    const auto period = std::chrono::microseconds(
      static_cast<int64_t>(std::lround(1e6 / loop_rate_hz_)));
    timer_ = this->create_wall_timer(period, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Control activo a %.0f Hz (Ts=%.1f ms). Ctrl+C para detener.",
      loop_rate_hz_, 1e3 * Ts_);
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
          !dxl_write2(id, ADDR_CURRENT_LIMIT, current_limit_register_, "set limit") ||
          !dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE_VAL, "enable torque")) {
        port_handler_->closePort();
        return false;
      }
      RCLCPP_INFO(get_logger(), "DXL ID %d listo", static_cast<int>(id));
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
  //  Ley de control FL
  // ─────────────────────────────────────────────────────────────────────────

  Vec4 compute_torque(const Vec4& q, const Vec4& dq, const Reference& ref)
  {
    // Aplicar escala de ganancia (gain_scale_)
    const Vec4 kp = KP * gain_scale_;
    const Vec4 kd = KD * std::sqrt(std::max(gain_scale_, 0.0));

    // Dinamica M(q) y nle(q, dq) via Pinocchio
    Eigen::VectorXd q_pin   = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin  = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head(NUM_JOINTS)  = q;
    dq_pin.head(NUM_JOINTS) = dq;

    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();
    pinocchio::nonLinearEffects(model_, data_, q_pin, dq_pin);

    const Eigen::Matrix4d M4   = data_.M.topLeftCorner<NUM_JOINTS, NUM_JOINTS>();
    const Eigen::Vector4d nle4 = data_.nle.head<NUM_JOINTS>();

    // ═══════════════════════════════════════════════════════════════════════
    //  SECCION 3 — LEY DE CONTROL (Feedback Linearization)
    //
    //  Disponible:
    //    q, dq                  — posicion y velocidad articular medidas
    //    ref.q, ref.dq, ref.ddq — referencia y sus derivadas analiticas
    //    M4   (4×4)             — matriz de masa inercial  M(q)
    //    nle4 (4×1)             — efectos no lineales      nle(q, dq)
    //    kp, kd                 — ganancias escaladas
    //
    //  Ley de control:
    //    e   = ref.q  - q              (error de posicion)
    //    de  = ref.dq - dq             (error de velocidad)
    //    v   = ref.ddq + kp.*e + kd.*de  (entrada auxiliar)
    //    tau = M4 * v + nle4           (torque de control)
    //
    //  Resultado: retornar tau (Vec4)
    // ═══════════════════════════════════════════════════════════════════════

    // -- completar --
    (void)M4; (void)nle4; (void)kp; (void)kd; (void)ref;
    return Vec4::Zero();  // reemplazar con tau calculado
    // ════════════════
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Conversion torque -> corriente via modelo OLS identificado
  //    I_cmd = alpha*tau + Fv*dq + fc_scale*Fc*tanh(dq_des/epsilon) + I_offset
  //  Fv usa la velocidad medida (termino suave); Fc usa la velocidad DESEADA:
  //  senal sin ruido → sin chattering, y aporta el empuje de despegue justo
  //  cuando la referencia arranca (con dq medida ≈ 0 el tanh muere pegado).
  // ─────────────────────────────────────────────────────────────────────────

  std::array<int16_t, NUM_JOINTS> torque_to_current(const Vec4& tau, const Vec4& dq,
                                                    const Vec4& dq_des)
  {
    std::array<int16_t, NUM_JOINTS> cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double I_model = motor_alpha_(i) * tau(i)
                           + motor_Fv_(i)    * dq(i)
                           + fc_scale_ * motor_Fc_(i) * std::tanh(dq_des(i) / motor_epsilon_)
                           + motor_I_offset_(i);
      cmd[i] = clampCurrent(current_sign_(i) * encoder_sign_(i) * I_model, current_cmd_limit_[i]);
    }
    return cmd;
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
  //  Callback del timer (loop_rate_hz)
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    if (!hw_active_) return;

    const auto tick_t0 = std::chrono::steady_clock::now();
    const auto   tp = std::chrono::high_resolution_clock::now();
    const double t  = std::chrono::duration<double>(tp - start_time_).count();

    const double t_shutdown = t_run_ + RETURN_TIME_S + HOLD_TIME_S;
    if (t_run_ > 0.0 && t >= t_shutdown) {
      RCLCPP_INFO(get_logger(), "Retorno a q_inicial completado. Deteniendo (t=%.1f s).", t);
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

    // 2. Filtro alpha-beta: estimacion conjunta de posicion y velocidad
    //   prediccion: q_pred = q_hat + Ts·dq_hat,  dq_pred = dq_hat
    //   residuo:    r      = q_meas - q_pred
    //   correccion: q_hat  = q_pred + alpha·r,    dq_hat = dq_pred + (beta/Ts)·r
    if (!ab_initialized_) {
      q_hat_          = q;
      dq_hat_         = dq;
      ab_initialized_ = true;
    } else {
      const Vec4 q_pred  = q_hat_ + Ts_ * dq_hat_;
      const Vec4 dq_pred = dq_hat_;
      const Vec4 r       = q - q_pred;
      q_hat_  = q_pred  + ab_alpha_ * r;
      dq_hat_ = dq_pred + (ab_beta_ / Ts_) * r;
    }

    // 3. Captura posicion inicial (primer tick)
    if (!q_initial_captured_) {
      q_initial_         = q;
      q_initial_captured_ = true;
      RCLCPP_INFO(get_logger(), "q_inicial=[%.3f %.3f %.3f %.3f] rad",
        q(0), q(1), q(2), q(3));
    }

    // 4. Referencia (transicion quintica + trayectoria + retorno final)
    Reference ref;
    if (t < RAMP_TIME_S) {
      ref = quinticTransition(t, RAMP_TIME_S, q_initial_);
    } else if (t_run_ <= 0.0 || t < t_run_) {
      ref = desiredTrajectory(t);
    } else if (t < t_run_ + RETURN_TIME_S) {
      if (!return_logged_) {
        RCLCPP_INFO(get_logger(), "Iniciando retorno quintico a q_inicial (t=%.1fs, dur=%.1fs)",
          t, RETURN_TIME_S);
        return_logged_ = true;
      }
      ref = quinticReturn(t - t_run_, RETURN_TIME_S, desiredTrajectory(t_run_), q_initial_);
    } else {
      ref.q = q_initial_;
      ref.dq.setZero();
      ref.ddq.setZero();
    }

    // 5. Verificar limites de la referencia
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (ref.q(i) < joint_lower_(i) + 0.02 || ref.q(i) > joint_upper_(i) - 0.02) {
        emergency_stop("Referencia fuera de limites articulares");
        return;
      }
    }

    // 6. Ley de control — usa dq_hat_ para reducir chattering; el torque se
    //    satura a +/-tau_max antes de convertirlo a corriente
    const Vec4 tau_unsat = compute_torque(q, dq_hat_, ref);
    const Vec4 tau = tau_unsat.cwiseMin(tau_max_).cwiseMax(-tau_max_);
    const auto cur_cmd = torque_to_current(tau, dq_hat_, ref.dq);

    // 7. Escribir corriente
    if (!send_currents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 8. Verificar corriente medida (seguridad)
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > current_measured_peak_) {
        emergency_stop("Corriente medida insegura en J" + std::to_string(i + 1)
                       + ": " + std::to_string(cur_meas[i]) + " ticks");
        return;
      }
    }

    // 9. Publicar JointState de monitoreo
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

    // 10. CSV (excluye el retorno final a q_inicial y la pausa en reposo,
    //     asi el CSV solo cubre la rampa inicial + trayectoria periodica)
    const bool in_return_or_hold = (t_run_ > 0.0 && t >= t_run_);
    if (csv_.is_open() && !in_return_or_hold) {
      csv_ << std::fixed << std::setprecision(6) << t
           << ',' << q(0)        << ',' << q(1)        << ',' << q(2)        << ',' << q(3)
           << ',' << dq(0)       << ',' << dq(1)       << ',' << dq(2)       << ',' << dq(3)
           << ',' << dq_hat_(0)  << ',' << dq_hat_(1)  << ',' << dq_hat_(2)  << ',' << dq_hat_(3)
           << ',' << ref.q(0)    << ',' << ref.q(1)    << ',' << ref.q(2)    << ',' << ref.q(3)
           << ',' << ref.dq(0)   << ',' << ref.dq(1)   << ',' << ref.dq(2)   << ',' << ref.dq(3)
           << ',' << tau(0)      << ',' << tau(1)       << ',' << tau(2)      << ',' << tau(3)
           << ',' << cur_cmd[0]  << ',' << cur_cmd[1]   << ',' << cur_cmd[2]  << ',' << cur_cmd[3]
           << ',' << cur_meas[0] << ',' << cur_meas[1]  << ',' << cur_meas[2] << ',' << cur_meas[3]
           << '\n';
    }

    // 11. Log periodico por consola (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      RCLCPP_INFO(get_logger(),
        "t=%.2fs  q=[%.3f %.3f %.3f %.3f]  |e|=%.4f  i=[%d %d %d %d]",
        t, q(0), q(1), q(2), q(3), (q - ref.q).norm(),
        cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }

    // 12. Deteccion de overrun: si el ciclo (bus + control) no cabe en Ts,
    //     el lazo real corre mas lento de lo configurado → bajar loop_rate_hz
    //     o revisar el latency_timer del adaptador USB-serial.
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
  double gain_scale_, t_run_;
  Vec4   motor_alpha_, motor_Fv_, motor_Fc_, motor_I_offset_;
  double motor_epsilon_;

  std::array<int32_t, NUM_JOINTS> joint_zero_tick_;
  Vec4     encoder_sign_, current_sign_;
  Vec4     joint_lower_, joint_upper_;
  uint16_t current_limit_register_;
  std::array<int16_t, NUM_JOINTS> current_cmd_limit_;
  int16_t  current_measured_peak_;
  double   tau_max_;

  double fc_scale_;
  double ab_alpha_, ab_beta_;
  double loop_rate_hz_{200.0};
  double Ts_{0.005};
  Vec4   q_hat_, dq_hat_;
  bool   ab_initialized_{false};

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead>  grp_read_;
  std::unique_ptr<dynamixel::GroupSyncWrite> grp_wcur_;

  bool hw_active_;
  bool q_initial_captured_;
  bool return_logged_{false};
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

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

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
    RCLCPP_INFO(rclcpp::get_logger("hw_fl_control_node"),
      "motorXM430W350T_params auto-cargado: %s", cfg.c_str());
  }

  try {
    rclcpp::spin(std::make_shared<HWFLControlNode>(opts));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_fl_control_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
