// ============================================================================
//  gz_SMC_cart_node.cpp
//  Control por Modo Deslizante en espacio cartesiano — OpenMANIPULATOR-X (Gazebo)
//
//  Salida cartesiana reducida:
//    y = [x_ee, y_ee, z_ee, phi]^T
//    phi = q2 + q3 + q4  (orientacion reducida del gripper)
//
//  Cinematica diferencial:
//    ydot  = J_y(q) * qdot
//    yddot = J_y(q) * qddot + Jydot(q,qdot) * qdot
//
//  Pseudo-inversa amortiguada DLS:
//    J_y^# = J_y^T (J_y J_y^T + lambda_J^2 * I)^{-1}
//
//  Superficie deslizante cartesiana:
//    e_y    = y - y_d,    edot_y = ydot - ydot_d
//    s_y    = edot_y + Lambda_y * e_y
//
//  Ley de control SMC cartesiana (ley de alcance exponencial + conmutacion):
//    v_y       = yddot_d - Lambda_y * edot_y - K_s * s_y - K_y * rho(s_y)
//    qddot_cmd = J_y^# * (v_y - Jydot * qdot)
//    tau       = M_hat(q) * qddot_cmd + Phi_hat(q,qdot) + tau_fric_hat(qdot)
//    tau_sat   = clamp(tau, -TAU_MAX, TAU_MAX)
//
//  Trayectoria cartesiana de referencia: FK analitica de la referencia articular
//    (misma que gz_SMC_art_node, omega = 1.0 rad/s)
//    q_d(t') = [(pi/4)*sin(t'), -0.45+0.5*sin(t'), 0.35-0.5*sin(t'), pi/4+0.25*sin(t')]
//    y_d(t') = FK(q_d(t'))              — open_manx_fkin.m, simplif. q2+q3=-0.10=cte
//    ydot_d  = J(q_d)*qdot_d            — Jacobiano analitico
//    yddot_d = J(q_d)*qddot_d + Jdot*qdot_d — segunda derivada exacta
//
//  Fases de control:
//    [0, T_PD)               — Fase 0: PD articular lleva el brazo a Q_DES_SMC
//                               tau = G(q) + Kp*(q_des - q) - Kd*qdot
//                               q_des = [0, -0.45, 0.35, pi/4] = q_d(0) articular
//    [T_PD, T_PD+T_TRANS)    — Fase 1: transicion quintica y0 → Y_START=FK(q_d(0))
//    [T_PD+T_TRANS, inf)     — Fase 2: trayectoria cartesiana activa (t' = t - T_PD - T_TRANS)
//
//  Funciones de conmutacion (aplicadas elemento a elemento):
//    "sign"  ->  rho(s) = sign(s)
//    "sat"   ->  rho(s) = sat(s / phi)     phi: capa limite
//    "tanh"  ->  rho(s) = tanh(alpha * s)
//
//  Suscriptor : /joint_states                    (sensor_msgs/JointState)
//  Publicador : /arm_effort_controller/commands   (std_msgs/Float64MultiArray)
//
//  Parametros ROS 2:
//    test_num  [int]     1        — identificador del CSV
//    t_sim     [double]  0.0      — duracion en segundos (0 = ilimitado)
//    rho_func  [string]  "sign"   — funcion de conmutacion: "sign"|"sat"|"tanh"
//    phi       [double]  0.02     — capa limite [m o rad]
//    alpha     [double]  100.0    — pendiente para tanh(alpha*s)
//
//  CSV: data/lab6/sim/act2/gz_smc_cart_<rho_func>_<test_num>.csv
//  Columnas: t, q1..q4, x,y,z,phi, x_des,y_des,z_des,phi_des,
//            xdot,ydot,zdot,phidot, xdot_des,ydot_des,zdot_des,phidot_des,
//            s1,s2,s3,s4, tau1..tau4, sat1..sat4, cond_J
//
//  Nota: usar sim_init_config.yaml con use_fixed_init: false.
//        La Fase 0 lleva el brazo a Q_DES_SMC = q_d(0) articular en T_PD segundos.
//        t_sim debe incluir T_PD + T_TRANS (ej: t_sim:=48.0 = 8 PD + 10 TRANS + 30 TRAY).
//
//  Ejemplos de uso:
//
//    ros2 run open_manipulator_x_torque_control gz_smc_cart_node
//      --ros-args -p rho_func:=sign -p test_num:=1 -p t_sim:=48.0
//
//    ros2 run open_manipulator_x_torque_control gz_smc_cart_node
//      --ros-args -p rho_func:=sat -p phi:=0.25 -p test_num:=2 -p t_sim:=48.0
//
//    ros2 run open_manipulator_x_torque_control gz_smc_cart_node
//      --ros-args -p rho_func:=tanh -p alpha:=100.0 -p test_num:=3 -p t_sim:=48.0
// ============================================================================

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
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

using namespace std::chrono_literals;

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif

static constexpr double PI      = M_PI;
static constexpr int    NARM    = 4;
static constexpr double TAU_MAX = 1.2;    // [N·m] limite de torque por articulacion
static constexpr double B_FRIC  = 0.001;  // [N·m·s/rad] friccion viscosa nominal
// Amortiguamiento DLS para pseudo-inversa (evita divergencia en singularidades)
// LAMBDA_DLS pequeno = solucion mas exacta; aumentar solo si kappa(J) > 100
static constexpr double LAMBDA_DLS    = 0.01;
static constexpr double LAMBDA_DLS_SQ = LAMBDA_DLS * LAMBDA_DLS;

// ─── Fase 0: PD articular ────────────────────────────────────────────────────
// q_des = q_d(0) de la trayectoria articular: [0, -0.45, 0.35, pi/4]
// FK(q_des) = [0.1988, 0.0, 0.1348, 0.6854] → Y_START
static constexpr double T_PD = 5.0;   // [s] duracion de la fase PD articular
static const Eigen::Vector4d Q_DES_SMC    = {0.0, -0.45, 0.35, PI/4.0};
static const Eigen::Vector4d KP_PD_JOINT  = {30.0, 50.0, 50.0, 50.0};   // [N·m/rad]
static const Eigen::Vector4d KD_PD_JOINT  = { 0.0,  0.0,  0.0,  0.0};   // [N·m·s/rad]
// ─────────────────────────────────────────────────────────────────────────────

// Duracion de la transicion quintica desde y0_ hasta Y_START
static constexpr double T_TRANS = 0.0;   // [s]

// Frame del efector final
static constexpr char EFF_FRAME[] = "end_effector_link";

// Punto de inicio de la trayectoria: y_d(t'=0)
// Y_START = FK(Q_DES_SMC) = FK([0, -0.45, 0.35, pi/4]) con q1=0 → y=0
static const Eigen::Vector4d Y_START {0.1988, 0.0, 0.1348, 0.6854};

// ═══════════════════════════════════════════════════════════════════════════
//  GANANCIAS SMC CARTESIANO  [x, y, z, phi]
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d LAMBDA_Y = {5.0,  5.0,  5.0,  50.0};   // superficie [1/s]
static const Eigen::Vector4d K_S      = {5.0,  5.0,  5.0,  20.0};   // proporcional a superficie (alcance exponencial)
static const Eigen::Vector4d K_Y      = {20.0, 20.0, 20.0, 40.0};   // ganancia de conmutacion
// ═══════════════════════════════════════════════════════════════════════════

// ── Referencia cartesiana ────────────────────────────────────────────────────
struct CartRef {
  Eigen::Vector4d y;      // posicion deseada  [x, y, z, phi]
  Eigen::Vector4d ydot;   // velocidad deseada
  Eigen::Vector4d yddot;  // aceleracion deseada
};

// Trayectoria cartesiana: FK analitica de la referencia articular (w=1.0 rad/s)
//   q_d = [(pi/4)*sin(t), -0.45+0.5*sin(t), 0.35-0.5*sin(t), pi/4+0.25*sin(t)]
// Simplificacion exacta: q2+q3 = -0.10 = cte → s23,c23 constantes; phidot = q4dot
static CartRef desiredTrajectory(double t)
{
  // Constantes de cinematica (open_manx_fkin.m)
  static constexpr double x_base = 0.012;
  static constexpr double z_base = 0.0765;   // 0.017 + 0.0595
  static constexpr double x23_   = 0.024;
  static constexpr double z23_   = 0.128;
  static constexpr double l34_   = 0.124;
  static constexpr double l4e_   = 0.126;
  // q2+q3 = -0.10 = cte
  static const double c23 = std::cos(-0.10);
  static const double s23 = std::sin(-0.10);

  // Trayectoria articular
  const double w   = 1.0;
  const double sw  = std::sin(w * t);
  const double cw  = std::cos(w * t);

  const double q1      = (PI/4.0) * sw;
  const double q2      = -0.45 + 0.5 * sw;
  const double phi_r   = -0.10 + PI/4.0 + 0.25 * sw;   // q2+q3+q4

  const double q1d     = (PI/4.0) * w * cw;
  const double q2d     =  0.5 * w * cw;
  const double phi_rd  =  0.25 * w * cw;                // phidot = q4dot (q2d+q3d=0)

  const double q1dd    = -(PI/4.0) * w * w * sw;
  const double q2dd    = -0.5 * w * w * sw;
  const double phi_rdd = -0.25 * w * w * sw;

  // Trigonometria
  const double c1  = std::cos(q1);
  const double s1  = std::sin(q1);
  const double cq2 = std::cos(q2);
  const double sq2 = std::sin(q2);
  const double cp  = std::cos(phi_r);
  const double sp  = std::sin(phi_r);

  // ── FK: posicion ──────────────────────────────────────────────────────
  const double r     = x23_*cq2 + z23_*sq2 + l34_*c23 + l4e_*cp;
  const double z_val = z_base + (-x23_*sq2 + z23_*cq2) - l34_*s23 - l4e_*sp;

  // ── Velocidades: ydot = J(q)*qdot  (q2d+q3d=0 → rdot solo usa q2d, phi_rd) ──
  const double rdot = (-x23_*sq2 + z23_*cq2)*q2d - l4e_*sp*phi_rd;
  const double zdot = (-x23_*cq2 - z23_*sq2)*q2d - l4e_*cp*phi_rd;

  // ── Aceleraciones: yddot = J*qddot + Jdot*qdot ───────────────────────
  const double rddot = (-x23_*cq2 - z23_*sq2)*q2d*q2d
                     + (-x23_*sq2 + z23_*cq2)*q2dd
                     - l4e_*cp*phi_rd*phi_rd
                     - l4e_*sp*phi_rdd;
  const double zddot = ( x23_*sq2 - z23_*cq2)*q2d*q2d
                     + (-x23_*cq2 - z23_*sq2)*q2dd
                     + l4e_*sp*phi_rd*phi_rd
                     - l4e_*cp*phi_rdd;

  CartRef ref;
  ref.y    << x_base + r*c1,                    r*s1,
              z_val,                             phi_r;
  ref.ydot << rdot*c1 - r*s1*q1d,              rdot*s1 + r*c1*q1d,
              zdot,                              phi_rd;
  ref.yddot << rddot*c1 - 2.0*rdot*s1*q1d - r*c1*q1d*q1d - r*s1*q1dd,
               rddot*s1 + 2.0*rdot*c1*q1d - r*s1*q1d*q1d + r*c1*q1dd,
               zddot,                            phi_rdd;
  return ref;
}

// Transicion quintica: y0 → y_goal en [0, T] con velocidad y aceleracion nulas en extremos
static CartRef transitionTrajectory(double t,
                                    const Eigen::Vector4d & y0,
                                    const Eigen::Vector4d & y_goal,
                                    double T)
{
  const double tau  = std::min(1.0, t / T);
  const double tau2 = tau  * tau;
  const double tau3 = tau2 * tau;
  const double tau4 = tau3 * tau;
  const double tau5 = tau4 * tau;

  const double s   =  10*tau3 - 15*tau4 +  6*tau5;
  const double sd  = (30*tau2 - 60*tau3 + 30*tau4) / T;
  const double sdd = (60*tau  - 180*tau2 + 120*tau3) / (T * T);

  const Eigen::Vector4d delta = y_goal - y0;

  CartRef ref;
  ref.y     = y0 + s   * delta;
  ref.ydot  =      sd  * delta;
  ref.yddot =      sdd * delta;
  return ref;
}

// ── Funciones de conmutacion ─────────────────────────────────────────────────
enum class RhoFunc { SIGN, SAT, TANH };

static double rho_scalar(double s, RhoFunc func, double phi, double alpha)
{
  switch (func) {
    case RhoFunc::SIGN:
      return (s > 0.0) ? 1.0 : (s < 0.0 ? -1.0 : 0.0);
    case RhoFunc::SAT:
      return std::max(-1.0, std::min(1.0, s / phi));
    case RhoFunc::TANH:
      return std::tanh(alpha * s);
    default:
      return 0.0;
  }
}

static Eigen::Vector4d rho_vec(const Eigen::Vector4d & s,
                                RhoFunc func, double phi, double alpha)
{
  Eigen::Vector4d r;
  for (int i = 0; i < NARM; ++i) {
    r[i] = rho_scalar(s[i], func, phi, alpha);
  }
  return r;
}

// ── Nodo principal ────────────────────────────────────────────────────────────
class SMCCartSimNode : public rclcpp::Node
{
public:
  SMCCartSimNode()
  : Node("gz_smc_cart_node"), t_(0.0), y0_initialized_(false)
  {
    // ── Parametros ────────────────────────────────────────────────────────────
    this->declare_parameter<int>        ("test_num", 1);
    this->declare_parameter<double>     ("t_sim",    0.0);
    this->declare_parameter<std::string>("rho_func", "sign");
    this->declare_parameter<double>     ("phi",      0.02);
    this->declare_parameter<double>     ("alpha",    100.0);

    const int         test_num = this->get_parameter("test_num").as_int();
    t_sim_                     = this->get_parameter("t_sim").as_double();
    phi_                       = this->get_parameter("phi").as_double();
    alpha_                     = this->get_parameter("alpha").as_double();
    const std::string rho_str  = this->get_parameter("rho_func").as_string();

    if (rho_str == "sat") {
      rho_func_ = RhoFunc::SAT;
      rho_str_  = "sat";
    } else if (rho_str == "tanh") {
      rho_func_ = RhoFunc::TANH;
      rho_str_  = "tanh";
    } else {
      rho_func_ = RhoFunc::SIGN;
      rho_str_  = "sign";
      if (rho_str != "sign") {
        RCLCPP_WARN(this->get_logger(),
          "rho_func '%s' desconocida — usando 'sign'", rho_str.c_str());
      }
    }

    // ── Modelo Pinocchio ──────────────────────────────────────────────────────
    const std::string urdf_path =
      std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf_path, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio buildModel: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    if (!model_.existFrame(EFF_FRAME)) {
      throw std::runtime_error(std::string("Frame no encontrado: ") + EFF_FRAME);
    }
    frame_id_ = model_.getFrameId(EFF_FRAME);

    RCLCPP_INFO(this->get_logger(),
      "SMC cartesiano — rho=%s  phi=%.4f  alpha=%.1f  tau_max=%.2f N·m",
      rho_str_.c_str(), phi_, alpha_, TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "Lambda_y=[%.1f %.1f %.1f %.1f]  K_s=[%.1f %.1f %.1f %.1f]  K_y=[%.1f %.1f %.1f %.1f]",
      LAMBDA_Y[0], LAMBDA_Y[1], LAMBDA_Y[2], LAMBDA_Y[3],
      K_S[0],      K_S[1],      K_S[2],      K_S[3],
      K_Y[0],      K_Y[1],      K_Y[2],      K_Y[3]);
    RCLCPP_INFO(this->get_logger(),
      "lambda_DLS=%.3f  B_fric=%.4f N·m·s/rad  T_trans=%.1f s",
      LAMBDA_DLS, B_FRIC, T_TRANS);
    RCLCPP_INFO(this->get_logger(),
      "Y_start=[%.3f %.3f %.3f %.3f]",
      Y_START[0], Y_START[1], Y_START[2], Y_START[3]);
    if (t_sim_ > 0.0) {
      RCLCPP_INFO(this->get_logger(), "t_sim = %.1f s", t_sim_);
    } else {
      RCLCPP_INFO(this->get_logger(), "t_sim = ilimitado");
    }

    open_csv(test_num);

    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_effort_controller/commands", 10);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_js_ = msg;
      });

    timer_ = this->create_wall_timer(10ms, [this]() { tick(); });
  }

  ~SMCCartSimNode()
  {
    if (csv_.is_open()) {
      csv_.close();
      RCLCPP_INFO(this->get_logger(), "CSV cerrado: %s", csv_path_.c_str());
    }
  }

private:
  // ── CSV ───────────────────────────────────────────────────────────────────
  void open_csv(int test_num)
  {
    std::filesystem::create_directories(
      std::string(PACKAGE_DATA_DIR) + "/lab6/sim/act2");

    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab6/sim/act2/gz_smc_cart_"
                + rho_str_ + "_" + std::to_string(test_num) + ".csv";
    csv_.open(csv_path_);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo crear: %s", csv_path_.c_str());
      return;
    }
    csv_ << "t,"
         << "q1,q2,q3,q4,"
         << "x,y,z,phi,"
         << "x_des,y_des,z_des,phi_des,"
         << "xdot,ydot,zdot,phidot,"
         << "xdot_des,ydot_des,zdot_des,phidot_des,"
         << "s1,s2,s3,s4,"
         << "tau1,tau2,tau3,tau4,"
         << "sat1,sat2,sat3,sat4,"
         << "cond_J\n";
    RCLCPP_INFO(this->get_logger(), "CSV: %s", csv_path_.c_str());
  }

  // ── Lectura de estados articulares (por nombre, orden independiente) ───────
  void read_js(Eigen::Vector4d & q, Eigen::Vector4d & dq)
  {
    static const std::array<std::string, NARM> names = {
      "joint1", "joint2", "joint3", "joint4"
    };
    q.setZero();
    dq.setZero();
    if (!last_js_) { return; }
    const auto & js = *last_js_;
    for (int j = 0; j < NARM; ++j) {
      for (std::size_t i = 0; i < js.name.size(); ++i) {
        if (js.name[i] == names[j]) {
          if (i < js.position.size()) { q[j]  = js.position[i]; }
          if (i < js.velocity.size()) { dq[j] = js.velocity[i]; }
          break;
        }
      }
    }
  }

  // ── Tick de control a 100 Hz ──────────────────────────────────────────────
  void tick()
  {
    if (!last_js_) { return; }

    // 1. Leer estados articulares
    Eigen::Vector4d q, dq;
    read_js(q, dq);

    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>()  = q;
    dq_pin.head<NARM>() = dq;

    // ── FASE 0: PD articular hacia Q_DES_SMC ─────────────────────────────────
    if (t_ < T_PD) {
      const Eigen::VectorXd zero_v = Eigen::VectorXd::Zero(model_.nv);
      const Eigen::Vector4d tau_grav =
        pinocchio::rnea(model_, data_, q_pin, zero_v, zero_v).head<NARM>();
      const Eigen::Vector4d e_pd = Q_DES_SMC - q;
      Eigen::Vector4d tau = tau_grav
                          + KP_PD_JOINT.asDiagonal() * e_pd
                          - KD_PD_JOINT.asDiagonal() * dq;
      tau = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

      std_msgs::msg::Float64MultiArray cmd;
      cmd.data.assign(tau.data(), tau.data() + NARM);
      torque_pub_->publish(cmd);

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[PD   ] t=%.2fs  |e_q|=%.4f rad  tau=[%.3f %.3f %.3f %.3f] N·m",
        t_, e_pd.norm(), tau[0], tau[1], tau[2], tau[3]);

      t_ += 0.01;
      return;
    }
    // ─────────────────────────────────────────────────────────────────────────

    // 2. Cinematica directa + Jacobiano reducido 4x4
    //    Filas: {vx, vy, vz, dphi/dq}  Columnas: {j1..j4}
    Eigen::MatrixXd J6 = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data_, q_pin, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J6);

    const Eigen::Vector3d p_ee = data_.oMf[frame_id_].translation();
    const double phi_ee = q[1] + q[2] + q[3];  // orientacion reducida analitica

    Eigen::Matrix4d J4;
    J4.row(0) = J6.row(0).head<NARM>();
    J4.row(1) = J6.row(1).head<NARM>();
    J4.row(2) = J6.row(2).head<NARM>();
    J4.row(3) << 0.0, 1.0, 1.0, 1.0;  // dphi/dq = [0,1,1,1] (constante analitico)

    // Velocidad cartesiana actual: ydot = J4 * qdot
    const Eigen::Vector4d ydot = J4 * dq;

    // 3. Termino de bias: Jdot*qdot (via aceleracion clasica con qddot=0)
    pinocchio::forwardKinematics(
      model_, data_, q_pin, dq_pin, Eigen::VectorXd::Zero(model_.nv));
    const pinocchio::Motion bias =
      pinocchio::getFrameClassicalAcceleration(
        model_, data_, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::Vector4d jdqd;
    jdqd << bias.linear()[0],
            bias.linear()[1],
            bias.linear()[2],
            0.0;  // Jdot_phi * qdot = 0 (fila [0,1,1,1] es constante)

    // 4. Dinamica nominal: M(q) y Phi(q,dq) = C*dq + g
    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();
    pinocchio::nonLinearEffects(model_, data_, q_pin, dq_pin);
    const Eigen::Matrix4d M4   = data_.M.topLeftCorner<NARM, NARM>();
    const Eigen::Vector4d nle4 = data_.nle.head<NARM>();
    const Eigen::Vector4d tau_fric = B_FRIC * dq;

    // 5. Pose cartesiana actual + captura de condicion inicial
    const Eigen::Vector4d y {p_ee[0], p_ee[1], p_ee[2], phi_ee};

    if (!y0_initialized_) {
      y0_ = y;
      y0_initialized_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Pose inicial capturada: x=%.3f  y=%.3f  z=%.3f  phi=%.3f rad",
        y0_[0], y0_[1], y0_[2], y0_[3]);
    }

    // 6. Referencia segun fase (t_smc = tiempo desde fin de Fase 0)
    const double t_smc  = t_ - T_PD;
    const bool in_trans = (t_smc < T_TRANS);
    const CartRef ref   = in_trans
      ? transitionTrajectory(t_smc, y0_, Y_START, T_TRANS)
      : desiredTrajectory(t_smc - T_TRANS);

    // 7. Superficie deslizante cartesiana: s_y = edot_y + Lambda_y * e_y
    const Eigen::Vector4d e_y    = y    - ref.y;
    const Eigen::Vector4d edot_y = ydot - ref.ydot;
    const Eigen::Vector4d s_y    = edot_y + LAMBDA_Y.asDiagonal() * e_y;

    // 8. Funcion de conmutacion rho(s_y)
    const Eigen::Vector4d rho = rho_vec(s_y, rho_func_, phi_, alpha_);

    // 9. Aceleracion cartesiana virtual: v_y = yddot_d - Lambda_y*edot_y - K_s*s_y - K_y*rho(s_y)
    //    Ley de alcance exponencial: sdot = -K_s*s - K_y*rho(s) → convergencia mas rapida a la variedad
    const Eigen::Vector4d v_y = ref.yddot
                               - LAMBDA_Y.asDiagonal() * edot_y
                               - K_S.asDiagonal() * s_y
                               - K_Y.asDiagonal() * rho;

    // 10. Aceleracion articular comandada via pseudo-inversa DLS:
    //     qddot_cmd = J_y^T (J_y J_y^T + lambda^2 I)^{-1} (v_y - Jydot*qdot)
    const Eigen::Matrix4d A =
      J4 * J4.transpose() + LAMBDA_DLS_SQ * Eigen::Matrix4d::Identity();
    const Eigen::Vector4d qddot_cmd = J4.transpose() * A.ldlt().solve(v_y - jdqd);

    // 11. Torque nominal y saturado
    const Eigen::Vector4d tau     = M4 * qddot_cmd + nle4 + tau_fric;
    const Eigen::Vector4d tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // 12. Condicionamiento del Jacobiano (para monitoreo de singularidades)
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(J4);
    const double sigma_min = svd.singularValues()(NARM - 1);
    const double cond_J    = (sigma_min > 1e-10) ?
      svd.singularValues()(0) / sigma_min : 1e10;

    // 13. Publicar torques
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    const char * phase = in_trans ? "TRANS" : "TRAY ";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[%s] t=%.2fs  |s|=%.4f  |e|=%.4f m  kJ=%.1f  tau=[%.3f %.3f %.3f %.3f] N·m",
      phase, t_smc, s_y.norm(), e_y.norm(), cond_J,
      tau_sat[0], tau_sat[1], tau_sat[2], tau_sat[3]);

    // 14. Registro CSV
    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t_
           << ',' << q[0]           << ',' << q[1]           << ',' << q[2]           << ',' << q[3]
           << ',' << y[0]           << ',' << y[1]           << ',' << y[2]           << ',' << y[3]
           << ',' << ref.y[0]       << ',' << ref.y[1]       << ',' << ref.y[2]       << ',' << ref.y[3]
           << ',' << ydot[0]        << ',' << ydot[1]        << ',' << ydot[2]        << ',' << ydot[3]
           << ',' << ref.ydot[0]    << ',' << ref.ydot[1]    << ',' << ref.ydot[2]    << ',' << ref.ydot[3]
           << ',' << s_y[0]         << ',' << s_y[1]         << ',' << s_y[2]         << ',' << s_y[3]
           << ',' << tau_sat[0]     << ',' << tau_sat[1]     << ',' << tau_sat[2]     << ',' << tau_sat[3]
           << ',' << (std::abs(tau[0]) > TAU_MAX ? 1 : 0)
           << ',' << (std::abs(tau[1]) > TAU_MAX ? 1 : 0)
           << ',' << (std::abs(tau[2]) > TAU_MAX ? 1 : 0)
           << ',' << (std::abs(tau[3]) > TAU_MAX ? 1 : 0)
           << ',' << cond_J
           << '\n';
    }

    t_ += 0.01;

    if (t_sim_ > 0.0 && t_ >= t_sim_) {
      RCLCPP_INFO(this->get_logger(),
        "Simulacion completada (%.1f s). Deteniendo control.", t_sim_);
      std_msgs::msg::Float64MultiArray zero;
      zero.data.assign(NARM, 0.0);
      torque_pub_->publish(zero);
      if (csv_.is_open()) {
        csv_.close();
        RCLCPP_INFO(this->get_logger(), "CSV cerrado: %s", csv_path_.c_str());
      }
      timer_->cancel();
    }
  }

  // ── Miembros ───────────────────────────────────────────────────────────────
  pinocchio::Model      model_;
  pinocchio::Data       data_;
  pinocchio::FrameIndex frame_id_;

  double      t_;
  double      t_sim_;
  RhoFunc     rho_func_;
  std::string rho_str_;
  double      phi_;
  double      alpha_;

  Eigen::Vector4d y0_;
  bool            y0_initialized_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState::SharedPtr last_js_;

  std::ofstream csv_;
  std::string   csv_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SMCCartSimNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gz_smc_cart_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
