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
//  Ley de control SMC cartesiana:
//    v_y       = yddot_d - Lambda_y * edot_y - K_y * rho(s_y)
//    qddot_cmd = J_y^# * (v_y - Jydot * qdot)
//    tau       = M_hat(q) * qddot_cmd + Phi_hat(q,qdot) + tau_fric_hat(qdot)
//    tau_sat   = clamp(tau, -TAU_MAX, TAU_MAX)
//
//  Trayectoria cartesiana de referencia (omega = 0.5 rad/s, Guia Lab 6 Sec. 6.2):
//    y_d    = [0.20 + 0.03*sin(w*t'), 0.03*cos(w*t'), 0.18 + 0.015*sin(2*w*t'), 0.60]
//    ydot_d = [0.03*w*cos(w*t'), -0.03*w*sin(w*t'), 0.03*w*cos(2*w*t'), 0]
//    yddot_d= [-0.03*w^2*sin(w*t'), -0.03*w^2*cos(w*t'), -0.06*w^2*sin(2*w*t'), 0]
//
//  Fases de referencia:
//    [0, T_TRANS)   — transicion quintica desde pose inicial hasta Y_START
//    [T_TRANS, inf) — trayectoria cartesiana activa (t' = t - T_TRANS)
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
//  Nota: usar sim_init_config.yaml con use_fixed_init: true
//        y q_init: [0.0, -0.5, 0.3, 0.785] (phi_init≈0.585, cercano a Y_START).
//
//  Ejemplos de uso:
//
//    ros2 run open_manipulator_x_torque_control gz_smc_cart_node
//      --ros-args -p rho_func:=sign -p test_num:=1 -p t_sim:=30.0
//
//    ros2 run open_manipulator_x_torque_control gz_smc_cart_node
//      --ros-args -p rho_func:=sat -p phi:=0.02 -p test_num:=2 -p t_sim:=30.0
//
//    ros2 run open_manipulator_x_torque_control gz_smc_cart_node
//      --ros-args -p rho_func:=tanh -p alpha:=100.0 -p test_num:=3 -p t_sim:=30.0
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

// Duracion de la transicion quintica: debe ser suficiente para cubrir la
// distancia desde la pose inicial hasta Y_START (tipicamente 4-5 s)
static constexpr double T_TRANS = 4.0;    // [s]

// Frame del efector final
static constexpr char EFF_FRAME[] = "end_effector_link";

// Punto de inicio de la trayectoria: y_d(t'=0)
// Verificar con FK(q_init) del sim_init_config.yaml que coincida con este valor
static const Eigen::Vector4d Y_START {0.20, 0.03, 0.18, 0.60};

// ═══════════════════════════════════════════════════════════════════════════
//  GANANCIAS SMC CARTESIANO  [x, y, z, phi]
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d LAMBDA_Y = {5.0,  5.0,  5.0,  50.0};   // superficie [1/s]
static const Eigen::Vector4d K_Y      = {20.0, 20.0, 20.0, 100.0};  // ganancia de conmutacion
// ═══════════════════════════════════════════════════════════════════════════

// ── Referencia cartesiana ────────────────────────────────────────────────────
struct CartRef {
  Eigen::Vector4d y;      // posicion deseada  [x, y, z, phi]
  Eigen::Vector4d ydot;   // velocidad deseada
  Eigen::Vector4d yddot;  // aceleracion deseada
};

// Trayectoria activa (Guia Lab 6 Sec. 6.2), parametro t' = t - T_TRANS
static CartRef desiredTrajectory(double t)
{
  const double w = 0.5;  // [rad/s]
  CartRef ref;

  ref.y <<
      0.20 + 0.03 * std::sin(w * t),
      0.03 * std::cos(w * t),
      0.18 + 0.015 * std::sin(2.0 * w * t),
      0.40 + 0.02 * std::sin(2.0 * w * t),

  ref.ydot <<
       0.03 * w * std::cos(w * t),
      -0.03 * w * std::sin(w * t),
       0.03 * w * std::cos(2.0 * w * t),
       0.04 * w * std::cos(2.0 * w * t);

  ref.yddot <<
      -0.03 * w * w * std::sin(w * t),
      -0.03 * w * w * std::cos(w * t),
      -0.06 * w * w * std::sin(2.0 * w * t),
      -0.08 * w * w * std::sin(2.0 * w * t);

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
      "Lambda_y=[%.1f %.1f %.1f %.1f]  K_y=[%.1f %.1f %.1f %.1f]",
      LAMBDA_Y[0], LAMBDA_Y[1], LAMBDA_Y[2], LAMBDA_Y[3],
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

    // 6. Referencia segun fase
    const bool in_trans = (t_ < T_TRANS);
    const CartRef ref = in_trans
      ? transitionTrajectory(t_, y0_, Y_START, T_TRANS)
      : desiredTrajectory(t_ - T_TRANS);

    // 7. Superficie deslizante cartesiana: s_y = edot_y + Lambda_y * e_y
    const Eigen::Vector4d e_y    = y    - ref.y;
    const Eigen::Vector4d edot_y = ydot - ref.ydot;
    const Eigen::Vector4d s_y    = edot_y + LAMBDA_Y.asDiagonal() * e_y;

    // 8. Funcion de conmutacion rho(s_y)
    const Eigen::Vector4d rho = rho_vec(s_y, rho_func_, phi_, alpha_);

    // 9. Aceleracion cartesiana virtual: v_y = yddot_d - Lambda_y*edot_y - K_y*rho(s_y)
    const Eigen::Vector4d v_y = ref.yddot
                               - LAMBDA_Y.asDiagonal() * edot_y
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
      phase, t_, s_y.norm(), e_y.norm(), cond_J,
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
