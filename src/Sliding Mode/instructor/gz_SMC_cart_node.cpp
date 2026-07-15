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
//    tau       = M_hat(q) * qddot_cmd + Phi_hat(q,qdot) + tau_fric
//    tau_sat   = clamp(tau, -TAU_MAX, TAU_MAX)
//
//  Feedforward de friccion del URDF:
//    tau_fric = Fv·dq + Fc·tanh(dq_d/eps)
//      Fv, Fc : <dynamics damping/friction> de joint1..4, leidos del modelo
//               Pinocchio — los mismos valores que simula Gazebo con
//               damping_scale = 1.0 y friction_scale = 1.0.
//      dq_d   : velocidad articular DESEADA = J_y^# * ydot_d (senal limpia;
//               con dq medida el tanh conmutaria con el ruido cerca de cero).
//
//  Trayectoria cartesiana de referencia (serie de Fourier, w=1.0 rad/s):
//    x_d   = 0.172 + 0.032*sin(t) + 0.027*cos(2t) + 0.003*sin(3t)
//    y_d   = 0.015 + 0.136*sin(t) - 0.014*cos(2t) + 0.003*sin(3t) - 0.001*cos(4t)
//    z_d   = 0.128 - 0.008*sin(t) + 0.006*cos(2t)
//    phi_d = 0.685 + 0.250*sin(t)
//    (derivadas: analiticas exactas de la serie de posicion)
//
//  Transicion inicial: quintica generalizada de duracion T_TRANS que empalma
//  posicion, velocidad Y aceleracion con la trayectoria de referencia en t=0
//  (empalme C2: sin escalon de ydot_d/yddot_d al iniciar la fase SMC).
//
//  Lazo de control: 200 Hz (Ts = 5 ms), alineado con los nodos gz de FL.
//
//  Funciones de conmutacion (aplicadas elemento a elemento):
//    "sign"  ->  rho(s) = sign(s)
//    "sat"   ->  rho(s) = sat(s / phi)     phi: capa limite
//
//  Suscriptor : /joint_states                    (sensor_msgs/JointState)
//  Publicador : /arm_effort_controller/commands   (std_msgs/Float64MultiArray)
//
//  Parametros ROS 2:
//    test_num  [int]     1        — identificador del CSV
//    t_sim     [double]  0.0      — duracion SMC en segundos (0 = ilimitado); total = T_TRANS + t_sim
//    rho_func  [string]  "sign"   — funcion de conmutacion: "sign" | "sat"
//    phi       [double]  0.15     — capa limite [m/s o rad/s]
//
//  CSV: data/lab6/sim/act2/gz_smc_cart_<rho_func>_<test_num>.csv
//  Columnas: t, q1..q4, x,y,z,phi, x_des,y_des,z_des,phi_des,
//            xdot,ydot,zdot,phidot, xdot_des,ydot_des,zdot_des,phidot_des,
//            s1,s2,s3,s4, tau1..tau4, cond_J
//  (Sat% se calcula en MATLAB desde tau con el criterio >= 0.99*tau_max.)
//
//  Nota: usar sim_init_config.yaml con use_fixed_init: false y escalas
//        nominales mass_inertia/damping/friction = 1.0.
//        El CSV almacena unicamente datos de la fase de seguimiento SMC.
//
//  Ejemplos de uso:
//
//    ros2 run open_manipulator_x_torque_control gz_smc_cart_node
//      --ros-args -p rho_func:=sign -p test_num:=1 -p t_sim:=30.0
//
//    ros2 run open_manipulator_x_torque_control gz_smc_cart_node
//      --ros-args -p rho_func:=sat -p phi:=0.15 -p test_num:=2 -p t_sim:=30.0
//
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
// Suavizado del tanh de Coulomb del feedforward de friccion.
// Se alimenta con la velocidad DESEADA (sin ruido): eps pequeno es seguro.
static constexpr double FRIC_EPS = 0.05;  // [rad/s]
// Amortiguamiento DLS para pseudo-inversa (evita divergencia en singularidades)
// LAMBDA_DLS pequeno = solucion mas exacta; aumentar solo si kappa(J) > 100
static constexpr double LAMBDA_DLS    = 0.01;
static constexpr double LAMBDA_DLS_SQ = LAMBDA_DLS * LAMBDA_DLS;

// Frame del efector final
static constexpr char EFF_FRAME[] = "end_effector_link";

// Duracion de la transicion quintica desde y0_ hasta Y_START
static constexpr double T_TRANS = 3.0;   // [s]

// Y_START = FK(Q_DES_SMC) = FK([0, -0.45, 0.35, pi/4]) con q1=0 → y=0
// FK(q_des) = [0.1988, 0.0, 0.1348, 0.6854]
static const Eigen::Vector4d Y_START {0.1988, 0.0, 0.1348, 0.6854};

// ═══════════════════════════════════════════════════════════════════════════
//  GANANCIAS SMC CARTESIANO  [x, y, z, phi]
//
//  Criterio de sintonia (lazo discreto a 200 Hz, Ts = 5 ms):
//   · K_S en unidades de aceleracion (m/s² para x,y,z; rad/s² para phi):
//     solo debe dominar la INCERTIDUMBRE acotada, no la dinamica completa.
//     Un K_S grande produce chattering |s| ~ K_S*Ts fuera de la capa limite
//     y sat(s/phi) degenera en sign(s).
//   · Ganancia efectiva dentro de la capa: K_V + K_S/phi <= (0.2~0.3)/Ts.
//   · Canal phi: dominado por joint4 (fila [0,1,1,1] del Jacobiano). El
//     residual de Coulomb de la muneca dividido por su inercia diminuta
//     (M44 ~ 0.001 kg·m²) equivale a ~10-20 rad/s² de perturbacion; por eso
//     Lambda4/K_V4/K_S4 > que en x,y,z (e_ss ~ s_ss/Lambda). Subir Lambda
//     mas alla de ~15 vuelve a demandar aceleraciones imposibles con
//     tau_max = 1.2 (saturacion y vibracion).
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d LAMBDA_Y = {5.0, 5.0, 5.0, 12.0};   // superficie [1/s]
static const Eigen::Vector4d K_V      = {5.0, 5.0, 5.0, 10.0};   // proporcional a superficie (alcance exponencial)
static const Eigen::Vector4d K_S      = {1.5, 1.5, 1.5,  6.0};   // ganancia de conmutacion [m/s² | rad/s²]
// ═══════════════════════════════════════════════════════════════════════════

// ── Referencia cartesiana ────────────────────────────────────────────────────
struct CartRef {
  Eigen::Vector4d y;      // posicion deseada  [x, y, z, phi]
  Eigen::Vector4d ydot;   // velocidad deseada
  Eigen::Vector4d yddot;  // aceleracion deseada
};

// Trayectoria cartesiana de referencia (serie de Fourier, w=1.0 rad/s)
// Derivadas son las derivadas analiticas exactas de la serie de posicion.
static CartRef desiredTrajectory(double t)
{
  const double s1 = std::sin(t);
  const double c1 = std::cos(t);
  const double s2 = std::sin(2.0 * t);
  const double c2 = std::cos(2.0 * t);
  const double s3 = std::sin(3.0 * t);
  const double c3 = std::cos(3.0 * t);
  const double s4 = std::sin(4.0 * t);
  const double c4 = std::cos(4.0 * t);

  CartRef ref;
  ref.y <<
      0.172 + 0.032*s1 + 0.027*c2 + 0.003*s3,
      0.015 + 0.136*s1 - 0.014*c2 + 0.003*s3 - 0.001*c4,
      0.128 - 0.008*s1 + 0.006*c2,
      0.685 + 0.250*s1;

  ref.ydot <<
       0.032*c1 - 0.054*s2 + 0.009*c3,
       0.136*c1 + 0.028*s2 + 0.009*c3 + 0.004*s4,
      -0.008*c1 - 0.012*s2,
       0.250*c1;

  ref.yddot <<
      -0.032*s1 - 0.108*c2 - 0.027*s3,
      -0.136*s1 + 0.056*c2 - 0.027*s3 + 0.016*c4,
       0.008*s1 - 0.024*c2,
      -0.250*s1;

  return ref;
}

// Transicion quintica generalizada: y0 (reposo) → goal en [0, T].
// Condiciones de borde: p(0)=y0, dp(0)=0, ddp(0)=0;
//                       p(T)=goal.y, dp(T)=goal.ydot, ddp(T)=goal.yddot.
// El empalme C2 con la trayectoria de referencia evita el escalon de
// ydot_d/yddot_d al iniciar la fase SMC (la quintica clasica terminaba en
// reposo mientras la serie de Fourier arranca con ydot_d(0) != 0, y ese
// salto pateaba la superficie justo donde empieza el registro CSV).
static CartRef transitionTrajectory(double t,
                                    const Eigen::Vector4d & y0,
                                    const CartRef & goal,
                                    double T)
{
  const double tc = std::min(t, T);
  const double T2 = T * T;

  CartRef ref;
  for (int i = 0; i < NARM; ++i) {
    const double D  = goal.y[i] - y0[i];
    const double vf = goal.ydot[i];
    const double af = goal.yddot[i];

    const double a3 = ( 20.0*D -  8.0*vf*T +       af*T2) / (2.0*T*T2);
    const double a4 = (-30.0*D + 14.0*vf*T - 2.0*af*T2) / (2.0*T2*T2);
    const double a5 = ( 12.0*D -  6.0*vf*T +       af*T2) / (2.0*T2*T2*T);

    const double t2 = tc * tc;
    const double t3 = t2 * tc;

    ref.y[i]     = y0[i] +      a3*t3 +      a4*t3*tc +      a5*t3*t2;
    ref.ydot[i]  =          3.0*a3*t2 +  4.0*a4*t3    +  5.0*a5*t2*t2;
    ref.yddot[i] =          6.0*a3*tc + 12.0*a4*t2    + 20.0*a5*t3;
  }
  return ref;
}

// ── Funciones de conmutacion ─────────────────────────────────────────────────
enum class RhoFunc { SIGN, SAT };

static double rho_scalar(double s, RhoFunc func, double phi)
{
  switch (func) {
    case RhoFunc::SIGN:
      return (s > 0.0) ? 1.0 : (s < 0.0 ? -1.0 : 0.0);
    case RhoFunc::SAT:
      return std::max(-1.0, std::min(1.0, s / phi));
    default:
      return 0.0;
  }
}

static Eigen::Vector4d rho_vec(const Eigen::Vector4d & s,
                                RhoFunc func, double phi)
{
  Eigen::Vector4d r;
  for (int i = 0; i < NARM; ++i) {
    r[i] = rho_scalar(s[i], func, phi);
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
    this->declare_parameter<double>     ("phi",      0.15);

    const int         test_num = this->get_parameter("test_num").as_int();
    t_sim_                     = this->get_parameter("t_sim").as_double();
    phi_                       = this->get_parameter("phi").as_double();
    const std::string rho_str  = this->get_parameter("rho_func").as_string();

    if (rho_str == "sat") {
      rho_func_ = RhoFunc::SAT;
      rho_str_  = "sat";
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

    // Friccion articular del URDF (<dynamics damping/friction> joint1..4):
    // feedforward de viscosa + Coulomb, mismos valores que simula Gazebo
    // con damping_scale = 1.0 y friction_scale = 1.0.
    fric_damping_ = model_.damping.head<NARM>();
    fric_coulomb_ = model_.friction.head<NARM>();

    RCLCPP_INFO(this->get_logger(),
      "SMC cartesiano — rho=%s  phi=%.4f  tau_max=%.2f N·m",
      rho_str_.c_str(), phi_, TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "Lambda_y=[%.1f %.1f %.1f %.1f]  K_s=[%.1f %.1f %.1f %.1f]  K_y=[%.1f %.1f %.1f %.1f]",
      LAMBDA_Y[0], LAMBDA_Y[1], LAMBDA_Y[2], LAMBDA_Y[3],
      K_V[0],      K_V[1],      K_V[2],      K_V[3],
      K_S[0],      K_S[1],      K_S[2],      K_S[3]);
    RCLCPP_INFO(this->get_logger(),
      "lambda_DLS=%.3f  T_trans=%.1f s  Friccion URDF — "
      "Fv=[%.4f %.4f %.4f %.4f] N·m·s/rad  Fc=[%.4f %.4f %.4f %.4f] N·m",
      LAMBDA_DLS, T_TRANS,
      fric_damping_[0], fric_damping_[1], fric_damping_[2], fric_damping_[3],
      fric_coulomb_[0], fric_coulomb_[1], fric_coulomb_[2], fric_coulomb_[3]);
    RCLCPP_INFO(this->get_logger(),
      "Y_start=[%.3f %.3f %.3f %.3f]",
      Y_START[0], Y_START[1], Y_START[2], Y_START[3]);
    if (t_sim_ > 0.0) {
      RCLCPP_INFO(this->get_logger(),
        "t_sim (SMC) = %.1f s  |  total = %.1f s (T_trans=%.1f + t_sim=%.1f)",
        t_sim_, T_TRANS + t_sim_, T_TRANS, t_sim_);
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

    timer_ = this->create_wall_timer(5ms, [this]() { tick(); });
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

  // ── Tick de control a 200 Hz ──────────────────────────────────────────────
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

    // 5. Pose cartesiana actual + captura de condicion inicial
    const Eigen::Vector4d y {p_ee[0], p_ee[1], p_ee[2], phi_ee};

    if (!y0_initialized_) {
      y0_ = y;
      y0_initialized_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Pose inicial capturada: x=%.3f  y=%.3f  z=%.3f  phi=%.3f rad",
        y0_[0], y0_[1], y0_[2], y0_[3]);
    }

    // 6. Referencia segun fase. La transicion quintica empalma en C2 con la
    //    trayectoria: posicion Y_START (= FK(q_d(0))) y velocidad/aceleracion
    //    de la serie de Fourier en t=0.
    CartRef goal0 = desiredTrajectory(0.0);
    goal0.y = Y_START;
    const bool in_trans = (t_ < T_TRANS);
    const CartRef ref   = in_trans
      ? transitionTrajectory(t_, y0_, goal0, T_TRANS)
      : desiredTrajectory(t_ - T_TRANS);

    // 6b. Factorizacion DLS: A = J_y J_y^T + lambda^2 I (se reutiliza en el
    //     feedforward de friccion y en la aceleracion articular comandada)
    const Eigen::Matrix4d A =
      J4 * J4.transpose() + LAMBDA_DLS_SQ * Eigen::Matrix4d::Identity();
    const Eigen::LDLT<Eigen::Matrix4d> A_ldlt = A.ldlt();

    // 6c. Feedforward de friccion del URDF (Coulomb con la velocidad
    //     articular DESEADA, mapeada de ydot_d via la pseudo-inversa DLS)
    const Eigen::Vector4d dq_des = J4.transpose() * A_ldlt.solve(ref.ydot);
    Eigen::Vector4d tau_fric;
    for (int i = 0; i < NARM; ++i) {
      tau_fric[i] = fric_damping_[i] * dq[i]
                  + fric_coulomb_[i] * std::tanh(dq_des[i] / FRIC_EPS);
    }

    // 7. Superficie deslizante cartesiana: s_y = edot_y + Lambda_y * e_y
    const Eigen::Vector4d e_y    = y    - ref.y;
    const Eigen::Vector4d edot_y = ydot - ref.ydot;
    const Eigen::Vector4d s_y    = edot_y + LAMBDA_Y.asDiagonal() * e_y;

    // 8. Funcion de conmutacion rho(s_y)
    const Eigen::Vector4d rho = rho_vec(s_y, rho_func_, phi_);

    // 9. Aceleracion cartesiana virtual: v_y = yddot_d - Lambda_y*edot_y - K_s*s_y - K_y*rho(s_y)
    //    Ley de alcance exponencial: sdot = -K_s*s - K_y*rho(s) → convergencia mas rapida a la variedad
    const Eigen::Vector4d v_y = ref.yddot
                               - LAMBDA_Y.asDiagonal() * edot_y
                               - K_V.asDiagonal() * s_y
                               - K_S.asDiagonal() * rho;

    // 10. Aceleracion articular comandada via pseudo-inversa DLS:
    //     qddot_cmd = J_y^T (J_y J_y^T + lambda^2 I)^{-1} (v_y - Jydot*qdot)
    const Eigen::Vector4d qddot_cmd = J4.transpose() * A_ldlt.solve(v_y - jdqd);

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

    // 14. Registro CSV (solo fase SMC, sin transicion quintica)
    if (!in_trans && csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t_
           << ',' << q[0]           << ',' << q[1]           << ',' << q[2]           << ',' << q[3]
           << ',' << y[0]           << ',' << y[1]           << ',' << y[2]           << ',' << y[3]
           << ',' << ref.y[0]       << ',' << ref.y[1]       << ',' << ref.y[2]       << ',' << ref.y[3]
           << ',' << ydot[0]        << ',' << ydot[1]        << ',' << ydot[2]        << ',' << ydot[3]
           << ',' << ref.ydot[0]    << ',' << ref.ydot[1]    << ',' << ref.ydot[2]    << ',' << ref.ydot[3]
           << ',' << s_y[0]         << ',' << s_y[1]         << ',' << s_y[2]         << ',' << s_y[3]
           << ',' << tau_sat[0]     << ',' << tau_sat[1]     << ',' << tau_sat[2]     << ',' << tau_sat[3]
           << ',' << cond_J
           << '\n';
    }

    t_ += 0.005;

    if (t_sim_ > 0.0 && t_ >= T_TRANS + t_sim_) {
      RCLCPP_INFO(this->get_logger(),
        "Simulacion completada: T_trans=%.1f s + t_sim=%.1f s = %.1f s total. Deteniendo control.",
        T_TRANS, t_sim_, T_TRANS + t_sim_);
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

  Eigen::Vector4d fric_damping_;   // Fv del URDF [N·m·s/rad]
  Eigen::Vector4d fric_coulomb_;   // Fc del URDF [N·m]

  double      t_;
  double      t_sim_;
  RhoFunc     rho_func_;
  std::string rho_str_;
  double      phi_;

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
