// ============================================================================
//  gz_SMCI_cart_node.cpp
//  Control por Modo Deslizante INTEGRAL (ISMC) en espacio cartesiano
//  OpenMANIPULATOR-X — Gazebo Fortress
//
//  Salida cartesiana reducida:
//    y = [x_ee, y_ee, z_ee, phi]^T       phi = q2 + q3 + q4
//
//  ── Formulacion ISMC ──────────────────────────────────────────────────────
//
//  Estado integral:
//    xi(t) = ∫₀ᵗ e(τ) dτ,    e = y − y_d             [m·s, m·s, m·s, rad·s]
//
//  Superficie deslizante integral:
//    s    = e + Lambda * xi                              [m, m, m, rad]
//    sdot = edot + Lambda * e                           [m/s, m/s, m/s, rad/s]
//
//  Grado relativo de s respecto a tau (Input-Output Linearization):
//
//    s(t)     = e + Lambda*xi            (r=0, no depende de qddot)
//    sdot(t)  = edot + Lambda*e          (r=1, edot=J_y*qdot - ydot_d)
//    sddot(t) = eddot + Lambda*edot      (r=2, eddot=J_y*qddot + Jdot_y*qdot - yddot_d)
//               = J_y * qddot + eta_I                 <- depende de qddot -> tau
//
//    => grado relativo r = 2
//
//    donde: eta_I = Jdot_y*qdot − yddot_d + Lambda*edot
//
//  Ley de control PD + conmutacion (actua sobre sddot):
//
//    sddot_cmd = −K_D * sdot − K_P * s − K_sw * rho(s)
//
//    Despejando qddot_cmd:
//      J_y * qddot_cmd = sddot_cmd − eta_I
//      v_s = yddot_d − Lambda*edot − Jdot_y*qdot − K_D*sdot − K_P*s − K_sw*rho(s)
//      qddot_cmd = J_y^# * v_s
//      tau   = M(q)*qddot_cmd + Phi(q,qdot) + tau_fric
//      tau_sat = clamp(tau, -TAU_MAX, TAU_MAX)
//
//  Feedforward de friccion del URDF:
//    tau_fric = Fv·dq + Fc·tanh(dq_d/eps)
//      Fv, Fc : <dynamics damping/friction> de joint1..4, leidos del modelo
//               Pinocchio — los mismos valores que simula Gazebo con
//               damping_scale = 1.0 y friction_scale = 1.0.
//      dq_d   : velocidad articular DESEADA = J_y^# * ydot_d (senal limpia).
//
//  Pseudo-inversa amortiguada DLS:
//    J_y^# = J_y^T (J_y*J_y^T + lambda_DLS^2 * I)^{-1}
//
//  Manejo del estado integral xi:
//    · xi se integra SOLO en la fase de seguimiento (durante la transicion
//      quintica el error es transitorio y sesgaria el integrador).
//    · anti-windup: |xi_i| <= XI_MAX (limita la contribucion Lambda*xi a s).
//
//  ── Trayectoria (identica a gz_SMC_cart_node, serie de Fourier w=1 rad/s) ─
//    Fase transicion quintica [0, T_TRANS):  y0 → Y_START con empalme C2
//    (posicion, velocidad y aceleracion de la serie en t'=0).
//    Fase seguimiento [T_TRANS, inf), t' = t − T_TRANS:
//      x_d   = 0.172 + 0.032*sin(t') + 0.027*cos(2t') + 0.003*sin(3t')
//      y_d   = 0.015 + 0.136*sin(t') - 0.014*cos(2t') + 0.003*sin(3t') - 0.001*cos(4t')
//      z_d   = 0.128 - 0.008*sin(t') + 0.006*cos(2t')
//      phi_d = 0.685 + 0.250*sin(t')
//
//  Lazo de control: 200 Hz (Ts = 5 ms), alineado con los nodos gz de FL/SMC.
//
//  ── CSV ───────────────────────────────────────────────────────────────────
//    data/lab6/sim/act2/gz_smci_cart_<rho_func>_<test_num>.csv
//    Columnas: t, q1..q4, x,y,z,phi, x_des,y_des,z_des,phi_des,
//              xdot,ydot,zdot,phidot, xdot_des,ydot_des,zdot_des,phidot_des,
//              s1,s2,s3,s4, tau1..tau4, cond_J, xi1..xi4
//    (Solo fase de seguimiento; Sat% se calcula en MATLAB desde tau.)
//    Mismas columnas que gz_smc_cart_*.csv (+xi): plots_SMC_cart.m puede
//    leerlo seleccionando controller = 'smci'.
//
//  Nota: usar sim_init_config.yaml con use_fixed_init: false y escalas
//        nominales mass_inertia/damping/friction = 1.0.
//
//  Ejemplos de uso:
//
//    ros2 run open_manipulator_x_torque_control gz_SMCI_cart_node
//      --ros-args -p rho_func:=sign -p test_num:=1 -p t_sim:=30.0
//
//    ros2 run open_manipulator_x_torque_control gz_SMCI_cart_node
//      --ros-args -p rho_func:=sat -p phi:=0.02 -p test_num:=2 -p t_sim:=30.0
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
static constexpr double TAU_MAX = 1.2;    // [N·m]
static constexpr double DT      = 0.005;  // [s] periodo de control 200 Hz
// Suavizado del tanh de Coulomb del feedforward de friccion.
// Se alimenta con la velocidad DESEADA (sin ruido): eps pequeno es seguro.
static constexpr double FRIC_EPS = 0.05;  // [rad/s]

// DLS: valor pequeno = solucion mas exacta; aumentar solo si kappa(J) > 100
static constexpr double LAMBDA_DLS    = 0.01;
static constexpr double LAMBDA_DLS_SQ = LAMBDA_DLS * LAMBDA_DLS;

// Duracion de la transicion quintica
static constexpr double T_TRANS = 3.0;   // [s]

// Anti-windup del estado integral: |xi_i| <= XI_MAX
// (limita la contribucion Lambda*xi a la superficie a Lambda*XI_MAX)
static constexpr double XI_MAX = 0.05;   // [m·s | rad·s]

// Frame del efector final (definido en el URDF)
static constexpr char EFF_FRAME[] = "end_effector_link";

// Y_START = FK(q_d(0)) = FK([0, -0.45, 0.35, pi/4]) — inicio de la
// trayectoria de referencia (identico a gz_SMC_cart_node)
static const Eigen::Vector4d Y_START {0.1988, 0.0, 0.1348, 0.6854};

// ═══════════════════════════════════════════════════════════════════════════
//  GANANCIAS ISMC CARTESIANO  [x, y, z, phi]
//
//  Lambda: polo de la superficie  s = e + Lambda*xi           [1/s]
//  K_P:    ganancia proporcional sobre s en sddot             [1/s²]
//  K_D:    ganancia derivativa  sobre sdot en sddot           [1/s]
//  K_sw:   ganancia de conmutacion rho(s)                     [m/s² o rad/s²]
//
//  Dinamica de s (parte lineal): sddot + K_D*sdot + K_P*s = 0
//    omega_n = sqrt(K_P)    [rad/s]
//    zeta    = K_D / (2*omega_n)
//
//  Criterio de sintonia (lazo discreto a 200 Hz, Ts = 5 ms):
//   · omega_n*Ts <= ~0.15 y K_D <= (0.2~0.3)/Ts.
//   · Lambda < omega_n/2 (separar el polo integral de la dinamica de s).
//   · K_sw solo domina la incertidumbre acotada; con sat, la pendiente
//     adicional K_sw/phi [1/s²] se suma a K_P — mantener K_sw/phi ~ K_P.
//   · Canal phi mas rigido: el residual de Coulomb de la muneca (M44
//     diminuta) equivale a ~10-20 rad/s² de perturbacion; el integrador
//     absorbe su componente lenta.
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d LAMBDA_I = {  2.5,   2.5,   2.5,   4.0};
static const Eigen::Vector4d K_P      = { 36.0,  36.0,  36.0, 100.0};  // omega_n = {6,6,6,10} rad/s
static const Eigen::Vector4d K_D      = { 12.0,  12.0,  12.0,  20.0};  // zeta = 1 (critico)
static const Eigen::Vector4d K_sw     = {  1.0,   1.0,   1.0,   4.0};
// ═══════════════════════════════════════════════════════════════════════════

// ── Referencia cartesiana ────────────────────────────────────────────────────
struct CartRef {
  Eigen::Vector4d y;
  Eigen::Vector4d ydot;
  Eigen::Vector4d yddot;
};

// Trayectoria activa (identica a gz_SMC_cart_node: serie de Fourier,
// w = 1.0 rad/s, derivadas analiticas exactas), t' = t - T_TRANS
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
// El empalme C2 con la trayectoria evita el escalon de ydot_d/yddot_d al
// iniciar la fase de seguimiento (la serie arranca con ydot_d(0) != 0).
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
class SMCICartSimNode : public rclcpp::Node
{
public:
  SMCICartSimNode()
  : Node("gz_smci_cart_node"),
    t_(0.0),
    y0_initialized_(false),
    xi_(Eigen::Vector4d::Zero())
  {
    // ── Parametros ────────────────────────────────────────────────────────────
    this->declare_parameter<int>        ("test_num", 1);
    this->declare_parameter<double>     ("t_sim",    0.0);
    this->declare_parameter<std::string>("rho_func", "sign");
    this->declare_parameter<double>     ("phi",      0.02);

    const int         test_num = this->get_parameter("test_num").as_int();
    t_sim_                     = this->get_parameter("t_sim").as_double();
    phi_                       = this->get_parameter("phi").as_double();
    const std::string rho_str  = this->get_parameter("rho_func").as_string();

    if (rho_str == "sat") {
      rho_func_ = RhoFunc::SAT;  rho_str_ = "sat";
    } else {
      rho_func_ = RhoFunc::SIGN; rho_str_ = "sign";
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
      "SMCI cart — rho=%s  phi=%.4f  tau_max=%.2f N·m",
      rho_str_.c_str(), phi_, TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "Lambda=[%.1f %.1f %.1f %.1f]  K_P=[%.1f %.1f %.1f %.1f]  "
      "K_D=[%.1f %.1f %.1f %.1f]  K_sw=[%.1f %.1f %.1f %.1f]",
      LAMBDA_I[0], LAMBDA_I[1], LAMBDA_I[2], LAMBDA_I[3],
      K_P[0],      K_P[1],      K_P[2],      K_P[3],
      K_D[0],      K_D[1],      K_D[2],      K_D[3],
      K_sw[0],     K_sw[1],     K_sw[2],     K_sw[3]);
    RCLCPP_INFO(this->get_logger(),
      "lambda_DLS=%.3f  T_trans=%.1f s  XI_max=%.2f  Friccion URDF — "
      "Fv=[%.4f %.4f %.4f %.4f] N·m·s/rad  Fc=[%.4f %.4f %.4f %.4f] N·m",
      LAMBDA_DLS, T_TRANS, XI_MAX,
      fric_damping_[0], fric_damping_[1], fric_damping_[2], fric_damping_[3],
      fric_coulomb_[0], fric_coulomb_[1], fric_coulomb_[2], fric_coulomb_[3]);
    if (t_sim_ > 0.0) {
      RCLCPP_INFO(this->get_logger(),
        "t_sim (seguimiento) = %.1f s  |  total = %.1f s (T_trans=%.1f + t_sim=%.1f)",
        t_sim_, T_TRANS + t_sim_, T_TRANS, t_sim_);
    } else {
      RCLCPP_INFO(this->get_logger(), "t_sim = ilimitado  (T_trans=%.1f s)", T_TRANS);
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

  ~SMCICartSimNode()
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

    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab6/sim/act2/gz_smci_cart_"
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
         << "cond_J,"
         << "xi1,xi2,xi3,xi4\n";
    RCLCPP_INFO(this->get_logger(), "CSV: %s", csv_path_.c_str());
  }

  // ── Lectura de estados articulares (por nombre, orden independiente) ───────
  void read_js(Eigen::Vector4d & q, Eigen::Vector4d & dq)
  {
    static const std::array<std::string, NARM> names = {
      "joint1", "joint2", "joint3", "joint4"
    };
    q.setZero(); dq.setZero();
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

    // 2. Jacobiano reducido 4×4: filas {vx, vy, vz, dphi/dq}
    Eigen::MatrixXd J6 = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data_, q_pin, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J6);

    const Eigen::Vector3d p_ee = data_.oMf[frame_id_].translation();
    const double phi_ee = q[1] + q[2] + q[3];

    Eigen::Matrix4d J4;
    J4.row(0) = J6.row(0).head<NARM>();
    J4.row(1) = J6.row(1).head<NARM>();
    J4.row(2) = J6.row(2).head<NARM>();
    J4.row(3) << 0.0, 1.0, 1.0, 1.0;  // dphi/dq (constante analitico)

    const Eigen::Vector4d ydot = J4 * dq;

    // 3. Termino bias Jdot*qdot (via aceleracion clasica con qddot=0)
    pinocchio::forwardKinematics(
      model_, data_, q_pin, dq_pin, Eigen::VectorXd::Zero(model_.nv));
    const pinocchio::Motion bias =
      pinocchio::getFrameClassicalAcceleration(
        model_, data_, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::Vector4d jdqd;
    jdqd << bias.linear()[0],
            bias.linear()[1],
            bias.linear()[2],
            0.0;  // fila [0,1,1,1] es constante → Jdot_phi*qdot = 0

    // 4. Dinamica nominal: M(q) y Phi(q,dq)
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
        "Pose inicial: x=%.3f  y=%.3f  z=%.3f  phi=%.3f rad",
        y0_[0], y0_[1], y0_[2], y0_[3]);
    }

    // 6. Referencia segun fase. La transicion quintica empalma en C2 con la
    //    trayectoria: posicion Y_START (= FK(q_d(0))) y velocidad/aceleracion
    //    de la serie de Fourier en t'=0.
    CartRef goal0 = desiredTrajectory(0.0);
    goal0.y = Y_START;
    const bool in_trans = (t_ < T_TRANS);
    const CartRef ref = in_trans
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

    // 7. Errores cartesianos
    const Eigen::Vector4d e_y    = y    - ref.y;
    const Eigen::Vector4d edot_y = ydot - ref.ydot;

    // 8. Estado integral: xi += e * dt  (Euler forward).
    //    Solo durante el seguimiento (el transitorio de la transicion
    //    sesgaria el integrador) y con anti-windup |xi_i| <= XI_MAX.
    if (!in_trans) {
      xi_ += e_y * DT;
      xi_ = xi_.cwiseMax(-XI_MAX).cwiseMin(XI_MAX);
    }

    // 9. Superficie deslizante integral:
    //      s    = e + Lambda * xi
    //      sdot = edot + Lambda * e
    const Eigen::Vector4d s_y    = e_y    + LAMBDA_I.asDiagonal() * xi_;
    const Eigen::Vector4d sdot_y = edot_y + LAMBDA_I.asDiagonal() * e_y;

    // 10. Funcion de conmutacion rho(s)
    const Eigen::Vector4d rho = rho_vec(s_y, rho_func_, phi_);

    // 11. Aceleracion cartesiana virtual (IOL de s con grado relativo 2):
    //
    //   sddot = J_y * qddot + eta_I,    eta_I = jdqd - yddot_d + Lambda*edot
    //   sddot_cmd = -K_D*sdot - K_P*s - K_sw*rho(s)
    //
    //   J_y * qddot_cmd = sddot_cmd - eta_I
    //   => v_s = yddot_d - Lambda*edot - jdqd - K_D*sdot - K_P*s - K_sw*rho(s)
    const Eigen::Vector4d v_s = ref.yddot
        - LAMBDA_I.asDiagonal() * edot_y
        - jdqd
        - K_D.asDiagonal()  * sdot_y
        - K_P.asDiagonal()  * s_y
        - K_sw.asDiagonal() * rho;

    // 12. Aceleracion articular comandada via pseudo-inversa DLS
    const Eigen::Vector4d qddot_cmd = J4.transpose() * A_ldlt.solve(v_s);

    // 13. Torque nominal y saturado
    const Eigen::Vector4d tau     = M4 * qddot_cmd + nle4 + tau_fric;
    const Eigen::Vector4d tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // 14. Condicionamiento del Jacobiano
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(J4);
    const double sigma_min = svd.singularValues()(NARM - 1);
    const double cond_J    = (sigma_min > 1e-10) ?
      svd.singularValues()(0) / sigma_min : 1e10;

    // 15. Publicar torques
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    const char * phase = in_trans ? "TRANS" : "TRAY ";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[%s] t=%.2fs  |s|=%.4f  |e|=%.4f m  |xi|=%.4f  kJ=%.1f  "
      "tau=[%.3f %.3f %.3f %.3f] N·m",
      phase, t_, s_y.norm(), e_y.norm(), xi_.norm(), cond_J,
      tau_sat[0], tau_sat[1], tau_sat[2], tau_sat[3]);

    // 16. Registro CSV (solo fase de seguimiento, sin transicion quintica)
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
           << ',' << xi_[0]         << ',' << xi_[1]         << ',' << xi_[2]         << ',' << xi_[3]
           << '\n';
    }

    t_ += DT;

    if (t_sim_ > 0.0 && t_ >= T_TRANS + t_sim_) {
      RCLCPP_INFO(this->get_logger(),
        "Simulacion completada: T_trans=%.1f s + t_sim=%.1f s = %.1f s total. Deteniendo.",
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
  Eigen::Vector4d xi_;           // estado integral: ∫e dt (solo fase de seguimiento)

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
    rclcpp::spin(std::make_shared<SMCICartSimNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gz_smci_cart_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
