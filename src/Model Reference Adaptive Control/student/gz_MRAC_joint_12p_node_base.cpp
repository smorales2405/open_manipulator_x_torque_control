// ============================================================================
//  gz_MRAC_joint_12p_node_base.cpp
//  Control Adaptativo (MRAC / Slotine-Li) articular — 12 PARAMETROS
//  OpenMANIPULATOR-X (Gazebo) — campana sim-to-real
//  [Archivo base para estudiantes — Lab 7]
//
//  Parametros adaptados (12):
//    a_hat = [ alpha1..alpha3 , dm, dmcx, dmcy, dmcz , Fv1 , Fc1..Fc4 ]^T
//
//    alpha_k (k=1..3): escala de inercia del cuerpo movido por jointk
//              (linkk+1) — masa + CoM + tensor completo, nominal = 1.0.
//    theta_load = [dm, dmcx, dmcy, dmcz]: EXCESO inercial del ultimo cuerpo
//              (link5 + lo que sujete el gripper), prior = 0. Una carga
//              rigida en el efector equivale exactamente a un delta en los
//              parametros inerciales del link5 (columnas [m, m·cx, m·cy,
//              m·cz] de su bloque del regresor).
//    Fv1     : friccion viscosa de joint1 (J2..J4 tienen Fv identificado = 0
//              y no se adaptan: un Fv_hat > 0 espurio con dq medida es
//              realimentacion positiva de velocidad).
//    Fc_j    : friccion de Coulomb por junta.
//
//  Ley de control (forma de desviacion respecto al prior, realimentacion
//  PREMULTIPLICADA por M(q) — las inercias reflejadas difieren 10-100x entre
//  ejes y un K_D crudo en N·m·s/rad no puede servir a todos):
//    s    = e_dq + Lambda*e_q       dq_r = dq_d - Lambda*e_q
//    ddq_r= ddq_d - Lambda*e_dq
//    tau  = tau_rigido_SlotineLi(q,dq,dqr,ddqr) + Fv0.*dq + Fc0.*tanh(dq_d/eps)
//           + Y*(a_hat - a_prior) - M(q)*( K_V.*s + K_S.*sat(s/phi) )
//    tau_sat = clamp(tau, -TAU_MAX, TAU_MAX)
//
//  Ley de adaptacion (proyeccion + sigma-modification, POR BLOQUE):
//    a_hat_dot = -Gamma .* ( Y^T*s + sigma .* (a_hat - a_prior) )
//    a_hat     = clamp(a_hat, A_MIN, A_MAX)
//  ANTI-WINDUP: la adaptacion se congela si |tau| comandado supera TAU_MAX
//  (el torque aplicado no es el de la ley) y fuera de la fase RUN.
//
//  Fases de arranque (infraestructura): HOMING (quintica T_HOME s desde la
//  pose medida hasta reposo en q_d(0)) -> SETTLE (hasta max|e|<SETTLE_TOL)
//  -> RUN (multiseno + adaptacion; el CSV y t_sim cuentan desde aqui).
//
//  RELOJ: use_sim_time=true, timer sobre /clock de Gazebo (robusto a RTF<1).
//
//  Suscriptor : /joint_states                    (sensor_msgs/JointState)
//  Publicador : /arm_effort_controller/commands   (std_msgs/Float64MultiArray)
//
//  Parametros ROS 2 (--ros-args -p nombre:=valor):
//    test_num        [int]     1     — identificador del CSV generado
//    t_sim           [double]  0.0   — duracion en segundos SIMULADOS (0 = ilimitado)
//    adaptive        [bool]    true  — true: adapta | false: a_hat fijo en a_prior
//    friction_prior  [bool]    true  — true: prior = friccion identificada
//                                      false: prior de friccion en cero
//
//  CSV: data/lab7/sim/mrac12p/gz_mrac_joint_12p_<modo>_<test_num>.csv
//  Columnas: t, q1..q4, dq1..dq4, q1_des..q4_des, dq1_des..dq4_des,
//            s1..s4, tau1..tau4, sat1..sat4,
//            a1_hat..a3_hat, dm_hat, dmcx_hat, dmcy_hat, dmcz_hat,
//            fv1_hat, fc1_hat..fc4_hat
//  (Graficar con plots_MRAC_joint_12p.m, mode='sim'.)
//
//  Ejemplos de uso:
//    ros2 run open_manipulator_x_torque_control gz_mrac_joint_12p_node_base --ros-args -p adaptive:=true -p test_num:=1 -p t_sim:=30.0
//    ros2 run open_manipulator_x_torque_control gz_mrac_joint_12p_node_base --ros-args -p adaptive:=false -p test_num:=2 -p t_sim:=30.0
//
//  ──────────────────────────────────────────────────────────────────────────
//  SECCIONES A COMPLETAR:
//    [1] Trayectoria articular de referencia   →  desiredTrajectory()
//    [2] Ganancias y parametros MRAC           →  LAMBDA_Q, K_V, K_S, PHI_BL,
//                                                 GAMMA, SIGMA_LEAK, A_MIN/A_MAX
//    [3] Friccion nominal identificada         →  FV_NOM, FC_NOM
//    [4] Ley de control y adaptacion           →  tick()
// ============================================================================

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
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/regressor.hpp>

using namespace std::chrono_literals;

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif

static constexpr double PI      = M_PI;
static constexpr int    NARM    = 4;       // articulaciones controladas
static constexpr int    NP      = 12;      // parametros: [alpha1..3, theta_load(4), Fv1, Fc1..4]
static constexpr double TAU_MAX = 1.2;     // [N·m] limite de torque (no modificar)
static constexpr double DT      = 0.005;   // [s] periodo de control, 200 Hz (no modificar)
static constexpr double T_HOME  = 5.0;     // [s] homing quintico inicial (no modificar)
static constexpr double T_RAMP  = 2.0;     // [s] envolvente 0->1 del multiseno (no modificar)
static constexpr double SETTLE_TOL  = 0.08;  // [rad] max|e| para dar por asentado (no modificar)
static constexpr double SETTLE_TMAX = 5.0;   // [s] timeout del asentamiento (no modificar)

// [rad/s] suavizado tanh del Coulomb (se alimenta con la velocidad DESEADA
// dq_d, senal limpia — no modificar)
static constexpr double EPS_FRICTION = 0.05;

using Vec4  = Eigen::Vector4d;
using Vec12 = Eigen::Matrix<double, NP, 1>;
using Vec10 = Eigen::Matrix<double, 10, 1>;
using MatY  = Eigen::Matrix<double, NARM, NP>;

// Indices de bloque dentro de a_hat (no modificar):
static constexpr int IDX_ALPHA = 0;   // [0..2]  alpha1..alpha3
static constexpr int IDX_LOAD  = 3;   // [3..6]  dm, dmcx, dmcy, dmcz (exceso del link5)
static constexpr int IDX_FV1   = 7;   // [7]     Fv1
static constexpr int IDX_FC    = 8;   // [8..11] Fc1..Fc4

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 3] Friccion nominal identificada — COMPLETAR
//
//  Valores por junta en DOMINIO DE TORQUE, derivados de la identificacion
//  del motor (config/motorXM430W350T_params.yaml):
//    Fv = motor_Fv / motor_alpha   [N·m·s/rad]
//    Fc = motor_Fc / motor_alpha   [N·m]
//  Son los mismos valores de <dynamics damping/friction> del Xacro a escala
//  1.0. FV_NOM completa se usa como feedforward FIJO; como parametro
//  ADAPTADO solo entra Fv1 (J2..J4 tienen Fv identificado = 0).
// ─────────────────────────────────────────────────────────────────────────────
static const Vec4 FV_NOM = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
static const Vec4 FC_NOM = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 2] GANANCIAS Y PARAMETROS MRAC — COMPLETAR
//  Indice articular: [joint1, joint2, joint3, joint4]
//  Indice parametro: [alpha1..3, dm, dmcx, dmcy, dmcz, Fv1, Fc1..4]
//
//  La realimentacion va PREMULTIPLICADA por M(q): LAMBDA_Q/K_V/K_S son
//  ACELERACIONES, con el mismo efecto en los 4 ejes.
//  Rangos recomendados (lazo discreto a 200 Hz):
//    LAMBDA_Q : 5 – 15   [1/s]    (joint4 tolera mas: su error lo domina la
//                                  friccion residual y su inercia es diminuta)
//    K_V      : 5 – 12   [1/s]
//    K_S      : 1 – 8    [rad/s²] (capa limite; K_S grande degenera en sign())
//    PHI_BL   : 0.1 – 0.3 [rad/s]
//    Criterio: ganancia efectiva dentro de la capa K_V + K_S/PHI_BL <= (0.2~0.3)/Ts.
//
//  GAMMA (tasas de adaptacion) POR BLOQUE — sugerencias:
//    alpha    : pequena (las masas se pesaron con balanza)
//    theta_load: la columna dm genera ~2.5 N·m/kg de gravedad y las d(m·c)
//               ~9.8 N·m/(kg·m): escalar Gamma por sensibilidad para tiempos
//               de convergencia parejos. OJO: columnas m y m·cx son casi
//               colineales — un gradiente caliente caza tope a tope.
//    friccion : mayor (lo peor identificado)
//
//  SIGMA_LEAK (fuga hacia el prior, [1/s]) POR BLOQUE: acota la deriva por
//  dinamica no modelada. En theta_load usar fuga DEBIL (su prior es cero y
//  una fuga fuerte pelea contra el valor verdadero con carga real).
//
//  A_MIN/A_MAX (proyeccion a la region fisica admisible): alpha estrecho
//  alrededor de 1; theta_load segun la carga maxima esperada (ej. 200 g a
//  17.5 cm); friccion >= 0 con margen sobre el nominal.
// ═══════════════════════════════════════════════════════════════════════════
static const Vec4 LAMBDA_Q = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
static const Vec4 K_V      = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
static const Vec4 K_S      = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
static constexpr double PHI_BL = 0.0;                                    // COMPLETAR

static const Vec12 GAMMA =
  (Vec12() << 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0,   0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR

static const Vec12 SIGMA_LEAK =
  (Vec12() << 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0,   0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR

static const Vec12 A_MIN =
  (Vec12() << 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0,   0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
static const Vec12 A_MAX =
  (Vec12() << 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0,   0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR
// ═══════════════════════════════════════════════════════════════════════════

// ── Estructura de referencia articular ───────────────────────────────────────
struct Reference {
  Vec4 q, dq, ddq;
};

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 1] Trayectoria articular de referencia — COMPLETAR
//
//  MULTISENO con envolvente de arranque (excitacion persistente con
//  inversiones frecuentes de velocidad, necesarias para separar Fv de Fc):
//    qi_d(t) = Ci + sg(t) * [ A1i*sin(W1i*t) + A2i*sin(W2i*t) ]
//  La envolvente quintica sg(t): 0->1 en T_RAMP ya esta implementada (no
//  modificar): garantiza q_d(0) = Ci en REPOSO (empalme con el asentamiento).
//
//  DISENO OBLIGATORIO antes de correr en Gazebo (verificar con Pinocchio o
//  MATLAB): limites articulares con holgura, altura del efector z_EE > 0 en
//  todo t (una trayectoria que "penetra" la base causa colisiones), y
//  torque de feedforward con margen respecto a TAU_MAX. Usar frecuencias
//  distintas por junta y no conmensuradas entre si.
// ─────────────────────────────────────────────────────────────────────────────
static Reference desiredTrajectory(double t)
{
  // COMPLETAR: centros, amplitudes y frecuencias del multiseno
  static const Vec4 C  = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR: centros [rad]
  static const Vec4 A1 = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR: amplitud 1 [rad]
  static const Vec4 W1 = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR: frecuencia 1 [rad/s]
  static const Vec4 A2 = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR: amplitud 2 [rad]
  static const Vec4 W2 = (Vec4() << 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR: frecuencia 2 [rad/s]
  (void)A1; (void)W1; (void)A2; (void)W2;

  // Envolvente quintica: sg = 10x^3 - 15x^4 + 6x^5, x = t/T_RAMP (no modificar)
  double sg = 1.0, dsg = 0.0, ddsg = 0.0;
  if (t < T_RAMP) {
    const double x = t / T_RAMP;
    sg   = ((6.0 * x - 15.0) * x + 10.0) * x * x * x;
    dsg  = ((30.0 * x - 60.0) * x + 30.0) * x * x / T_RAMP;
    ddsg = ((120.0 * x - 180.0) * x + 60.0) * x / (T_RAMP * T_RAMP);
  }
  (void)sg; (void)dsg; (void)ddsg;

  Reference ref;
  for (int i = 0; i < NARM; ++i) {
    // COMPLETAR: parte oscilatoria y sus derivadas
    //   osc   =  A1*sin(W1*t) + A2*sin(W2*t)
    //   dosc  =  A1*W1*cos(W1*t) + A2*W2*cos(W2*t)
    //   ddosc = -A1*W1^2*sin(W1*t) - A2*W2^2*sin(W2*t)
    const double osc   = 0.0;   // COMPLETAR
    const double dosc  = 0.0;   // COMPLETAR
    const double ddosc = 0.0;   // COMPLETAR

    // COMPLETAR: composicion con la envolvente (regla del producto):
    //   q   = C + sg*osc
    //   dq  = dsg*osc + sg*dosc
    //   ddq = ddsg*osc + 2*dsg*dosc + sg*ddosc
    ref.q[i]   = C[i];   // COMPLETAR
    ref.dq[i]  = 0.0;    // COMPLETAR
    ref.ddq[i] = 0.0;    // COMPLETAR
    (void)osc; (void)dosc; (void)ddosc;
  }
  return ref;
}

// ── Homing quintico: de la postura medida (q0,v0, acc 0) en t=0 hasta el ────
//   REPOSO en los centros del multiseno (q_d(0), 0, 0) en t=T_HOME.
//   (Infraestructura, no modificar)
static Reference homingTrajectory(double t, const Vec4 & q0, const Vec4 & v0)
{
  const Reference end = desiredTrajectory(0.0);
  const double T = T_HOME;
  Reference ref;
  for (int i = 0; i < NARM; ++i) {
    const double h  = end.q[i] - q0[i];
    const double vf = end.dq[i];
    const double af = end.ddq[i];
    const double c0 = q0[i];
    const double c1 = v0[i];
    const double c3 = ( 20.0*h - (8.0*vf + 12.0*v0[i])*T + af*T*T) / (2.0*T*T*T);
    const double c4 = (-30.0*h + (14.0*vf + 16.0*v0[i])*T - 2.0*af*T*T) / (2.0*T*T*T*T);
    const double c5 = ( 12.0*h -  6.0*(vf + v0[i])*T + af*T*T) / (2.0*T*T*T*T*T);
    ref.q[i]   = c0 + c1*t + c3*t*t*t + c4*t*t*t*t + c5*t*t*t*t*t;
    ref.dq[i]  = c1 + 3.0*c3*t*t + 4.0*c4*t*t*t + 5.0*c5*t*t*t*t;
    ref.ddq[i] = 6.0*c3*t + 12.0*c4*t*t + 20.0*c5*t*t*t;
  }
  return ref;
}

// ── Termino Slotine-Li rigido:  M(q)*ddq_r + C(q,dq)*dq_r + g(q) via RNEA ────
//   (Infraestructura, no modificar.) Identidades (rnea(q,v,a) = M a + C v + g):
//     g(q)         = rnea(q, 0, 0)
//     M(q)*ddq_r   = rnea(q, 0, ddq_r) - g
//     C(q,dq)*dq_r = 1/2 [ rnea(q, dq+dq_r, 0) - rnea(q, dq, 0) - rnea(q, dqr, 0) + g ]
static Eigen::VectorXd slotineLiTorque(
    const pinocchio::Model & model, pinocchio::Data & data,
    const Eigen::VectorXd & q,   const Eigen::VectorXd & dq,
    const Eigen::VectorXd & dqr, const Eigen::VectorXd & ddqr)
{
  const Eigen::VectorXd zero  = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd g     = pinocchio::rnea(model, data, q, zero,      zero);
  const Eigen::VectorXd Mddqr = pinocchio::rnea(model, data, q, zero,      ddqr);
  const Eigen::VectorXd r_sum = pinocchio::rnea(model, data, q, dq + dqr,  zero);
  const Eigen::VectorXd r_dq  = pinocchio::rnea(model, data, q, dq,        zero);
  const Eigen::VectorXd r_dqr = pinocchio::rnea(model, data, q, dqr,       zero);
  const Eigen::VectorXd Cqr   = 0.5 * (r_sum - r_dq - r_dqr + g);
  return Mddqr + Cqr;
}

// ── Regresor Slotine-Li: identidad de polarizacion sobre el regresor ────────
//   (Infraestructura, no modificar.) R(q,v,a)*pi = M a + C v + g para
//   cualquier vector de parametros dinamicos pi (10 por cuerpo).
static Eigen::MatrixXd slotineLiRegressor(
    const pinocchio::Model & model, pinocchio::Data & data,
    const Eigen::VectorXd & q,   const Eigen::VectorXd & dq,
    const Eigen::VectorXd & dqr, const Eigen::VectorXd & ddqr)
{
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(model.nv);
  const Eigen::MatrixXd R_g     = pinocchio::computeJointTorqueRegressor(model, data, q, zero,     zero);
  const Eigen::MatrixXd R_Mddqr = pinocchio::computeJointTorqueRegressor(model, data, q, zero,     ddqr);
  const Eigen::MatrixXd R_sum   = pinocchio::computeJointTorqueRegressor(model, data, q, dq + dqr, zero);
  const Eigen::MatrixXd R_dq    = pinocchio::computeJointTorqueRegressor(model, data, q, dq,       zero);
  const Eigen::MatrixXd R_dqr   = pinocchio::computeJointTorqueRegressor(model, data, q, dqr,      zero);
  return R_Mddqr + 0.5 * (R_sum - R_dq - R_dqr + R_g);
}

// ── Fases de arranque ─────────────────────────────────────────────────────────
enum class Phase { HOMING, SETTLE, RUN };

// ── Nodo principal ────────────────────────────────────────────────────────────
class MRACJoint12pSimNode : public rclcpp::Node
{
public:
  MRACJoint12pSimNode()
  : Node("gz_mrac_joint_12p_node"), t_(0.0)
  {
    // Reloj de simulacion: el timer avanza con /clock de Gazebo (no modificar)
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // ── Parametros ────────────────────────────────────────────────────────────
    this->declare_parameter<int>   ("test_num",       1);
    this->declare_parameter<double>("t_sim",          0.0);
    this->declare_parameter<bool>  ("adaptive",       true);
    this->declare_parameter<bool>  ("friction_prior", true);

    const int test_num = this->get_parameter("test_num").as_int();
    t_sim_             = this->get_parameter("t_sim").as_double();
    adaptive_          = this->get_parameter("adaptive").as_bool();
    friction_prior_    = this->get_parameter("friction_prior").as_bool();

    mode_str_ = adaptive_ ? "adaptive" : "fixed";
    if (!friction_prior_) { mode_str_ += "_noprior"; }

    // Prior de parametros (= a_hat(0) y referencia de la forma de desviacion).
    // fv_prior_ es el feedforward viscoso FIJO de las 4 juntas; como parametro
    // adaptado solo entra Fv1. theta_load parte de cero. (No modificar.)
    fv_prior_ = friction_prior_ ? FV_NOM : Vec4::Zero().eval();
    a_prior_.setZero();
    a_prior_.segment<3>(IDX_ALPHA).setOnes();
    a_prior_[IDX_FV1]              = fv_prior_[0];
    a_prior_.segment<NARM>(IDX_FC) = friction_prior_ ? FC_NOM : Vec4::Zero().eval();
    a_hat_ = a_prior_;

    // ── Modelo Pinocchio (brazo nominal, escala 1.0) ────────────────────────────
    const std::string urdf_path =
      std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf_path, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio buildModel: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    for (int i = 0; i < NARM; ++i) {
      const std::string jname = "joint" + std::to_string(i + 1);
      jid_[i]    = model_.getJointId(jname);
      pi_nom_[i] = model_.inertias[jid_[i]].toDynamicParameters();
    }

    RCLCPP_INFO(this->get_logger(),
      "MRAC articular 12p [BASE] — modo=%s  friction_prior=%s  tau_max=%.2f N·m",
      mode_str_.c_str(), friction_prior_ ? "identificado" : "cero", TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "Lambda=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]  phi=%.2f",
      LAMBDA_Q[0], LAMBDA_Q[1], LAMBDA_Q[2], LAMBDA_Q[3],
      K_V[0],      K_V[1],      K_V[2],      K_V[3],
      K_S[0],      K_S[1],      K_S[2],      K_S[3],  PHI_BL);
    if (t_sim_ > 0.0) {
      RCLCPP_INFO(this->get_logger(),
        "t_sim = %.1f s (contados desde el inicio del multiseno, tras homing+asentamiento)",
        t_sim_);
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

    timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(DT),
      [this]() { tick(); });
  }

  ~MRACJoint12pSimNode()
  {
    if (csv_.is_open()) {
      csv_.close();
      RCLCPP_INFO(this->get_logger(), "CSV cerrado: %s", csv_path_.c_str());
    }
  }

private:
  // ── CSV (infraestructura, no modificar) ────────────────────────────────────
  void open_csv(int test_num)
  {
    std::filesystem::create_directories(
      std::string(PACKAGE_DATA_DIR) + "/lab7/sim/mrac12p");

    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab7/sim/mrac12p/gz_mrac_joint_12p_"
                + mode_str_ + "_" + std::to_string(test_num) + ".csv";
    csv_.open(csv_path_);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo crear: %s", csv_path_.c_str());
      return;
    }
    csv_ << "t,"
         << "q1,q2,q3,q4,"
         << "dq1,dq2,dq3,dq4,"
         << "q1_des,q2_des,q3_des,q4_des,"
         << "dq1_des,dq2_des,dq3_des,dq4_des,"
         << "s1,s2,s3,s4,"
         << "tau1,tau2,tau3,tau4,"
         << "sat1,sat2,sat3,sat4,"
         << "a1_hat,a2_hat,a3_hat,"
         << "dm_hat,dmcx_hat,dmcy_hat,dmcz_hat,"
         << "fv1_hat,fc1_hat,fc2_hat,fc3_hat,fc4_hat\n";
    RCLCPP_INFO(this->get_logger(), "CSV: %s", csv_path_.c_str());
  }

  // ── Lectura de estados articulares (infraestructura, no modificar) ─────────
  void read_js(Vec4 & q, Vec4 & dq)
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

  // ── Tick de control a 200 Hz (tiempo simulado) ────────────────────────────
  void tick()
  {
    if (!last_js_) { return; }

    Vec4 q, dq;
    read_js(q, dq);

    // ── Maquina de fases: HOMING -> SETTLE -> RUN (no modificar) ────────────
    if (!home_init_) {
      q_home0_  = q;
      dq_home0_ = dq;
      home_init_ = true;
      const Reference r0 = desiredTrajectory(0.0);
      RCLCPP_INFO(this->get_logger(),
        "Homing (%.1f s): q0=[%.3f %.3f %.3f %.3f] -> q_d(0)=[%.2f %.2f %.2f %.2f]",
        T_HOME, q[0], q[1], q[2], q[3], r0.q[0], r0.q[1], r0.q[2], r0.q[3]);
    }

    Reference ref;
    if (phase_ == Phase::HOMING) {
      if (t_ < T_HOME) {
        ref = homingTrajectory(t_, q_home0_, dq_home0_);
      } else {
        phase_ = Phase::SETTLE;
        RCLCPP_INFO(this->get_logger(),
          "Homing completado -> asentamiento en q_d(0) (tol %.2f rad, max %.1f s).",
          SETTLE_TOL, SETTLE_TMAX);
      }
    }
    if (phase_ == Phase::SETTLE) {
      ref = desiredTrajectory(0.0);
      const double e_max = (q - ref.q).cwiseAbs().maxCoeff();
      if (e_max < SETTLE_TOL || t_settle_ >= SETTLE_TMAX) {
        phase_ = Phase::RUN;
        RCLCPP_INFO(this->get_logger(),
          "Asentado (max|e|=%.3f rad tras %.1f s). Inicia multiseno + adaptacion. CSV desde t=0.",
          e_max, t_settle_);
      }
    }
    if (phase_ == Phase::RUN) {
      ref = desiredTrajectory(t_run_);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  [SECCION 4] Ley de control y adaptacion — COMPLETAR
    //  Disponible: q, dq (medidos), ref.q, ref.dq, ref.ddq, y las funciones
    //  slotineLiTorque() / slotineLiRegressor() (infraestructura).
    // ─────────────────────────────────────────────────────────────────────────

    //  3.1 Errores, superficie y referencias auxiliares:
    //      e_q = q - q_d          e_dq = dq - dq_d
    //      s   = e_dq + Lambda*e_q
    //      dq_r  = dq_d  - Lambda*e_q
    //      ddq_r = ddq_d - Lambda*e_dq
    const Vec4 e_q  = Vec4::Zero();   // COMPLETAR
    const Vec4 e_dq = Vec4::Zero();   // COMPLETAR
    const Vec4 s_q  = Vec4::Zero();   // COMPLETAR
    const Vec4 dqr  = Vec4::Zero();   // COMPLETAR
    const Vec4 ddqr = Vec4::Zero();   // COMPLETAR
    (void)e_dq;

    //  3.2 Coulomb suavizado EN LAZO ABIERTO sobre la velocidad DESEADA dq_d
    //      (no sobre dq_r: una Fc_hat sobre/subestimada realimentaria la
    //      oscilacion): tanh_dqd_i = tanh(dq_d_i / EPS_FRICTION)
    const Vec4 tanh_dqd = Vec4::Zero();   // COMPLETAR

    // ── Vectores de tamano nv para Pinocchio (no modificar) ─────────────────
    Eigen::VectorXd q_pin    = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin   = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dqr_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd ddqr_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>()    = q;
    dq_pin.head<NARM>()   = dq;
    dqr_pin.head<NARM>()  = dqr;
    ddqr_pin.head<NARM>() = ddqr;

    // ── Matriz de inercia nominal M(q) via CRBA (no modificar) ──────────────
    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();
    const Eigen::Matrix4d M = data_.M.topLeftCorner<NARM, NARM>();

    // ── Termino rigido nominal Slotine-Li (no modificar) ────────────────────
    const Vec4 tau_rigid =
      slotineLiTorque(model_, data_, q_pin, dq_pin, dqr_pin, ddqr_pin).head<NARM>();

    //  3.3 Feedforward de friccion nominal del prior:
    //      tau_fric_nom = fv_prior .* dq + fc_prior .* tanh_dqd
    //      (fc_prior = a_prior_.segment<NARM>(IDX_FC))
    const Vec4 tau_fric_nom = Vec4::Zero();   // COMPLETAR
    const Vec4 tau_nom = tau_rigid + tau_fric_nom;

    // ── Regresor 4x12 (infraestructura, no modificar): usa dqr/ddqr/tanh_dqd ─
    const Eigen::MatrixXd Y_SL = slotineLiRegressor(model_, data_, q_pin, dq_pin, dqr_pin, ddqr_pin);
    MatY Y = MatY::Zero();
    for (int i = 0; i < 3; ++i) {                                        // Y_alpha1..3
      const int col0 = 10 * (static_cast<int>(jid_[i]) - 1);
      Y.col(IDX_ALPHA + i) = (Y_SL.middleCols(col0, 10) * pi_nom_[i]).head<NARM>();
    }
    const int col0_load = 10 * (static_cast<int>(jid_[3]) - 1);          // Y_load
    Y.block<NARM, 4>(0, IDX_LOAD) = Y_SL.block(0, col0_load, NARM, 4);
    Y(0, IDX_FV1) = dq[0];                                               // Y_Fv1 = dq1 medida
    for (int i = 0; i < NARM; ++i) {
      Y(i, IDX_FC + i) = tanh_dqd[i];                                    // Y_Fc = diag(tanh(dq_d/eps))
    }

    //  3.4 Ley de control (realimentacion M(q)-escalada con capa limite):
    //      sat_s_i = clamp(s_i / PHI_BL, -1, 1)
    //      tau = tau_nom + Y*(a_hat - a_prior)
    //            - M * ( K_V .* s + K_S .* sat_s )
    const double phi_eff = std::max(PHI_BL, 1e-9);   // evita division por cero (no modificar)
    (void)phi_eff;
    Vec4 sat_s = Vec4::Zero();                       // COMPLETAR
    const Vec4 tau = Vec4::Zero();                   // COMPLETAR
    (void)M; (void)tau_nom; (void)sat_s;
    const Vec4 tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // ── Publicar torques (no modificar) ─────────────────────────────────────
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[%s] t=%.2fs  |s|=%.4f  |e|=%.4f rad  alpha=[%.3f %.3f %.3f]  "
      "load=[%.3fkg %.4f %.4f %.4f]  Fv1=%.4f  Fc=[%.4f %.4f %.4f %.4f]",
      phase_ == Phase::RUN ? "RUN" : (phase_ == Phase::SETTLE ? "SETTLE" : "HOME"),
      t_, s_q.norm(), e_q.norm(),
      a_hat_[0], a_hat_[1], a_hat_[2],
      a_hat_[3], a_hat_[4], a_hat_[5],  a_hat_[6],  a_hat_[7],
      a_hat_[8], a_hat_[9], a_hat_[10], a_hat_[11]);

    // ── Registro CSV (solo fase RUN; no modificar) ──────────────────────────
    if (csv_.is_open() && phase_ == Phase::RUN) {
      csv_ << std::fixed << std::setprecision(6)
           << t_run_
           << ',' << q[0]        << ',' << q[1]        << ',' << q[2]        << ',' << q[3]
           << ',' << dq[0]       << ',' << dq[1]       << ',' << dq[2]       << ',' << dq[3]
           << ',' << ref.q[0]    << ',' << ref.q[1]    << ',' << ref.q[2]    << ',' << ref.q[3]
           << ',' << ref.dq[0]   << ',' << ref.dq[1]   << ',' << ref.dq[2]   << ',' << ref.dq[3]
           << ',' << s_q[0]      << ',' << s_q[1]      << ',' << s_q[2]      << ',' << s_q[3]
           << ',' << tau_sat[0]  << ',' << tau_sat[1]  << ',' << tau_sat[2]  << ',' << tau_sat[3]
           << ',' << (std::abs(tau[0]) > TAU_MAX ? 1 : 0)
           << ',' << (std::abs(tau[1]) > TAU_MAX ? 1 : 0)
           << ',' << (std::abs(tau[2]) > TAU_MAX ? 1 : 0)
           << ',' << (std::abs(tau[3]) > TAU_MAX ? 1 : 0)
           << ',' << a_hat_[0]   << ',' << a_hat_[1]   << ',' << a_hat_[2]
           << ',' << a_hat_[3]   << ',' << a_hat_[4]   << ',' << a_hat_[5]   << ',' << a_hat_[6]
           << ',' << a_hat_[7]   << ',' << a_hat_[8]   << ',' << a_hat_[9]   << ',' << a_hat_[10] << ',' << a_hat_[11]
           << '\n';
    }

    //  3.5 Ley de adaptacion (Euler + proyeccion + anti-windup + sigma-mod):
    //      a_dot = -Gamma .* ( Y^T * s + SIGMA_LEAK .* (a_hat - a_prior) )
    //      a_hat = a_hat + DT * a_dot        (luego proyeccion a [A_MIN, A_MAX])
    //      El congelamiento por saturacion y fase (anti-windup) ya esta dado.
    const bool tau_clipped = (tau.array().abs() > TAU_MAX).any();
    if (adaptive_ && phase_ == Phase::RUN && !tau_clipped) {
      const Vec12 a_dot = Vec12::Zero();   // COMPLETAR
      a_hat_ += DT * a_dot;
      // Proyeccion (activa solo si las cotas estan configuradas; no modificar)
      if ((A_MAX - A_MIN).minCoeff() > 0.0) {
        a_hat_ = a_hat_.cwiseMax(A_MIN).cwiseMin(A_MAX);
      }
    }

    t_ += DT;
    if (phase_ == Phase::SETTLE) { t_settle_ += DT; }
    if (phase_ == Phase::RUN)    { t_run_    += DT; }

    if (t_sim_ > 0.0 && phase_ == Phase::RUN && t_run_ >= t_sim_) {
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
  pinocchio::Model model_;
  pinocchio::Data  data_;

  std::array<pinocchio::JointIndex, NARM> jid_;
  std::array<Vec10, NARM> pi_nom_;

  double      t_;
  double      t_sim_;
  bool        adaptive_;
  bool        friction_prior_;
  Phase       phase_     = Phase::HOMING;
  bool        home_init_ = false;
  Vec4        q_home0_, dq_home0_;
  double      t_settle_  = 0.0;
  double      t_run_     = 0.0;
  std::string mode_str_;
  Vec12       a_prior_;
  Vec12       a_hat_;
  Vec4        fv_prior_;

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
    rclcpp::spin(std::make_shared<MRACJoint12pSimNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gz_mrac_joint_12p_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
