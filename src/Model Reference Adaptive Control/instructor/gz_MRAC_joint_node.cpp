// ============================================================================
//  gz_MRAC_joint_node.cpp
//  Control Adaptativo (MRAC / Slotine-Li) en espacio articular
//  OpenMANIPULATOR-X (Gazebo) — Lab 7, Actividad 2
//
//  Modelo dinamico nominal (rigido, via Pinocchio):
//    M(q)*ddq + C(q,dq)*dq + g(q) + B_NOM*dq = tau     (catalogo, escala 1.0)
//
//  La PLANTA real (Gazebo) difiere del modelo nominal por dos escalas del
//  Xacro (config/sim_init_config.yaml):
//    mass_inertia_scale : 1.0 -> 1.2   (masa+inercia de link1..link5, TODO el tensor)
//    damping_scale       : 1.0 -> 1.1   (damping viscoso de joint1..joint4)
//  friction_scale se mantiene en 0.0 (friccion de Coulomb no modelada, igual
//  que en SMC) y no hay carga en el efector (spawn_load: false).
//
//  El controlador ADAPTA en linea 8 parametros fisicos interpretables:
//    a_hat = [ alpha1..alpha4 , b1..b4 ]^T
//    alpha_k : escala de inercia del cuerpo movido por jointk (linkk+1)
//              nominal = 1.0  |  real = 1.2
//    b_j     : damping viscoso de jointj
//              nominal = catalogo (B_NOM, desde el URDF)  |  real = 1.1*B_NOM
//
//  Superficie de seguimiento (igual estructura que SMC/MRAC articular previo):
//    e_q  = q  - q_d                 (error de posicion)
//    e_dq = dq - dq_d                (error de velocidad)
//    s    = e_dq + Lambda_q * e_q  =  dq - dq_r
//    dq_r  = dq_d  - Lambda_q * e_q       (velocidad de referencia)
//    ddq_r = ddq_d - Lambda_q * e_dq      (aceleracion de referencia)
//
//  Regresor Slotine-Li exacto via pinocchio::computeJointTorqueRegressor
//  (generaliza la identidad de polarizacion que antes se aplicaba a RNEA,
//  ahora aplicada al REGRESOR R(q,v,a), con R(q,v,a)*pi = M(q)a+C(q,v)v+g(q)
//  para cualquier vector de parametros dinamicos pi):
//
//    Y_SL(q,dq,dqr,ddqr) = R(q,0,ddqr)
//                         + 0.5*[ R(q,dq+dqr,0) - R(q,dq,0) - R(q,dqr,0) + R(q,0,0) ]
//
//  Y_SL es (nv x 10*ncuerpos). La columna alpha_k se obtiene contrayendo el
//  bloque de 10 columnas del cuerpo k con sus parametros dinamicos NOMINALES
//  pi_nom_k = model.inertias[jointk].toDynamicParameters()
//           = [ m, m*cx, m*cy, m*cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz ]  (orden interno Pinocchio):
//
//    Y_alpha.col(k) = ( Y_SL.middleCols(10*(jointk-1), 10) * pi_nom_k ).head<NARM>()
//
//  Esto captura el escalado UNIFORME de masa + centro de masa + tensor de
//  inercia completo del cuerpo (a diferencia de una masa puntual, que solo
//  aproxima la masa y no el tensor). Los 2 grados de libertad del gripper
//  (no escalados por mass_inertia_scale en la planta) quedan como termino
//  RIGIDO CONOCIDO dentro de tau_nom (no se adaptan ni se reconstruyen).
//
//  Regresor completo:  Y = [ Y_alpha | diag(dq_r) ]   (4x8)
//    - Y_alpha    : 4 columnas, escala de inercia por eslabon
//    - diag(dq_r) : 4 columnas, damping viscoso por junta
//
//  Ley de control adaptativo (FORMA DE DESVIACION respecto al catalogo, evita
//  contar dos veces la parte nominal que ya esta dentro de tau_nom):
//    tau_nom  = tau_nom_rigido(q,dq,dqr,ddqr) + B_NOM .* dq_r      (RNEA + damping nominal)
//    tau      = tau_nom + Y*(a_hat - A_HAT_0) - K_D*s
//    tau_sat  = clamp(tau, -TAU_MAX, TAU_MAX)
//  En a_hat = A_HAT_0 (catalogo)  ->  tau = tau_nom - K_D*s  (mejor modelo fijo + PD).
//  En a_hat = a_real (1.2 | 1.1*B_NOM) -> tau reproduce exactamente la planta real + PD.
//
//  Ley de adaptacion (con proyeccion a la region fisica):
//    a_hat_dot = -Gamma .* (Y^T * s)      (solo si adaptive = true)
//    a_hat     = clamp(a_hat, A_MIN, A_MAX)     (alpha > 0 , b >= 0)
//
//  Estabilidad (Lyapunov, a_tilde = a_hat - a_real):
//    V = 1/2 s^T M(q) s + 1/2 a_tilde^T Gamma^-1 a_tilde
//    V_dot = -s^T K_D s <= 0   (bajo excitacion persistente, a_hat -> a_real)
//
//  Caso no adaptativo (comparacion, adaptive:=false):
//    a_hat se congela en A_HAT_0 (catalogo nominal) durante toda la simulacion.
//
//  Trayectoria articular de referencia — MULTISENO, frecuencias distintas por
//  junta (excitacion persistente; la senoidal en fase anterior era pobre para
//  identificar 8 parametros). qi_d(t) = Ci + Ai*sin(wai*t) + Bi*sin(wbi*t):
//    q1_d = 0.00 + 0.60*sin(0.6t) + 0.20*sin(1.5t)
//    q2_d = -0.30 + 0.45*sin(0.9t) + 0.15*sin(1.9t)
//    q3_d = 0.30 + 0.45*sin(1.3t) + 0.15*sin(0.7t)
//    q4_d = 0.80 + 0.40*sin(1.7t) + 0.15*sin(1.1t)
//  (verificado dentro de limites de posicion/velocidad del URDF; en t=0 todos
//  los senos valen 0 -> arranca en los centros [0, -0.30, 0.30, 0.80]).
//
//  Suscriptor : /joint_states                    (sensor_msgs/JointState)
//  Publicador : /arm_effort_controller/commands   (std_msgs/Float64MultiArray)
//
//  Parametros ROS 2 (--ros-args -p nombre:=valor):
//    test_num  [int]     1      — identificador del CSV generado
//    t_sim     [double]  0.0    — duracion en segundos (0 = ilimitado)
//    adaptive  [bool]    true   — true: adapta a_hat | false: a_hat fijo en A_HAT_0
//
//  CSV generado: data/lab7/sim/act2/gz_mrac_joint_<modo>_<test_num>.csv
//    <modo> = "adaptive" | "fixed"
//  Columnas: t, q1..q4, dq1..dq4, q1_des..q4_des, dq1_des..dq4_des,
//            s1..s4, tau1..tau4, sat1..sat4,
//            a1_hat..a4_hat, b1_hat..b4_hat
//
//  Notas de configuracion (config/sim_init_config.yaml):
//    - Arrancar en q_d(0):  use_fixed_init: true   q_init: [0.0, -0.30, 0.30, 0.80]
//    - Planta real        :  mass_inertia_scale: 1.2   damping_scale: 1.1
//    - Sin friccion Coulomb: friction_scale: 0.0  (igual que SMC)
//    - Sin carga en efector: spawn_load: false
//    - Damping nominal joint2/joint3 = 0 en el URDF -> b2_hat, b3_hat deben
//      permanecer cerca de 0 (no hay damping viscoso real que identificar ahi).
//
//  Ejemplos de uso:
//
//    # Caso adaptativo (estima escala de inercia y damping en linea):
//    ros2 run open_manipulator_x_torque_control gz_mrac_joint_node
//      --ros-args -p adaptive:=true -p test_num:=1 -p t_sim:=30.0
//
//    # Caso no adaptativo (catalogo nominal fijo, referencia de comparacion):
//    ros2 run open_manipulator_x_torque_control gz_mrac_joint_node
//      --ros-args -p adaptive:=false -p test_num:=2 -p t_sim:=30.0
//
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
static constexpr int    NP      = 8;       // parametros adaptados: [alpha1..4, b1..4]
static constexpr double TAU_MAX = 1.2;     // [N·m] limite de torque por articulacion
static constexpr double DT      = 0.01;    // [s] periodo de control (100 Hz)

using Vec4  = Eigen::Vector4d;
using Vec8  = Eigen::Matrix<double, NP, 1>;
using Vec10 = Eigen::Matrix<double, 10, 1>;
using Mat48 = Eigen::Matrix<double, NARM, NP>;

// Damping nominal por junta (URDF, escala 1.0) — xacro/open_manipulator_x.urdf.xacro
static const Vec4 B_NOM = (Vec4() << 0.0367, 0.0000, 0.0000, 0.0050).finished();

// ═══════════════════════════════════════════════════════════════════════════
//  GANANCIAS MRAC — ajustar aqui
//  Indice articular: [joint1, joint2, joint3, joint4]
//  Indice parametro: [alpha1, alpha2, alpha3, alpha4, b1, b2, b3, b4]
// ═══════════════════════════════════════════════════════════════════════════
static const Vec4 LAMBDA_Q = {5.0,  5.0,  10.0,  25.0};   // superficie de seguimiento [1/s]
static const Vec4 K_D      = {5.0,  5.0,  10.0,  25.0};   // amortiguamiento sobre s (proporcional a s)

// Tasas de adaptacion Gamma = diag(gamma_alpha1..4, gamma_b1..4)
static const Vec8 GAMMA =
  (Vec8() << 0.5, 0.5, 0.5, 0.5,   2.0, 2.0, 2.0, 2.0).finished();

// Estimacion inicial a_hat(0) = catalogo nominal (mass_inertia_scale=1.0, damping_scale=1.0).
// Con esto el caso no adaptativo (adaptive:=false) es el mejor modelo fijo posible.
static const Vec8 A_HAT_0 =
  (Vec8() << 1.0, 1.0, 1.0, 1.0,   B_NOM[0], B_NOM[1], B_NOM[2], B_NOM[3]).finished();

// Proyeccion a la region fisica admisible (anti-deriva parametrica): alpha > 0, b >= 0
static const Vec8 A_MIN =
  (Vec8() << 0.5, 0.5, 0.5, 0.5,   0.0, 0.0, 0.0, 0.0).finished();
static const Vec8 A_MAX =
  (Vec8() << 2.0, 2.0, 2.0, 2.0,   0.2, 0.2, 0.2, 0.2).finished();
// ═══════════════════════════════════════════════════════════════════════════

// ── Trayectoria de referencia articular (multiseno, excitacion persistente) ──
struct Reference {
  Vec4 q, dq, ddq;
};

static Reference desiredTrajectory(double t)
{
  Reference ref;

  ref.q <<
       0.00 + 0.60 * std::sin(0.6 * t) + 0.20 * std::sin(1.5 * t),
      -0.30 + 0.45 * std::sin(0.9 * t) + 0.15 * std::sin(1.9 * t),
       0.30 + 0.45 * std::sin(1.3 * t) + 0.15 * std::sin(0.7 * t),
       0.80 + 0.40 * std::sin(1.7 * t) + 0.15 * std::sin(1.1 * t);

  ref.dq <<
      0.60 * 0.6 * std::cos(0.6 * t) + 0.20 * 1.5 * std::cos(1.5 * t),
      0.45 * 0.9 * std::cos(0.9 * t) + 0.15 * 1.9 * std::cos(1.9 * t),
      0.45 * 1.3 * std::cos(1.3 * t) + 0.15 * 0.7 * std::cos(0.7 * t),
      0.40 * 1.7 * std::cos(1.7 * t) + 0.15 * 1.1 * std::cos(1.1 * t);

  ref.ddq <<
      -0.60 * 0.6 * 0.6 * std::sin(0.6 * t) - 0.20 * 1.5 * 1.5 * std::sin(1.5 * t),
      -0.45 * 0.9 * 0.9 * std::sin(0.9 * t) - 0.15 * 1.9 * 1.9 * std::sin(1.9 * t),
      -0.45 * 1.3 * 1.3 * std::sin(1.3 * t) - 0.15 * 0.7 * 0.7 * std::sin(0.7 * t),
      -0.40 * 1.7 * 1.7 * std::sin(1.7 * t) - 0.15 * 1.1 * 1.1 * std::sin(1.1 * t);

  return ref;
}

// ── Termino Slotine-Li rigido:  M(q)*ddq_r + C(q,dq)*dq_r + g(q) via RNEA ────
//   Identidades (rnea(q,v,a) = M(q)a + C(q,v)v + g(q)):
//     g(q)        = rnea(q, 0,   0)
//     M(q)*ddq_r  = rnea(q, 0,   ddq_r) - g
//     C(q,dq)*dq_r= 1/2 [ rnea(q, dq+dq_r, 0) - rnea(q, dq, 0) - rnea(q, dq_r, 0) + g ]
static Eigen::VectorXd slotineLiTorque(
    const pinocchio::Model & model, pinocchio::Data & data,
    const Eigen::VectorXd & q,   const Eigen::VectorXd & dq,
    const Eigen::VectorXd & dqr, const Eigen::VectorXd & ddqr)
{
  const Eigen::VectorXd zero  = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd g     = pinocchio::rnea(model, data, q, zero,      zero);
  const Eigen::VectorXd Mddqr = pinocchio::rnea(model, data, q, zero,      ddqr);  // M*ddq_r + g
  const Eigen::VectorXd r_sum = pinocchio::rnea(model, data, q, dq + dqr,  zero);
  const Eigen::VectorXd r_dq  = pinocchio::rnea(model, data, q, dq,        zero);
  const Eigen::VectorXd r_dqr = pinocchio::rnea(model, data, q, dqr,       zero);
  const Eigen::VectorXd Cqr   = 0.5 * (r_sum - r_dq - r_dqr + g);                 // C(q,dq)*dq_r
  return Mddqr + Cqr;                                                            // M*ddq_r + C*dq_r + g
}

// ── Regresor Slotine-Li:  misma identidad de polarizacion, aplicada al ──────
//   regresor R(q,v,a) en vez de RNEA. R(q,v,a)*pi = M(q)a+C(q,v)v+g(q) para
//   cualquier vector de parametros dinamicos pi (uno por cuerpo, 10 c/u).
//   Cada llamada a computeJointTorqueRegressor sobrescribe data.jointTorqueRegressor,
//   por eso cada resultado se copia (Eigen::MatrixXd) antes de la siguiente.
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

// ── Nodo principal ────────────────────────────────────────────────────────────
class MRACJointSimNode : public rclcpp::Node
{
public:
  MRACJointSimNode()
  : Node("gz_mrac_joint_node"), t_(0.0), a_hat_(A_HAT_0)
  {
    // ── Parametros ────────────────────────────────────────────────────────────
    this->declare_parameter<int>   ("test_num", 1);
    this->declare_parameter<double>("t_sim",    0.0);
    this->declare_parameter<bool>  ("adaptive", true);

    const int test_num = this->get_parameter("test_num").as_int();
    t_sim_             = this->get_parameter("t_sim").as_double();
    adaptive_          = this->get_parameter("adaptive").as_bool();
    mode_str_          = adaptive_ ? "adaptive" : "fixed";

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

    precompute_regressor_bases();

    RCLCPP_INFO(this->get_logger(),
      "MRAC articular — modo=%s  tau_max=%.2f N·m", mode_str_.c_str(), TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "Lambda=[%.1f %.1f %.1f %.1f]  Kd=[%.1f %.1f %.1f %.1f]",
      LAMBDA_Q[0], LAMBDA_Q[1], LAMBDA_Q[2], LAMBDA_Q[3],
      K_D[0],      K_D[1],      K_D[2],      K_D[3]);
    RCLCPP_INFO(this->get_logger(),
      "Gamma_alpha=[%.2f %.2f %.2f %.2f]  Gamma_b=[%.2f %.2f %.2f %.2f]",
      GAMMA[0], GAMMA[1], GAMMA[2], GAMMA[3],
      GAMMA[4], GAMMA[5], GAMMA[6], GAMMA[7]);
    RCLCPP_INFO(this->get_logger(),
      "a_hat(0): alpha=[%.2f %.2f %.2f %.2f]  b=[%.4f %.4f %.4f %.4f]  (catalogo nominal)",
      A_HAT_0[0], A_HAT_0[1], A_HAT_0[2], A_HAT_0[3],
      A_HAT_0[4], A_HAT_0[5], A_HAT_0[6], A_HAT_0[7]);
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

  ~MRACJointSimNode()
  {
    if (csv_.is_open()) {
      csv_.close();
      RCLCPP_INFO(this->get_logger(), "CSV cerrado: %s", csv_path_.c_str());
    }
  }

private:
  // ── Indices de junta y parametros dinamicos nominales (para Y_alpha) ───────
  //   pi_nom_[i] = parametros dinamicos nominales (10) del cuerpo movido por
  //   jointi (linki+1), en el orden interno de Pinocchio (ver
  //   pinocchio::Inertia::toDynamicParameters()). Y_alpha.col(i) contrae el
  //   bloque de 10 columnas de jointi en el regresor Slotine-Li con pi_nom_[i]:
  //   captura el escalado UNIFORME de masa + CoM + tensor de inercia completo
  //   del cuerpo (no una masa puntual).
  void precompute_regressor_bases()
  {
    for (int i = 0; i < NARM; ++i) {
      const std::string jname = "joint" + std::to_string(i + 1);
      jid_[i]    = model_.getJointId(jname);
      pi_nom_[i] = model_.inertias[jid_[i]].toDynamicParameters();
    }
  }

  // ── CSV ───────────────────────────────────────────────────────────────────
  void open_csv(int test_num)
  {
    std::filesystem::create_directories(
      std::string(PACKAGE_DATA_DIR) + "/lab7/sim/act2");

    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab7/sim/act2/gz_mrac_joint_"
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
         << "a1_hat,a2_hat,a3_hat,a4_hat,b1_hat,b2_hat,b3_hat,b4_hat\n";
    RCLCPP_INFO(this->get_logger(), "CSV: %s", csv_path_.c_str());
  }

  // ── Lectura de estados articulares (por nombre, orden independiente) ───────
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

  // ── Tick de control a 100 Hz ──────────────────────────────────────────────
  void tick()
  {
    if (!last_js_) { return; }

    Vec4 q, dq;
    read_js(q, dq);

    const Reference ref = desiredTrajectory(t_);

    // ── Errores, superficie y referencias auxiliares ───────────────────────
    const Vec4 e_q   = q  - ref.q;
    const Vec4 e_dq  = dq - ref.dq;
    const Vec4 s_q   = e_dq + LAMBDA_Q.asDiagonal() * e_q;          // s = dq - dq_r
    const Vec4 dqr   = ref.dq  - LAMBDA_Q.asDiagonal() * e_q;       // dq_r
    const Vec4 ddqr  = ref.ddq - LAMBDA_Q.asDiagonal() * e_dq;      // ddq_r

    // ── Vectores de tamano nv para Pinocchio (gripper en cero) ─────────────
    Eigen::VectorXd q_pin    = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin   = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dqr_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd ddqr_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>()    = q;
    dq_pin.head<NARM>()   = dq;
    dqr_pin.head<NARM>()  = dqr;
    ddqr_pin.head<NARM>() = ddqr;

    // ── Termino rigido nominal (RNEA) + damping nominal ─────────────────────
    const Eigen::VectorXd tau_nom_rigid_full =
      slotineLiTorque(model_, data_, q_pin, dq_pin, dqr_pin, ddqr_pin);
    const Vec4 tau_nom = tau_nom_rigid_full.head<NARM>() + B_NOM.cwiseProduct(dqr);

    // ── Regresor Slotine-Li completo y columnas de escala de inercia ───────
    const Eigen::MatrixXd Y_SL = slotineLiRegressor(model_, data_, q_pin, dq_pin, dqr_pin, ddqr_pin);

    Mat48 Y = Mat48::Zero();
    for (int i = 0; i < NARM; ++i) {
      const int col0 = 10 * (static_cast<int>(jid_[i]) - 1);
      Y.col(i) = (Y_SL.middleCols(col0, 10) * pi_nom_[i]).head<NARM>();  // Y_alpha
    }
    for (int i = 0; i < NARM; ++i) {
      Y(i, NARM + i) = dqr[i];                                          // Y_b = diag(dq_r)
    }

    // ── Ley de control adaptativo (forma de desviacion respecto al catalogo) ─
    const Vec4 tau     = tau_nom + Y * (a_hat_ - A_HAT_0) - K_D.asDiagonal() * s_q;
    const Vec4 tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // ── Publicar torques ───────────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "t=%.2fs  |s|=%.4f  |e|=%.4f rad  alpha=[%.3f %.3f %.3f %.3f]  b=[%.4f %.4f %.4f %.4f]",
      t_, s_q.norm(), e_q.norm(),
      a_hat_[0], a_hat_[1], a_hat_[2], a_hat_[3],
      a_hat_[4], a_hat_[5], a_hat_[6], a_hat_[7]);

    // ── Registro CSV (a_hat usado en este tick) ────────────────────────────
    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t_
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
           << ',' << a_hat_[0]   << ',' << a_hat_[1]   << ',' << a_hat_[2]   << ',' << a_hat_[3]
           << ',' << a_hat_[4]   << ',' << a_hat_[5]   << ',' << a_hat_[6]   << ',' << a_hat_[7]
           << '\n';
    }

    // ── Ley de adaptacion (Euler + proyeccion) ─────────────────────────────
    if (adaptive_) {
      const Vec8 a_dot = -GAMMA.cwiseProduct(Y.transpose() * s_q);
      a_hat_ += DT * a_dot;
      a_hat_ = a_hat_.cwiseMax(A_MIN).cwiseMin(A_MAX);
    }

    t_ += DT;

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
  pinocchio::Model model_;
  pinocchio::Data  data_;

  std::array<pinocchio::JointIndex, NARM> jid_;
  std::array<Vec10, NARM> pi_nom_;

  double      t_;
  double      t_sim_;
  bool        adaptive_;
  std::string mode_str_;
  Vec8        a_hat_;     // estimacion actual [alpha1..4, b1..4]

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
    rclcpp::spin(std::make_shared<MRACJointSimNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gz_mrac_joint_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
