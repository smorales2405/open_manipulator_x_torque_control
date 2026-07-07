// ============================================================================
//  gz_MRAC_joint_node_base.cpp
//  Control Adaptativo (MRAC / Slotine-Li) en espacio articular
//  OpenMANIPULATOR-X (Gazebo)
//  [Archivo base para estudiantes — Lab 7, Actividad 1]
//
//  Modelo dinamico nominal (rigido, via Pinocchio):
//    M(q)*ddq + C(q,dq)*dq + g(q) + tau_fric(dq) + tau_carga = tau
//    tau_fric  = diag(b)*dq          (friccion viscosa, b desconocida)
//    tau_carga                       (carga en el efector, masa m_L desconocida)
//
//  El controlador conoce la dinamica RIGIDA del brazo (URDF nominal) y ADAPTA
//  en linea un vector pequeno de parametros fisicos desconocidos:
//    a_hat = [ m_L , b1 , b2 , b3 , b4 ]^T   (carga + fricciones viscosas)
//
//  Superficie de seguimiento (igual estructura que SMC articular):
//    e_q  = q  - q_d                 (error de posicion)
//    e_dq = dq - dq_d                (error de velocidad)
//    s    = e_dq + Lambda_q * e_q  =  dq - dq_r
//    dq_r  = dq_d  - Lambda_q * e_q       (velocidad de referencia)
//    ddq_r = ddq_d - Lambda_q * e_dq      (aceleracion de referencia)
//
//  Forma lineal en parametros (Slotine-Li):
//    M(q)*ddq_r + C(q,dq)*dq_r + g(q) = tau_nom  (parte rigida nominal, Pinocchio)
//    Y(q,dq,dq_r,ddq_r) * a           = parte adaptada
//    Regresor:  Y = [ Y_mL | diag(dq_r) ]
//      - Y_mL     : columna de masa de carga (diferencia RNEA con/sin carga unitaria)
//      - diag(dq_r): columnas de friccion viscosa
//
//  Ley de control adaptativo:
//    tau     = tau_nom(q,dq,dq_r,ddq_r) + Y * a_hat - K_D * s
//    tau_sat = clamp(tau, -TAU_MAX, TAU_MAX)
//
//  Ley de adaptacion (con proyeccion a la region fisica):
//    a_hat_dot = -Gamma .* (Y^T * s)      (solo si adaptive = true)
//    a_hat     = clamp(a_hat, A_MIN, A_MAX)
//
//  Estabilidad (Lyapunov):
//    V = 1/2 s^T M(q) s + 1/2 a_tilde^T Gamma^-1 a_tilde,  a_tilde = a_hat - a
//    V_dot = -s^T K_D s <= 0
//
//  Caso no adaptativo (comparacion, adaptive:=false):
//    a_hat se congela en A_HAT_0 durante toda la simulacion.
//
//  Suscriptor : /joint_states                    (sensor_msgs/JointState)
//  Publicador : /arm_effort_controller/commands   (std_msgs/Float64MultiArray)
//
//  Parametros ROS 2 (--ros-args -p nombre:=valor):
//    test_num  [int]     1      — identificador del CSV generado
//    t_sim     [double]  0.0    — duracion en segundos (0 = ilimitado)
//    adaptive  [bool]    true   — true: adapta a_hat | false: a_hat fijo en A_HAT_0
//
//  CSV generado: data/lab7/sim/act1/gz_mrac_joint_<modo>_<test_num>.csv
//    <modo> = "adaptive" | "fixed"
//  Columnas: t, q1..q4, dq1..dq4, q1_des..q4_des, dq1_des..dq4_des,
//            s1..s4, tau1..tau4, sat1..sat4, mL_hat, b1_hat, b2_hat, b3_hat, b4_hat
//
//  Notas de configuracion (config/sim_init_config.yaml):
//    - Arrancar en q_d(0):  use_fixed_init: true   q_init: [0.0, -0.45, 0.35, 0.785]
//    - Carga desconocida :  spawn_load: true        (cilindro ~0.1 kg en el efector)
//    - Friccion de planta:  friction_scale: 1.0
//    - Incertidumbre param: mass_inertia_scale: 1.2  damping_scale: 1.1
//
//  Ejemplos de uso:
//
//    ros2 run open_manipulator_x_torque_control gz_mrac_joint_node_base --ros-args -p adaptive:=true  -p test_num:=1 -p t_sim:=20.0
//    ros2 run open_manipulator_x_torque_control gz_mrac_joint_node_base --ros-args -p adaptive:=false -p test_num:=2 -p t_sim:=20.0
//
//  ──────────────────────────────────────────────────────────────────────────
//  SECCIONES A COMPLETAR:
//    [1] Trayectoria articular de referencia  →  desiredTrajectory()
//    [2] Ganancias MRAC                       →  LAMBDA_Q, K_D, GAMMA, A_HAT_0
//    [3] Termino nominal Slotine-Li           →  slotineLiTorque()
//    [4] Ley de control y adaptacion          →  tick()
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
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

using namespace std::chrono_literals;

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif

static constexpr double PI      = M_PI;
static constexpr int    NARM    = 4;       // articulaciones controladas
static constexpr int    NP      = 5;       // parametros adaptados: [m_L, b1..b4]
static constexpr double TAU_MAX = 1.2;     // [N·m] limite de torque por articulacion
static constexpr double DT      = 0.01;    // [s] periodo de control (100 Hz)
static constexpr double LOAD_X_OFFSET = 0.015;  // [m] offset de la carga en el efector

using Vec4  = Eigen::Vector4d;
using Vec5  = Eigen::Matrix<double, NP, 1>;
using Mat45 = Eigen::Matrix<double, NARM, NP>;

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 2] GANANCIAS MRAC — COMPLETAR
//  Indice articular: [joint1, joint2, joint3, joint4]
//  Indice parametro: [m_L, b1, b2, b3, b4]
//
//  Rangos recomendados:
//    LAMBDA_Q : 1.0  – 30.0  [1/s]   (superficie de seguimiento)
//    K_D      : 1.0  – 30.0          (amortiguamiento sobre s)
//    GAMMA    : m_L: [0.5, 5.0]  |  b_i: [0.1, 2.0]   (tasas de adaptacion)
//    A_HAT_0  : estimacion inicial, tipicamente ceros si no hay conocimiento previo
// ═══════════════════════════════════════════════════════════════════════════
static const Vec4 LAMBDA_Q = {0.0, 0.0, 0.0, 0.0};  // COMPLETAR
static const Vec4 K_D      = {0.0, 0.0, 0.0, 0.0};  // COMPLETAR

// Tasas de adaptacion Gamma = diag(gamma_mL, gamma_b1..b4)
static const Vec5 GAMMA   = (Vec5() << 0.0, 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR

// Estimacion inicial a_hat(0) = [m_L, b1, b2, b3, b4]
static const Vec5 A_HAT_0 = (Vec5() << 0.0, 0.0, 0.0, 0.0, 0.0).finished();  // COMPLETAR

// Proyeccion a la region fisica admisible (anti-deriva parametrica) — no modificar
static const Vec5 A_MIN = (Vec5() << 0.0, 0.0, 0.0, 0.0, 0.0).finished();
static const Vec5 A_MAX = (Vec5() << 2.0, 1.0, 1.0, 1.0, 1.0).finished();
// ═══════════════════════════════════════════════════════════════════════════

// ── Trayectoria de referencia articular ──────────────────────────────────────
struct Reference {
  Vec4 q, dq, ddq;
};

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 1] Trayectoria articular de referencia — COMPLETAR
//
//  Definir la trayectoria para cada articulacion de la forma:
//    qi_d(t)   = Ai + Bi * sin(w*t)
//    dqi_d(t)  = Bi * w * cos(w*t)
//    ddqi_d(t) = -Bi * w^2 * sin(w*t)
//
//  Usar w = 1.0 rad/s (no modificar).
// ─────────────────────────────────────────────────────────────────────────────
static Reference desiredTrajectory(double t)
{
  Reference ref;
  const double w = 1.0;  // [rad/s] — no modificar

  // COMPLETAR: posicion deseada qi_d(t) = Ai + Bi*sin(w*t)
  ref.q <<
      0.0,   // joint1: COMPLETAR
      0.0,   // joint2: COMPLETAR
      0.0,   // joint3: COMPLETAR
      0.0;   // joint4: COMPLETAR

  // COMPLETAR: velocidad deseada dqi_d(t) = Bi*w*cos(w*t)
  ref.dq <<
      0.0,   // joint1: COMPLETAR
      0.0,   // joint2: COMPLETAR
      0.0,   // joint3: COMPLETAR
      0.0;   // joint4: COMPLETAR

  // COMPLETAR: aceleracion deseada ddqi_d(t) = -Bi*w^2*sin(w*t)
  ref.ddq <<
      0.0,   // joint1: COMPLETAR
      0.0,   // joint2: COMPLETAR
      0.0,   // joint3: COMPLETAR
      0.0;   // joint4: COMPLETAR

  (void)t;  // eliminar cuando se use t en las expresiones
  return ref;
}

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 3] Termino nominal Slotine-Li — COMPLETAR
//
//  Retorna: M(q)*ddq_r + C(q,dq)*dq_r + g(q)  usando RNEA de Pinocchio.
//
//  Identidades (rnea(q, v, a) = M(q)*a + C(q,v)*v + g(q)):
//    g(q)         = rnea(q, 0,       0    )
//    M(q)*ddq_r   = rnea(q, 0,       ddq_r) - g
//    C(q,dq)*dq_r = 1/2 * [ rnea(q, dq+dq_r, 0)
//                           - rnea(q, dq,     0)
//                           - rnea(q, dq_r,   0) + g ]
//                   (identidad de polarizacion de Christoffel)
//
//  Nota: Mddqr = rnea(q, 0, ddqr) ya incluye g internamente,
//        por lo que el retorno final Mddqr + Cqr da M*ddq_r + C*dq_r + g.
// ─────────────────────────────────────────────────────────────────────────────
static Eigen::VectorXd slotineLiTorque(
    const pinocchio::Model & model, pinocchio::Data & data,
    const Eigen::VectorXd & q,   const Eigen::VectorXd & dq,
    const Eigen::VectorXd & dqr, const Eigen::VectorXd & ddqr)
{
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(model.nv);

  // COMPLETAR: g(q) = rnea(q, zero, zero)
  const Eigen::VectorXd g     = Eigen::VectorXd::Zero(model.nv);

  // COMPLETAR: Mddqr = rnea(q, zero, ddqr)   [equivale a M(q)*ddq_r + g]
  const Eigen::VectorXd Mddqr = Eigen::VectorXd::Zero(model.nv);

  // COMPLETAR: Cqr = C(q,dq)*dq_r via identidad de polarizacion
  //   Calcular r_sum  = rnea(q, dq+dqr, zero)
  //            r_dq   = rnea(q, dq,     zero)
  //            r_dqr  = rnea(q, dqr,    zero)
  //   Luego Cqr = 0.5 * (r_sum - r_dq - r_dqr + g)
  const Eigen::VectorXd Cqr   = Eigen::VectorXd::Zero(model.nv);

  (void)zero; (void)g; (void)dq; (void)dqr; (void)ddqr;  // eliminar al completar
  return Mddqr + Cqr;  // M*ddq_r + C*dq_r + g — no modificar
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

    // ── Modelo Pinocchio (brazo nominal) ──────────────────────────────────────
    const std::string urdf_path =
      std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf_path, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio buildModel: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    // ── Modelo con carga unitaria en el efector (para el regresor Y_mL) ───────
    build_load_model();

    RCLCPP_INFO(this->get_logger(),
      "MRAC articular — modo=%s  tau_max=%.2f N·m", mode_str_.c_str(), TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "Lambda=[%.1f %.1f %.1f %.1f]  Kd=[%.1f %.1f %.1f %.1f]",
      LAMBDA_Q[0], LAMBDA_Q[1], LAMBDA_Q[2], LAMBDA_Q[3],
      K_D[0],      K_D[1],      K_D[2],      K_D[3]);
    RCLCPP_INFO(this->get_logger(),
      "Gamma=[%.2f | %.2f %.2f %.2f %.2f]  a_hat(0)=[%.2f | %.2f %.2f %.2f %.2f]",
      GAMMA[0], GAMMA[1], GAMMA[2], GAMMA[3], GAMMA[4],
      A_HAT_0[0], A_HAT_0[1], A_HAT_0[2], A_HAT_0[3], A_HAT_0[4]);
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
  // ── Modelo con carga unitaria en el efector (no modificar) ────────────────
  //   model_load_ = model_ + masa puntual unitaria en end_effector_link.
  //   La diferencia slotineLi(model_load_) - slotineLi(model_) entrega la
  //   columna del regresor para la masa de carga m_L.
  void build_load_model()
  {
    model_load_ = model_;

    pinocchio::JointIndex jid = static_cast<pinocchio::JointIndex>(model_load_.njoints - 1);
    pinocchio::SE3 placement  = pinocchio::SE3::Identity();

    if (model_load_.existFrame("end_effector_link")) {
      const pinocchio::FrameIndex fid = model_load_.getFrameId("end_effector_link");
      const pinocchio::Frame & f      = model_load_.frames[fid];
      jid       = f.parentJoint;
      placement = f.placement;
    } else {
      RCLCPP_WARN(this->get_logger(),
        "Frame 'end_effector_link' no encontrado — carga anclada al ultimo joint.");
    }

    const pinocchio::Inertia unit_load(
      1.0, Eigen::Vector3d(LOAD_X_OFFSET, 0.0, 0.0), Eigen::Matrix3d::Zero());
    model_load_.appendBodyToJoint(jid, unit_load, placement);
    data_load_ = pinocchio::Data(model_load_);
  }

  // ── CSV ───────────────────────────────────────────────────────────────────
  void open_csv(int test_num)
  {
    std::filesystem::create_directories(
      std::string(PACKAGE_DATA_DIR) + "/lab7/sim/act1");

    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab7/sim/act1/gz_mrac_joint_"
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
         << "mL_hat,b1_hat,b2_hat,b3_hat,b4_hat\n";
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

    // ── Errores articulares (no modificar) ────────────────────────────────
    const Vec4 e_q  = q  - ref.q;
    const Vec4 e_dq = dq - ref.dq;

    // ─────────────────────────────────────────────────────────────────────────
    //  [SECCION 4] Ley de control y adaptacion — COMPLETAR
    // ─────────────────────────────────────────────────────────────────────────

    // 4.1 Superficie de seguimiento s y referencias auxiliares:
    //   s    = e_dq + Lambda_Q .* e_q
    //   dq_r  = ref.dq  - Lambda_Q .* e_q
    //   ddq_r = ref.ddq - Lambda_Q .* e_dq
    const Vec4 s_q  = Vec4::Zero();  // COMPLETAR
    const Vec4 dqr  = Vec4::Zero();  // COMPLETAR
    const Vec4 ddqr = Vec4::Zero();  // COMPLETAR

    // ── Vectores de tamano nv para Pinocchio (no modificar) ───────────────
    Eigen::VectorXd q_pin    = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin   = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dqr_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd ddqr_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>()    = q;
    dq_pin.head<NARM>()   = dq;
    dqr_pin.head<NARM>()  = dqr;
    ddqr_pin.head<NARM>() = ddqr;

    // ── Termino rigido nominal y columna de carga del regresor (no modificar) ─
    const Eigen::VectorXd tau_nom_full =
      slotineLiTorque(model_,      data_,      q_pin, dq_pin, dqr_pin, ddqr_pin);
    const Eigen::VectorXd tau_load_full =
      slotineLiTorque(model_load_, data_load_, q_pin, dq_pin, dqr_pin, ddqr_pin);

    const Vec4 tau_nom = tau_nom_full.head<NARM>();
    const Vec4 Y_mL    = (tau_load_full - tau_nom_full).head<NARM>();  // columna m_L

    // 4.2 Regresor adaptado Y = [ Y_mL | diag(dq_r) ]  (NARM x NP)
    //   Columna 0    : Y_mL            (efecto de la carga de masa)
    //   Columnas 1..4: Y(i,1+i)=dqr[i] (fricciones viscosas por articulacion)
    Mat45 Y = Mat45::Zero();
    // COMPLETAR: Y.col(0) = Y_mL
    // COMPLETAR: for (int i = 0; i < NARM; ++i) { Y(i, 1+i) = dqr[i]; }

    // 4.3 Ley de control:
    //   tau = tau_nom + Y * a_hat_ - K_D .* s
    const Vec4 tau     = Vec4::Zero();  // COMPLETAR
    const Vec4 tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // ─────────────────────────────────────────────────────────────────────────

    // ── Publicar torques ───────────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "t=%.2fs  |s|=%.4f  |e|=%.4f rad  mL=%.3f kg  b=[%.3f %.3f %.3f %.3f]",
      t_, s_q.norm(), e_q.norm(),
      a_hat_[0], a_hat_[1], a_hat_[2], a_hat_[3], a_hat_[4]);

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
           << ',' << a_hat_[0]   << ',' << a_hat_[1]   << ',' << a_hat_[2]
           << ',' << a_hat_[3]   << ',' << a_hat_[4]
           << '\n';
    }

    // ── Ley de adaptacion (Euler + proyeccion) ─────────────────────────────
    if (adaptive_) {
      // 4.4 Ley de adaptacion:
      //   a_hat_dot = -Gamma .* (Y^T * s)
      const Vec5 a_dot = Vec5::Zero();  // COMPLETAR
      a_hat_ += DT * a_dot;
      a_hat_ = a_hat_.cwiseMax(A_MIN).cwiseMin(A_MAX);  // proyeccion (no modificar)
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
  pinocchio::Model model_,  model_load_;
  pinocchio::Data  data_,   data_load_;

  double      t_;
  double      t_sim_;
  bool        adaptive_;
  std::string mode_str_;
  Vec5        a_hat_;     // estimacion actual [m_L, b1, b2, b3, b4]

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
