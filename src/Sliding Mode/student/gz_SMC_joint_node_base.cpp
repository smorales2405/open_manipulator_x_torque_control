// ============================================================================
//  gz_SMC_joint_node_base.cpp
//  Control por Modo Deslizante en espacio articular — OpenMANIPULATOR-X (Gazebo)
//  [Archivo base para estudiantes — Lab 6, Actividad 1]
//
//  Modelo dinamico nominal (Guia Lab 6, Sec. 6.1):
//    M(q)*ddq + Phi(q,dq) = tau - tau_fric
//    tau_fric = Fv·dq + Fc·tanh(dq_d/eps)    (feedforward de friccion del URDF)
//      Fv, Fc : <dynamics damping/friction> de joint1..4, leidos del modelo
//               Pinocchio — los mismos valores que simula Gazebo con
//               damping_scale = 1.0 y friction_scale = 1.0.
//      El Coulomb usa la velocidad DESEADA dq_d (senal limpia): con dq medida
//      el tanh conmutaria con el ruido cerca de velocidad cero.
//
//  Funciones de conmutacion (aplicadas elemento a elemento):
//    "sign"  ->  rho(s) = sign(s)
//    "sat"   ->  rho(s) = sat(s / phi)        phi: capa limite [rad/s]
//
//  Lazo de control: 200 Hz (Ts = 5 ms).
//
//  Suscriptor : /joint_states                   (sensor_msgs/JointState)
//  Publicador : /arm_effort_controller/commands  (std_msgs/Float64MultiArray)
//
//  Parametros ROS 2 (--ros-args -p nombre:=valor):
//    test_num  [int]     1       — identificador del CSV generado
//    t_sim     [double]  0.0     — duracion en segundos (0 = ilimitado)
//    rho_func  [string]  "sign"  — funcion de conmutacion: "sign" | "sat"
//    phi       [double]  0.1     — capa limite para sat(s/phi)  [rad/s]
//
//  CSV generado: data/lab6/sim/act1/gz_smc_joint_<rho_func>_<test_num>.csv
//  Columnas: t, q1..q4, dq1..dq4, q1_des..q4_des, dq1_des..dq4_des,
//            s1..s4, tau1..tau4
//  (Sat% se calcula en MATLAB desde tau con el criterio >= 0.99*tau_max.)
//
//  Nota: usar sim_init_config.yaml con use_fixed_init: true,
//        q_init: [0.0, -0.45, 0.35, 0.785] (= q_d(0)) y escalas nominales
//        mass_inertia/damping/friction = 1.0.
//
//  Ejemplos de uso:
//
//    ros2 run open_manipulator_x_torque_control gz_smc_joint_node_base --ros-args -p rho_func:=sign -p test_num:=1 -p t_sim:=20.0
//
//    ros2 run open_manipulator_x_torque_control gz_smc_joint_node_base --ros-args -p rho_func:=sat -p phi:=0.1 -p test_num:=2 -p t_sim:=20.0
//
//  ──────────────────────────────────────────────────────────────────────────
//  SECCIONES A COMPLETAR:
//    [1] Trayectoria articular de referencia  →  desiredTrajectory()
//    [2] Funciones de conmutacion             →  rho_scalar()
//    [3] Ganancias SMC                        →  LAMBDA_Q, K_V, K_S
//    [4] Ley SMC articular                    →  tick()
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
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

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
// Suavizado del tanh de Coulomb del feedforward de friccion (no modificar).
// Se alimenta con la velocidad DESEADA (sin ruido): eps pequeno es seguro.
static constexpr double FRIC_EPS = 0.05;  // [rad/s]

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 3] GANANCIAS SMC — COMPLETAR
//  Ajustar los valores para cada articulacion [joint1, joint2, joint3, joint4]
//
//  Rangos recomendados (lazo discreto a 200 Hz, Ts = 5 ms):
//    LAMBDA_Q : 5.0 – 15.0   [1/s]     (ancho de banda de la superficie)
//    K_V      : 5.0 – 15.0             (alcance exponencial; mayor = convergencia mas rapida)
//    K_S      : 1.0 – 10.0   [rad/s²]  (ganancia de conmutacion)
//
//  Criterio: K_S solo debe dominar la INCERTIDUMBRE acotada (±20% de masa,
//  carga de 100 g), no la dinamica completa. Un K_S grande produce un ciclo
//  limite de chattering de amplitud |s| ~ K_S*Ts que nunca entra en la capa
//  limite (sat(s/phi) degenera en sign(s)). Ganancia efectiva dentro de la
//  capa: K_V + K_S/phi <= (0.2~0.3)/Ts.
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d LAMBDA_Q = {0.0, 0.0, 0.0, 0.0};  // COMPLETAR
static const Eigen::Vector4d K_V      = {0.0, 0.0, 0.0, 0.0};  // COMPLETAR
static const Eigen::Vector4d K_S      = {0.0, 0.0, 0.0, 0.0};  // COMPLETAR
// ═══════════════════════════════════════════════════════════════════════════

// ── Estructura de referencia articular ───────────────────────────────────────
struct Reference {
  Eigen::Vector4d q;    // posicion deseada   [rad]
  Eigen::Vector4d dq;   // velocidad deseada  [rad/s]
  Eigen::Vector4d ddq;  // aceleracion deseada [rad/s^2]
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

  (void)t;
  const double w = 1.0;  // [rad/s] — no modificar
  (void)w;

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

  return ref;
}

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 2] Funciones de conmutacion — COMPLETAR
//
//  Implementar rho(s) para cada tipo:
//    sign : rho(s) = +1 si s > 0,  -1 si s < 0,  0 si s == 0
//    sat  : rho(s) = clamp(s / phi, -1, 1)        (phi: capa limite)
// ─────────────────────────────────────────────────────────────────────────────
enum class RhoFunc { SIGN, SAT };

static double rho_scalar(double s, RhoFunc func, double phi)
{
  (void)s; (void)phi;

  switch (func) {
    case RhoFunc::SIGN:
      return 0.0;  // COMPLETAR: sign(s)
    case RhoFunc::SAT:
      return 0.0;  // COMPLETAR: sat(s/phi) = clamp(s/phi, -1, 1)
    default:
      return 0.0;
  }
}

// Aplica rho_scalar elemento a elemento sobre el vector s (no modificar)
[[maybe_unused]] static Eigen::Vector4d rho_vec(const Eigen::Vector4d & s,
                                RhoFunc func, double phi)
{
  Eigen::Vector4d r;
  for (int i = 0; i < NARM; ++i) {
    r[i] = rho_scalar(s[i], func, phi);
  }
  return r;
}

// ── Nodo principal ────────────────────────────────────────────────────────────
class SMCJointSimNode : public rclcpp::Node
{
public:
  SMCJointSimNode()
  : Node("gz_smc_joint_node"), t_(0.0)
  {
    // ── Parametros ────────────────────────────────────────────────────────────
    this->declare_parameter<int>        ("test_num", 1);
    this->declare_parameter<double>     ("t_sim",    0.0);
    this->declare_parameter<std::string>("rho_func", "sign");
    this->declare_parameter<double>     ("phi",      0.1);

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

    // Friccion articular del URDF (<dynamics damping/friction> joint1..4):
    // feedforward de viscosa + Coulomb, mismos valores que simula Gazebo
    // con damping_scale = 1.0 y friction_scale = 1.0. (No modificar)
    fric_damping_ = model_.damping.head<NARM>();
    fric_coulomb_ = model_.friction.head<NARM>();

    RCLCPP_INFO(this->get_logger(),
      "SMC articular — rho=%s  phi=%.3f  tau_max=%.2f N·m",
      rho_str_.c_str(), phi_, TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "Lambda=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]",
      LAMBDA_Q[0], LAMBDA_Q[1], LAMBDA_Q[2], LAMBDA_Q[3],
      K_V[0],      K_V[1],      K_V[2],      K_V[3],
      K_S[0],      K_S[1],      K_S[2],      K_S[3]);
    RCLCPP_INFO(this->get_logger(),
      "Friccion URDF — Fv=[%.4f %.4f %.4f %.4f] N·m·s/rad  "
      "Fc=[%.4f %.4f %.4f %.4f] N·m  (eps=%.2f)",
      fric_damping_[0], fric_damping_[1], fric_damping_[2], fric_damping_[3],
      fric_coulomb_[0], fric_coulomb_[1], fric_coulomb_[2], fric_coulomb_[3],
      FRIC_EPS);
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

    timer_ = this->create_wall_timer(5ms, [this]() { tick(); });
  }

  ~SMCJointSimNode()
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
      std::string(PACKAGE_DATA_DIR) + "/lab6/sim/act1");

    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab6/sim/act1/gz_smc_joint_"
                + rho_str_ + "_" + std::to_string(test_num) + ".csv";
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
         << "tau1,tau2,tau3,tau4\n";
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

    // Leer estados articulares
    Eigen::Vector4d q, dq;
    read_js(q, dq);

    // Trayectoria de referencia en el instante t_
    const Reference ref = desiredTrajectory(t_);

    // ── Dinamica nominal via Pinocchio ─────────────────────────────────────
    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>()  = q;
    dq_pin.head<NARM>() = dq;

    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();
    const Eigen::Matrix4d M = data_.M.topLeftCorner<NARM, NARM>();

    pinocchio::nonLinearEffects(model_, data_, q_pin, dq_pin);
    const Eigen::Vector4d phi_nle = data_.nle.head<NARM>();

    // Feedforward de friccion del URDF (infraestructura, no modificar):
    // viscosa con dq medida + Coulomb con la velocidad DESEADA ref.dq
    Eigen::Vector4d tau_fric;
    for (int i = 0; i < NARM; ++i) {
      tau_fric[i] = fric_damping_[i] * dq[i]
                  + fric_coulomb_[i] * std::tanh(ref.dq[i] / FRIC_EPS);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  [SECCION 4] Ley SMC articular — COMPLETAR
    //  Disponible: M (4x4), phi_nle (4x1), tau_fric (4x1), ref.q, ref.dq, ref.ddq
    // ─────────────────────────────────────────────────────────────────────────

    //  4.1 Errores articulares:
    const Eigen::Vector4d e_q  = Eigen::Vector4d::Zero();
    const Eigen::Vector4d e_dq = Eigen::Vector4d::Zero();

    // 4.2 Superficie deslizante:
    // COMPLETAR
    const Eigen::Vector4d s_q = Eigen::Vector4d::Zero();

    // 4.3 Funcion de conmutacion: rho = rho_vec(s_q, ...)
    // COMPLETAR
    const Eigen::Vector4d rho = Eigen::Vector4d::Zero();

    // 4.4 Aceleracion articular virtual:
    // COMPLETAR
    const Eigen::Vector4d v_q = Eigen::Vector4d::Zero();

    // 4.5 Torque:
    const Eigen::Vector4d tau     = Eigen::Vector4d::Zero();
    const Eigen::Vector4d tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // Suprimir warnings mientras la seccion esta incompleta (eliminar al completarla):
    (void)M; (void)phi_nle; (void)tau_fric; (void)e_dq; (void)rho; (void)v_q;
    // ─────────────────────────────────────────────────────────────────────────

    // ── Publicar torques ───────────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "t=%.2fs  |s|=%.4f  |e|=%.4f rad  tau=[%.3f %.3f %.3f %.3f] N·m",
      t_, s_q.norm(), e_q.norm(),
      tau_sat[0], tau_sat[1], tau_sat[2], tau_sat[3]);

    // ── Registro CSV ───────────────────────────────────────────────────────
    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t_
           << ',' << q[0]       << ',' << q[1]       << ',' << q[2]       << ',' << q[3]
           << ',' << dq[0]      << ',' << dq[1]      << ',' << dq[2]      << ',' << dq[3]
           << ',' << ref.q[0]   << ',' << ref.q[1]   << ',' << ref.q[2]   << ',' << ref.q[3]
           << ',' << ref.dq[0]  << ',' << ref.dq[1]  << ',' << ref.dq[2]  << ',' << ref.dq[3]
           << ',' << s_q[0]     << ',' << s_q[1]     << ',' << s_q[2]     << ',' << s_q[3]
           << ',' << tau_sat[0] << ',' << tau_sat[1] << ',' << tau_sat[2] << ',' << tau_sat[3]
           << '\n';
    }

    t_ += 0.005;

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

  Eigen::Vector4d fric_damping_;   // Fv del URDF [N·m·s/rad]
  Eigen::Vector4d fric_coulomb_;   // Fc del URDF [N·m]

  double      t_;
  double      t_sim_;
  RhoFunc     rho_func_;
  std::string rho_str_;
  double      phi_;

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
    rclcpp::spin(std::make_shared<SMCJointSimNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gz_smc_joint_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
