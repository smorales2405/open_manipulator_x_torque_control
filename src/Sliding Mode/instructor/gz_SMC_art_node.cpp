// ============================================================================
//  gz_SMC_art_node.cpp
//  Control por Modo Deslizante en espacio articular — OpenMANIPULATOR-X (Gazebo)
//
//  Modelo dinamico nominal (Guia Lab 6, Sec. 6.1):
//    M(q)*ddq + Phi(q,dq) = tau - tau_fric(dq)
//    tau_fric_i = B_FRIC * dq_i     (B_FRIC = 0.001 N·m·s/rad)
//
//  Superficie deslizante articular:
//    e_q  = q  - q_d               (error de posicion)
//    e_dq = dq - dq_d              (error de velocidad)
//    s_q  = e_dq + Lambda_q * e_q
//
//  Ley de control SMC:
//    v_q  = ddq_d - Lambda_q*e_dq - K_q*rho(s_q)
//    tau  = M(q)*v_q + Phi(q,dq) + tau_fric(dq)
//    tau_sat = clamp(tau, -TAU_MAX, TAU_MAX)
//
//  Funciones de conmutacion (aplicadas elemento a elemento):
//    "sign"  ->  rho(s) = sign(s)
//    "sat"   ->  rho(s) = sat(s / phi)        phi: capa limite [rad/s]
//    "tanh"  ->  rho(s) = tanh(alpha * s)
//
//  Trayectoria articular de referencia (omega = 1.0 rad/s):
//    q_d   = [ (pi/4)*sin(w*t),  -0.5+0.5*sin(w*t),  0.3-0.5*sin(w*t),  pi/4 ]
//    dq_d  = [ (pi/4)w*cos(w*t), 0.5w*cos(w*t),      -0.5w*cos(w*t),      0   ]
//    ddq_d = [-(pi/4)w²*sin(w*t),-0.5w²*sin(w*t),    0.5w²*sin(w*t),       0   ]
//
//  Suscriptor : /joint_states                   (sensor_msgs/JointState)
//  Publicador : /arm_effort_controller/commands  (std_msgs/Float64MultiArray)
//
//  Parametros ROS 2 (--ros-args -p nombre:=valor):
//    test_num  [int]     1       — identificador del CSV generado
//    t_sim     [double]  0.0     — duracion en segundos (0 = ilimitado)
//    rho_func  [string]  "sign"  — funcion de conmutacion: "sign" | "sat" | "tanh"
//    phi       [double]  0.05    — capa limite para sat(s/phi)  [rad/s]
//    alpha     [double]  50.0    — pendiente para tanh(alpha*s)
//
//  CSV generado: data/lab6/sim/act1/gz_smc_art_<rho_func>_<test_num>.csv
//  Columnas: t, q1..q4, dq1..dq4, q1_des..q4_des, dq1_des..dq4_des,
//            s1..s4, tau1..tau4, sat1..sat4
//
//  Nota: usar sim_init_config.yaml con use_fixed_init: true
//        y q_init: [0.0, -0.5, 0.3, 0.785] para arrancar en q_d(0).
//
//  Ejemplos de uso:
//
//    # Funcion signo (chattering maximo, referencia de comparacion):
//    ros2 run open_manipulator_x_torque_control gz_smc_art_node
//      --ros-args -p rho_func:=sign -p test_num:=1 -p t_sim:=30.0
//
//    # Saturacion con capa limite phi = 0.05 rad/s:
//    ros2 run open_manipulator_x_torque_control gz_smc_art_node
//      --ros-args -p rho_func:=sat -p phi:=0.05 -p test_num:=2 -p t_sim:=30.0
//
//    # Tangente hiperbolica con alpha = 50:
//    ros2 run open_manipulator_x_torque_control gz_smc_art_node
//      --ros-args -p rho_func:=tanh -p alpha:=50.0 -p test_num:=3 -p t_sim:=30.0
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
static constexpr double B_FRIC  = 0.001;  // [N·m·s/rad] friccion viscosa nominal

// ═══════════════════════════════════════════════════════════════════════════
//  GANANCIAS SMC — ajustar aqui para cada articulacion
//  Indice: [joint1, joint2, joint3, joint4]
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d LAMBDA_Q = {5.0, 5.0, 5.0, 100.0};  // superficie [1/s]
static const Eigen::Vector4d K_Q      = {50.0, 30.0, 50.0, 500.0};  // ganancia conmutacion
// ═══════════════════════════════════════════════════════════════════════════

// ── Trayectoria de referencia articular ──────────────────────────────────────
struct Reference {
  Eigen::Vector4d q, dq, ddq;
};

static Reference desiredTrajectory(double t)
{
  Reference ref;
  const double w = 1.0;  // [rad/s] 

  ref.q <<
       0.35 * std::sin(w * t),
      -0.45 + 0.25 * std::sin(w * t),
       0.35 - 0.25 * std::sin(w * t),
      (PI/4.0) + 0.4*std::sin(w * t);
       
  ref.dq <<
       0.35 * w * std::cos(w * t),
       0.25 * w * std::cos(w * t),
      -0.25 * w * std::cos(w * t),
      0.4 * w * std::cos(w * t);

  ref.ddq <<
      -0.35 * w * w * std::sin(w * t),
      -0.25 * w * w * std::sin(w * t),
       0.25 * w * w * std::sin(w * t),
      -0.4 * w * w * std::sin(w * t);

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
class SMCJointSimNode : public rclcpp::Node
{
public:
  SMCJointSimNode()
  : Node("gz_smc_art_node"), t_(0.0)
  {
    // ── Parametros ────────────────────────────────────────────────────────────
    this->declare_parameter<int>        ("test_num", 1);
    this->declare_parameter<double>     ("t_sim",    0.0);
    this->declare_parameter<std::string>("rho_func", "sign");
    this->declare_parameter<double>     ("phi",      0.05);
    this->declare_parameter<double>     ("alpha",    50.0);

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

    RCLCPP_INFO(this->get_logger(),
      "SMC articular — rho=%s  phi=%.3f  alpha=%.1f  tau_max=%.2f N·m",
      rho_str_.c_str(), phi_, alpha_, TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "Lambda=[%.1f %.1f %.1f %.1f]  K=[%.1f %.1f %.1f %.1f]",
      LAMBDA_Q[0], LAMBDA_Q[1], LAMBDA_Q[2], LAMBDA_Q[3],
      K_Q[0],      K_Q[1],      K_Q[2],      K_Q[3]);
    RCLCPP_INFO(this->get_logger(),
      "B_fric=%.4f N·m·s/rad  (modelo nominal)",  B_FRIC);
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

    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab6/sim/act1/gz_smc_art_"
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
         << "tau1,tau2,tau3,tau4,"
         << "sat1,sat2,sat3,sat4\n";
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

    Eigen::Vector4d q, dq;
    read_js(q, dq);

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

    const Eigen::Vector4d tau_fric = B_FRIC * dq;

    // ── Ley SMC articular ──────────────────────────────────────────────────
    const Eigen::Vector4d e_q  = q  - ref.q;
    const Eigen::Vector4d e_dq = dq - ref.dq;

    const Eigen::Vector4d s_q = e_dq + LAMBDA_Q.asDiagonal() * e_q;

    const Eigen::Vector4d rho  = rho_vec(s_q, rho_func_, phi_, alpha_);
    const Eigen::Vector4d v_q  = ref.ddq
                                - LAMBDA_Q.asDiagonal() * e_dq
                                - K_Q.asDiagonal() * rho;

    const Eigen::Vector4d tau     = M * v_q + phi_nle + tau_fric;
    const Eigen::Vector4d tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

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
  pinocchio::Model model_;
  pinocchio::Data  data_;

  double      t_;
  double      t_sim_;
  RhoFunc     rho_func_;
  std::string rho_str_;
  double      phi_;
  double      alpha_;

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
    RCLCPP_FATAL(rclcpp::get_logger("gz_smc_art_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
