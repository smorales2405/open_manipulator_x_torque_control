// ============================================================================
//  gz_fl_control_node.cpp
//  Feedback Linearization / Computed Torque — OpenMANIPULATOR-X (simulacion Gazebo)
//
//  Ley de control:
//    v   = ddq_des - Kd*(dq - dq_des) - Kp*(q - q_des)
//    tau = M(q)*v + h(q, dq)           <- RNEA (Pinocchio)
//
//  Fases de referencia (identicas al nodo hw_fl_control_node):
//    [0, RAMP_TIME_S) — rampa quintica desde la pose inicial medida hasta
//                       el punto de la trayectoria en t = RAMP_TIME_S
//    [RAMP_TIME_S, ∞) — trayectoria sinusoidal articular (w = 1 rad/s):
//                      q1_des =  (pi/4)*sin(t)
//                      q2_des = -0.5 + 0.5*sin(t)
//                      q3_des =  0.3 - 0.5*sin(t)
//                      q4_des =  pi/4
//
//  Suscriptor: /joint_states           (sensor_msgs/JointState)
//  Publicador: /arm_effort_controller/commands  (std_msgs/Float64MultiArray)
//
//  Parametros ROS:
//    test_num      — identificador del CSV generado (gz_fl_data_<test_num>.csv)
//    t_sim         — duracion de la simulacion en segundos (0 = ilimitado)
//    friction_ffwd — bool (default false): feedforward de la friccion del URDF
//                    (damping·dq + friction·tanh(dq_des/eps)), replicando la
//                    compensacion del nodo hw. Pinocchio no incluye <dynamics>
//                    en M/nle, asi que sin esto la friccion de Gazebo queda
//                    sin compensar. Valido con damping/friction_scale = 1.0.
//
//  CSV generado: gz_fl_data_<test_num>.csv  en PACKAGE_DATA_DIR/lab4/sim/act1/
//
//  Ejemplo de ejecucion (con la simulacion Gazebo ya lanzada):
//    ros2 launch open_manipulator_x_torque_control torque_sim.launch.py
//    ros2 run open_manipulator_x_torque_control gz_fl_control_node --ros-args -p test_num:=1 -p t_sim:=20.0 -p friction_ffwd:=true
// ============================================================================

#include <chrono>
#include <cmath>
#include <ctime>
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

static constexpr double PI   = M_PI;
static constexpr int    NARM = 4;    // controlled joints (joint1..joint4)

// ═══════════════════════════════════════════════════════════════════════════
//  GANANCIAS DEL CONTROLADOR  — editar aqui para cada articulacion
//  Indice:  [joint1, joint2, joint3, joint4]
// ═══════════════════════════════════════════════════════════════════════════
// Alineadas con hw_fl_control_node (kd = 1.4·sqrt(kp), zeta = 0.7): mismo
// controlador en sim y en el robot real para que los CSV sean comparables.
// kp4 alto porque la rigidez de feedback es M44·kp4 y M44 ≈ 0.001 kg·m².
static const Eigen::Vector4d KP = {400.0, 400.0, 600.0, 3000.0};
static const Eigen::Vector4d KD = { 28.0,  28.0,  34.0,   77.0};
static constexpr double TAU_MAX = 1.2;    // [N·m] igual que tau_max del robot real
// ═══════════════════════════════════════════════════════════════════════════

// Duracion de la rampa quintica inicial [s] — igual que el nodo hw
static constexpr double RAMP_TIME_S = 3.0;

// Suavizado del tanh en el feedforward de Coulomb [rad/s] — igual que
// motor_epsilon_friction del nodo hw (seguro: usa la velocidad deseada)
static constexpr double FRIC_EPS = 0.05;

// ── Trajectory ──────────────────────────────────────────────────────────────
struct Reference {
  Eigen::Vector4d q, dq, ddq;
};

static Reference desiredTrajectory(double t)
{
  Reference ref;
  const double w = 1.0;

  ref.q <<
    (PI / 4.0) * std::sin(w * t),
    -0.5 + 0.5 * std::sin(w * t),
     0.3 - 0.5 * std::sin(w * t),
     (PI/4.0);

  ref.dq <<
     (PI / 4.0) * w * std::cos(w * t),
     0.5 * w * std::cos(w * t),
    -0.5 * w * std::cos(w * t),
     0.0;

  ref.ddq <<
    -(PI / 4.0) * w * w * std::sin(w * t),
    -0.5 * w * w * std::sin(w * t),
     0.5 * w * w * std::sin(w * t),
     0.0;

  return ref;
}

// ── Rampa quintica inicial (igual que el nodo hw) ───────────────────────────
// Lleva suavemente desde q0 hasta el punto de la trayectoria en t = T, con
// velocidad y aceleracion continuas en el empalme.
static Reference quinticTransition(double t, double T, const Eigen::Vector4d & q0)
{
  const Reference target = desiredTrajectory(T);
  if (T <= 0.0) {return target;}

  const Eigen::Vector4d v0 = Eigen::Vector4d::Zero();
  const Eigen::Vector4d a0 = Eigen::Vector4d::Zero();
  const Eigen::Vector4d qf = target.q, vf = target.dq, af = target.ddq;

  const double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;
  const Eigen::Vector4d c0 = q0;
  const Eigen::Vector4d c1 = v0;
  const Eigen::Vector4d c2 = 0.5 * a0;
  const Eigen::Vector4d c3 = (20.0*(qf-q0) - (8.0*vf+12.0*v0)*T - (3.0*a0-af)*T2) / (2.0*T3);
  const Eigen::Vector4d c4 = (30.0*(q0-qf) + (14.0*vf+16.0*v0)*T + (3.0*a0-2.0*af)*T2) / (2.0*T4);
  const Eigen::Vector4d c5 = (12.0*(qf-q0) - (6.0*vf+6.0*v0)*T - (a0-af)*T2) / (2.0*T5);

  const double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
  Reference ref;
  ref.q   = c0 + c1*t  + c2*t2  + c3*t3  + c4*t4  + c5*t5;
  ref.dq  = c1 + 2.0*c2*t + 3.0*c3*t2 + 4.0*c4*t3 + 5.0*c5*t4;
  ref.ddq = 2.0*c2 + 6.0*c3*t + 12.0*c4*t2 + 20.0*c5*t3;
  return ref;
}

// ── Node ────────────────────────────────────────────────────────────────────
class FLControlNode : public rclcpp::Node
{
public:
  FLControlNode()
  : Node("gz_fl_control_node"), t_(0.0)
  {
    // Unico parametro en tiempo de ejecucion: numero de prueba
    // Uso: ros2 run ... fl_control_node --ros-args -p test_num:=3
    this->declare_parameter<int>("test_num", 1);
    const int test_num = this->get_parameter("test_num").as_int();

    this->declare_parameter<double>("t_sim", 0.0);   // 0 = sin limite
    t_sim_ = this->get_parameter("t_sim").as_double();

    this->declare_parameter<bool>("friction_ffwd", false);
    friction_ffwd_ = this->get_parameter("friction_ffwd").as_bool();

    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";

    // Load Pinocchio model
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    // Friccion articular del URDF (<dynamics damping/friction>), parseada por
    // Pinocchio pero NO incluida en M/nle: se usa para el feedforward opcional.
    fric_damping_ = model_.damping.head<NARM>();
    fric_coulomb_ = model_.friction.head<NARM>();

    RCLCPP_INFO(this->get_logger(), "Modelo cargado: nv=%d", model_.nv);
    if (friction_ffwd_) {
      RCLCPP_INFO(this->get_logger(),
        "Feedforward de friccion URDF ACTIVO: damping=[%.4f %.4f %.4f %.4f]  "
        "coulomb=[%.4f %.4f %.4f %.4f]",
        fric_damping_[0], fric_damping_[1], fric_damping_[2], fric_damping_[3],
        fric_coulomb_[0], fric_coulomb_[1], fric_coulomb_[2], fric_coulomb_[3]);
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Feedforward de friccion URDF desactivado (-p friction_ffwd:=true para activar).");
    }
    RCLCPP_INFO(this->get_logger(),
      "Kp=[%.1f %.1f %.1f %.1f]  Kd=[%.1f %.1f %.1f %.1f]",
      KP[0], KP[1], KP[2], KP[3],
      KD[0], KD[1], KD[2], KD[3]);
    if (t_sim_ > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Tiempo de simulacion: %.1f s", t_sim_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Tiempo de simulacion: ilimitado");
    }

    open_csv(test_num);

    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_effort_controller/commands", 10);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_js_ = msg;
      });

    // Lazo de control a 200 Hz (dt = 5 ms), igual que el nodo hw
    timer_ = this->create_wall_timer(5ms, [this]() { tick(); });
  }

  ~FLControlNode()
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
    std::filesystem::create_directories(std::string(PACKAGE_DATA_DIR) + "/lab4/sim/act1");
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab4/sim/act1/gz_fl_data_"
                + std::to_string(test_num) + ".csv";
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
         << "tau1,tau2,tau3,tau4\n";
    RCLCPP_INFO(this->get_logger(), "CSV: %s", csv_path_.c_str());
  }

  // ── Joint state reader (by name, order-independent) ───────────────────────
  void read_js(Eigen::Vector4d & q, Eigen::Vector4d & dq)
  {
    static const std::array<std::string, NARM> names = {
      "joint1", "joint2", "joint3", "joint4"
    };
    q.setZero();
    dq.setZero();
    if (!last_js_) {return;}
    const auto & js = *last_js_;
    for (int j = 0; j < NARM; ++j) {
      for (std::size_t i = 0; i < js.name.size(); ++i) {
        if (js.name[i] == names[j]) {
          if (i < js.position.size()) {q[j]  = js.position[i];}
          if (i < js.velocity.size()) {dq[j] = js.velocity[i];}
          break;
        }
      }
    }
  }

  // ── Control tick ──────────────────────────────────────────────────────────
  void tick()
  {
    if (!last_js_) {return;}

    Eigen::Vector4d q, dq;
    read_js(q, dq);

    // Captura de la pose inicial (primer tick con /joint_states valido)
    if (!q_initial_captured_) {
      q_initial_ = q;
      q_initial_captured_ = true;
      RCLCPP_INFO(this->get_logger(), "q_inicial=[%.3f %.3f %.3f %.3f] rad",
        q[0], q[1], q[2], q[3]);
    }

    // Referencia: rampa quintica inicial + trayectoria sinusoidal (igual que hw)
    const Reference ref = (t_ < RAMP_TIME_S)
      ? quinticTransition(t_, RAMP_TIME_S, q_initial_)
      : desiredTrajectory(t_);

    // Full Pinocchio state (nv=6); gripper joints locked at 0
    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>()  = q;
    dq_pin.head<NARM>() = dq;

    // Mass matrix (CRBA fills upper triangle only; symmetrize)
    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();

    // NLE = C(q,dq)*dq + g(q)
    pinocchio::nonLinearEffects(model_, data_, q_pin, dq_pin);

    const Eigen::Matrix4d M   = data_.M.topLeftCorner<NARM, NARM>();
    const Eigen::Vector4d nle = data_.nle.head<NARM>();

    // Feedback linearization:
    // tau = M(q) * (ddq_des + Kp*(q_des-q) + Kd*(dq_des-dq)) + NLE
    const Eigen::Vector4d e   = ref.q  - q;
    const Eigen::Vector4d de  = ref.dq - dq;
    const Eigen::Vector4d v   = ref.ddq
                                + KP.asDiagonal() * e
                                + KD.asDiagonal() * de;
    Eigen::Vector4d tau = M * v + nle;

    // Feedforward opcional de la friccion del URDF (la que Gazebo simula):
    // viscosa con la velocidad medida; Coulomb con la velocidad DESEADA
    // (senal limpia, mismo criterio que el nodo hw).
    if (friction_ffwd_) {
      for (int i = 0; i < NARM; ++i) {
        tau[i] += fric_damping_[i] * dq[i]
                + fric_coulomb_[i] * std::tanh(ref.dq[i] / FRIC_EPS);
      }
    }

    const Eigen::Vector4d tau_sat =
      tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "t=%.2fs  |e|=%.4f rad  tau=[%.3f %.3f %.3f %.3f] Nm",
      t_, e.norm(), tau_sat[0], tau_sat[1], tau_sat[2], tau_sat[3]);

    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t_
           << ',' << q[0]          << ',' << q[1]          << ',' << q[2]          << ',' << q[3]
           << ',' << dq[0]         << ',' << dq[1]         << ',' << dq[2]         << ',' << dq[3]
           << ',' << ref.q[0]      << ',' << ref.q[1]      << ',' << ref.q[2]      << ',' << ref.q[3]
           << ',' << ref.dq[0]     << ',' << ref.dq[1]     << ',' << ref.dq[2]     << ',' << ref.dq[3]
           << ',' << tau_sat[0]    << ',' << tau_sat[1]    << ',' << tau_sat[2]    << ',' << tau_sat[3]
           << '\n';
    }

    t_ += 0.005;   // Ts del lazo de 200 Hz

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

  // ── Members ───────────────────────────────────────────────────────────────
  pinocchio::Model  model_;
  pinocchio::Data   data_;
  double t_;
  double t_sim_;

  bool friction_ffwd_{false};
  Eigen::Vector4d fric_damping_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d fric_coulomb_{Eigen::Vector4d::Zero()};

  Eigen::Vector4d q_initial_;
  bool q_initial_captured_{false};

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
  rclcpp::spin(std::make_shared<FLControlNode>());
  rclcpp::shutdown();
  return 0;
}
