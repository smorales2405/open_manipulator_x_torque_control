// ============================================================================
//  gz_fl_control_node.cpp
//  Feedback Linearization / Computed Torque — OpenMANIPULATOR-X (simulacion Gazebo)
//
//  Ley de control:
//    v   = ddq_des - Kd*(dq - dq_des) - Kp*(q - q_des)
//    tau = M(q)*v + h(q, dq)           <- RNEA (Pinocchio)
//
//  Fases de referencia:
//    [0, T_TRANS)  — transicion suave con polinomio de 5to orden
//                    desde la posicion inicial medida hasta el inicio de la trayectoria
//    [T_TRANS, ∞)  — trayectoria sinusoidal articular (w = 1 rad/s):
//                      q1_des =  (pi/4)*sin(t')
//                      q2_des = -0.5 + 0.5*sin(t')
//                      q3_des =  0.3 - 0.5*sin(t')
//                      q4_des =  pi/4
//
//  Suscriptor: /joint_states           (sensor_msgs/JointState)
//  Publicador: /arm_effort_controller/commands  (std_msgs/Float64MultiArray)
//
//  Parametros ROS:
//    test_num — identificador del CSV generado (gz_fl_data_<test_num>.csv)
//    t_sim    — duracion de la simulacion en segundos (0 = ilimitado)
//
//  CSV generado: gz_fl_data_<test_num>.csv  en PACKAGE_DATA_DIR/sim/act1/
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
static const Eigen::Vector4d KP = {200.0, 200.0, 200.0, 200.0};
static const Eigen::Vector4d KD = { 20.0,  20.0,  20.0,  20.0};
static constexpr double TAU_MAX = 0.82;   // [N·m] limite por articulacion
// ═══════════════════════════════════════════════════════════════════════════

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

    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/openmani.urdf";

    // Load Pinocchio model
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    RCLCPP_INFO(this->get_logger(), "Modelo cargado: nv=%d", model_.nv);
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

    // 100 Hz control loop (dt = 10 ms)
    timer_ = this->create_wall_timer(10ms, [this]() { tick(); });
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

    const Reference ref = desiredTrajectory(t_);

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
    const Eigen::Vector4d tau = M * v + nle;
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

  // ── Members ───────────────────────────────────────────────────────────────
  pinocchio::Model  model_;
  pinocchio::Data   data_;
  double t_;
  double t_sim_;

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
