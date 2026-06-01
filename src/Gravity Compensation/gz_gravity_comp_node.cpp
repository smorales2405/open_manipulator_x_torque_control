// ============================================================================
//  gz_gravity_comp_node.cpp
//  Compensacion gravitacional — OpenMANIPULATOR-X en Gazebo Fortress.
//
//  Ley de control:
//    tau = gravity_scale * G(q) + Kp*(q_des - q) + Kd*(0 - dq)
//
//  El termino G(q) compensa la gravedad en la posicion actual.
//  El termino PD es necesario en simulacion para dos razones:
//    1. El robot cae ~0.15 s antes de que el controlador de esfuerzo
//       reciba el primer comando — el PD recupera la posicion q_des.
//    2. Mantiene q_des con precision frente a errores del modelo URDF.
//
//  Con Kp y Kd pequeños (default: 30, 3), el robot se mantiene en q_des
//  de forma suave y puede moverse manualmente con resistencia moderada.
//
//  Suscriptor: /joint_states                    (sensor_msgs/JointState)
//  Publicador: /arm_effort_controller/commands  (std_msgs/Float64MultiArray)
//
//  Parametros ROS 2:
//    gravity_scale  [double]  1.0   (ajuste fino del modelo de gravedad)
//    Kp             [double]  30.0  (ganancia proporcional PD de recuperacion)
//    Kd             [double]   3.0  (ganancia derivativa PD de recuperacion)
//    test_num       [int]      1    (CSV: gz_gc_data_<test_num>.csv)
//    t_sim          [double]   0.0  (duracion en s; 0 = ilimitado)
//
//  CSV: data/gravity_comp/gz_gc_data_<test_num>.csv
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

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

using namespace std::chrono_literals;

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif

static constexpr int    NARM    = 4;
static constexpr double TAU_MAX = 1.5;   // [N·m] — limite por articulacion (effort="1.5")

// ============================================================================
//  Nodo ROS 2
// ============================================================================

class GZGravityCompNode : public rclcpp::Node
{
public:
  GZGravityCompNode()
  : Node("gz_gravity_comp_node")
  {
    // ── Parametros ───────────────────────────────────────────────────────────
    this->declare_parameter<double>("gravity_scale", 1.0);
    this->declare_parameter<double>("Kp",             3.0);
    this->declare_parameter<double>("Kd",             0.5);
    this->declare_parameter<int>   ("test_num",       1);
    this->declare_parameter<double>("t_sim",          0.0);

    gravity_scale_ = this->get_parameter("gravity_scale").as_double();
    Kp_            = this->get_parameter("Kp").as_double();
    Kd_            = this->get_parameter("Kd").as_double();
    const int test_num = this->get_parameter("test_num").as_int();
    t_sim_         = this->get_parameter("t_sim").as_double();

    RCLCPP_INFO(get_logger(),
      "gravity_scale=%.2f  Kp=%.1f  Kd=%.1f  test_num=%d  t_sim=%.1fs",
      gravity_scale_, Kp_, Kd_, test_num, t_sim_);
    RCLCPP_INFO(get_logger(),
      "Referencia: q_des = [0, 0, 0, 0] rad");

    // ── Pinocchio ─────────────────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(get_logger(), "Modelo Pinocchio cargado: nv=%d", model_.nv);

    // ── CSV ───────────────────────────────────────────────────────────────────
    const std::string csv_dir = std::string(PACKAGE_DATA_DIR) + "/gravity_comp";
    std::filesystem::create_directories(csv_dir);
    csv_path_ = csv_dir + "/gz_gc_data_" + std::to_string(test_num) + ".csv";
    csv_.open(csv_path_);
    if (csv_.is_open()) {
      csv_ << "t,"
              "q1,q2,q3,q4,"
              "dq1,dq2,dq3,dq4,"
              "tau_g1,tau_g2,tau_g3,tau_g4,"
              "tau_pd1,tau_pd2,tau_pd3,tau_pd4,"
              "tau1,tau2,tau3,tau4\n";
      RCLCPP_INFO(get_logger(), "CSV: %s", csv_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "No se pudo crear CSV: %s", csv_path_.c_str());
    }

    // ── Comunicaciones ────────────────────────────────────────────────────────
    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_effort_controller/commands", 10);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_js_ = msg;
      });

    // ── Timer 100 Hz ──────────────────────────────────────────────────────────
    start_time_ = std::chrono::high_resolution_clock::now();
    timer_ = this->create_wall_timer(10ms, [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
      "Nodo activo. Esperando /joint_states de Gazebo...");
  }

  ~GZGravityCompNode()
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
      RCLCPP_INFO(get_logger(), "CSV cerrado: %s", csv_path_.c_str());
    }
  }

private:
  // ── Lectura de /joint_states por nombre (orden-independiente) ────────────────

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

  // ── G(q) = RNEA(q, 0, 0) — torque de gravedad puro ──────────────────────────

  Eigen::Vector4d compute_gravity(const Eigen::Vector4d & q)
  {
    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd zero_v = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>() = q;
    const Eigen::VectorXd tau_full =
      pinocchio::rnea(model_, data_, q_pin, zero_v, zero_v);
    return gravity_scale_ * tau_full.head<NARM>();
  }

  // ── Tick de control (100 Hz) ──────────────────────────────────────────────────

  void tick()
  {
    if (!last_js_) { return; }

    const auto tp = std::chrono::high_resolution_clock::now();
    const double t = std::chrono::duration<double>(tp - start_time_).count();

    if (t_sim_ > 0.0 && t >= t_sim_) {
      RCLCPP_INFO(get_logger(), "Simulacion completada (%.1f s).", t_sim_);
      std_msgs::msg::Float64MultiArray zero;
      zero.data.assign(NARM, 0.0);
      torque_pub_->publish(zero);
      if (csv_.is_open()) { csv_.flush(); csv_.close(); }
      timer_->cancel();
      return;
    }

    // 1. Leer estado articular
    Eigen::Vector4d q, dq;
    read_js(q, dq);

    // 2. Termino de compensacion gravitacional: G(q)
    const Eigen::Vector4d tau_grav = compute_gravity(q);

    // 3. Termino PD alrededor de q_des = [0,0,0,0]:
    //    tau_pd = Kp*(q_des - q) + Kd*(0 - dq)
    //    Recupera el robot de la caida inicial y lo mantiene en q_des.
    const Eigen::Vector4d e      = -q;    // q_des = 0 → e = 0 - q
    const Eigen::Vector4d tau_pd = Kp_ * e + Kd_ * (-dq);

    // 4. Torque total saturado al limite del URDF
    Eigen::Vector4d tau = tau_grav + tau_pd;
    tau = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // 5. Publicar
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau.data(), tau.data() + NARM);
    torque_pub_->publish(cmd);

    // 6. CSV
    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t
           << ',' << q[0]         << ',' << q[1]         << ',' << q[2]         << ',' << q[3]
           << ',' << dq[0]        << ',' << dq[1]        << ',' << dq[2]        << ',' << dq[3]
           << ',' << tau_grav[0]  << ',' << tau_grav[1]  << ',' << tau_grav[2]  << ',' << tau_grav[3]
           << ',' << tau_pd[0]    << ',' << tau_pd[1]    << ',' << tau_pd[2]    << ',' << tau_pd[3]
           << ',' << tau[0]       << ',' << tau[1]       << ',' << tau[2]       << ',' << tau[3]
           << '\n';
    }

    // 7. Log periodico (1 Hz)
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
      "t=%.1fs  q=[%.3f %.3f %.3f %.3f]  "
      "tau_g=[%.3f %.3f %.3f %.3f]  "
      "tau_pd=[%.3f %.3f %.3f %.3f]",
      t,
      q[0], q[1], q[2], q[3],
      tau_grav[0], tau_grav[1], tau_grav[2], tau_grav[3],
      tau_pd[0], tau_pd[1], tau_pd[2], tau_pd[3]);
  }

  // ── Miembros ─────────────────────────────────────────────────────────────────

  pinocchio::Model model_;
  pinocchio::Data  data_;

  double gravity_scale_;
  double Kp_, Kd_;
  double t_sim_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState::SharedPtr last_js_;

  std::chrono::high_resolution_clock::time_point start_time_;

  std::ofstream csv_;
  std::string   csv_path_;
};

// ============================================================================
//  main
// ============================================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GZGravityCompNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gz_gravity_comp_node"),
      "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
