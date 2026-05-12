#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

// Resolved at compile time via CMakeLists.txt target_compile_definitions.
#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif

// Nodo de prueba de control por esfuerzo.
//
// Modos (parametro ROS 'mode'):
//   zero  — todos los torques en 0  (robot cae por gravedad)
//   sine  — tau2 = 0.05*sin(2π·0.5·t)  Nm  (respuesta dinamica)
//
// Salidas:
//   /arm_effort_controller/commands  (Float64MultiArray) — [joint1..joint4]
//   data/torque_data_YYYYMMDD_HHMMSS.csv  — t, q1..q4, dq1..dq4, tau1..tau4

class TorqueTestNode : public rclcpp::Node
{
public:
  TorqueTestNode()
  : Node("torque_test_node"), t_(0.0)
  {
    this->declare_parameter<std::string>("mode", "zero");
    mode_ = this->get_parameter("mode").as_string();

    open_csv();

    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_effort_controller/commands", 10);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_js_ = msg;
      });

    timer_ = this->create_wall_timer(20ms, [this]() { tick(); });

    RCLCPP_INFO(this->get_logger(),
      "Iniciado en modo '%s'. Publicando en /arm_effort_controller/commands.",
      mode_.c_str());
  }

  ~TorqueTestNode()
  {
    if (csv_.is_open()) {
      csv_.close();
      RCLCPP_INFO(this->get_logger(), "CSV cerrado: %s", csv_path_.c_str());
    }
  }

private:
  void open_csv()
  {
    std::filesystem::create_directories(PACKAGE_DATA_DIR);

    auto now = std::chrono::system_clock::now();
    std::time_t t_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t_c, &tm);

    char fname[64];
    std::strftime(fname, sizeof(fname), "torque_data_%Y%m%d_%H%M%S.csv", &tm);
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/" + fname;

    csv_.open(csv_path_);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo crear: %s", csv_path_.c_str());
      return;
    }
    csv_ << "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,tau1,tau2,tau3,tau4\n";
    RCLCPP_INFO(this->get_logger(), "CSV: %s", csv_path_.c_str());
  }

  // Extrae q y dq para joint1..joint4 buscando por nombre (orden-independiente).
  void read_joint_states(std::array<double, 4> & q, std::array<double, 4> & dq)
  {
    static const std::array<std::string, 4> names = {
      "joint1", "joint2", "joint3", "joint4"
    };
    q.fill(0.0);
    dq.fill(0.0);
    if (!last_js_) {return;}
    const auto & js = *last_js_;
    for (std::size_t j = 0; j < 4; ++j) {
      for (std::size_t i = 0; i < js.name.size(); ++i) {
        if (js.name[i] == names[j]) {
          if (i < js.position.size()) {q[j]  = js.position[i];}
          if (i < js.velocity.size()) {dq[j] = js.velocity[i];}
          break;
        }
      }
    }
  }

  void tick()
  {
    std::array<double, 4> q{}, dq{};
    read_joint_states(q, dq);

    double tau1 = 0.0, tau2 = 0.0, tau3 = 0.0, tau4 = 0.0;
    if (mode_ == "sine") {
      tau2 = 1.5 * std::sin(2.0 * M_PI * 0.5 * t_);
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {tau1, tau2, tau3, tau4};
    torque_pub_->publish(msg);

    if (last_js_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "t=%.2fs  q=[%.3f %.3f %.3f %.3f] rad  tau=[%.4f %.4f %.4f %.4f] Nm",
        t_, q[0], q[1], q[2], q[3], tau1, tau2, tau3, tau4);
    }

    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t_   << ','
           << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3] << ','
           << dq[0] << ',' << dq[1] << ',' << dq[2] << ',' << dq[3] << ','
           << tau1 << ',' << tau2 << ',' << tau3 << ',' << tau4 << '\n';
    }

    t_ += 0.02;
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState::SharedPtr last_js_;
  std::string mode_;
  double t_;
  std::ofstream csv_;
  std::string csv_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueTestNode>());
  rclcpp::shutdown();
  return 0;
}
