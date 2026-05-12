#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

// Nodo minimo de prueba de control por esfuerzo.
//
// Por defecto publica torque cero en las 4 articulaciones.
// Para verificar respuesta dinamica activa el modo sinusoidal:
//   ros2 run open_manipulator_x_torque_control torque_test_node \
//     --ros-args -p mode:=sine
//
// Topico de salida : /arm_effort_controller/commands  (Float64MultiArray)
// Topico de entrada: /joint_states                    (JointState)
// Orden del vector : [joint1, joint2, joint3, joint4]

class TorqueTestNode : public rclcpp::Node
{
public:
  TorqueTestNode()
  : Node("torque_test_node"), t_(0.0)
  {
    this->declare_parameter<std::string>("mode", "zero");
    mode_ = this->get_parameter("mode").as_string();

    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_effort_controller/commands", 10);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_joint_state_ = msg;
      });

    // 50 Hz de publicacion
    timer_ = this->create_wall_timer(20ms, [this]() { tick(); });

    RCLCPP_INFO(this->get_logger(),
      "Iniciado en modo '%s'. Publicando en /arm_effort_controller/commands",
      mode_.c_str());
  }

private:
  void tick()
  {
    auto msg = std_msgs::msg::Float64MultiArray();

    double tau1 = 0.0;
    double tau2 = 0.0;
    double tau3 = 0.0;
    double tau4 = 0.0;

    if (mode_ == "sine") {
      // Torque sinusoidal pequeno en joint2 para verificar respuesta dinamica.
      // Amplitud 0.05 Nm (limite URDF: 1 Nm). Frecuencia: 0.5 Hz.
      tau2 = 0.05 * std::sin(2.0 * M_PI * 0.5 * t_);
    }
    // modo "zero": todos los torques son 0.0 (robot cae por gravedad en Gazebo)

    msg.data = {tau1, tau2, tau3, tau4};
    torque_pub_->publish(msg);

    // Log a 1 Hz
    if (last_joint_state_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "t=%.2fs  tau=[%.4f, %.4f, %.4f, %.4f] Nm",
        t_, tau1, tau2, tau3, tau4);
    }

    t_ += 0.02;  // dt = 20 ms
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
  std::string mode_;
  double t_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueTestNode>());
  rclcpp::shutdown();
  return 0;
}
