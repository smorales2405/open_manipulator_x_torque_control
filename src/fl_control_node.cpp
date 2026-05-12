#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>

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

static constexpr double PI  = M_PI;
static constexpr int    NARM = 4;   // controlled joints (joint1..joint4)

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
     1.5;

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
  : Node("fl_control_node"), t_(0.0)
  {
    this->declare_parameter<double>("kp", 100.0);
    this->declare_parameter<double>("kd",  20.0);
    this->declare_parameter<std::string>("urdf_path",
      std::string(PACKAGE_URDF_DIR) + "/openmani.urdf");

    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();
    const std::string urdf = this->get_parameter("urdf_path").as_string();

    // Load Pinocchio model
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(this->get_logger(),
      "Modelo cargado: nv=%d  nq=%d", model_.nv, model_.nq);
    RCLCPP_INFO(this->get_logger(),
      "Kp=%.1f  Kd=%.1f", kp_, kd_);

    open_csv();

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
  void open_csv()
  {
    std::filesystem::create_directories(PACKAGE_DATA_DIR);
    auto now = std::chrono::system_clock::now();
    std::time_t t_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t_c, &tm);
    char fname[64];
    std::strftime(fname, sizeof(fname), "fl_data_%Y%m%d_%H%M%S.csv", &tm);
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/" + fname;
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
    // Wait for first joint state before starting
    if (!last_js_) {return;}

    // Read current state
    Eigen::Vector4d q, dq;
    read_js(q, dq);

    // Desired trajectory at current time
    const Reference ref = desiredTrajectory(t_);

    // Build full Pinocchio state (nv=6); gripper joints stay at 0
    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>()  = q;
    dq_pin.head<NARM>() = dq;

    // ── Dynamics (Pinocchio) ─────────────────────────────────────────────
    // Mass matrix — CRBA fills only upper triangle; symmetrize
    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();

    // Nonlinear effects: NLE = C(q,dq)*dq + g(q), stored in data_.nle
    pinocchio::nonLinearEffects(model_, data_, q_pin, dq_pin);

    // Extract arm sub-system (first NARM rows/columns)
    const Eigen::Matrix4d M   = data_.M.topLeftCorner<NARM, NARM>();
    const Eigen::Vector4d nle = data_.nle.head<NARM>();

    // ── Feedback Linearization ───────────────────────────────────────────
    // tau = M(q) * (ddq_des + Kp*(q_des - q) + Kd*(dq_des - dq)) + NLE
    const Eigen::Vector4d e   = ref.q  - q;
    const Eigen::Vector4d de  = ref.dq - dq;
    const Eigen::Vector4d v   = ref.ddq + kp_ * e + kd_ * de;
    const Eigen::Vector4d tau = M * v + nle;

    // Publish torque command
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau.data(), tau.data() + NARM);
    torque_pub_->publish(cmd);

    // Log at 1 Hz
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "t=%.2fs  |e|=%.4f rad  tau=[%.3f %.3f %.3f %.3f] Nm",
      t_, e.norm(), tau[0], tau[1], tau[2], tau[3]);

    // Write CSV row
    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t_
           << ',' << q[0]       << ',' << q[1]       << ',' << q[2]       << ',' << q[3]
           << ',' << dq[0]      << ',' << dq[1]      << ',' << dq[2]      << ',' << dq[3]
           << ',' << ref.q[0]   << ',' << ref.q[1]   << ',' << ref.q[2]   << ',' << ref.q[3]
           << ',' << ref.dq[0]  << ',' << ref.dq[1]  << ',' << ref.dq[2]  << ',' << ref.dq[3]
           << ',' << tau[0]     << ',' << tau[1]     << ',' << tau[2]     << ',' << tau[3]
           << '\n';
    }

    t_ += 0.01;   // dt = 10 ms
  }

  // ── Members ───────────────────────────────────────────────────────────────
  pinocchio::Model  model_;
  pinocchio::Data   data_;
  double kp_, kd_;
  double t_;

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
