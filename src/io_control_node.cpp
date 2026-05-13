// ============================================================================
//  io_control_node.cpp
//  Input-Output Linearization en espacio cartesiano — OpenMANIPULATOR-X
//
//  Salida de tarea:
//    y = [x, y, z, phi]^T
//    donde (x,y,z) es la posicion cartesiana del efector final y phi es el
//    angulo de inclinacion (pitch ZYX) del mismo.
//
//  Cinematica diferencial:
//    ydot  = J(q) * qdot
//    yddot = J(q) * qddot + Jdot(q,qdot) * qdot
//
//  Ley de control:
//    v   = yddot_des + Kp*(y_des - y) + Kd*(ydot_des - ydot)
//    tau = M(q) * J^{-1}(q) * (v - Jdot*qdot) + b(q,qdot)
//
//  Trayectoria cartesiana (frecuencia w = 2 rad/s):
//    x_des   = 0.08 + 0.05*sin(2t)   [m]
//    y_des   = 0.05*cos(2t)           [m]
//    z_des   = 0.10                   [m]
//    phi_des = pi/2                   [rad]
//
//  Parametro ROS (unico):
//    t_sim  — duracion de la simulacion en segundos (0 = ilimitado)
//
//  CSV generado: fl_xyz_data.csv  en PACKAGE_DATA_DIR
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
#include <Eigen/LU>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

using namespace std::chrono_literals;

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif

static constexpr double PI   = M_PI;
static constexpr int    NARM = 4;   // articulaciones controladas (joint1..joint4)

// Nombre del frame del efector final en el URDF
static constexpr char EFF_FRAME[] = "end_effector_link";

// ============================================================================
//  GANANCIAS DEL CONTROLADOR CARTESIANO  (editar aqui)
//  Indice: [x,  y,  z,  phi]
// ============================================================================
static const Eigen::Vector4d KP = {50.0, 50.0, 50.0, 50.0};
static const Eigen::Vector4d KD = {14.0, 14.0, 14.0, 14.0};
static constexpr double TAU_MAX  = 0.82;   // [N·m] limite de saturacion por articulacion
// ============================================================================

// ── Referencia cartesiana ────────────────────────────────────────────────────
struct CartRef {
  Eigen::Vector4d y;      // [x, y, z, phi]
  Eigen::Vector4d ydot;   // primera derivada analitica
  Eigen::Vector4d yddot;  // segunda derivada analitica
};

static CartRef desiredTrajectory(double t)
{
  const double w = 2.0;
  CartRef ref;

  ref.y <<
    0.08 + 0.05 * std::sin(w * t),
    0.05 * std::cos(w * t),
    0.10,
    PI / 2.0;

  ref.ydot <<
     0.05 * w * std::cos(w * t),
    -0.05 * w * std::sin(w * t),
     0.0,
     0.0;

  ref.yddot <<
    -0.05 * w * w * std::sin(w * t),
    -0.05 * w * w * std::cos(w * t),
     0.0,
     0.0;

  return ref;
}

// ── Nodo de control ──────────────────────────────────────────────────────────
class IOControlNode : public rclcpp::Node
{
public:
  IOControlNode()
  : Node("io_control_node"), t_(0.0)
  {
    // Parametro: duracion de simulacion en segundos (0 = sin limite)
    this->declare_parameter<double>("t_sim", 0.0);
    t_sim_ = this->get_parameter("t_sim").as_double();

    // Cargar modelo Pinocchio desde URDF
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/openmani.urdf";
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    // Verificar y obtener el ID del frame del efector final
    if (!model_.existFrame(EFF_FRAME)) {
      throw std::runtime_error(std::string("Frame no encontrado en el URDF: ") + EFF_FRAME);
    }
    frame_id_ = model_.getFrameId(EFF_FRAME);

    RCLCPP_INFO(this->get_logger(),
      "Modelo cargado: nv=%d  frame='%s' (id=%lu)",
      model_.nv, EFF_FRAME, frame_id_);
    RCLCPP_INFO(this->get_logger(),
      "Kp=[%.1f %.1f %.1f %.1f]  Kd=[%.1f %.1f %.1f %.1f]  tau_max=%.2f N·m",
      KP[0], KP[1], KP[2], KP[3], KD[0], KD[1], KD[2], KD[3], TAU_MAX);
    if (t_sim_ > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Tiempo de simulacion: %.1f s", t_sim_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Tiempo de simulacion: ilimitado");
    }

    open_csv();

    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_effort_controller/commands", 10);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_js_ = msg;
      });

    // Lazo de control a 100 Hz (dt = 10 ms)
    timer_ = this->create_wall_timer(10ms, [this]() { tick(); });
  }

  ~IOControlNode()
  {
    if (csv_.is_open()) {
      csv_.close();
      RCLCPP_INFO(this->get_logger(), "CSV cerrado: %s", csv_path_.c_str());
    }
  }

private:
  // ── Apertura del archivo CSV ──────────────────────────────────────────────
  void open_csv()
  {
    std::filesystem::create_directories(PACKAGE_DATA_DIR);
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/fl_xyz_data.csv";
    csv_.open(csv_path_);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo crear: %s", csv_path_.c_str());
      return;
    }
    csv_ << "t,"
         << "q1,q2,q3,q4,"
         << "x,y,z,phi,"
         << "x_des,y_des,z_des,phi_des,"
         << "tau1,tau2,tau3,tau4\n";
    RCLCPP_INFO(this->get_logger(), "CSV: %s", csv_path_.c_str());
  }

  // ── Lectura de estados articulares (busqueda por nombre) ──────────────────
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

    // 1. Estados articulares del brazo
    Eigen::Vector4d q, dq;
    read_js(q, dq);

    // Vector completo para Pinocchio (gripper bloqueado en 0)
    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>()  = q;
    dq_pin.head<NARM>() = dq;

    // ── 2. Cinematica directa y Jacobiano en LOCAL_WORLD_ALIGNED ─────────────
    // computeFrameJacobian: llama FK, actualiza data.oMf y data.J internamente
    Eigen::MatrixXd J6 = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data_, q_pin, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J6);

    // Posicion cartesiana del efector final
    const Eigen::Vector3d p = data_.oMf[frame_id_].translation();
    const Eigen::Matrix3d R = data_.oMf[frame_id_].rotation();

    // Angulo de inclinacion phi: pitch de la descomposicion ZYX
    const double phi = std::atan2(-R(2, 0),
                                   std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));

    // Jacobiano de tarea 4x4: filas {vx, vy, vz, wy} — columnas {j1..j4}
    // wy (row 4 de J6) es la tasa de cambio de phi en LOCAL_WORLD_ALIGNED
    Eigen::Matrix4d J4;
    J4.row(0) = J6.row(0).head<NARM>();   // vx
    J4.row(1) = J6.row(1).head<NARM>();   // vy
    J4.row(2) = J6.row(2).head<NARM>();   // vz
    J4.row(3) = J6.row(4).head<NARM>();   // wy  (pitch)

    // Velocidad cartesiana actual: ydot = J4 * dq
    const Eigen::Vector4d ydot = J4 * dq;

    // ── 3. Termino de bias  Jdot*qdot  via cinematica con aceleracion = 0 ───
    // forwardKinematics(q, dq, 0): llena data.v y data.a con acel articular = 0
    // => data.a contiene exclusivamente el termino centripeto/Coriolis (Jdot*qdot)
    pinocchio::forwardKinematics(
      model_, data_, q_pin, dq_pin, Eigen::VectorXd::Zero(model_.nv));

    // getFrameClassicalAcceleration devuelve la aceleracion clasica del frame
    // (equivale a Jdot*qdot cuando qddot = 0)
    const pinocchio::Motion bias =
      pinocchio::getFrameClassicalAcceleration(
        model_, data_, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED);

    Eigen::Vector4d jdqd;
    jdqd << bias.linear()[0],    // componente x
            bias.linear()[1],    // componente y
            bias.linear()[2],    // componente z
            bias.angular()[1];   // componente wy (pitch)

    // ── 4. Matrices de dinamica ───────────────────────────────────────────────
    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();
    pinocchio::nonLinearEffects(model_, data_, q_pin, dq_pin);

    const Eigen::Matrix4d M4   = data_.M.topLeftCorner<NARM, NARM>();
    const Eigen::Vector4d nle4 = data_.nle.head<NARM>();

    // ── 5. Referencia cartesiana y errores ────────────────────────────────────
    const CartRef ref = desiredTrajectory(t_);
    const Eigen::Vector4d y { p[0], p[1], p[2], phi };

    const Eigen::Vector4d ey    = ref.y    - y;
    const Eigen::Vector4d eydot = ref.ydot - ydot;

    // ── 6. Entrada auxiliar v y ley de control IO ─────────────────────────────
    // v = yddot_des + Kp*(y_des - y) + Kd*(ydot_des - ydot)
    const Eigen::Vector4d v =
      ref.yddot + KP.asDiagonal() * ey + KD.asDiagonal() * eydot;

    // Aceleracion articular deseada: qddot = J4^{-1} * (v - Jdot*qdot)
    // Se usa fullPivLu para mayor estabilidad numerica cerca de singularidades
    const Eigen::Vector4d qddot = J4.fullPivLu().solve(v - jdqd);

    // Torque: tau = M(q) * qddot + b(q, qdot)
    const Eigen::Vector4d tau     = M4 * qddot + nle4;
    const Eigen::Vector4d tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // ── 7. Publicar torques ───────────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "t=%.2fs  y=[%.3f %.3f %.3f %.3f]  |ey|=%.4f  tau=[%.3f %.3f %.3f %.3f]",
      t_, y[0], y[1], y[2], y[3], ey.norm(),
      tau_sat[0], tau_sat[1], tau_sat[2], tau_sat[3]);

    // ── 8. Registro CSV ───────────────────────────────────────────────────────
    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t_
           << ',' << q[0]         << ',' << q[1]         << ',' << q[2]         << ',' << q[3]
           << ',' << y[0]         << ',' << y[1]         << ',' << y[2]         << ',' << y[3]
           << ',' << ref.y[0]     << ',' << ref.y[1]     << ',' << ref.y[2]     << ',' << ref.y[3]
           << ',' << tau_sat[0]   << ',' << tau_sat[1]   << ',' << tau_sat[2]   << ',' << tau_sat[3]
           << '\n';
    }

    t_ += 0.01;

    // ── 9. Detener si se cumplio el tiempo de simulacion ─────────────────────
    if (t_sim_ > 0.0 && t_ >= t_sim_) {
      RCLCPP_INFO(this->get_logger(),
        "Simulacion completada (%.1f s). Deteniendo control.", t_sim_);
      if (csv_.is_open()) {
        csv_.close();
        RCLCPP_INFO(this->get_logger(), "CSV cerrado: %s", csv_path_.c_str());
      }
      timer_->cancel();
    }
  }

  // ── Miembros ──────────────────────────────────────────────────────────────
  pinocchio::Model      model_;
  pinocchio::Data       data_;
  pinocchio::FrameIndex frame_id_;
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
  rclcpp::spin(std::make_shared<IOControlNode>());
  rclcpp::shutdown();
  return 0;
}
