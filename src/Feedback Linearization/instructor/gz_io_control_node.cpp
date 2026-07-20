// ============================================================================
//  io_control_node.cpp
//  Input-Output Linearization en espacio cartesiano — OpenMANIPULATOR-X
//
//  Salida de tarea:
//    y = [x, y, z, phi]^T
//    donde (x,y,z) es la posicion cartesiana del efector final y
//    phi = q2 + q3 + q4  es el angulo de inclinacion analitico del efector final.
//
//  Cinematica diferencial:
//    ydot  = J(q) * qdot
//    yddot = J(q) * qddot + Jdot(q,qdot) * qdot
//
//  Ley de control:
//    v   = yddot_des + Kp*(y_des - y) + Kd*(ydot_des - ydot)
//    qddot = J4_dls^+ * (v - Jdot*qdot)     <- pseudo-inversa amortiguada DLS
//    tau = M(q) * qddot + b(q,qdot)
//
//  Fases de referencia (identicas al nodo hw_io_control_node):
//    [0, T_TRANS)  — transicion suave con polinomio de 5to orden
//                    desde la pose inicial medida hasta el inicio de la trayectoria
//    [T_TRANS, ∞)  — trayectoria 3D cartesiana (w = 1 rad/s):
//                      x_des   = 0.20 + 0.05*sin(t')   [extension max. 0.25 m]
//                      y_des   = 0.05*cos(t')
//                      z_des   = 0.15 - 0.05*sin(t')   [z ∈ 0.10..0.20 m,
//                                CONTRAFASE con x: z minimo con brazo extendido]
//                      phi_des = 0.22 rad
//
//  Parametros ROS:
//    test_num      — identificador del CSV generado (gz_io_data_<test_num>.csv)
//    t_run         — duracion de la simulacion en segundos (0 = ilimitado)
//    friction_ffwd — bool (default false): feedforward de la friccion del URDF
//                    (damping·dq + friction·tanh(dq_des/eps), con dq_des =
//                    J4⁺·ydot_des), replicando la compensacion del nodo hw.
//                    Pinocchio no incluye <dynamics> en M/nle, asi que sin
//                    esto la friccion de Gazebo queda sin compensar.
//                    Valido con damping/friction_scale = 1.0.
//
//  CSV generado: gz_io_data_<test_num>.csv  en PACKAGE_DATA_DIR/lab4/sim/act2/
//
//  Ejemplo de ejecucion (con la simulacion Gazebo ya lanzada):
//    ros2 launch open_manipulator_x_torque_control torque_sim.launch.py
//    ros2 run open_manipulator_x_torque_control gz_io_control_node --ros-args -p test_num:=1 -p t_run:=20.0 -p friction_ffwd:=true
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
static constexpr int    NARM = 4;

static constexpr char EFF_FRAME[] = "end_effector_link";

// ============================================================================
//  PARAMETROS DEL CONTROLADOR  (editar aqui)
// ============================================================================

// Ganancias cartesianas  [x,  y,  z,  phi]
// Alineadas con hw_io_control_node (KD = 1.4·sqrt(KP), zeta = 0.7): mismo
// controlador en sim y en el robot real para que los CSV sean comparables.
// z y phi van mas altos porque esas direcciones las dominan las munecas
// (inercia diminuta): en hardware con KP bajos quedaban pegadas por stiction.
static const Eigen::Vector4d KP = {400.0, 200.0, 800.0, 1500.0};
static const Eigen::Vector4d KD = { 28.0,  20.0,  40.0,   54.0};

// Saturacion de torque por articulacion [N·m] — igual que tau_max del robot real
static constexpr double TAU_MAX = 1.2;

// Duracion de la fase de transicion inicial [s] — igual que el nodo hw
static constexpr double T_TRANS = 3.0;

// Suavizado del tanh en el feedforward de Coulomb [rad/s] — igual que
// motor_epsilon_friction del nodo hw (seguro: usa la velocidad deseada)
static constexpr double FRIC_EPS = 0.05;

// Factor de amortiguamiento para la pseudo-inversa DLS  (Damped Least Squares)
// Limita la norma de qddot cerca de singularidades sin afectar la inversion
// cuando el Jacobiano esta bien condicionado. Alineado con el nodo hw (0.01);
// la trayectoria verificada por IK se mantiene lejos de singularidades.
static constexpr double LAMBDA    = 0.01;
static constexpr double LAMBDA_SQ = LAMBDA * LAMBDA;

// Inicio de la trayectoria (= valor de la referencia en t'=0), igual que hw
static const Eigen::Vector4d Y_START {0.20, 0.05, 0.15, 0.22};

// Altura minima del efector sobre la placa de la base [m] — igual que el
// nodo hw: guarda para la referencia y para el efector medido (durante la
// transicion solo se exige no descender por debajo de la pose inicial).
static constexpr double Z_MIN_FLOOR = 0.075;

// ============================================================================

// ── Estructura de referencia cartesiana ─────────────────────────────────────
struct CartRef {
  Eigen::Vector4d y;      // posicion deseada  [x, y, z, phi]
  Eigen::Vector4d ydot;   // velocidad deseada (1ra derivada analitica)
  Eigen::Vector4d yddot;  // aceleracion deseada (2da derivada analitica)
};

// ── Trayectoria circular (activa para t' = t - T_TRANS >= 0) ────────────────
static CartRef circularTrajectory(double tp)
{
  const double w = 1.0;   // 2.0→1.0 rad/s: periodo 6.3 s, menor acoplamiento en phi       
  CartRef ref;

  ref.y <<
     0.20 + 0.05 * std::sin(w * tp),
     0.05 * std::cos(w * tp),
     0.15 - 0.05 * std::sin(w * tp),
     0.22;

  ref.ydot <<
     0.05 * w * std::cos(w * tp),
    -0.05 * w * std::sin(w * tp),
    -0.05 * w * std::cos(w * tp),
     0.0;

  ref.yddot <<
    -0.05 * w * w * std::sin(w * tp),
    -0.05 * w * w * std::cos(w * tp),
     0.05 * w * w * std::sin(w * tp),
     0.0;

  return ref;
}

// ── Transicion con polinomio de 5to orden (zero-velocity, zero-accel en extremos) ─
// Genera referencias suaves de y0 → y_goal durante [0, T] segundos.
static CartRef transitionTrajectory(double t,
                                    const Eigen::Vector4d & y0,
                                    const Eigen::Vector4d & y_goal,
                                    double T)
{
  const double tau   = std::min(1.0, t / T);
  const double tau2  = tau  * tau;
  const double tau3  = tau2 * tau;
  const double tau4  = tau3 * tau;
  const double tau5  = tau4 * tau;

  // Polinomio s(tau) = 10τ³ - 15τ⁴ + 6τ⁵  →  s(0)=0, s(1)=1, s'(0)=s'(1)=0
  const double s    =  10*tau3 - 15*tau4 +  6*tau5;
  const double sd   = (30*tau2 - 60*tau3 + 30*tau4) / T;
  const double sdd  = (60*tau  - 180*tau2 + 120*tau3) / (T * T);

  const Eigen::Vector4d delta = y_goal - y0;

  CartRef ref;
  ref.y     = y0    +  s  * delta;
  ref.ydot  =           sd  * delta;
  ref.yddot =           sdd * delta;
  return ref;
}

// ── Nodo de control ──────────────────────────────────────────────────────────
class IOControlNode : public rclcpp::Node
{
public:
  IOControlNode()
  : Node("gz_io_control_node"), t_(0.0), y0_initialized_(false)
  {
    this->declare_parameter<int>("test_num", 1);
    const int test_num = this->get_parameter("test_num").as_int();

    this->declare_parameter<double>("t_run", 0.0);
    t_run_ = this->get_parameter("t_run").as_double();

    this->declare_parameter<bool>("friction_ffwd", false);
    friction_ffwd_ = this->get_parameter("friction_ffwd").as_bool();

    // Cargar modelo Pinocchio
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    if (!model_.existFrame(EFF_FRAME)) {
      throw std::runtime_error(std::string("Frame no encontrado: ") + EFF_FRAME);
    }
    frame_id_ = model_.getFrameId(EFF_FRAME);

    // Friccion articular del URDF (<dynamics damping/friction>), parseada por
    // Pinocchio pero NO incluida en M/nle: se usa para el feedforward opcional.
    fric_damping_ = model_.damping.head<NARM>();
    fric_coulomb_ = model_.friction.head<NARM>();

    RCLCPP_INFO(this->get_logger(),
      "Modelo cargado: nv=%d  frame='%s' (id=%lu)",
      model_.nv, EFF_FRAME, frame_id_);
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
      "Kp=[%.1f]  Kd=[%.1f]  tau_max=%.2f N·m  lambda=%.3f  T_trans=%.1f s",
      KP[0], KD[0], TAU_MAX, LAMBDA, T_TRANS);
    if (t_run_ > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Tiempo de simulacion: %.1f s", t_run_);
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

  ~IOControlNode()
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
    std::filesystem::create_directories(std::string(PACKAGE_DATA_DIR) + "/lab4/sim/act2");
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab4/sim/act2/gz_io_data_"
                + std::to_string(test_num) + ".csv";
    csv_.open(csv_path_);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo crear: %s", csv_path_.c_str());
      return;
    }
    csv_ << "t,"
         << "q1,q2,q3,q4,"
         << "x,y,z,phi,"
         << "x_des,y_des,z_des,phi_des,"
         << "xdot,ydot,zdot,phidot,"
         << "xdot_des,ydot_des,zdot_des,phidot_des,"
         << "tau1,tau2,tau3,tau4\n";
    RCLCPP_INFO(this->get_logger(), "CSV: %s", csv_path_.c_str());
  }

  // ── Lectura de estados articulares (por nombre, independiente del orden) ───
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

  // ── Parada segura: torque cero y fin del lazo (guardas de seguridad) ──────
  void stop_control(const std::string & reason)
  {
    RCLCPP_ERROR(this->get_logger(), "PARADA: %s", reason.c_str());
    std_msgs::msg::Float64MultiArray zero;
    zero.data.assign(NARM, 0.0);
    torque_pub_->publish(zero);
    if (csv_.is_open()) { csv_.close(); }
    timer_->cancel();
  }

  // ── Tick de control (200 Hz) ──────────────────────────────────────────────
  void tick()
  {
    if (!last_js_) { return; }

    // 1. Leer estados articulares
    Eigen::Vector4d q, dq;
    read_js(q, dq);

    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>()  = q;
    dq_pin.head<NARM>() = dq;

    // ── 2. Cinematica directa y Jacobiano ─────────────────────────────────
    Eigen::MatrixXd J6 = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data_, q_pin, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J6);

    const Eigen::Vector3d p = data_.oMf[frame_id_].translation();
    const double phi = q[1] + q[2] + q[3];  // analitico: phi = q2 + q3 + q4

    // Jacobiano de tarea 4x4: filas {vx, vy, vz, dphi/dq} — columnas {j1..j4}
    Eigen::Matrix4d J4;
    J4.row(0) = J6.row(0).head<NARM>();
    J4.row(1) = J6.row(1).head<NARM>();
    J4.row(2) = J6.row(2).head<NARM>();
    J4.row(3) << 0.0, 1.0, 1.0, 1.0;   // dphi/dq = [0,1,1,1] (constante analitico)

    // Velocidad cartesiana actual
    const Eigen::Vector4d ydot = J4 * dq;

    // ── 3. Bias  Jdot*qdot  (FK con aceleracion articular = 0) ───────────
    pinocchio::forwardKinematics(
      model_, data_, q_pin, dq_pin, Eigen::VectorXd::Zero(model_.nv));
    const pinocchio::Motion bias =
      pinocchio::getFrameClassicalAcceleration(
        model_, data_, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::Vector4d jdqd;
    jdqd << bias.linear()[0],
            bias.linear()[1],
            bias.linear()[2],
            0.0;   // Jdot_phi * qdot = 0 (fila [0,1,1,1] es constante)

    // ── 4. Dinamica: M(q) y b(q, qdot) ───────────────────────────────────
    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();
    pinocchio::nonLinearEffects(model_, data_, q_pin, dq_pin);
    const Eigen::Matrix4d M4   = data_.M.topLeftCorner<NARM, NARM>();
    const Eigen::Vector4d nle4 = data_.nle.head<NARM>();

    // ── 5. Pose cartesiana actual y captura de condicion inicial ──────────
    const Eigen::Vector4d y { p[0], p[1], p[2], phi };

    if (!y0_initialized_) {
      y0_ = y;
      y0_initialized_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Pose inicial capturada: x=%.3f  y=%.3f  z=%.3f  phi=%.3f rad",
        y0_[0], y0_[1], y0_[2], y0_[3]);
    }

    // ── 6. Referencia segun fase ──────────────────────────────────────────
    CartRef ref;
    if (t_ < T_TRANS) {
      // Fase de transicion: polinomio 5to orden  y0 → Y_START
      ref = transitionTrajectory(t_, y0_, Y_START, T_TRANS);
    } else {
      // Fase de trayectoria: parametro de tiempo relativo al fin de la transicion
      ref = circularTrajectory(t_ - T_TRANS);
    }

    // ── 6b. Guardas de altura minima (igual que el nodo hw) ───────────────
    const double z_floor = (t_ < T_TRANS)
      ? std::min(Z_MIN_FLOOR, y0_[2] - 0.03)
      : Z_MIN_FLOOR;
    if (ref.y[2] < z_floor) {
      stop_control("Referencia z bajo altura minima: " + std::to_string(ref.y[2])
                   + " m (piso " + std::to_string(z_floor) + ")");
      return;
    }
    if (y[2] < z_floor) {
      stop_control("Efector bajo altura minima: z=" + std::to_string(y[2])
                   + " m (piso " + std::to_string(z_floor) + ")");
      return;
    }

    // ── 7. Errores cartesianos ─────────────────────────────────────────────
    const Eigen::Vector4d ey    = ref.y    - y;
    const Eigen::Vector4d eydot = ref.ydot - ydot;

    // ── 8. Entrada auxiliar v ─────────────────────────────────────────────
    const Eigen::Vector4d v =
      ref.yddot + KP.asDiagonal() * ey + KD.asDiagonal() * eydot;

    // ── 9. Pseudo-inversa amortiguada DLS ─────────────────────────────────
    // qddot = J4ᵀ (J4 J4ᵀ + λ²I)⁻¹ (v - Jdot*qdot)
    // Cuando el Jacobiano es bien condicionado: equivalente a J4⁻¹(v - jdqd)
    // Cerca de singularidades: limita la norma de qddot evitando divergencia
    const Eigen::Matrix4d A =
      J4 * J4.transpose() + LAMBDA_SQ * Eigen::Matrix4d::Identity();
    const auto A_ldlt = A.ldlt();
    const Eigen::Vector4d qddot = J4.transpose() * A_ldlt.solve(v - jdqd);

    // ── 10. Torque y saturacion ───────────────────────────────────────────
    Eigen::Vector4d tau = M4 * qddot + nle4;

    // Feedforward opcional de la friccion del URDF (la que Gazebo simula):
    // viscosa con la velocidad medida; Coulomb con la velocidad articular
    // DESEADA dq_des = J4⁺·ydot_des (senal limpia, igual que el nodo hw).
    if (friction_ffwd_) {
      const Eigen::Vector4d dq_des = J4.transpose() * A_ldlt.solve(ref.ydot);
      for (int i = 0; i < NARM; ++i) {
        tau[i] += fric_damping_[i] * dq[i]
                + fric_coulomb_[i] * std::tanh(dq_des[i] / FRIC_EPS);
      }
    }

    const Eigen::Vector4d tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // ── 11. Publicar ──────────────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    const char * phase = (t_ < T_TRANS) ? "TRANS" : "CIRC ";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[%s] t=%.2fs  y=[%.3f %.3f %.3f %.3f]  |ey|=%.4f  tau=[%.3f %.3f %.3f %.3f]",
      phase, t_, y[0], y[1], y[2], y[3], ey.norm(),
      tau_sat[0], tau_sat[1], tau_sat[2], tau_sat[3]);

    // ── 12. CSV ───────────────────────────────────────────────────────────
    if (csv_.is_open()) {
      csv_ << std::fixed << std::setprecision(6)
           << t_
           << ',' << q[0]           << ',' << q[1]           << ',' << q[2]           << ',' << q[3]
           << ',' << y[0]           << ',' << y[1]           << ',' << y[2]           << ',' << y[3]
           << ',' << ref.y[0]       << ',' << ref.y[1]       << ',' << ref.y[2]       << ',' << ref.y[3]
           << ',' << ydot[0]        << ',' << ydot[1]        << ',' << ydot[2]        << ',' << ydot[3]
           << ',' << ref.ydot[0]    << ',' << ref.ydot[1]    << ',' << ref.ydot[2]    << ',' << ref.ydot[3]
           << ',' << tau_sat[0]     << ',' << tau_sat[1]     << ',' << tau_sat[2]     << ',' << tau_sat[3]
           << '\n';
    }

    t_ += 0.005;   // Ts del lazo de 200 Hz

    // ── 13. Parar al cumplirse el tiempo de simulacion ────────────────────
    if (t_run_ > 0.0 && t_ >= t_run_) {
      RCLCPP_INFO(this->get_logger(),
        "Simulacion completada (%.1f s). Deteniendo control.", t_run_);
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

  // ── Miembros ──────────────────────────────────────────────────────────────
  pinocchio::Model      model_;
  pinocchio::Data       data_;
  pinocchio::FrameIndex frame_id_;

  double t_;
  double t_run_;

  bool friction_ffwd_{false};
  Eigen::Vector4d fric_damping_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d fric_coulomb_{Eigen::Vector4d::Zero()};

  Eigen::Vector4d y0_;          // pose cartesiana inicial medida
  bool            y0_initialized_;

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
