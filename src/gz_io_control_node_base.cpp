// ============================================================================
//  gz_io_control_node_base.cpp
//  Plantilla: Controlador Input-Output Linearization — OpenMANIPULATOR-X (simulacion Gazebo)
//
//  Salida de tarea:
//    y = [x, y, z, phi]^T
//    donde (x,y,z) es la posicion cartesiana del efector final y
//    phi = q2 + q3 + q4  es el angulo de inclinacion analitico del efector final.
//
//  Ley de control (a implementar):
//    v     = yddot_des + Kp_y*(y_des - y) + Kd_y*(ydot_des - ydot)
//    qddot = J4_dls^+ * (v - Jdot*qdot)     (pseudo-inversa DLS)
//    tau   = M(q) * qddot + b(q,dq)
//
//  Se debe completar las secciones marcadas con COMPLETAR:
//    SECCION 1 — Ganancias y parametros del controlador
//    SECCION 2 — Trayectoria de referencia cartesiana
//    SECCION 3 — Ley de control
//
//  Infraestructura proporcionada (NO modificar):
//    - Carga del modelo Pinocchio / URDF
//    - Cinematica directa: posicion (x,y,z), angulo phi, Jacobiano J4
//    - Termino de bias Jdot*qdot via getFrameClassicalAcceleration
//    - Calculo de M(q) y NLE(q,dq) mediante Pinocchio
//    - Transicion inicial suave (polinomio de 5to orden): y0 → Y_START
//    - Captura automatica de la pose inicial del robot
//    - Suscriptor /joint_states  →  q, dq
//    - Publicador /arm_effort_controller/commands  →  tau
//    - Escritura de CSV:  gz_io_data_<test_num>.csv
//    - Timer de control a 100 Hz
//    - Parametros ROS:  test_num, t_sim
//
//  Ejecucion:
//    ros2 run open_manipulator_x_torque_control io_control_node
//         --ros-args -p test_num:=1 -p t_sim:=15.0
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

// ═══════════════════════════════════════════════════════════════════════════
//  SECCION 1 — TRAYECTORIA DE REFERENCIA CARTESIANA
//
//  COMPLETAR: Definir y_des(t'), ydot_des(t') e yddot_des(t').
//
//  Parametro de entrada:
//    tp  — tiempo relativo al inicio de la fase circular [s]  (tp = t - T_TRANS)
//
//  Salida esperada (llenar ref.y, ref.ydot, ref.yddot):
//    ref.y    = [x_des(tp), y_des(tp), z_des(tp), phi_des(tp)]
//    ref.ydot = primera derivada temporal de ref.y (analitica)
//    ref.yddot= segunda derivada temporal de ref.y (analitica)
//
//  Parametros configurables (ajustar si es necesario):
//    T_TRANS — duracion de la fase de transicion inicial [s]
// ═══════════════════════════════════════════════════════════════════════════

static constexpr double T_TRANS   = 2.0;    // duracion de la transicion inicial  [s]
static const Eigen::Vector4d Y_START {0.0, 0.0, 0.0, 0.0};  // COMPLETAR 1b  [x, y, z, phi]

// ── Estructura de referencia cartesiana ─────────────────────────────────────
struct CartRef {
  Eigen::Vector4d y;      // posicion deseada    [x, y, z, phi]
  Eigen::Vector4d ydot;   // velocidad deseada   (1ra derivada analitica)
  Eigen::Vector4d yddot;  // aceleracion deseada (2da derivada analitica)
};
// ═══════════════════════════════════════════════════════════════════════════
static CartRef circularTrajectory(double tp)
{
  CartRef ref;

  // -------------------------------------------------------------------
  // COMPLETAR: implementar aqui
  // -------------------------------------------------------------------

  (void)tp;  // suprimir advertencia de compilador hasta que tp sea usado
  ref.y.setZero();
  ref.ydot.setZero();
  ref.yddot.setZero();

  return ref;
}
// ═══════════════════════════════════════════════════════════════════════════

// ── Transicion suave con polinomio de 5to orden ─────────────────────────────
// Genera referencias continuas de y0 → y_goal en el intervalo [0, T] segundos.
// Condiciones de frontera: s(0)=0, s(T)=1, s'(0)=s'(T)=0, s''(0)=s''(T)=0
// (INFRAESTRUCTURA — no es necesario modificar)
static CartRef transitionTrajectory(double t,
                                    const Eigen::Vector4d & y0,
                                    const Eigen::Vector4d & y_goal,
                                    double T)
{
  const double tau  = std::min(1.0, t / T);
  const double tau2 = tau  * tau;
  const double tau3 = tau2 * tau;
  const double tau4 = tau3 * tau;
  const double tau5 = tau4 * tau;

  const double s   =  10*tau3 - 15*tau4 +  6*tau5;
  const double sd  = (30*tau2 - 60*tau3 + 30*tau4) / T;
  const double sdd = (60*tau  - 180*tau2 + 120*tau3) / (T * T);

  const Eigen::Vector4d delta = y_goal - y0;

  CartRef ref;
  ref.y     = y0 + s   * delta;
  ref.ydot  =      sd  * delta;
  ref.yddot =      sdd * delta;
  return ref;
}
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
//  SECCION 2 — GANANCIAS Y PARAMETROS DEL CONTROLADOR
//
//  COMPLETAR 1a: Asignar las ganancias cartesianas Kp_y y Kd_y.
//    Indice:  [x, y, z, phi]
//    Referencia de disenio:
//      - Sistema de 2do orden desacoplado: wn = sqrt(Kp_y),  zeta = Kd_y/(2*wn)
//      - Para amortiguamiento critico:     Kd_y = 2*sqrt(Kp_y)
//      - La frecuencia natural debe ser >> frecuencia de la trayectoria
//
//  COMPLETAR 1b: Definir Y_START — punto inicial de la trayectoria circular.
//    Debe coincidir con ref.y en t'=0 (i.e., circularTrajectory(0)).
//    El robot parte de su pose Gazebo y transiciona hacia Y_START en T_TRANS s.
//
//  Parametros configurables (ajustar si es necesario):
//    TAU_MAX — saturacion de torque por articulacion [N·m]  
//    LAMBDA  — factor de amortiguamiento DLS (rango tipico: 0.01 – 0.05)
// ═══════════════════════════════════════════════════════════════════════════

static const Eigen::Vector4d KP_Y = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR [x, y, z, phi]
static const Eigen::Vector4d KD_Y = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR [x, y, z, phi]

static constexpr double TAU_MAX   = 0.0;   // COMPLETAR limite de torque por articulacion [N·m]
// ═══════════════════════════════════════════════════════════════════════════

// ── Nodo de control ──────────────────────────────────────────────────────────
class IOControlNode : public rclcpp::Node
{
public:
  IOControlNode()
  : Node("gz_io_control_node"), t_(0.0), y0_initialized_(false)
  {
    this->declare_parameter<int>("test_num", 1);
    const int test_num = this->get_parameter("test_num").as_int();

    this->declare_parameter<double>("t_sim", 0.0);
    t_sim_ = this->get_parameter("t_sim").as_double();

    // ── Carga del modelo Pinocchio ─────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/openmani.urdf";
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

    RCLCPP_INFO(this->get_logger(),
      "Modelo cargado: nv=%d  frame='%s' (id=%lu)",
      model_.nv, EFF_FRAME, frame_id_);
    RCLCPP_INFO(this->get_logger(),
      "Kp_y=[%.1f %.1f %.1f %.1f]  Kd_y=[%.1f %.1f %.1f %.1f]  tau_max=%.2f N·m",
      KP_Y[0], KP_Y[1], KP_Y[2], KP_Y[3],
      KD_Y[0], KD_Y[1], KD_Y[2], KD_Y[3], TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "lambda=%.3f  T_trans=%.1f s", LAMBDA, T_TRANS);
    if (t_sim_ > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Tiempo de simulacion: %.1f s", t_sim_);
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
    std::filesystem::create_directories(std::string(PACKAGE_DATA_DIR) + "/sim/act2");
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/sim/act2/gz_io_data_"
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

  // ── Lectura de estados articulares (por nombre, independiente del orden) ──
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

    // ── 1. Leer estado articular ───────────────────────────────────────────
    Eigen::Vector4d q, dq;
    read_js(q, dq);

    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>()  = q;
    dq_pin.head<NARM>() = dq;

    // ── 2. Cinematica directa y Jacobiano ─────────────────────────────────
    // computeFrameJacobian actualiza la FK internamente.
    // J6 (6×nv): filas {vx,vy,vz,wx,wy,wz} en marco LOCAL_WORLD_ALIGNED.
    Eigen::MatrixXd J6 = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data_, q_pin, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J6);

    // Posicion del efector final
    const Eigen::Vector3d p = data_.oMf[frame_id_].translation();
    const double phi = q[1] + q[2] + q[3];  // analitico: phi = q2 + q3 + q4

    // Jacobiano de tarea 4×4: filas {vx, vy, vz, dphi/dq} — columnas {j1..j4}
    Eigen::Matrix4d J4;
    J4.row(0) = J6.row(0).head<NARM>();
    J4.row(1) = J6.row(1).head<NARM>();
    J4.row(2) = J6.row(2).head<NARM>();
    J4.row(3) << 0.0, 1.0, 1.0, 1.0;   // dphi/dq = [0,1,1,1] (constante analitico)

    // Velocidad de la salida de tarea actual:  ydot = J4 * dq
    const Eigen::Vector4d ydot = J4 * dq;

    // ── 3. Termino de bias  Jdot*qdot ─────────────────────────────────────
    // Se obtiene como la aceleracion clasica del frame con qddot = 0:
    //   a_clasica(q, dq, 0) = Jdot(q,dq) * dq
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

    // ── 4. Dinamica: M(q) y b(q,dq) ──────────────────────────────────────
    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();
    pinocchio::nonLinearEffects(model_, data_, q_pin, dq_pin);

    const Eigen::Matrix4d M4   = data_.M.topLeftCorner<NARM, NARM>();
    const Eigen::Vector4d nle4 = data_.nle.head<NARM>();

    // ── 5. Salida de tarea y captura de condicion inicial ─────────────────
    const Eigen::Vector4d y { p[0], p[1], p[2], phi };

    if (!y0_initialized_) {
      y0_ = y;
      y0_initialized_ = true;
      RCLCPP_INFO(this->get_logger(),
        "Pose inicial capturada: x=%.3f  y=%.3f  z=%.3f  phi=%.3f rad",
        y0_[0], y0_[1], y0_[2], y0_[3]);
    }

    // ── 6. Referencia segun fase ──────────────────────────────────────────
    // [0, T_TRANS): transicion suave desde la pose inicial hasta Y_START
    // [T_TRANS, ∞): trayectoria definida en circularTrajectory()
    CartRef ref;
    if (t_ < T_TRANS) {
      ref = transitionTrajectory(t_, y0_, Y_START, T_TRANS);
    } else {
      ref = circularTrajectory(t_ - T_TRANS);
    }

    // ══════════════════════════════════════════════════════════════════════
    //  SECCION 3 — LEY DE CONTROL  (COMPLETAR)
    //
    //  Disponible:
    //    y      (4×1)  — salida de tarea actual        [x, y, z, phi]
    //    ydot   (4×1)  — velocidad de la salida actual
    //    ref.y, ref.ydot, ref.yddot  — referencia y sus derivadas
    //    J4     (4×4)  — Jacobiano de tarea
    //    jdqd   (4×1)  — termino Jdot(q,dq)*dq  (bias de aceleracion)
    //    M4     (4×4)  — matriz de masa inercial  M(q)
    //    nle4   (4×1)  — efectos no lineales      b(q,dq)
    //    KP_Y, KD_Y        — ganancias (definidas en Seccion 1)
    //    LAMBDA        — lambda para la pseudo-inversa DLS
    //    TAU_MAX       — saturacion de torque
    //
    //  Ley de control IO Linearization:
    //    ey     = ref.y - y
    //    eydot  = ref.ydot - ydot
    //    v      = ref.yddot + Kp_y*ey + Kd_y*eydot
    //    A      = J4*J4^T + lambda^2 * I          (matriz auxiliar DLS)
    //    qddot  = J4^T * A^-1 * (v - Jdot*dq)    (pseudo-inversa DLS)
    //    tau    = M(q) * qddot + b(q,dq)
    //    tau_sat = clamp(tau, -TAU_MAX, TAU_MAX)
    // ══════════════════════════════════════════════════════════════════════

    static constexpr double LAMBDA    = 0.01;   // factor DLS

    Eigen::Vector4d tau_sat = Eigen::Vector4d::Zero();   // <-- reemplazar con implementacion

    // ══════════════════════════════════════════════════════════════════════

    // ── 7. Publicar torques ────────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    const char * phase = (t_ < T_TRANS) ? "TRANS" : "CIRC ";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[%s] t=%.2fs  y=[%.3f %.3f %.3f %.3f]  tau=[%.3f %.3f %.3f %.3f]",
      phase, t_, y[0], y[1], y[2], y[3],
      tau_sat[0], tau_sat[1], tau_sat[2], tau_sat[3]);

    // ── 8. CSV ────────────────────────────────────────────────────────────
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

    t_ += 0.01;

    // ── 9. Detener al cumplirse el tiempo de simulacion ───────────────────
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

  // ── Miembros ──────────────────────────────────────────────────────────────
  pinocchio::Model      model_;
  pinocchio::Data       data_;
  pinocchio::FrameIndex frame_id_;

  double t_;
  double t_sim_;

  Eigen::Vector4d y0_;           // pose cartesiana inicial capturada
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
