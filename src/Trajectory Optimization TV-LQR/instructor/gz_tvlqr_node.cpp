// ============================================================================
//  gz_tvlqr_node.cpp
//  Seguimiento TV-LQR — OpenMANIPULATOR-X (simulacion Gazebo)
//
//  Ley de control TV-LQR:
//    x(t)    = [q; dq]                                 (8 x 1)
//    x_ref,k = [q_ref_k; dq_ref_k]                    (8 x 1)
//    tau     = u_ref_k - K_k * (x(t) - x_ref,k)       (4 x 1)
//    tau_sat = clamp(tau, -TAU_MAX, TAU_MAX)
//
//  Referencias precargadas desde archivos de texto en reference_dir/:
//    time_ref.txt   N x 1    — instantes de muestreo [s]
//    q_ref.txt      N x 4    — posiciones articulares de referencia [rad]
//    dq_ref.txt     N x 4    — velocidades articulares de referencia [rad/s]
//    u_ref.txt      N x 4    — entradas optimizadas (torques) [N.m]
//    K_TV.txt       N x 32   — ganancias TV-LQR (reshape col-major de K_k 4x8)
//
//  Parametros ROS 2 (--ros-args -p nombre:=valor):
//    test_num      [int]    1       — identificador del CSV generado
//    t_sim         [double] 0.0     — duracion en segundos (0 = sin limite)
//    reference_dir [string] "src/Trajectory Optimization TV-LQR/references"
//
//  Suscriptor : /joint_states                   (sensor_msgs/JointState)
//  Publicador : /arm_effort_controller/commands  (std_msgs/Float64MultiArray)
//
//  CSV generado: data/lab5/sim/data_log_sim_lab5_<test_num>.csv
// ============================================================================

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std::chrono_literals;

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_SHARE_DIR
#define PACKAGE_SHARE_DIR "."
#endif

static constexpr int    NARM    = 4;
static constexpr double TAU_MAX = 1.0;   // [N·m] limite de torque por articulacion

// ── Cinematica directa analitica (equivalente a open_manx_fkin.m) ────────────
//   Entrada: q = [q1 q2 q3 q4]^T [rad]
//   Salida:  [x, y, z, phi]  con phi = q2 + q3 + q4
static Eigen::Vector4d fkin(const Eigen::Vector4d & q)
{
  const double x_base = 0.012,  z_base = 0.017 + 0.0595;
  const double x23 = 0.024,  z23 = 0.128;
  const double l34 = 0.124,  l4e = 0.126;

  const double r = x23*std::cos(q[1]) + z23*std::sin(q[1])
                 + l34*std::cos(q[1]+q[2])
                 + l4e*std::cos(q[1]+q[2]+q[3]);
  const double z = z_base
                 + (-x23*std::sin(q[1]) + z23*std::cos(q[1]))
                 - l34*std::sin(q[1]+q[2])
                 - l4e*std::sin(q[1]+q[2]+q[3]);
  return { x_base + r*std::cos(q[0]),
           r*std::sin(q[0]),
           z,
           q[1]+q[2]+q[3] };
}

// ── Utilidad: leer una matriz de texto con 'cols' columnas ───────────────────
//   Formato: valores separados por espacios, una fila por linea.
static bool load_matrix(const std::string & path, int cols,
                         std::vector<std::vector<double>> & rows)
{
  std::ifstream f(path);
  if (!f.is_open()) { return false; }
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) { continue; }
    std::istringstream ss(line);
    std::vector<double> row;
    double v;
    while (ss >> v) { row.push_back(v); }
    if (static_cast<int>(row.size()) != cols) { return false; }
    rows.push_back(row);
  }
  return !rows.empty();
}

// ── Nodo ROS 2 ───────────────────────────────────────────────────────────────
class TVLQRSimNode : public rclcpp::Node
{
public:
  TVLQRSimNode()
  : Node("gz_tvlqr_node"),
    t_(0.0), refs_loaded_(false), N_(0), Ts_(0.05),
    warmup_ticks_(0), tau_gravity_(Eigen::Vector4d::Zero())
  {
    // ── Parametros ──────────────────────────────────────────────────────────
    this->declare_parameter<int>        ("test_num",      1);
    this->declare_parameter<double>     ("t_sim",         0.0);
    this->declare_parameter<double>     ("t_warmup",      2.0);
    this->declare_parameter<std::string>("reference_dir",
      "src/Trajectory Optimization TV-LQR/references");

    const int test_num    = this->get_parameter("test_num").as_int();
    t_sim_                = this->get_parameter("t_sim").as_double();
    const std::string ref_name = this->get_parameter("reference_dir").as_string();
    ref_dir_ = std::string(PACKAGE_SHARE_DIR) + "/" + ref_name;

    RCLCPP_INFO(get_logger(), "reference_dir: %s", ref_dir_.c_str());
    if (t_sim_ > 0.0)
      RCLCPP_INFO(get_logger(), "t_sim: %.2f s", t_sim_);
    else
      RCLCPP_INFO(get_logger(), "t_sim: ilimitado");

    // ── Seccion 1: Carga de referencias ─────────────────────────────────────
    auto fatal_load = [&](const std::string & path) {
      RCLCPP_FATAL(get_logger(), "No se pudo cargar: %s", path.c_str());
      throw std::runtime_error("Archivo de referencia no encontrado: " + path);
    };

    // time_ref.txt — 1 columna
    {
      std::vector<std::vector<double>> rows;
      const std::string path = ref_dir_ + "/time_ref.txt";
      if (!load_matrix(path, 1, rows)) { fatal_load(path); }
      N_  = static_cast<int>(rows.size());
      Ts_ = rows[0][0];
      t_ref_.reserve(N_);
      for (const auto & r : rows) { t_ref_.push_back(r[0]); }
    }

    // q_ref.txt — 4 columnas
    {
      std::vector<std::vector<double>> rows;
      const std::string path = ref_dir_ + "/q_ref.txt";
      if (!load_matrix(path, 4, rows)) { fatal_load(path); }
      q_ref_.reserve(N_);
      for (const auto & r : rows)
        q_ref_.emplace_back(r[0], r[1], r[2], r[3]);
    }

    // dq_ref.txt — 4 columnas
    {
      std::vector<std::vector<double>> rows;
      const std::string path = ref_dir_ + "/dq_ref.txt";
      if (!load_matrix(path, 4, rows)) { fatal_load(path); }
      dq_ref_.reserve(N_);
      for (const auto & r : rows)
        dq_ref_.emplace_back(r[0], r[1], r[2], r[3]);
    }

    // u_ref.txt — 4 columnas
    {
      std::vector<std::vector<double>> rows;
      const std::string path = ref_dir_ + "/u_ref.txt";
      if (!load_matrix(path, 4, rows)) { fatal_load(path); }
      u_ref_.reserve(N_);
      for (const auto & r : rows)
        u_ref_.emplace_back(r[0], r[1], r[2], r[3]);
    }

    // K_TV.txt — 32 columnas (col-major, reshape de K_k 4x8)
    {
      std::vector<std::vector<double>> rows;
      const std::string path = ref_dir_ + "/K_TV.txt";
      if (!load_matrix(path, 32, rows)) { fatal_load(path); }
      K_TV_.reserve(N_);
      for (const auto & r : rows) {
        Eigen::Map<const Eigen::Matrix<double,4,8,Eigen::ColMajor>> K(r.data());
        K_TV_.push_back(K);
      }
    }

    refs_loaded_ = true;
    RCLCPP_INFO(get_logger(), "Referencias cargadas: N=%d  Ts=%.3f s", N_, Ts_);

    // Compensacion gravitatoria: torque de referencia en k=0 sostiene x0
    tau_gravity_ = u_ref_[0];
    const double t_warmup = this->get_parameter("t_warmup").as_double();
    warmup_ticks_ = static_cast<int>(std::round(t_warmup / 0.01));
    RCLCPP_INFO(get_logger(),
      "Compensacion gravitatoria: [%.3f %.3f %.3f %.3f] Nm  (t_warmup=%.1f s)",
      tau_gravity_[0], tau_gravity_[1], tau_gravity_[2], tau_gravity_[3], t_warmup);

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

  ~TVLQRSimNode()
  {
    if (csv_.is_open()) {
      csv_.close();
      RCLCPP_INFO(get_logger(), "CSV cerrado: %s", csv_path_.c_str());
    }
  }

private:
  // ── CSV ───────────────────────────────────────────────────────────────────
  void open_csv(int test_num)
  {
    std::filesystem::create_directories(std::string(PACKAGE_DATA_DIR) + "/lab5/sim");
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab5/sim/data_log_sim_lab5_"
                + std::to_string(test_num) + ".csv";
    csv_.open(csv_path_);
    if (!csv_.is_open()) {
      RCLCPP_ERROR(get_logger(), "No se pudo crear: %s", csv_path_.c_str());
      return;
    }
    csv_ << "t,"
         << "q1,q2,q3,q4,"
         << "dq1,dq2,dq3,dq4,"
         << "q1_ref,q2_ref,q3_ref,q4_ref,"
         << "dq1_ref,dq2_ref,dq3_ref,dq4_ref,"
         << "tau1,tau2,tau3,tau4,"
         << "x,y,z,phi,"
         << "x_ref,y_ref,z_ref,phi_ref\n";
    RCLCPP_INFO(get_logger(), "CSV: %s", csv_path_.c_str());
  }

  // ── Lectura de estados articulares ───────────────────────────────────────
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

  // ── Tick de control a 100 Hz ─────────────────────────────────────────────
  void tick()
  {
    if (!last_js_ || !refs_loaded_) { return; }

    // ── Fase de inicializacion: compensacion gravitatoria ─────────────────
    if (warmup_ticks_ > 0) {
      std_msgs::msg::Float64MultiArray cmd;
      cmd.data.assign(tau_gravity_.data(), tau_gravity_.data() + NARM);
      torque_pub_->publish(cmd);
      --warmup_ticks_;
      if (warmup_ticks_ == 0) {
        RCLCPP_INFO(get_logger(), "Compensacion gravitatoria completada. Iniciando TV-LQR.");
      }
      return;
    }

    // ── Leer estado articular ─────────────────────────────────────────────
    Eigen::Vector4d q, dq;
    read_js(q, dq);

    const Eigen::Vector4d y_actual = fkin(q);

    // ── Seccion 2: Seleccion del indice temporal k ────────────────────────
    int k = static_cast<int>(std::floor(t_ / Ts_));
    k = std::max(0, std::min(k, N_ - 1));

    // ── Seccion 3: Ley de control TV-LQR ─────────────────────────────────
    Eigen::Matrix<double,8,1> x_state, x_ref_k;
    x_state << q, dq;
    x_ref_k << q_ref_[k], dq_ref_[k];

    const Eigen::Vector4d tau = u_ref_[k] - K_TV_[k] * (x_state - x_ref_k);

    // ── Seccion 4: Saturacion de torque ───────────────────────────────────
    const Eigen::Vector4d tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // ── Publicar torques ──────────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
      "t=%.2fs  k=%d/%d  y=[%.3f %.3f %.3f]  tau=[%.3f %.3f %.3f %.3f] Nm",
      t_, k, N_-1, y_actual[0], y_actual[1], y_actual[2],
      tau_sat[0], tau_sat[1], tau_sat[2], tau_sat[3]);

    // ── Seccion 5: Registro de datos en CSV ───────────────────────────────
    if (csv_.is_open()) {
      const Eigen::Vector4d y_ref_k = fkin(q_ref_[k]);
      csv_ << std::fixed << std::setprecision(6)
           << t_               << ","
           << q[0]             << "," << q[1]  << "," << q[2]  << "," << q[3]  << ","
           << dq[0]            << "," << dq[1] << "," << dq[2] << "," << dq[3] << ","
           << q_ref_[k][0]    << "," << q_ref_[k][1]  << "," << q_ref_[k][2]  << "," << q_ref_[k][3]  << ","
           << dq_ref_[k][0]   << "," << dq_ref_[k][1] << "," << dq_ref_[k][2] << "," << dq_ref_[k][3] << ","
           << tau_sat[0]       << "," << tau_sat[1] << "," << tau_sat[2] << "," << tau_sat[3] << ","
           << y_actual[0]      << "," << y_actual[1] << "," << y_actual[2] << "," << y_actual[3] << ","
           << y_ref_k[0]       << "," << y_ref_k[1]  << "," << y_ref_k[2]  << "," << y_ref_k[3]
           << "\n";
    }

    t_ += 0.01;

    if (t_sim_ > 0.0 && t_ >= t_sim_) {
      RCLCPP_INFO(get_logger(), "Simulacion completada (%.2f s).", t_sim_);
      std_msgs::msg::Float64MultiArray zero;
      zero.data.assign(NARM, 0.0);
      torque_pub_->publish(zero);
      if (csv_.is_open()) { csv_.close(); }
      timer_->cancel();
    }
  }

  // ── Miembros ──────────────────────────────────────────────────────────────
  std::string ref_dir_;
  double      t_;
  double      t_sim_;
  bool        refs_loaded_;

  int    N_;
  double Ts_;

  std::vector<double>                                    t_ref_;
  std::vector<Eigen::Vector4d>                           q_ref_;
  std::vector<Eigen::Vector4d>                           dq_ref_;
  std::vector<Eigen::Vector4d>                           u_ref_;
  std::vector<Eigen::Matrix<double,4,8,Eigen::ColMajor>> K_TV_;

  int             warmup_ticks_;
  Eigen::Vector4d tau_gravity_;

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
    rclcpp::spin(std::make_shared<TVLQRSimNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gz_tvlqr_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
