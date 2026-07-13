/*
 * hw_fkin_node.cpp
 * Cinematica directa en hardware real — OpenMANIPULATOR-X
 * vía Dynamixel SDK directo, sin ros2_control.
 *
 * Que hace:
 *   1) Se conecta al robot real (Dynamixel SDK) como hw_fl_control_node.cpp.
 *   2) Deja los motores en torque cero: Torque Enable = OFF (bobinados en
 *      circuito abierto). Asi no hay frenado electromagnetico del lazo de
 *      corriente y el brazo se mueve a mano con la minima resistencia posible
 *      (solo queda la friccion mecanica del tren de engranajes, irreducible
 *      por software).
 *   3) Lee las posiciones articulares.
 *   4) Imprime las posiciones articulares actuales [rad].
 *   5) Imprime la posicion y orientacion del efector final (x, y, z, phi)
 *      obtenidos con la cinematica directa analitica (misma fkin que
 *      gz_tvlqr_node.cpp / open_manx_fkin.m).
 *
 * Nota: este nodo NO habilita torque ni entra en modo control de corriente.
 *   El encoder reporta Present Position igual con el torque deshabilitado, y
 *   con Torque Enable = ON aunque la corriente objetivo sea 0 el lazo de
 *   corriente activo introduce un amortiguamiento que se siente como
 *   resistencia extra al mover el brazo. Por eso aqui se deja torque OFF.
 *
 * Parametros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name   [string]  "/dev/ttyUSB0"
 *
 * Publisher: /hw/joint_states (sensor_msgs/JointState) — solo monitoreo
 *
 * ADVERTENCIA: No ejecutar junto a hardware.launch.py ni ningun proceso
 * que acceda a /dev/ttyUSB0 (ros2_control_node, dynamixel_hardware_interface).
 */

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <Eigen/Dense>

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;

// ============================================================
// Constantes Dynamixel / conversion de unidades
//   (identicas a hw_fl_control_node.cpp)
// ============================================================

static constexpr double PI = 3.14159265358979323846;
static constexpr int    NUM_JOINTS = 4;

static const std::array<uint8_t, NUM_JOINTS> DXL_ID = {11, 12, 13, 14};
static constexpr int    BAUDRATE         = 1000000;
static constexpr double PROTOCOL_VERSION = 2.0;

static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
static constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
static constexpr uint16_t ADDR_PRESENT_POSITION = 132;

static constexpr uint16_t LEN_PRESENT_VELOCITY = 4;
static constexpr uint16_t LEN_PRESENT_POSITION = 4;

static constexpr uint8_t TORQUE_DISABLE_VAL = 0;

static constexpr double POS_UNIT_RAD   = 2.0 * PI / 4096.0;
static constexpr double VEL_UNIT_RAD_S = 0.229 * 2.0 * PI / 60.0;

static const std::array<int32_t, NUM_JOINTS> JOINT_ZERO_TICK = {2048, 2048, 2048, 2048};
static const std::array<double,  NUM_JOINTS> ENCODER_SIGN    = {+1.0, +1.0, +1.0, +1.0};

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ============================================================
// Utilidades de conversion (identicas a hw_fl_control_node.cpp)
// ============================================================

static int32_t toSigned32(uint32_t v)
{
  if (v > 0x7FFFFFFFu) return -static_cast<int32_t>(0xFFFFFFFFu - v + 1u);
  return static_cast<int32_t>(v);
}

static int32_t wrappedTickDiff(int32_t raw, int32_t zero)
{
  int32_t d = raw - zero;
  while (d >  2048) d -= 4096;
  while (d < -2048) d += 4096;
  return d;
}

// ============================================================
// Cinematica directa analitica (equivalente a open_manx_fkin.m
//   y a la fkin() de gz_tvlqr_node.cpp)
//   Entrada: q = [q1 q2 q3 q4]^T [rad]
//   Salida:  [x, y, z, phi]  con phi = q2 + q3 + q4
// ============================================================

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

// ============================================================
// Nodo ROS 2
// ============================================================

class HWFkinNode : public rclcpp::Node
{
public:
  HWFkinNode()
  : Node("hw_fkin_node"), hw_active_(false)
  {
    // ── Parametros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    port_name_ = this->get_parameter("port_name").as_string();

    RCLCPP_INFO(get_logger(), "puerto=%s", port_name_.c_str());

    // ── Publisher de monitoreo ────────────────────────────────────────────────
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── SDK ──────────────────────────────────────────────────────────────────
    if (!init_hardware()) {
      RCLCPP_FATAL(get_logger(), "Fallo hardware init. Abortando.");
      throw std::runtime_error("Hardware init failed");
    }

    // ── Timer 10 Hz (lectura + impresion) ─────────────────────────────────────
    timer_ = this->create_wall_timer(100ms, [this]() { tick(); });
    RCLCPP_INFO(get_logger(),
      "Torque deshabilitado (brazo libre). Lectura de cinematica directa a 10 Hz. Ctrl+C para detener.");
  }

  ~HWFkinNode()
  {
    if (timer_) timer_->cancel();
    shutdown_hardware();
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Hardware
  // ─────────────────────────────────────────────────────────────────────────

  bool init_hardware()
  {
    port_handler_   = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!port_handler_->openPort()) {
      RCLCPP_ERROR(get_logger(), "No se pudo abrir %s", port_name_.c_str());
      return false;
    }
    if (!port_handler_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(get_logger(), "No se pudo configurar baudrate");
      port_handler_->closePort();
      return false;
    }

    // Torque OFF: bobinados en circuito abierto -> sin frenado electromagnetico.
    // El encoder sigue reportando Present Position con el torque deshabilitado.
    for (const auto id : DXL_ID) {
      if (!dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "torque disable")) {
        port_handler_->closePort();
        return false;
      }
      RCLCPP_INFO(get_logger(), "DXL ID %d listo (torque OFF)", static_cast<int>(id));
    }

    grp_pos_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    grp_vel_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);

    for (const auto id : DXL_ID) {
      grp_pos_->addParam(id);
      grp_vel_->addParam(id);
    }

    hw_active_ = true;
    RCLCPP_INFO(get_logger(), "Hardware inicializado en %s", port_name_.c_str());
    return true;
  }

  void shutdown_hardware()
  {
    if (!hw_active_) return;
    hw_active_ = false;

    for (const auto id : DXL_ID)
      dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "shutdown disable");

    if (port_handler_) {
      port_handler_->closePort();
      RCLCPP_INFO(get_logger(), "Puerto cerrado.");
    }
  }

  bool dxl_write1(uint8_t id, uint16_t addr, uint8_t val, const char* lbl)
  {
    uint8_t err = 0;
    const int r = packet_handler_->write1ByteTxRx(port_handler_, id, addr, val, &err);
    if (r != COMM_SUCCESS || err != 0) {
      RCLCPP_WARN(get_logger(), "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
      return false;
    }
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Lectura de estado (posiciones y velocidades)
  // ─────────────────────────────────────────────────────────────────────────

  bool read_state(Vec4& q, Vec4& dq)
  {
    if (grp_pos_->txRxPacket() != COMM_SUCCESS ||
        grp_vel_->txRxPacket() != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncRead fallo");
      return false;
    }
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const uint8_t id = DXL_ID[i];
      if (!grp_pos_->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) ||
          !grp_vel_->isAvailable(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)) {
        RCLCPP_ERROR(get_logger(), "[ID %d] dato no disponible", id);
        return false;
      }
      const int32_t rp = toSigned32(static_cast<uint32_t>(
        grp_pos_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)));
      const int32_t rv = toSigned32(static_cast<uint32_t>(
        grp_vel_->getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)));

      q(i)  = ENCODER_SIGN[i] * static_cast<double>(wrappedTickDiff(rp, JOINT_ZERO_TICK[i])) * POS_UNIT_RAD;
      dq(i) = ENCODER_SIGN[i] * static_cast<double>(rv) * VEL_UNIT_RAD_S;
    }
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Callback del timer (10 Hz)
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    if (!hw_active_) return;

    // 1. Lectura de posiciones (y velocidades para monitoreo)
    Vec4 q, dq;
    if (!read_state(q, dq)) {
      RCLCPP_ERROR(get_logger(), "Lectura de estado fallida.");
      return;
    }

    // 2. Cinematica directa: posicion y orientacion del efector final
    const Eigen::Vector4d y = fkin(q);

    // 3. Imprimir posiciones articulares y pose del efector final
    RCLCPP_INFO(get_logger(),
      "q=[%.4f %.4f %.4f %.4f] rad  |  EE: x=%.4f y=%.4f z=%.4f m  phi=%.4f rad",
      q(0), q(1), q(2), q(3), y(0), y(1), y(2), y(3));

    // 4. Publicar JointState de monitoreo
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name         = {"joint1", "joint2", "joint3", "joint4"};
    js.position     = {q(0), q(1), q(2), q(3)};
    js.velocity     = {dq(0), dq(1), dq(2), dq(3)};
    js_pub_->publish(js);
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  std::string port_name_;

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead> grp_pos_, grp_vel_;

  bool hw_active_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================
// main
// ============================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HWFkinNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_fkin_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
