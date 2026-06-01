/*
 * hw_torque_probe_node.cpp
 * Diagnostico de polaridad torque->corriente — OpenMANIPULATOR-X hardware real.
 *
 * OBJETIVO: aplicar una corriente PEQUENA y FIJA a UN solo joint (los demas
 * quedan libres/backdrivable) y registrar hacia donde gira, en marco CRUDO
 * del encoder (sin ENCODER_SIGN). Sirve para validar empiricamente la relacion
 *   (signo de Goal_Current)  ->  (signo del cambio de Present_Position crudo)
 *   ->  (direccion fisica que sientes con la mano).
 *
 * Con esto se determina el CURRENT_SIGN correcto para los nodos de control,
 * de forma independiente al ENCODER_SIGN.
 *
 * Parametros ROS 2:
 *   port_name      [string]  "/dev/ttyUSB0"
 *   joint          [int]     2     (1..4  ->  DXL ID 11..14)
 *   current_ticks  [int]     30    (corriente fija con signo; |valor| acotado a SAFE_MAX)
 *   duration_s     [double]  3.0   (tiempo de aplicacion)
 *
 * USO TIPICO (un joint a la vez, sosteniendolo con la mano al inicio):
 *   ros2 run open_manipulator_x_torque_control hw_torque_probe_node \
 *        --ros-args -p joint:=2 -p current_ticks:=30 -p duration_s:=3.0
 *
 * SEGURIDAD:
 *   - Solo se energiza el joint seleccionado; los otros quedan con torque OFF.
 *   - |current_ticks| se acota a SAFE_MAX_TICKS (corriente baja).
 *   - Al terminar (o con Ctrl+C) se envia corriente 0 y se deshabilita torque.
 *   - Sosten el eslabon con la mano antes de arrancar: aunque la corriente es
 *     baja, el joint puede moverse.
 *
 * ADVERTENCIA: No ejecutar junto a ningun proceso que use /dev/ttyUSB0.
 */

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;

// ============================================================
// Constantes Dynamixel (identicas a los nodos hw_)
// ============================================================

static constexpr double PI = 3.14159265358979323846;
static constexpr int    NUM_JOINTS = 4;

static const std::array<uint8_t, NUM_JOINTS> DXL_ID = {11, 12, 13, 14};
static constexpr int    BAUDRATE         = 1000000;
static constexpr double PROTOCOL_VERSION = 2.0;

static constexpr uint16_t ADDR_OPERATING_MODE   = 11;
static constexpr uint16_t ADDR_CURRENT_LIMIT    = 38;
static constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
static constexpr uint16_t ADDR_GOAL_CURRENT     = 102;
static constexpr uint16_t ADDR_PRESENT_CURRENT  = 126;
static constexpr uint16_t ADDR_PRESENT_POSITION = 132;

static constexpr uint8_t CURRENT_CONTROL_MODE = 0;
static constexpr uint8_t TORQUE_ENABLE_VAL    = 1;
static constexpr uint8_t TORQUE_DISABLE_VAL   = 0;

static constexpr double POS_UNIT_RAD         = 2.0 * PI / 4096.0;
static constexpr double POS_UNIT_DEG         = 360.0 / 4096.0;
static constexpr double CURRENT_UNIT_A       = 0.00269;
static constexpr double TORQUE_CONSTANT_NM_A = 1.666;
static constexpr double TORQUE_PER_CURRENT_TICK = TORQUE_CONSTANT_NM_A * CURRENT_UNIT_A;

// Limites de seguridad para el diagnostico (corriente baja)
static constexpr uint16_t CURRENT_LIMIT_REGISTER = 150;   // tope HW conservador
static constexpr int16_t  SAFE_MAX_TICKS         = 100;   // |comando| maximo permitido

// ============================================================
// Utilidades
// ============================================================

static int32_t toSigned32(uint32_t v)
{
  if (v > 0x7FFFFFFFu) return -static_cast<int32_t>(0xFFFFFFFFu - v + 1u);
  return static_cast<int32_t>(v);
}

static int16_t toSigned16(uint32_t v)
{
  const uint16_t w = static_cast<uint16_t>(v & 0xFFFFu);
  if (w > 0x7FFFu) return -static_cast<int16_t>(0xFFFFu - w + 1u);
  return static_cast<int16_t>(w);
}

// ============================================================
// Nodo
// ============================================================

class HWTorqueProbeNode : public rclcpp::Node
{
public:
  HWTorqueProbeNode()
  : Node("hw_torque_probe_node"), hw_active_(false)
  {
    this->declare_parameter<std::string>("port_name",     "/dev/ttyUSB0");
    this->declare_parameter<int>        ("joint",          2);
    this->declare_parameter<int>        ("current_ticks",  30);
    this->declare_parameter<double>     ("duration_s",     3.0);

    port_name_ = this->get_parameter("port_name").as_string();
    const int joint = this->get_parameter("joint").as_int();
    int cmd_ticks   = this->get_parameter("current_ticks").as_int();
    duration_s_     = this->get_parameter("duration_s").as_double();

    if (joint < 1 || joint > NUM_JOINTS) {
      throw std::runtime_error("joint debe estar en 1..4");
    }
    joint_idx_ = joint - 1;

    // Acotar corriente al limite seguro
    if (std::abs(cmd_ticks) > SAFE_MAX_TICKS) {
      RCLCPP_WARN(get_logger(),
        "current_ticks=%d acotado a +/-%d por seguridad", cmd_ticks, SAFE_MAX_TICKS);
      cmd_ticks = (cmd_ticks > 0 ? SAFE_MAX_TICKS : -SAFE_MAX_TICKS);
    }
    cmd_ticks_ = static_cast<int16_t>(cmd_ticks);

    RCLCPP_INFO(get_logger(),
      "Probe joint%d (DXL ID %d)  current_ticks=%+d (%.4f N·m esperado)  dur=%.1fs",
      joint, static_cast<int>(DXL_ID[joint_idx_]),
      cmd_ticks_, cmd_ticks_ * TORQUE_PER_CURRENT_TICK, duration_s_);
    RCLCPP_WARN(get_logger(),
      "Solo se energiza joint%d. Sosten el eslabon con la mano antes de arrancar.", joint);

    if (!init_hardware()) {
      throw std::runtime_error("Fallo init hardware");
    }

    // Leer posicion cruda inicial
    raw_start_ = read_raw_position(DXL_ID[joint_idx_]);
    RCLCPP_INFO(get_logger(), "Posicion cruda inicial: %d ticks", raw_start_);

    start_time_ = std::chrono::high_resolution_clock::now();
    timer_ = this->create_wall_timer(50ms, [this]() { tick(); });  // 20 Hz
  }

  ~HWTorqueProbeNode()
  {
    if (timer_) timer_->cancel();
    shutdown_hardware();
  }

private:
  // ── Hardware: solo se configura/energiza el joint seleccionado ─────────────
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

    const uint8_t id = DXL_ID[joint_idx_];
    // Deshabilitar torque antes de cambiar modo
    dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "pre-disable");
    if (!dxl_write1(id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE, "set mode") ||
        !dxl_write2(id, ADDR_CURRENT_LIMIT,  CURRENT_LIMIT_REGISTER, "set limit") ||
        !dxl_write1(id, ADDR_TORQUE_ENABLE,  TORQUE_ENABLE_VAL, "enable torque")) {
      port_handler_->closePort();
      return false;
    }

    hw_active_ = true;
    RCLCPP_INFO(get_logger(), "Joint%d energizado (Current Mode). Resto: libre.",
      joint_idx_ + 1);
    return true;
  }

  void shutdown_hardware()
  {
    if (!hw_active_) return;
    hw_active_ = false;
    const uint8_t id = DXL_ID[joint_idx_];
    send_current(id, 0);
    rclcpp::sleep_for(std::chrono::milliseconds(20));
    dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "shutdown disable");
    if (port_handler_) {
      port_handler_->closePort();
      RCLCPP_INFO(get_logger(), "Puerto cerrado.");
    }
  }

  bool dxl_write1(uint8_t id, uint16_t addr, uint8_t val, const char * lbl)
  {
    uint8_t err = 0;
    const int r = packet_handler_->write1ByteTxRx(port_handler_, id, addr, val, &err);
    if (r != COMM_SUCCESS || err != 0) {
      RCLCPP_WARN(get_logger(), "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
      return false;
    }
    return true;
  }

  bool dxl_write2(uint8_t id, uint16_t addr, uint16_t val, const char * lbl)
  {
    uint8_t err = 0;
    const int r = packet_handler_->write2ByteTxRx(port_handler_, id, addr, val, &err);
    if (r != COMM_SUCCESS || err != 0) {
      RCLCPP_WARN(get_logger(), "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
      return false;
    }
    return true;
  }

  void send_current(uint8_t id, int16_t cur)
  {
    uint8_t err = 0;
    packet_handler_->write2ByteTxRx(port_handler_, id, ADDR_GOAL_CURRENT,
                                    static_cast<uint16_t>(cur), &err);
  }

  int32_t read_raw_position(uint8_t id)
  {
    uint32_t data = 0; uint8_t err = 0;
    packet_handler_->read4ByteTxRx(port_handler_, id, ADDR_PRESENT_POSITION, &data, &err);
    return toSigned32(data);
  }

  int16_t read_raw_current(uint8_t id)
  {
    uint16_t data = 0; uint8_t err = 0;
    packet_handler_->read2ByteTxRx(port_handler_, id, ADDR_PRESENT_CURRENT, &data, &err);
    return toSigned16(data);
  }

  // ── Tick: aplica corriente fija y registra el movimiento crudo ─────────────
  void tick()
  {
    if (!hw_active_) return;
    const auto tp = std::chrono::high_resolution_clock::now();
    const double t = std::chrono::duration<double>(tp - start_time_).count();
    const uint8_t id = DXL_ID[joint_idx_];

    if (t >= duration_s_) {
      finish();
      return;
    }

    send_current(id, cmd_ticks_);

    const int32_t raw = read_raw_position(id);
    const int16_t cur = read_raw_current(id);
    const int32_t draw = raw - raw_start_;

    if (++log_cnt_ % 4 == 0) {  // ~5 Hz a consola
      RCLCPP_INFO(get_logger(),
        "t=%.1fs  cmd=%+d ticks  raw=%d (Δ=%+d ticks = %+.1f°)  i_meas=%+d",
        t, cmd_ticks_, raw, draw, draw * POS_UNIT_DEG, cur);
    }
  }

  void finish()
  {
    const uint8_t id = DXL_ID[joint_idx_];
    const int32_t raw_end = read_raw_position(id);
    const int32_t draw = raw_end - raw_start_;
    send_current(id, 0);
    timer_->cancel();

    RCLCPP_INFO(get_logger(), "════════════════ RESULTADO joint%d ════════════════",
      joint_idx_ + 1);
    RCLCPP_INFO(get_logger(),
      "  Goal_Current comandado : %+d ticks", cmd_ticks_);
    RCLCPP_INFO(get_logger(),
      "  Δ Present_Position crudo: %+d ticks (%+.1f°)", draw, draw * POS_UNIT_DEG);
    if (std::abs(draw) < 5) {
      RCLCPP_WARN(get_logger(),
        "  Movimiento casi nulo: sube |current_ticks| o reduce la carga/friccion.");
    } else {
      const char * rel = (cmd_ticks_ > 0) == (draw > 0)
        ? "Goal_Current (+) -> Present_Position SUBE  (convencion estandar Dynamixel)"
        : "Goal_Current (+) -> Present_Position BAJA  (polaridad invertida)";
      RCLCPP_INFO(get_logger(), "  Relacion: %s", rel);
    }
    RCLCPP_INFO(get_logger(),
      "  Anota la direccion FISICA que sentiste (arriba/abajo) para fijar CURRENT_SIGN.");
    RCLCPP_INFO(get_logger(), "════════════════════════════════════════════════════");

    shutdown_hardware();
    rclcpp::shutdown();
  }

  // ── Miembros ───────────────────────────────────────────────────────────────
  std::string port_name_;
  int      joint_idx_{1};
  int16_t  cmd_ticks_{30};
  double   duration_s_{3.0};
  int32_t  raw_start_{0};

  dynamixel::PortHandler  * port_handler_{nullptr};
  dynamixel::PacketHandler * packet_handler_{nullptr};

  bool hw_active_;
  int  log_cnt_{0};
  std::chrono::high_resolution_clock::time_point start_time_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HWTorqueProbeNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_torque_probe_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
