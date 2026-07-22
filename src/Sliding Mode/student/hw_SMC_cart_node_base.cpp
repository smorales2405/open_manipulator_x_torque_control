/*
 * hw_SMC_cart_node_base.cpp
 * Control por Modo Deslizante en espacio cartesiano — OpenMANIPULATOR-X
 * hardware real vía Dynamixel SDK directo, sin ros2_control.
 * [Archivo base para estudiantes — Lab 6, implementacion]
 *
 * Soporta DOS leyes de control, seleccionables con el parametro booleano
 * use_integral (default: false):
 *   · use_integral:=false → SMC clasico (superficie en velocidad)
 *   · use_integral:=true  → ISMC, SMC con termino INTEGRAL (superficie en
 *     posicion con estado xi = ∫e dt) — elimina los sesgos DC que el SMC
 *     clasico deja por el residual del modelo de gravedad/COM del robot.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * SECCIONES A COMPLETAR:
 *   [1] Trayectoria cartesiana de referencia  →  desiredTrajectory()
 *   [2] Vector Y_START                        →  constante Y_START
 *   [3] Funciones de conmutacion              →  rho_scalar()
 *   [4] Ganancias SMC clasico                 →  LAMBDA_Y, K_V, K_S
 *   [5] Ganancias ISMC                        →  LAMBDA_I, K_P, K_D, K_sw
 *   [6] Ley SMC clasica                       →  compute_control() (rama false)
 *   [7] Ley ISMC                              →  compute_control() (rama true)
 *   [8] Torque                                →  compute_control() (comun)
 * El resto (hardware Dynamixel, filtro alpha-beta, transicion quintica,
 * DLS, integracion de xi con anti-windup, conversion torque→corriente,
 * guardas de seguridad, CSV) es INFRAESTRUCTURA: no modificar.
 * ──────────────────────────────────────────────────────────────────────────
 *
 * Salida de tarea:  y = [x, y, z, phi]^T
 *   (x,y,z) posición cartesiana del efector final (frame end_effector_link).
 *   phi = q2 + q3 + q4  ángulo de inclinación analítico.
 *
 * Ley SMC clasica (use_integral:=false):
 *   e_y  = y - y_d,   edot_y = ydot - ydot_d
 *   s_y  = edot_y + Lambda_y * e_y                       [m/s | rad/s]
 *   v_y  = yddot_d - Lambda_y*edot_y - K_v*s_y - K_s*rho(s_y)
 *   qddot = J4^T (J4 J4^T + lambda^2 I)^-1 (v_y - Jdot*qdot)   [DLS]
 *   tau   = M(q)*qddot + Phi(q,qdot)
 *
 * Ley ISMC (use_integral:=true), formulacion de hw_SMCI_cart_node:
 *   xi   = ∫ e dt  (lo integra la INFRAESTRUCTURA, con anti-windup)
 *   s    = e + Lambda_I * xi                             [m | rad]
 *   sdot = edot + Lambda_I * e
 *   v_s  = yddot_d - Lambda_I*edot - Jdot*qdot - K_D*sdot - K_P*s - K_sw*rho(s)
 *   qddot = J4^T (J4 J4^T + lambda^2 I)^-1 v_s           [DLS]
 *   tau   = M(q)*qddot + Phi(q,qdot)
 *
 * NOTA sobre la friccion: a diferencia de los nodos gz (feedforward del URDF
 * a nivel de torque), en hardware la friccion REAL se compensa en el modelo
 * torque→corriente identificado (Fv, Fc en ticks, con Fc alimentado por la
 * velocidad articular DESEADA qdot_des = J4⁺·ydot_des). NO agregar tau_fric
 * del URDF aqui: seria doble compensacion. El residual de friccion es la
 * incertidumbre acotada que el termino de conmutacion debe dominar.
 *
 * Funciones de conmutacion (elemento a elemento):
 *   "sign" -> rho(s) = sign(s)
 *   "sat"  -> rho(s) = sat(s/phi)     phi: capa limite [m/s o rad/s]
 *
 * Trayectoria cartesiana de referencia (serie de Fourier w=1.0 rad/s,
 * identica a gz_SMC_cart_node; FK de la trayectoria articular del Lab 6):
 *   x_d   = 0.172 + 0.032*sin(t') + 0.027*cos(2t') + 0.003*sin(3t')
 *   y_d   = 0.015 + 0.136*sin(t') - 0.014*cos(2t') + 0.003*sin(3t') - 0.001*cos(4t')
 *   z_d   = 0.128 - 0.008*sin(t') + 0.006*cos(2t')      [z ∈ 0.114..0.142 m]
 *   phi_d = 0.685 + 0.250*sin(t')
 * Transicion inicial [0, T_TRANS): quintica generalizada desde la pose
 * medida hasta Y_START = FK(q_d(0)) con empalme C2 (velocidad y aceleracion
 * de la serie en t'=0).
 *
 * Fases temporales completas (ver constantes T_TRANS/RETURN_TIME_S/HOLD_TIME_S):
 *   [0, T_TRANS)                                  transicion quintica a Y_START
 *   [T_TRANS, T_TRANS+t_run)                      seguimiento periodico
 *   [T_TRANS+t_run, T_TRANS+t_run+RETURN_TIME_S)  retorno quintico a y_inicial
 *   [...+RETURN_TIME_S, ...+HOLD_TIME_S)          pausa en reposo (asienta)
 *   >= ...+RETURN_TIME_S+HOLD_TIME_S              corte de corriente y fin
 * El retorno evita que el brazo caiga por gravedad al recibir torque cero de
 * golpe en medio de la trayectoria (t_run<=0 desactiva el retorno: corre
 * indefinidamente). El estado integral xi_ (si use_integral:=true) NO se
 * actualiza durante la transicion, el retorno ni la pausa (solo durante el
 * seguimiento). El CSV registra la transicion inicial + el seguimiento (en
 * hardware las fallas suelen ocurrir en la transicion); plots_SMC_cart.m en
 * modo 'real' descarta t < T_TRANS para que las metricas sean solo de
 * seguimiento. El retorno y la pausa final NO se guardan en el CSV.
 *
 * PROCEDIMIENTO DE ARRANQUE: posicionar el brazo a mano cerca de Y_START
 * (codo arriba, efector a ~0.20 m y ~0.13 m de altura) antes de lanzar el
 * nodo. Arrancar desde la pose de reposo caida (brazo extendido-bajo) exige
 * levantar el brazo extendido contra gravedad+stiction durante la
 * transicion y fue la causa de la divergencia del test1 hw.
 *
 * Guardas de altura mínima (placa metálica de la base):
 *   La referencia y el efector medido nunca pueden bajar de Z_MIN_FLOOR
 *   (0.075 m; la referencia baja hasta 0.114 → margen de tracking de ~4 cm).
 *   Durante la transición solo se exige no descender por debajo de la pose
 *   inicial (el brazo puede arrancar plegado, con z < Z_MIN_FLOOR).
 *
 * Guarda de singularidad:
 *   Parada de emergencia si cond(J4) > COND_J_MAX (150). En el test1 hw la
 *   configuracion derivo a una singularidad (kJ 23 → 18869) y la DLS
 *   repartio comandos grandes en direcciones mal condicionadas hasta llevar
 *   J1 a su limite; esta guarda corta esa divergencia ~1 s antes.
 *
 * Parametros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name          [string]  "/dev/ttyUSB0"
 *   use_integral       [bool]    false  (false = SMC clasico | true = ISMC)
 *   gain_scale         [double]  1.0    (escala lineal de las ganancias de la
 *                                        ley activa; Lambdas fijas)
 *   t_run              [double]  20.0   (duracion del SEGUIMIENTO [s];
 *                                        total = T_TRANS + t_run)
 *   test_num           [int]     1      (CSV: hw_smc_cart_... o hw_smci_cart_...
 *                                        segun use_integral)
 *   rho_func           [string]  "sat"  (funcion de conmutacion: "sign" | "sat")
 *   phi                [double]  0.3    (capa limite para sat(s/phi); OJO con
 *                                        las unidades: SMC clasico s en [m/s |
 *                                        rad/s] → usar ~0.3; ISMC s en [m |
 *                                        rad] → usar ~0.02, -p phi:=0.02)
 *   ab_alpha           [double]  0.2    (filtro α-β: ganancia de posicion)
 *   ab_beta            [double]  0.02   (filtro α-β: ganancia de velocidad)
 *   friction_fc_scale  [double]  0.95   (fraccion de Fc compensada)
 *   loop_rate_hz       [double]  200.0  (frecuencia del lazo [Hz], acotada [50,400])
 *
 * Parametros del modelo identificado (auto-cargados desde
 * config/motorXM430W350T_params.yaml): motor_alpha, motor_Fv, motor_Fc,
 * motor_I_offset, motor_epsilon_friction, joint_zero_tick, encoder_sign,
 * current_sign, joint_lower/upper, current_*_limit, tau_max.
 *
 * CSV: data/lab6/real/act2/hw_smc_cart_<rho_func>_<test_num>.csv   (SMC)
 *      data/lab6/real/act2/hw_smci_cart_<rho_func>_<test_num>.csv  (ISMC)
 * Columnas (compatibles con plots_SMC_cart.m, mode='real',
 * controller='smc'|'smci' segun use_integral):
 *   t, q1..q4, x,y,z,phi, x_des..phi_des, xdot..phidot, xdot_des..phidot_des,
 *   s1..s4, tau1..tau4, cond_J, dq1..dq4, dq1_filt..dq4_filt,
 *   curr_cmd1..4, curr_meas1..4, xi1..xi4
 * (Sat% se calcula en MATLAB desde tau con el criterio >= 0.99*tau_max.)
 *
 * Publisher: /hw/joint_states (sensor_msgs/JointState) — monitoreo
 *
 * Ejemplos de ejecucion (primer run SIEMPRE con gain_scale:=0.5 y el brazo
 * pre-posicionado a mano cerca de Y_START):
 *   # SMC clasico:
 *   ros2 run open_manipulator_x_torque_control hw_smc_cart_node_base \
 *     --ros-args -p gain_scale:=0.5 -p rho_func:=sat -p test_num:=1 -p t_run:=30.0
 *
 *   # ISMC (termino integral habilitado; phi en unidades de POSICION):
 *   ros2 run open_manipulator_x_torque_control hw_smc_cart_node_base \
 *     --ros-args -p use_integral:=true -p phi:=0.02 -p rho_func:=sat \
 *     -p test_num:=2 -p t_run:=30.0
 *
 * ADVERTENCIA: No ejecutar junto a hardware.launch.py ni ningun proceso
 * que acceda a /dev/ttyUSB0 (ros2_control_node, dynamixel_hardware_interface).
 */

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "dynamixel_sdk/dynamixel_sdk.h"

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif
#ifndef PACKAGE_CONFIG_DIR
#define PACKAGE_CONFIG_DIR "."
#endif

using namespace std::chrono_literals;

// ============================================================
// Constantes Dynamixel / conversión de unidades
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
static constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
static constexpr uint16_t ADDR_PRESENT_POSITION = 132;

static constexpr uint16_t LEN_GOAL_CURRENT     = 2;
static constexpr uint16_t LEN_PRESENT_CURRENT  = 2;
static constexpr uint16_t LEN_PRESENT_VELOCITY = 4;
static constexpr uint16_t LEN_PRESENT_POSITION = 4;

// Bloque contiguo current(126,2)+velocity(128,4)+position(132,4): una sola
// GroupSyncRead por ciclo en vez de tres → latencia de bus ≈ 1/3.
static constexpr uint16_t ADDR_STATE_BLOCK = ADDR_PRESENT_CURRENT;
static constexpr uint16_t LEN_STATE_BLOCK  = 10;

static constexpr uint8_t CURRENT_CONTROL_MODE = 0;
static constexpr uint8_t TORQUE_ENABLE_VAL    = 1;
static constexpr uint8_t TORQUE_DISABLE_VAL   = 0;

static constexpr double POS_UNIT_RAD           = 2.0 * PI / 4096.0;
static constexpr double VEL_UNIT_RAD_S         = 0.229 * 2.0 * PI / 60.0;
static constexpr double CURRENT_UNIT_A         = 0.00269;
static constexpr double TORQUE_CONSTANT_NM_A   = 1.654;
static constexpr double TORQUE_PER_CURRENT_TICK = TORQUE_CONSTANT_NM_A * CURRENT_UNIT_A;

// JOINT_ZERO_TICK, ENCODER_SIGN, CURRENT_SIGN, JOINT_LOWER/UPPER,
// CURRENT_LIMIT_REGISTER, CURRENT_CMD_LIMIT, CURRENT_MEASURED_PEAK, TAU_MAX
// → cargados desde config/motorXM430W350T_params.yaml como parámetros ROS 2.

// Amortiguamiento DLS para pseudo-inversa (evita divergencia en singularidades)
static constexpr double LAMBDA_DLS    = 0.01;
static constexpr double LAMBDA_DLS_SQ = LAMBDA_DLS * LAMBDA_DLS;

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 2] Vector Y_START — COMPLETAR
//
//  Y_START es la pose cartesiana [x, y, z, phi] de inicio de la trayectoria
//  de referencia, obtenida de la cinematica directa FK evaluada en q_d(0):
//    Y_START = FK(q_d(0))
//
//  Formato: {x [m], y [m], z [m], phi [rad]}
// ─────────────────────────────────────────────────────────────────────────────
static const Eigen::Vector4d Y_START {0.0, 0.0, 0.0, 0.0};  // COMPLETAR

// Transicion mas larga que en sim (5 s vs 3 s): levantar el brazo real
// contra gravedad+stiction con demandas suaves. (No modificar)
static constexpr double T_TRANS = 5.0;   // [s]
// Retorno con la misma duracion que la transicion: bajar el brazo tambien
// enfrenta gravedad+stiction. (No modificar)
static constexpr double RETURN_TIME_S = 5.0;   // [s]
static constexpr double HOLD_TIME_S   = 0.5;   // pausa en reposo antes de cortar corriente

// Umbral de parada por mal condicionamiento del Jacobiano de tarea
// (no modificar)
static constexpr double COND_J_MAX = 150.0;

// ── Anti-windup del estado integral xi (INFRAESTRUCTURA, no modificar) ──────
// Tope POR CANAL |xi_i| <= XI_MAX_i (~2-3x el xi de equilibrio esperado por
// los sesgos del robot; un tope holgado deja que Lambda*xi almacene una
// falsa referencia y desestabilice el brazo en los picos de demanda).
static const Eigen::Vector4d XI_MAX = {0.015, 0.015, 0.015, 0.03};  // [m·s | rad·s]
// Compuerta POR CANAL: xi_i solo integra con |s_i| <= gate_i (en unidades de
// POSICION). x/y/z: 0.02 m (engrana en regimen, se congela en las rafagas de
// saturacion). phi: 0.15 rad (su sesgo tipico ~0.11 debe quedar dentro).
static const Eigen::Vector4d XI_S_GATE = {0.02, 0.02, 0.02, 0.15};  // [m | rad]

// Altura mínima del efector sobre la placa metálica de la base [m].
// Se verifica en cada tick para la referencia y para el efector medido.
// La referencia baja hasta z=0.114 → margen de tracking de ~4 cm antes
// de la parada de emergencia.
static constexpr double Z_MIN_FLOOR = 0.075;

static constexpr char EFF_FRAME_NAME[] = "end_effector_link";

using Vec4 = Eigen::Matrix<double, NUM_JOINTS, 1>;

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 4] GANANCIAS SMC CLASICO  [x, y, z, phi] — COMPLETAR
//  (se usan cuando use_integral:=false)
//
//  En HARDWARE la stiction real exige rigidez de tarea varias veces mayor
//  que en Gazebo. Dentro de la capa el SMC equivale a un PD con:
//    kp_eq = Lambda*(K_V + K_S/phi)   kd_eq = Lambda + K_V + K_S/phi
//  Apuntar a la rigidez validada por FL: kp_eq ~ [400, 200, 800, 1500].
//
//  Rangos recomendados (con phi = 0.3):
//    LAMBDA_Y : 10 – 40   [1/s]
//    K_V      : 8  – 30
//    K_S      : 1  – 5    [m/s² | rad/s²]
//  Primer run SIEMPRE con gain_scale:=0.5.
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d LAMBDA_Y = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_V      = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_S      = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
//  [SECCION 5] GANANCIAS ISMC  [x, y, z, phi] — COMPLETAR
//  (se usan cuando use_integral:=true)
//
//  Lambda_I: polo de la superficie  s = e + Lambda_I*xi        [1/s]
//  K_P:      ganancia proporcional sobre s en sddot            [1/s²]
//  K_D:      ganancia derivativa  sobre sdot en sddot          [1/s]
//  K_sw:     ganancia de conmutacion rho(s)                    [m/s² | rad/s²]
//
//  Dentro de la capa (rho = s/phi) la dinamica del error es un PID con:
//    kp_eq = Lambda_I*K_D + K_P + K_sw/phi     kd_eq = Lambda_I + K_D
//    ki_eq = (K_P + K_sw/phi)*Lambda_I
//  Apuntar a los mismos kp_eq/kd_eq del SMC clasico (la novedad es ki_eq,
//  que elimina los sesgos DC). Lambda_I ~ sqrt(kp_eq)/4 separa el polo
//  integral de la dinamica PD.
//
//  Rangos recomendados (con phi = 0.02 — ¡s en unidades de POSICION!):
//    LAMBDA_I : 3  – 12   [1/s]
//    K_P      : 50 – 500  [1/s²]
//    K_D      : 20 – 70   [1/s]
//    K_sw     : 1  – 10   [m/s² | rad/s²]
// ═══════════════════════════════════════════════════════════════════════════
static const Eigen::Vector4d LAMBDA_I = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_P      = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_D      = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
static const Eigen::Vector4d K_sw     = {0.0, 0.0, 0.0, 0.0};   // COMPLETAR
// ═══════════════════════════════════════════════════════════════════════════

// ============================================================
// Utilidades de conversión (idénticas a hw_io_control_node)
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

static int32_t wrappedTickDiff(int32_t raw, int32_t zero)
{
  int32_t d = raw - zero;
  while (d >  2048) d -= 4096;
  while (d < -2048) d += 4096;
  return d;
}

static void currentToBytes(int16_t cur, uint8_t p[2])
{
  const uint16_t w = static_cast<uint16_t>(cur);
  p[0] = DXL_LOBYTE(w);
  p[1] = DXL_HIBYTE(w);
}

static int16_t clampCurrent(double x, int16_t lim)
{
  return static_cast<int16_t>(std::lround(
    std::min(std::max(x, -static_cast<double>(lim)), static_cast<double>(lim))));
}

// ============================================================
// Trayectoria cartesiana (identica a gz_SMC_cart_node)
// ============================================================

struct CartRef {
  Eigen::Vector4d y, ydot, yddot;
};

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 1] Trayectoria cartesiana de referencia — COMPLETAR
//
//  Definir la trayectoria como una serie de Fourier (w = 1.0 rad/s, misma
//  trayectoria que en Gazebo):
//    y_d(t)    = a0 + sum_n [ An*cos(n*w*t) + Bn*sin(n*w*t) ]
//    ydot_d(t) = derivada analitica exacta de y_d(t)
//    yddot_d(t)= derivada analitica exacta de ydot_d(t)
//
//  Las variables auxiliares sN = sin(N*t) y cN = cos(N*t) ya estan
//  declaradas para facilitar la escritura de las expresiones.
// ─────────────────────────────────────────────────────────────────────────────
static CartRef desiredTrajectory(double t)
{
  const double s1 = std::sin(t);
  const double c1 = std::cos(t);
  const double s2 = std::sin(2.0 * t);
  const double c2 = std::cos(2.0 * t);
  const double s3 = std::sin(3.0 * t);
  const double c3 = std::cos(3.0 * t);
  const double s4 = std::sin(4.0 * t);
  const double c4 = std::cos(4.0 * t);
  (void)s1; (void)c1; (void)s2; (void)c2;   // suprimir warnings mientras la
  (void)s3; (void)c3; (void)s4; (void)c4;   // seccion esta incompleta

  CartRef ref;

  // COMPLETAR: posicion deseada y_d(t) = [x_d, y_d, z_d, phi_d]
  ref.y <<
      0.0,   // x_d   : COMPLETAR
      0.0,   // y_d   : COMPLETAR
      0.0,   // z_d   : COMPLETAR
      0.0;   // phi_d : COMPLETAR

  // COMPLETAR: velocidad deseada ydot_d(t) (derivada analitica de y_d)
  ref.ydot <<
      0.0,   // xdot_d   : COMPLETAR
      0.0,   // ydot_d   : COMPLETAR
      0.0,   // zdot_d   : COMPLETAR
      0.0;   // phidot_d : COMPLETAR

  // COMPLETAR: aceleracion deseada yddot_d(t) (derivada analitica de ydot_d)
  ref.yddot <<
      0.0,   // xddot_d   : COMPLETAR
      0.0,   // yddot_d   : COMPLETAR
      0.0,   // zddot_d   : COMPLETAR
      0.0;   // phiddot_d : COMPLETAR

  return ref;
}

// Polinomio quintico general: interpola (y,ydot,yddot) desde el estado de
// borde (y0,v0,a0) en t=0 hasta (yf,vf,af) en t=T. transitionTrajectory
// (transicion inicial) y returnTrajectory (retorno final) son casos
// particulares de este blend. INFRAESTRUCTURA: no modificar.
static CartRef quinticBlend(double t, double T,
                            const Eigen::Vector4d& y0, const Eigen::Vector4d& v0, const Eigen::Vector4d& a0,
                            const Eigen::Vector4d& yf, const Eigen::Vector4d& vf, const Eigen::Vector4d& af)
{
  const double tc = std::min(t, T);
  const double T2=T*T, T3=T2*T, T4=T3*T, T5=T4*T;
  const Eigen::Vector4d c0 = y0;
  const Eigen::Vector4d c1 = v0;
  const Eigen::Vector4d c2 = 0.5 * a0;
  const Eigen::Vector4d c3 = (20.0*(yf-y0) - (8.0*vf+12.0*v0)*T - (3.0*a0-af)*T2) / (2.0*T3);
  const Eigen::Vector4d c4 = (30.0*(y0-yf) + (14.0*vf+16.0*v0)*T + (3.0*a0-2.0*af)*T2) / (2.0*T4);
  const Eigen::Vector4d c5 = (12.0*(yf-y0) - (6.0*vf+6.0*v0)*T - (a0-af)*T2) / (2.0*T5);

  const double t2=tc*tc, t3=t2*tc, t4=t3*tc, t5=t4*tc;
  CartRef r;
  r.y     = c0 + c1*tc + c2*t2 + c3*t3 + c4*t4 + c5*t5;
  r.ydot  = c1 + 2.0*c2*tc + 3.0*c3*t2 + 4.0*c4*t3 + 5.0*c5*t4;
  r.yddot = 2.0*c2 + 6.0*c3*tc + 12.0*c4*t2 + 20.0*c5*t3;
  return r;
}

// Transicion inicial: parte en reposo (v0=a0=0) en y0 y llega a la
// trayectoria periodica (y,ydot,yddot)(0) — empalme C2 (ydot_d(0) != 0).
// INFRAESTRUCTURA: no modificar.
static CartRef transitionTrajectory(double t, const Eigen::Vector4d& y0, const CartRef& goal, double T)
{
  return quinticBlend(t, T, y0, Eigen::Vector4d::Zero(), Eigen::Vector4d::Zero(),
                      goal.y, goal.ydot, goal.yddot);
}

// Retorno final: parte del estado de la trayectoria periodica en el
// instante en que termina t_run (start = desiredTrajectory(t_run)) y llega
// en reposo (vf=af=0) a yf (y_inicial) — evita el frenazo/caida por corte
// de torque. INFRAESTRUCTURA: no modificar.
static CartRef returnTrajectory(double t, double T, const CartRef& start, const Eigen::Vector4d& yf)
{
  return quinticBlend(t, T, start.y, start.ydot, start.yddot,
                      yf, Eigen::Vector4d::Zero(), Eigen::Vector4d::Zero());
}

// ─────────────────────────────────────────────────────────────────────────────
//  [SECCION 3] Funciones de conmutacion — COMPLETAR
//
//  Implementar rho(s) para cada tipo:
//    sign : rho(s) = +1 si s > 0,  -1 si s < 0,  0 si s == 0
//    sat  : rho(s) = clamp(s / phi, -1, 1)        (phi: capa limite)
// ─────────────────────────────────────────────────────────────────────────────
enum class RhoFunc { SIGN, SAT };

static double rho_scalar(double s, RhoFunc func, double phi)
{
  (void)s; (void)phi;

  switch (func) {
    case RhoFunc::SIGN:
      return 0.0;  // COMPLETAR: sign(s)
    case RhoFunc::SAT:
      return 0.0;  // COMPLETAR: sat(s/phi) = clamp(s/phi, -1, 1)
    default:
      return 0.0;
  }
}

// Aplica rho_scalar elemento a elemento sobre el vector s (no modificar)
[[maybe_unused]] static Eigen::Vector4d rho_vec(const Eigen::Vector4d & s, RhoFunc func, double phi)
{
  Eigen::Vector4d r;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    r[i] = rho_scalar(s[i], func, phi);
  }
  return r;
}

// ============================================================
// Nodo ROS 2
// ============================================================

class HWSMCCartNode : public rclcpp::Node
{
public:
  explicit HWSMCCartNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("hw_smc_cart_node", opts),
    hw_active_(false), y0_initialized_(false)
  {
    // ── Parámetros ──────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",  "/dev/ttyUSB0");
    this->declare_parameter<bool>       ("use_integral", false);
    this->declare_parameter<double>     ("gain_scale",  1.0);
    this->declare_parameter<double>     ("t_run",       20.0);
    this->declare_parameter<int>        ("test_num",    1);
    this->declare_parameter<std::string>("rho_func",    "sat");
    this->declare_parameter<double>     ("phi",         0.3);

    using dvec = std::vector<double>;
    this->declare_parameter<dvec>("motor_alpha",            dvec{208.5, 208.5, 208.5, 208.5});
    this->declare_parameter<dvec>("motor_Fv",               dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<dvec>("motor_Fc",               dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<dvec>("motor_I_offset",         dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<double>("motor_epsilon_friction", 0.05);
    this->declare_parameter<double>("friction_fc_scale",      0.95);
    this->declare_parameter<double>("ab_alpha", 0.2);
    this->declare_parameter<double>("ab_beta",  0.02);
    this->declare_parameter<double>("loop_rate_hz", 200.0);

    using ivec = std::vector<int64_t>;
    this->declare_parameter<ivec>  ("joint_zero_tick",        ivec{2048, 2048, 2048, 2048});
    this->declare_parameter<dvec>  ("encoder_sign",           dvec{+1.0, +1.0, +1.0, +1.0});
    this->declare_parameter<dvec>  ("current_sign",           dvec{+1.0, +1.0, +1.0, +1.0});
    this->declare_parameter<dvec>  ("joint_lower",            dvec{-2.356194, -1.919862, -1.919862, -1.8});
    this->declare_parameter<dvec>  ("joint_upper",            dvec{+2.356194, +1.745329, +1.570796, +2.1});
    this->declare_parameter<int>   ("current_limit_register", 350);
    this->declare_parameter<ivec>  ("current_cmd_limit",      ivec{257, 257, 257, 257});
    this->declare_parameter<int>   ("current_measured_peak",  313);
    this->declare_parameter<double>("tau_max",                1.2);

    port_name_    = this->get_parameter("port_name").as_string();
    use_integral_ = this->get_parameter("use_integral").as_bool();
    gain_scale_   = this->get_parameter("gain_scale").as_double();
    t_run_        = this->get_parameter("t_run").as_double();
    const int test_num = this->get_parameter("test_num").as_int();
    phi_        = this->get_parameter("phi").as_double();
    const std::string rho_str = this->get_parameter("rho_func").as_string();

    if (rho_str == "sign") {
      rho_func_ = RhoFunc::SIGN;
      rho_str_  = "sign";
    } else {
      rho_func_ = RhoFunc::SAT;
      rho_str_  = "sat";
      if (rho_str != "sat") {
        RCLCPP_WARN(get_logger(),
          "rho_func '%s' desconocida — usando 'sat'", rho_str.c_str());
      }
    }

    auto load_vec4 = [this](const std::string& name) {
      auto v = get_parameter(name).as_double_array();
      return Vec4(v[0], v[1], v[2], v[3]);
    };
    motor_alpha_    = load_vec4("motor_alpha");
    motor_Fv_       = load_vec4("motor_Fv");
    motor_Fc_       = load_vec4("motor_Fc");
    motor_I_offset_ = load_vec4("motor_I_offset");
    motor_epsilon_  = get_parameter("motor_epsilon_friction").as_double();
    fc_scale_       = get_parameter("friction_fc_scale").as_double();

    ab_alpha_ = get_parameter("ab_alpha").as_double();
    ab_beta_  = get_parameter("ab_beta").as_double();
    loop_rate_hz_ = std::min(std::max(get_parameter("loop_rate_hz").as_double(), 50.0), 400.0);
    Ts_ = 1.0 / loop_rate_hz_;

    {
      const auto zt = get_parameter("joint_zero_tick").as_integer_array();
      for (int i = 0; i < NUM_JOINTS; ++i) joint_zero_tick_[i] = static_cast<int32_t>(zt[i]);
    }
    encoder_sign_ = load_vec4("encoder_sign");
    current_sign_ = load_vec4("current_sign");
    joint_lower_  = load_vec4("joint_lower");
    joint_upper_  = load_vec4("joint_upper");
    current_limit_register_ = static_cast<uint16_t>(get_parameter("current_limit_register").as_int());
    {
      const auto cl = get_parameter("current_cmd_limit").as_integer_array();
      for (int i = 0; i < NUM_JOINTS; ++i) current_cmd_limit_[i] = static_cast<int16_t>(cl[i]);
    }
    current_measured_peak_ = static_cast<int16_t>(get_parameter("current_measured_peak").as_int());
    tau_max_ = get_parameter("tau_max").as_double();

    RCLCPP_INFO(get_logger(), "puerto=%s  gain=%.2f  t_run(seguimiento)=%.1fs  test=%d",
      port_name_.c_str(), gain_scale_, t_run_, test_num);
    RCLCPP_INFO(get_logger(),
      "%s cartesiano hw — rho=%s  phi=%.4f  tau_max=%.2f N·m  T_trans=%.1f s",
      use_integral_ ? "ISMC" : "SMC",
      rho_str_.c_str(), phi_, tau_max_, T_TRANS);
    if (use_integral_) {
      RCLCPP_INFO(get_logger(),
        "Lambda_I=[%.1f %.1f %.1f %.1f]  K_P=[%.0f %.0f %.0f %.0f]  "
        "K_D=[%.0f %.0f %.0f %.0f]  K_sw=[%.1f %.1f %.1f %.1f]  lambda_DLS=%.3f",
        LAMBDA_I[0], LAMBDA_I[1], LAMBDA_I[2], LAMBDA_I[3],
        K_P[0],      K_P[1],      K_P[2],      K_P[3],
        K_D[0],      K_D[1],      K_D[2],      K_D[3],
        K_sw[0],     K_sw[1],     K_sw[2],     K_sw[3],
        LAMBDA_DLS);
      RCLCPP_INFO(get_logger(),
        "XI_max=[%.3f %.3f %.3f %.3f]  XI_s_gate=[%.2f %.2f %.2f %.2f]",
        XI_MAX[0], XI_MAX[1], XI_MAX[2], XI_MAX[3],
        XI_S_GATE[0], XI_S_GATE[1], XI_S_GATE[2], XI_S_GATE[3]);
    } else {
      RCLCPP_INFO(get_logger(),
        "Lambda_y=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  Ks=[%.1f %.1f %.1f %.1f]  lambda_DLS=%.3f",
        LAMBDA_Y[0], LAMBDA_Y[1], LAMBDA_Y[2], LAMBDA_Y[3],
        K_V[0],      K_V[1],      K_V[2],      K_V[3],
        K_S[0],      K_S[1],      K_S[2],      K_S[3],
        LAMBDA_DLS);
    }
    RCLCPP_INFO(get_logger(),
      "Y_start=[%.3f %.3f %.3f %.3f]  Z_min_floor=%.3f m",
      Y_START[0], Y_START[1], Y_START[2], Y_START[3], Z_MIN_FLOOR);
    RCLCPP_INFO(get_logger(),
      "motor α=[%.1f %.1f %.1f %.1f]  Fv=[%.2f %.2f %.2f %.2f]  ε=%.3f",
      motor_alpha_(0), motor_alpha_(1), motor_alpha_(2), motor_alpha_(3),
      motor_Fv_(0), motor_Fv_(1), motor_Fv_(2), motor_Fv_(3), motor_epsilon_);
    RCLCPP_INFO(get_logger(),
      "Filtro α-β: α=%.3f  β=%.4f  |  Fc_scale=%.2f  |  lazo=%.0f Hz",
      ab_alpha_, ab_beta_, fc_scale_, loop_rate_hz_);

    // ── Pinocchio ────────────────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    if (!model_.existFrame(EFF_FRAME_NAME)) {
      RCLCPP_FATAL(get_logger(), "Frame '%s' no encontrado en URDF", EFF_FRAME_NAME);
      throw std::runtime_error("Frame not found");
    }
    frame_id_ = model_.getFrameId(EFF_FRAME_NAME);
    RCLCPP_INFO(get_logger(), "Pinocchio: nv=%d  frame='%s' (id=%lu)",
      model_.nv, EFF_FRAME_NAME, frame_id_);

    // ── CSV (prefijo segun la ley activa; compatible con plots_SMC_cart.m
    //    controller='smc'|'smci') ────────────────────────────────────────────
    std::filesystem::create_directories(std::string(PACKAGE_DATA_DIR) + "/lab6/real/act2");
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab6/real/act2/"
                + (use_integral_ ? "hw_smci_cart_" : "hw_smc_cart_")
                + rho_str_ + "_" + std::to_string(test_num) + ".csv";
    csv_.open(csv_path_);
    if (csv_.is_open()) {
      csv_ << "t,q1,q2,q3,q4,"
              "x,y,z,phi,"
              "x_des,y_des,z_des,phi_des,"
              "xdot,ydot,zdot,phidot,"
              "xdot_des,ydot_des,zdot_des,phidot_des,"
              "s1,s2,s3,s4,"
              "tau1,tau2,tau3,tau4,"
              "cond_J,"
              "dq1,dq2,dq3,dq4,"
              "dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
              "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
              "curr_meas1,curr_meas2,curr_meas3,curr_meas4,"
              "xi1,xi2,xi3,xi4\n";
      RCLCPP_INFO(get_logger(), "CSV: %s", csv_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "No se pudo crear CSV: %s", csv_path_.c_str());
    }

    // ── Publisher de monitoreo ────────────────────────────────────────────────
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── SDK ──────────────────────────────────────────────────────────────────
    if (!init_hardware()) {
      RCLCPP_FATAL(get_logger(), "Fallo hardware init. Abortando.");
      throw std::runtime_error("Hardware init failed");
    }

    // ── Timer de control (loop_rate_hz) ──────────────────────────────────────
    start_time_ = std::chrono::high_resolution_clock::now();
    const auto period = std::chrono::microseconds(
      static_cast<int64_t>(std::lround(1e6 / loop_rate_hz_)));
    timer_ = this->create_wall_timer(period, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Control SMC cartesiano activo a %.0f Hz (Ts=%.1f ms). Ctrl+C para detener.",
      loop_rate_hz_, 1e3 * Ts_);
  }

  ~HWSMCCartNode()
  {
    if (timer_) timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
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

    for (const auto id : DXL_ID)
      dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE_VAL, "pre-disable");

    for (const auto id : DXL_ID) {
      if (!dxl_write1(id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE, "set mode") ||
          !dxl_write2(id, ADDR_CURRENT_LIMIT, current_limit_register_, "set limit") ||
          !dxl_write1(id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE_VAL, "enable torque")) {
        port_handler_->closePort();
        return false;
      }
      RCLCPP_INFO(get_logger(), "DXL ID %d listo", static_cast<int>(id));
    }

    grp_read_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_handler_, packet_handler_, ADDR_STATE_BLOCK, LEN_STATE_BLOCK);
    grp_wcur_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_handler_, packet_handler_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    for (const auto id : DXL_ID)
      grp_read_->addParam(id);

    hw_active_ = true;
    RCLCPP_INFO(get_logger(), "Hardware inicializado en %s", port_name_.c_str());
    return true;
  }

  void shutdown_hardware()
  {
    if (!hw_active_) return;
    hw_active_ = false;

    if (grp_wcur_) {
      const std::array<int16_t, NUM_JOINTS> zero = {0, 0, 0, 0};
      send_currents(zero);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
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

  bool dxl_write2(uint8_t id, uint16_t addr, uint16_t val, const char* lbl)
  {
    uint8_t err = 0;
    const int r = packet_handler_->write2ByteTxRx(port_handler_, id, addr, val, &err);
    if (r != COMM_SUCCESS || err != 0) {
      RCLCPP_WARN(get_logger(), "[ID %d] %s: r=%d err=%d", id, lbl, r, err);
      return false;
    }
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Lectura / escritura SDK
  // ─────────────────────────────────────────────────────────────────────────

  bool read_state(Vec4& q, Vec4& dq, std::array<int16_t, NUM_JOINTS>& cur)
  {
    if (grp_read_->txRxPacket() != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncRead fallo");
      return false;
    }
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const uint8_t id = DXL_ID[i];
      if (!grp_read_->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) ||
          !grp_read_->isAvailable(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY) ||
          !grp_read_->isAvailable(id, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT)) {
        RCLCPP_ERROR(get_logger(), "[ID %d] dato no disponible", id);
        return false;
      }
      const int32_t rp = toSigned32(static_cast<uint32_t>(
        grp_read_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)));
      const int32_t rv = toSigned32(static_cast<uint32_t>(
        grp_read_->getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)));
      const int16_t rc = toSigned16(static_cast<uint32_t>(
        grp_read_->getData(id, ADDR_PRESENT_CURRENT,  LEN_PRESENT_CURRENT)));

      q(i)   = encoder_sign_(i) * static_cast<double>(wrappedTickDiff(rp, joint_zero_tick_[i])) * POS_UNIT_RAD;
      dq(i)  = encoder_sign_(i) * static_cast<double>(rv) * VEL_UNIT_RAD_S;
      cur[i] = rc;
    }
    return true;
  }

  bool send_currents(const std::array<int16_t, NUM_JOINTS>& cmd)
  {
    uint8_t p[NUM_JOINTS][2];
    for (int i = 0; i < NUM_JOINTS; ++i) {
      currentToBytes(cmd[i], p[i]);
      grp_wcur_->addParam(DXL_ID[i], p[i]);
    }
    const int r = grp_wcur_->txPacket();
    grp_wcur_->clearParam();
    if (r != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "SyncWrite fallo");
      return false;
    }
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Conversión torque → corriente
  //  Fv usa la velocidad medida (término suave); Fc usa la velocidad articular
  //  DESEADA (mapeo DLS de ydot_des): señal sin ruido → sin chattering, y
  //  aporta el empuje de despegue cuando la referencia arranca.
  // ─────────────────────────────────────────────────────────────────────────

  std::array<int16_t, NUM_JOINTS> torque_to_current(const Vec4& tau, const Vec4& dq,
                                                    const Vec4& dq_des)
  {
    std::array<int16_t, NUM_JOINTS> cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double I_model = motor_alpha_(i) * tau(i)
                           + motor_Fv_(i)    * dq(i)
                           + fc_scale_ * motor_Fc_(i) * std::tanh(dq_des(i) / motor_epsilon_)
                           + motor_I_offset_(i);
      cmd[i] = clampCurrent(current_sign_(i) * encoder_sign_(i) * I_model, current_cmd_limit_[i]);
    }
    return cmd;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Ley de control cartesiana (SMC clasico o ISMC segun use_integral)
  //  gain_scale escala las ganancias de la ley activa; las Lambdas definen
  //  la superficie y no se escalan. Actualiza el estado integral xi_ (solo
  //  con use_integral:=true).
  // ─────────────────────────────────────────────────────────────────────────

  struct SMCOut {
    Vec4            tau;
    Vec4            dq_des;   // velocidad articular deseada (DLS de ydot_des)
    Eigen::Vector4d y_actual, ydot_actual, s_y;
    double          cond_J;
  };

  SMCOut compute_control(const Vec4& q, const Vec4& dq, const CartRef& ref,
                         bool in_tracking)
  {
    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head(NUM_JOINTS)  = q;
    dq_pin.head(NUM_JOINTS) = dq;

    // FK + Jacobiano 6×nv (infraestructura, no modificar)
    Eigen::MatrixXd J6 = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::computeFrameJacobian(model_, data_, q_pin, frame_id_,
                                     pinocchio::LOCAL_WORLD_ALIGNED, J6);

    const Eigen::Vector3d p   = data_.oMf[frame_id_].translation();
    const double phi_ee       = q[1] + q[2] + q[3];

    // Jacobiano de tarea 4×4
    Eigen::Matrix4d J4;
    J4.row(0) = J6.row(0).head<NUM_JOINTS>();
    J4.row(1) = J6.row(1).head<NUM_JOINTS>();
    J4.row(2) = J6.row(2).head<NUM_JOINTS>();
    J4.row(3) << 0.0, 1.0, 1.0, 1.0;

    const Eigen::Vector4d ydot = J4 * dq;

    // Término de bias  Jdot*qdot  (FK con qddot=0)
    pinocchio::forwardKinematics(model_, data_, q_pin, dq_pin,
                                  Eigen::VectorXd::Zero(model_.nv));
    const pinocchio::Motion bias =
      pinocchio::getFrameClassicalAcceleration(model_, data_, frame_id_,
                                                pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::Vector4d jdqd;
    jdqd << bias.linear()[0], bias.linear()[1], bias.linear()[2], 0.0;

    // Dinámica M(q) y Phi(q,qdot)
    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();
    pinocchio::nonLinearEffects(model_, data_, q_pin, dq_pin);

    const Eigen::Matrix4d M4   = data_.M.topLeftCorner<NUM_JOINTS, NUM_JOINTS>();
    const Eigen::Vector4d nle4 = data_.nle.head<NUM_JOINTS>();

    // Errores cartesianos (infraestructura)
    const Eigen::Vector4d y_actual{p[0], p[1], p[2], phi_ee};
    const Eigen::Vector4d e_y    = y_actual - ref.y;
    const Eigen::Vector4d edot_y = ydot     - ref.ydot;

    // Estado integral xi (INFRAESTRUCTURA, no modificar): solo con
    // use_integral y en fase de seguimiento (in_tracking — ni transicion,
    // ni retorno, ni pausa); compuerta por canal |s_i| <= XI_S_GATE_i,
    // congelado si el tick anterior saturo algun torque, y acotado a
    // |xi_i| <= XI_MAX_i.
    if (use_integral_ && in_tracking && !tau_saturated_prev_) {
      for (int i = 0; i < NUM_JOINTS; ++i) {
        const double s_pre = e_y[i] + LAMBDA_I[i] * xi_[i];
        if (std::abs(s_pre) <= XI_S_GATE[i]) {
          xi_[i] += e_y[i] * Ts_;
          xi_[i] = std::max(-XI_MAX[i], std::min(XI_MAX[i], xi_[i]));
        }
      }
    }

    // Pseudo-inversa DLS (infraestructura; factorizacion reutilizada)
    const Eigen::Matrix4d A = J4*J4.transpose() + LAMBDA_DLS_SQ*Eigen::Matrix4d::Identity();
    const Eigen::LDLT<Eigen::Matrix4d> A_ldlt = A.ldlt();

    Eigen::Vector4d s_y   = Eigen::Vector4d::Zero();
    Eigen::Vector4d qddot = Eigen::Vector4d::Zero();

    if (!use_integral_) {
      // ═════════════════════════════════════════════════════════════════════
      //  [SECCION 6] Ley SMC clasica — COMPLETAR
      //
      //  Disponible: e_y, edot_y, ref.yddot, jdqd, LAMBDA_Y, K_V, K_S,
      //              gain_scale_, rho_vec(s, rho_func_, phi_), A_ldlt, J4
      //
      //  6.1  s_y = edot_y + Lambda_y * e_y            (superficie, ya declarada)
      //  6.2  rho = rho_vec(s_y, rho_func_, phi_)
      //  6.3  v_y = yddot_d - Lambda_y*edot_y - k_v*s_y - k_s*rho
      //       (k_v = K_V*gain_scale_, k_s = K_S*gain_scale_)
      //  6.4  qddot = J4^T * A_ldlt.solve(v_y - jdqd)  (ya dada abajo)
      // ═════════════════════════════════════════════════════════════════════

      // COMPLETAR: s_y (asignar, no redeclarar), rho y v_y
      const Eigen::Vector4d v_y = Eigen::Vector4d::Zero();   // COMPLETAR

      // Aceleracion articular via DLS (no modificar):
      qddot = J4.transpose() * A_ldlt.solve(v_y - jdqd);
      // ═════════════════════════════════════════════════════════════════════
    } else {
      // ═════════════════════════════════════════════════════════════════════
      //  [SECCION 7] Ley ISMC — COMPLETAR
      //
      //  Disponible: e_y, edot_y, ref.yddot, jdqd, xi_ (ya actualizado por
      //              la infraestructura), LAMBDA_I, K_P, K_D, K_sw,
      //              gain_scale_, rho_vec(s, rho_func_, phi_), A_ldlt, J4
      //
      //  7.1  s_y  = e_y + Lambda_I * xi_              (superficie integral)
      //  7.2  sdot_y = edot_y + Lambda_I * e_y
      //  7.3  rho  = rho_vec(s_y, rho_func_, phi_)     (¡phi en unidades de posicion!)
      //  7.4  v_s  = yddot_d - Lambda_I*edot_y - jdqd
      //              - k_d*sdot_y - k_p*s_y - k_sw*rho
      //       (k_p = K_P*gain_scale_, etc.)
      //  7.5  qddot = J4^T * A_ldlt.solve(v_s)         (ya dada abajo)
      // ═════════════════════════════════════════════════════════════════════

      // COMPLETAR: s_y (asignar, no redeclarar), sdot_y, rho y v_s
      const Eigen::Vector4d v_s = Eigen::Vector4d::Zero();   // COMPLETAR

      // Aceleracion articular via DLS (no modificar):
      qddot = J4.transpose() * A_ldlt.solve(v_s);
      // ═════════════════════════════════════════════════════════════════════
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  [SECCION 8] Torque — COMPLETAR (comun a ambas leyes)
    //
    //  tau_unsat = M4 * qddot + nle4
    //  (en hardware SIN tau_fric del URDF: la friccion real se compensa en
    //   torque_to_current con el modelo identificado del motor)
    // ═══════════════════════════════════════════════════════════════════════
    const Eigen::Vector4d tau_unsat = Eigen::Vector4d::Zero();  // COMPLETAR

    // Suprimir warnings mientras las secciones estan incompletas
    // (eliminar al completarlas):
    (void)M4; (void)nle4; (void)qddot; (void)edot_y;
    // ═══════════════════════════════════════════════════════════════════════

    // Saturacion + flag de anti-windup (infraestructura, no modificar)
    const Eigen::Vector4d tau_sat = tau_unsat.cwiseMin(tau_max_).cwiseMax(-tau_max_);
    tau_saturated_prev_ = (tau_unsat.cwiseAbs().maxCoeff() >= tau_max_);

    // Condicionamiento del Jacobiano (monitoreo de singularidades)
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(J4);
    const double sigma_min = svd.singularValues()(NUM_JOINTS - 1);

    SMCOut out;
    out.tau         = tau_sat;
    out.dq_des      = J4.transpose() * A_ldlt.solve(ref.ydot);
    out.y_actual    = y_actual;
    out.ydot_actual = ydot;
    out.s_y         = s_y;
    out.cond_J      = (sigma_min > 1e-10) ? svd.singularValues()(0) / sigma_min : 1e10;
    return out;
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Parada de emergencia
  // ─────────────────────────────────────────────────────────────────────────

  void emergency_stop(const std::string& reason)
  {
    RCLCPP_ERROR(get_logger(), "PARADA: %s", reason.c_str());
    timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    rclcpp::shutdown();
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Callback del timer (loop_rate_hz)
  // ─────────────────────────────────────────────────────────────────────────

  void tick()
  {
    if (!hw_active_) return;

    const auto tick_t0 = std::chrono::steady_clock::now();
    const auto tp  = std::chrono::high_resolution_clock::now();
    const double t = std::chrono::duration<double>(tp - start_time_).count();

    const double t_shutdown = T_TRANS + t_run_ + RETURN_TIME_S + HOLD_TIME_S;
    if (t_run_ > 0.0 && t >= t_shutdown) {
      RCLCPP_INFO(get_logger(), "Retorno a y_inicial completado. Deteniendo (t=%.1f s).", t);
      timer_->cancel();
      shutdown_hardware();
      if (csv_.is_open()) { csv_.flush(); csv_.close(); }
      rclcpp::shutdown();
      return;
    }

    // 1. Lectura de estado
    Vec4 q, dq;
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    if (!read_state(q, dq, cur_meas)) {
      emergency_stop("SyncRead fallido");
      return;
    }

    // 2. Filtro α-β: estimación conjunta de posición y velocidad
    //   predicción: q_pred = q_hat + Ts·dq_hat,  dq_pred = dq_hat
    //   residuo:    r      = q_meas - q_pred
    //   corrección: q_hat  = q_pred + α·r,        dq_hat = dq_pred + (β/Ts)·r
    if (!ab_initialized_) {
      q_hat_          = q;
      dq_hat_         = dq;
      ab_initialized_ = true;
    } else {
      const Vec4 q_pred  = q_hat_ + Ts_ * dq_hat_;
      const Vec4 dq_pred = dq_hat_;
      const Vec4 r       = q - q_pred;
      q_hat_  = q_pred  + ab_alpha_ * r;
      dq_hat_ = dq_pred + (ab_beta_ / Ts_) * r;
    }

    Eigen::VectorXd q_pin  = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head(NUM_JOINTS)  = q;

    // 3. Capturar pose cartesiana inicial (primer tick)
    if (!y0_initialized_) {
      pinocchio::forwardKinematics(model_, data_, q_pin);
      pinocchio::updateFramePlacement(model_, data_, frame_id_);
      const Eigen::Vector3d p0 = data_.oMf[frame_id_].translation();
      const double phi0        = q[1] + q[2] + q[3];
      y0_ = Eigen::Vector4d{p0[0], p0[1], p0[2], phi0};
      y0_initialized_ = true;
      RCLCPP_INFO(get_logger(),
        "y_inicial=[%.3f %.3f %.3f %.3f]  Y_START=[%.3f %.3f %.3f %.3f]",
        y0_[0], y0_[1], y0_[2], y0_[3],
        Y_START[0], Y_START[1], Y_START[2], Y_START[3]);
    }

    // 4. Verificar límites articulares del estado real
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (q(i) < joint_lower_(i) || q(i) > joint_upper_(i)) {
        emergency_stop("Articulacion " + std::to_string(i+1) + " fuera de limites: "
                       + std::to_string(q(i)) + " rad");
        return;
      }
    }

    // 5. Referencia cartesiana: transicion quintica C2 hacia Y_START (con
    //    velocidad/aceleracion de la serie en t'=0), seguimiento, y retorno
    //    quintico final + pausa en reposo (ver RETURN_TIME_S/HOLD_TIME_S)
    CartRef goal0 = desiredTrajectory(0.0);
    goal0.y = Y_START;
    const bool in_trans    = (t < T_TRANS);
    const bool in_tracking = !in_trans && (t_run_ <= 0.0 || t < T_TRANS + t_run_);
    CartRef ref;
    if (in_trans) {
      ref = transitionTrajectory(t, y0_, goal0, T_TRANS);
    } else if (in_tracking) {
      ref = desiredTrajectory(t - T_TRANS);
    } else if (t < T_TRANS + t_run_ + RETURN_TIME_S) {
      if (!return_logged_) {
        RCLCPP_INFO(get_logger(), "Iniciando retorno quintico a y_inicial (t=%.1fs, dur=%.1fs)",
          t, RETURN_TIME_S);
        return_logged_ = true;
      }
      ref = returnTrajectory(t - (T_TRANS + t_run_), RETURN_TIME_S,
                            desiredTrajectory(t_run_), y0_);
    } else {
      ref.y = y0_;
      ref.ydot.setZero();
      ref.yddot.setZero();
    }

    // 5b. Guarda de altura mínima de la REFERENCIA (placa base metálica).
    //     Cerca de la pose inicial (transición de entrada, retorno final o
    //     pausa en reposo) el brazo puede estar bajo (y0_[2] < Z_MIN_FLOOR):
    //     solo se exige no descender más de 3 cm bajo la pose inicial.
    const bool near_home = in_trans || (t_run_ > 0.0 && t >= T_TRANS + t_run_);
    const double z_floor = near_home
      ? std::min(Z_MIN_FLOOR, y0_[2] - 0.03)
      : Z_MIN_FLOOR;
    if (ref.y[2] < z_floor) {
      emergency_stop("Referencia z bajo altura minima: "
                     + std::to_string(ref.y[2]) + " m (piso " + std::to_string(z_floor) + ")");
      return;
    }

    // 6. Ley de control (SMC clasico o ISMC) — usa dq_hat_ para reducir
    //    chattering; actualiza el estado integral xi_ internamente (solo
    //    en in_tracking)
    const SMCOut ctrl = compute_control(q, dq_hat_, ref, in_tracking);

    // 6b. Guarda de altura mínima del EFECTOR medido (FK de Pinocchio)
    if (ctrl.y_actual[2] < z_floor) {
      emergency_stop("Efector bajo altura minima: z="
                     + std::to_string(ctrl.y_actual[2]) + " m (piso " + std::to_string(z_floor) + ")");
      return;
    }

    // 6c. Guarda de singularidad: cerca de una singularidad la DLS reparte
    //     comandos grandes en direcciones mal condicionadas (test1 hw:
    //     kJ 23 → 18869 y J1 se fue a su limite).
    if (ctrl.cond_J > COND_J_MAX) {
      emergency_stop("Jacobiano mal condicionado: cond(J4)="
                     + std::to_string(ctrl.cond_J) + " (umbral " + std::to_string(COND_J_MAX) + ")");
      return;
    }

    // 7. Conversión torque → corriente (Fc con la velocidad articular deseada)
    const auto cur_cmd = torque_to_current(ctrl.tau, dq_hat_, ctrl.dq_des);

    // 8. Escribir corriente
    if (!send_currents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 9. Verificar corriente medida
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > current_measured_peak_) {
        emergency_stop("Corriente insegura J" + std::to_string(i+1)
                       + ": " + std::to_string(cur_meas[i]) + " ticks");
        return;
      }
    }

    // 10. Publicar JointState de monitoreo
    {
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();
      js.name         = {"joint1", "joint2", "joint3", "joint4"};
      js.position     = {q(0), q(1), q(2), q(3)};
      js.velocity     = {dq(0), dq(1), dq(2), dq(3)};
      js.effort = {
        static_cast<double>(cur_meas[0]) * TORQUE_PER_CURRENT_TICK,
        static_cast<double>(cur_meas[1]) * TORQUE_PER_CURRENT_TICK,
        static_cast<double>(cur_meas[2]) * TORQUE_PER_CURRENT_TICK,
        static_cast<double>(cur_meas[3]) * TORQUE_PER_CURRENT_TICK
      };
      js_pub_->publish(js);
    }

    // 11. CSV (incluye la transicion inicial y el seguimiento; excluye el
    //     retorno final a y_inicial y la pausa en reposo. En hardware las
    //     fallas suelen ocurrir en la transicion; plots_SMC_cart.m modo
    //     'real' descarta t < T_TRANS)
    const bool in_return_or_hold = (t_run_ > 0.0 && t >= T_TRANS + t_run_);
    if (csv_.is_open() && !in_return_or_hold) {
      csv_ << std::fixed << std::setprecision(6)
           << t
           << ',' << q(0) << ',' << q(1) << ',' << q(2) << ',' << q(3)
           << ',' << ctrl.y_actual[0] << ',' << ctrl.y_actual[1]
           << ',' << ctrl.y_actual[2] << ',' << ctrl.y_actual[3]
           << ',' << ref.y[0] << ',' << ref.y[1] << ',' << ref.y[2] << ',' << ref.y[3]
           << ',' << ctrl.ydot_actual[0] << ',' << ctrl.ydot_actual[1]
           << ',' << ctrl.ydot_actual[2] << ',' << ctrl.ydot_actual[3]
           << ',' << ref.ydot[0] << ',' << ref.ydot[1]
           << ',' << ref.ydot[2] << ',' << ref.ydot[3]
           << ',' << ctrl.s_y[0] << ',' << ctrl.s_y[1]
           << ',' << ctrl.s_y[2] << ',' << ctrl.s_y[3]
           << ',' << ctrl.tau[0] << ',' << ctrl.tau[1]
           << ',' << ctrl.tau[2] << ',' << ctrl.tau[3]
           << ',' << ctrl.cond_J
           << ',' << dq(0) << ',' << dq(1) << ',' << dq(2) << ',' << dq(3)
           << ',' << dq_hat_(0) << ',' << dq_hat_(1) << ',' << dq_hat_(2) << ',' << dq_hat_(3)
           << ',' << cur_cmd[0] << ',' << cur_cmd[1]
           << ',' << cur_cmd[2] << ',' << cur_cmd[3]
           << ',' << cur_meas[0] << ',' << cur_meas[1]
           << ',' << cur_meas[2] << ',' << cur_meas[3]
           << ',' << xi_[0] << ',' << xi_[1] << ',' << xi_[2] << ',' << xi_[3]
           << '\n';
    }

    // 12. Log periódico (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      const char* phase = in_trans    ? "TRANS" :
                          in_tracking ? "TRAY " :
                          (t < T_TRANS + t_run_ + RETURN_TIME_S) ? "RET  " : "HOME ";
      RCLCPP_INFO(get_logger(),
        "[%s] t=%.2fs  |s|=%.4f  |ey|=%.4f m  |xi|=%.4f  kJ=%.1f  i=[%d %d %d %d]",
        phase, t, ctrl.s_y.norm(), (ctrl.y_actual - ref.y).norm(), xi_.norm(),
        ctrl.cond_J, cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }

    // 13. Detección de overrun: si el ciclo (bus + control) no cabe en Ts,
    //     el lazo real corre más lento de lo configurado → bajar loop_rate_hz
    //     o revisar el latency_timer del adaptador USB-serial.
    const double tick_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - tick_t0).count();
    if (tick_ms > 1e3 * Ts_) {
      if (++overrun_cnt_ % 50 == 1) {
        RCLCPP_WARN(get_logger(), "Overrun del lazo: tick=%.2f ms > Ts=%.1f ms (n=%d)",
          tick_ms, 1e3 * Ts_, overrun_cnt_);
      }
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  //  Miembros
  // ─────────────────────────────────────────────────────────────────────────

  pinocchio::Model      model_;
  pinocchio::Data       data_;
  pinocchio::FrameIndex frame_id_;

  std::string port_name_;
  double gain_scale_, t_run_;
  RhoFunc     rho_func_{RhoFunc::SAT};
  std::string rho_str_;
  double      phi_;

  Vec4   motor_alpha_, motor_Fv_, motor_Fc_, motor_I_offset_;
  double motor_epsilon_;
  double fc_scale_;

  std::array<int32_t, NUM_JOINTS> joint_zero_tick_;
  Vec4     encoder_sign_, current_sign_;
  Vec4     joint_lower_, joint_upper_;
  uint16_t current_limit_register_;
  std::array<int16_t, NUM_JOINTS> current_cmd_limit_;
  int16_t  current_measured_peak_;
  double   tau_max_;

  double ab_alpha_, ab_beta_;
  double loop_rate_hz_{200.0};
  double Ts_{0.005};
  Vec4   q_hat_, dq_hat_;
  bool   ab_initialized_{false};

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead>  grp_read_;
  std::unique_ptr<dynamixel::GroupSyncWrite> grp_wcur_;

  bool hw_active_;
  bool y0_initialized_;
  Eigen::Vector4d y0_;
  bool return_logged_{false};

  bool            use_integral_{false};
  Eigen::Vector4d xi_{Eigen::Vector4d::Zero()};  // estado integral ∫e dt
  bool            tau_saturated_prev_{false};    // anti-windup condicional

  std::chrono::high_resolution_clock::time_point start_time_;
  int log_cnt_{0};
  int overrun_cnt_{0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::ofstream csv_;
  std::string   csv_path_;
};

// ============================================================
// main
// ============================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  const std::string cfg = std::string(PACKAGE_CONFIG_DIR) + "/motorXM430W350T_params.yaml";
  if (std::filesystem::exists(cfg)) {
    std::vector<std::string> args = {"--ros-args"};
    bool in_ros_args = false;
    for (int i = 1; i < argc; ++i) {
      const std::string a(argv[i]);
      if (a == "--ros-args") { in_ros_args = true; continue; }
      if (in_ros_args) args.push_back(a);
    }
    // params-file AL FINAL → el YAML del motor tiene prioridad sobre los -p del CLI:
    // los parámetros del motor NO se pueden sobreescribir; los params propios del
    // nodo (no presentes en el YAML) sí se ajustan con -p.
    args.push_back("--params-file");
    args.push_back(cfg);
    opts.arguments(args);
    opts.use_global_arguments(false);
    RCLCPP_INFO(rclcpp::get_logger("hw_smc_cart_node"),
      "motorXM430W350T_params auto-cargado: %s", cfg.c_str());
  } else {
    RCLCPP_WARN(rclcpp::get_logger("hw_smc_cart_node"),
      "motorXM430W350T_params no encontrado: %s — usando defaults del código.", cfg.c_str());
  }

  try {
    rclcpp::spin(std::make_shared<HWSMCCartNode>(opts));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_smc_cart_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
