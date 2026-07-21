/*
 * hw_MRAC_joint_12p_node.cpp
 * Control Adaptativo (MRAC / Slotine-Li) articular de 12 PARAMETROS —
 * OpenMANIPULATOR-X hardware real via Dynamixel SDK directo, sin ros2_control.
 *
 * Port del gz_MRAC_joint_12p_node.cpp (validado en sim: campana C1-C5) sobre
 * el andamiaje hardware del hw_SMC_joint_node.cpp (SDK, mapeo torque→corriente,
 * filtro α-β, seguridad).
 *
 * Parametros adaptados (12):
 *   a_hat = [ alpha1..alpha3 , dm, dmcx, dmcy, dmcz , Fv1_res , Fc1..Fc4_res ]^T
 *
 *   alpha_k (k=1..3): escala de inercia del cuerpo movido por jointk
 *             (linkk+1), nominal = 1.0 (masas pesadas con balanza).
 *   theta_load = [dm, dmcx, dmcy, dmcz]: EXCESO inercial del ultimo cuerpo
 *             (link5 + lo que sujete el gripper) en el frame de joint4,
 *             prior = 0. Una carga rigida en el gripper equivale exactamente
 *             a un delta en los parametros inerciales del link5 → la
 *             adaptacion la estima en linea (en sim: dm al 1% del real).
 *             IMPORTANTE: cerrar el gripper sobre la carga ANTES de correr el
 *             nodo (este nodo NO comanda el gripper, ID 15).
 *   Fv1_res / Fc_res: friccion RESIDUAL en dominio de torque. A diferencia
 *             del nodo gz, aqui la friccion identificada YA se compensa en el
 *             dominio de CORRIENTE con el modelo del motor (motor_Fv,
 *             motor_Fc en ticks — ver torque_to_current); agregar el prior de
 *             friccion del URDF a nivel de torque seria doble compensacion.
 *             Los parametros adaptivos estiman la DESVIACION de la friccion
 *             real respecto a la identificada (temperatura, unidad a unidad),
 *             con prior 0 y cotas simetricas alrededor de cero.
 *             Fv de J2..J4 no se adapta (identificado = 0 en la familia OMX;
 *             un Fv_hat > 0 espurio con dq medida es realimentacion positiva).
 *
 * Ley de control (diag(M)-escalada, con ERRORES RECORTADOS — ver lecciones):
 *   e_c  = clamp(e_q,  ±E_CLAMP)     edq_c = clamp(e_dq, ±DE_CLAMP)
 *   s    = edq_c + Lambda*e_c        dq_r  = dq_d - Lambda*e_c
 *   ddq_r= ddq_d - clamp(Lambda*edq_c, ±DDQR_CLAMP)
 *   tau  = rnea(q, dq, ddq_r) + Y*(a_hat - a_prior)
 *          - diag(M(q)) * gain_scale * ( K_V.*s + K_S.*sat(s/phi) )
 *   tau_sat = clamp(tau, -tau_max, tau_max)
 *   I_cmd   = alpha_m*tau_sat + Fv_m*dq + fc_scale*Fc_m*tanh(dq_d/eps) + I_off
 *
 * LECCION HW (test 9) — por que diag(M) y el recorte de ddq_r:
 *   La reconstruccion offline de la ley desde el CSV dio corr 1.0000 con el
 *   torque registrado: NO hay bug — lo que fallaba es la ley misma en el
 *   regimen de |edq| ~ 2 rad/s. Con la matriz M completa, los terminos
 *   cruzados (M23*ddq_r3 en el feedforward y -M23*Kv3*s3 en el feedback) son
 *   el "desacople" del torque computado, correcto SOLO con modelo exacto;
 *   con el ~20% de error modelo/actuador real se convierten en lanzamientos
 *   entre ejes (medidos +0.54 y +0.60 N·m sobre J2 en el instante critico):
 *   J3 corrige -> lanza a J2 -> J2 corrige -> lanza a J3, creciendo al ritmo
 *   del batido del multiseno (0.17 Hz) hasta el limite articular. Con
 *   diag(M) el feedback es estrictamente disipativo por eje (tau_i*s_i < 0
 *   siempre) y el recorte DDQR_CLAMP limita el lanzamiento cruzado del
 *   feedforward a ~0.08 N·m sin tocar la operacion normal (|edq| < 0.5).
 *
 *   Y = [ Y_alpha1..3 | Y_load(4 col del bloque de link5) | dq1 |
 *         diag(tanh(dq_d/eps)) ]   (regresor Slotine-Li via identidad de
 *   polarizacion sobre pinocchio::computeJointTorqueRegressor).
 *
 * LECCION HW (tests 1-4, 2026-07-19) — por que el recorte de errores:
 *   Sin recorte, con Lambda=20..40 un error moderado (e~0.3 rad tras un
 *   asentamiento con stiction) produce demandas M*(ddq_d - Lambda*edq) +
 *   feedback que superan tau_max: el lazo entra al REGIMEN DE RELE (torque
 *   riel a riel, 30-42% de saturacion) y se auto-sostiene un ciclo limite
 *   LENTO de ~0.22 Hz de gran amplitud (test 3: divergencia hasta el limite
 *   articular; test 4: estable 12 s y luego bascula → parada por corriente).
 *   El recorte E_CLAMP/DE_CLAMP acota la demanda dentro del presupuesto
 *   lineal: cerca de la referencia (|e| < E_CLAMP) la ley es EXACTAMENTE la
 *   original; con error grande el robot regresa a velocidad acotada (como un
 *   homing) en vez de entrar al rele. El termino rigido usa la convencion
 *   NLE a dq MEDIDA (rnea(q,dq,ddq_r), como FL/SMC hw): la variante
 *   C(q,dq)*dq_r de Slotine-Li difiere solo en -C*s (<=0.03 N·m RMS medido
 *   en los tests, irrelevante) y costaba 5 RNEA en vez de 1.
 *
 * Ley de adaptacion (solo RUN, anti-windup, ZONA MUERTA, COMPUERTA de
 * tracking, sigma POR BLOQUE):
 *   s_dz  = signo(s) * max(|s| - adapt_deadzone, 0)      (por articulacion)
 *   a_dot = -Gamma .* ( Y^T*s_dz + sigma .* (a_hat - a_prior) )
 *   a_hat = clamp(a_hat + Ts*a_dot, A_MIN, A_MAX)
 *   Se CONGELA si el torque o la corriente comandada saturan (el actuador no
 *   aplica la ley → integrar seria windup), fuera de la fase RUN, y cuando
 *   max|e_q| > ADAPT_EMAX (lejos de la referencia el error es de
 *   stiction-parking/recuperacion, no de parametros — test 8).
 *   LECCION HW: en el robot |s| es 10-50x mayor que en sim (stick-slip,
 *   ruido); con las Gamma de sim los residuales Fc cruzaban toda su banda en
 *   ~35 ms actuando como un rele adicional sincronizado con la oscilacion
 *   (rieles 30-54% del tiempo en tests 1-2). Por eso: Gamma ~20x mas frias
 *   que en sim (promedian SOBRE el ciclo de 2-10 Hz en vez de perseguirlo),
 *   zona muerta en s (el ciclo residual y el ruido no bombean parametros) y
 *   cotas de Fc residual estrechas.
 *
 * GANANCIAS: moderadas (set gz validado escalado ~1.5x) — ver bloque de
 * ganancias. LECCION HW (test 5): el recorte de errores dio 12 s de tracking
 * limpio (24-37 mrad), pero con las ganancias heredadas del hw_SMC (nunca
 * validadas en hw) un modo de ~10 Hz se encendia en la configuracion mas
 * profunda del multiseno (t~12.2 s, maxima gravedad en q2): kd_eq 41-115 1/s
 * excedia el ancho de banda del estimador alfa-beta. La oscilacion railed
 * sostenida llevo al SERVO J2 a su proteccion de sobrecarga (err=128 en cada
 * cierre del ID 12): el motor corto torque por si mismo (cmd -257 vs meas
 * -69..+36) y el brazo cayo por gravedad hasta el limite articular. De ahi:
 * ganancias moderadas + alfa-beta mas rapido + watchdog de sobrecarga que
 * aborta ANTES de que la proteccion interna del servo actue en movimiento.
 * gain_scale escala K_V y K_S; evidencia tests 1-2: 0.5 fue PEOR que 1.0
 * (la stiction exige rigidez) — operar con 1.0.
 *
 * Fases (como el gz 12p; el CSV registra solo RUN, t=0 al iniciar multiseno):
 *   1) HOMING : quintica T_HOME s desde la pose medida hasta reposo en los
 *               centros del multiseno seguro [0, -0.55, 0.10, 0.50].
 *   2) SETTLE : mantiene los centros hasta max|e| < SETTLE_TOL (o timeout).
 *   3) RUN    : multiseno seguro con envolvente + adaptacion activa.
 *   4) RETURN : al completar t_run, transicion quintica de RETURN_TIME_S s
 *               (como hw_fl_control_node) desde la ultima referencia hasta
 *               el reposo en q_inicial (pose medida en el primer tick, antes
 *               de HOMING) + pausa HOLD_TIME_S antes de cortar corriente.
 *               Evita el frenazo/caida por corte de torque en medio del
 *               multiseno. NO se registra en el CSV (t_run<=0 la desactiva:
 *               RUN corre indefinido).
 *
 * Trayectoria: multiseno seguro del 12p (z_EE >= 83 mm verificado, tau_ff
 * <= 0.63 N·m), identico a la sim → resultados comparables sim vs real.
 *
 * Parametros ROS 2 (--ros-args -p nombre:=valor):
 *   port_name    [string] "/dev/ttyUSB0"
 *   adaptive     [bool]   true   (false: a_hat fijo en a_prior — baseline)
 *   gain_scale   [double] 1.0    (escala K_V y K_S; mantener 1.0 — ver arriba)
 *   t_run        [double] 30.0   (duracion de RUN [s]; total += homing+settle)
 *   test_num     [int]    1
 *   phi          [double] 0.3    (capa limite sat(s/phi) [rad/s])
 *   adapt_deadzone [double] 0.3  (zona muerta de |s| para la adaptacion [rad/s])
 *   traj_scale   [double] 1.0    (escala las AMPLITUDES del multiseno; los
 *                                 centros no cambian. Para validacion gradual
 *                                 en hw: 0.7 primero, luego 1.0)
 *   ab_alpha     [double] 0.35 / ab_beta [double] 0.06 — filtro alfa-beta con
 *                ancho de banda ~8 Hz (0.2/0.02 = ~4.5 Hz dejaba el kd de la
 *                ley sin margen de fase en la banda de 10 Hz).
 *
 * WATCHDOG de sobrecarga: si la corriente comandada de una articulacion
 * promedia (EMA tau=1.5 s) mas del 80% de su limite, el nodo aborta limpio —
 * esa condicion sostenida es la antesala de la proteccion interna del servo
 * (que corta torque en pleno movimiento, el modo de falla mas peligroso).
 *   ab_alpha     [double] 0.2    (filtro α-β: ganancia de posicion)
 *   ab_beta      [double] 0.02   (filtro α-β: ganancia de velocidad)
 *   friction_fc_scale [double] 0.95 (fraccion de Fc del motor compensada)
 *   loop_rate_hz [double] 200.0  (frecuencia del lazo [Hz], acotada [50,400])
 *
 * Parametros del modelo identificado auto-cargados desde
 * config/motorXM430W350T_params.yaml (prioridad sobre -p del CLI):
 * motor_alpha, motor_Fv, motor_Fc, motor_I_offset, motor_epsilon_friction,
 * joint_zero_tick, encoder_sign, current_sign, joint_lower/upper,
 * current_*_limit, tau_max.
 *
 * CSV: data/lab7/real/mrac12p/hw_mrac_joint_12p_<modo>_<test_num>.csv
 * Columnas: las mismas del gz 12p (compatibles con plots_MRAC_joint_12p.m,
 * mode='real') + extras de hardware al final:
 *   t, q1..4, dq1..4, q*_des, dq*_des, s1..4, tau1..4, sat1..4,
 *   a1..a3_hat, dm/dmcx/dmcy/dmcz_hat, fv1_hat, fc1..fc4_hat,
 *   dq1_filt..dq4_filt, curr_cmd1..4, curr_meas1..4
 *
 * Publisher: /hw/joint_states (sensor_msgs/JointState) — solo monitoreo
 *
 * Ejemplos de ejecucion:
 *   # Baseline no adaptativo primero (valida el lazo base con recorte):
 *   ros2 run open_manipulator_x_torque_control hw_mrac_joint_12p_node \
 *     --ros-args -p adaptive:=false -p test_num:=5 -p t_run:=30.0
 *
 *   # Adaptativo (Gamma fria + zona muerta):
 *   ros2 run open_manipulator_x_torque_control hw_mrac_joint_12p_node \
 *     --ros-args -p adaptive:=true -p test_num:=6 -p t_run:=30.0
 *
 *   # Con carga en el gripper (cerrar el gripper sobre el disco ANTES):
 *   ros2 run open_manipulator_x_torque_control hw_mrac_joint_12p_node \
 *     --ros-args -p adaptive:=true -p test_num:=3 -p t_run:=30.0
 *
 * ADVERTENCIA: No ejecutar junto a hardware.launch.py ni ningun proceso
 * que acceda a /dev/ttyUSB0 (ros2_control_node, dynamixel_hardware_interface).
 */

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <vector>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/regressor.hpp>
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
static constexpr int    NP         = 12;   // [alpha1..3, theta_load(4), Fv1, Fc1..4]

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

// ── Fases de arranque (como el gz 12p) ───────────────────────────────────────
static constexpr double T_HOME      = 5.0;   // [s] homing quintico inicial
static constexpr double T_RAMP      = 2.0;   // [s] envolvente 0->1 del multiseno
static constexpr double SETTLE_TOL  = 0.08;  // [rad] max|e| para dar por asentado
static constexpr double SETTLE_TMAX = 5.0;   // [s] timeout del asentamiento

// ── Fase de retorno final (como hw_fl_control_node) ──────────────────────────
// Al completar RUN, en vez de cortar corriente de golpe en medio del
// multiseno (el brazo caeria por gravedad), el nodo hace una transicion
// quintica de RETURN_TIME_S s desde el ultimo punto de la referencia hasta
// el REPOSO en q_inicial (la pose medida en el primer tick, antes de
// HOMING), luego una pausa HOLD_TIME_S para asentar antes de cortar. El CSV
// NO registra el retorno (se excluye igual que la fase RUN lo exige, ver
// gate en el punto 10 de tick()).
static constexpr double RETURN_TIME_S = 3.0;   // [s] duracion del retorno quintico a q_inicial
static constexpr double HOLD_TIME_S   = 0.5;   // [s] pausa en reposo antes de cortar corriente

// ── Recorte de errores para la ley de control (anti-rele, tests 3-4) ─────────
//   Acota la demanda de torque dentro del presupuesto lineal (|tau|<tau_max).
//   Cerca de la referencia la ley no cambia; con error grande el retorno es a
//   velocidad acotada en vez del ciclo limite de rele de ~0.22 Hz.
//   E_CLAMP = 0.5 tras los tests 6-7: con 0.15 el techo de correccion
//   estatica era M*Kv*Lambda*0.15 ~ 0.15-0.3 N·m, menor que el breakaway
//   real + error de modelo → el brazo quedaba ESTACIONADO por stiction a
//   ~1.1 rad de la referencia (estable pero colgado). Con 0.5 y las
//   ganancias ya moderadas, el techo sube a ~0.5-0.7 N·m y la demanda total
//   (gravedad <=0.78 + feedback) sigue dentro del presupuesto salvo recortes
//   transitorios en la pose mas profunda.
static constexpr double E_CLAMP  = 0.5;   // [rad]   recorte de e_q
static constexpr double DE_CLAMP = 1.5;   // [rad/s] recorte de e_dq

// Recorte del termino de velocidad del feedforward de aceleracion (test 9):
// ddq_r = ddq_d - clamp(Lambda*edq_c, ±DDQR_CLAMP). En operacion normal
// (|edq| < 0.5 rad/s) no actua; en el regimen de columpio limita el
// lanzamiento cruzado M_ij*ddq_r_j a ~0.08 N·m (medido sin recorte: +0.54).
static constexpr double DDQR_CLAMP = 8.0;  // [rad/s²]

// Compuerta de adaptacion por calidad de tracking: adaptar SOLO cerca de la
// referencia. Lejos de ella (test 8: brazo estacionado por stiction a ~1 rad)
// la firma del error no es de gravedad ni de friccion y el gradiente del
// regresor apunta a cualquier lado: los parametros paseaban riel a riel.
static constexpr double ADAPT_EMAX = 0.25;  // [rad] max|e_q| para adaptar

using Vec4  = Eigen::Matrix<double, NUM_JOINTS, 1>;
using Vec12 = Eigen::Matrix<double, NP, 1>;
using Vec10 = Eigen::Matrix<double, 10, 1>;
using MatY  = Eigen::Matrix<double, NUM_JOINTS, NP>;

// Indices de bloque dentro de a_hat:
static constexpr int IDX_ALPHA = 0;   // [0..2]  alpha1..alpha3
static constexpr int IDX_LOAD  = 3;   // [3..6]  dm, dmcx, dmcy, dmcz
static constexpr int IDX_FV1   = 7;   // [7]     Fv1 residual
static constexpr int IDX_FC    = 8;   // [8..11] Fc1..Fc4 residual

// ═══════════════════════════════════════════════════════════════════════════
//  GANANCIAS MRAC 12p hw — realimentacion M(q)-escalada
//  Indice articular: [joint1, joint2, joint3, joint4]
//
//  MODERADAS con evidencia del test 5 (set gz validado escalado ~1.5x). Las
//  heredadas del hw_SMC (Lambda 20-40, K_V 14-55, mapeadas de FL y NUNCA
//  validadas en hw) daban kd_eq = Lambda+K_V+K_S/phi = 41-115 1/s, mas alla
//  de lo que soporta el estimador de velocidad alfa-beta (~4.5-8 Hz de ancho
//  de banda): un modo marginal de ~10 Hz se encendia en la configuracion mas
//  profunda del multiseno (t~12.2 s, ambos senos de q2 alineados, maxima
//  gravedad) y crecia hasta sobrecargar el servo J2 (proteccion err=128).
//  Set actual: kd_eq = [27, 27, 32, 58], kp_eq = Lambda*(K_V+K_S/phi) =
//  [176, 176, 317, 833] — la stiction la cubre el feedforward de Fc en
//  corriente (fc_scale) + K_S; no hace falta la rigidez FL completa.
//  gain_scale escala K_V y K_S (Lambda no). Evidencia tests 1-2: 0.5 fue
//  PEOR que 1.0 — operar con 1.0.
// ═══════════════════════════════════════════════════════════════════════════
static const Vec4 LAMBDA_Q = (Vec4() << 12.0, 12.0, 15.0, 25.0).finished();  // superficie [1/s]
static const Vec4 K_V      = (Vec4() <<  8.0,  8.0, 10.0, 20.0).finished();  // alcance exponencial [1/s]
static const Vec4 K_S      = (Vec4() <<  2.0,  2.0,  2.0,  4.0).finished();  // conmutacion sat(s/phi) [rad/s²]

// Tasas de adaptacion por bloque — ~20x MAS FRIAS que en sim (tests hw 1-2:
// con las Gamma de sim y |s|~1-4, los residuales Fc cruzaban su banda en
// ~35 ms y actuaban como rele sincronizado con el ciclo limite; frios, la
// adaptacion promedia sobre el ciclo y estima solo la componente DC).
static const Vec12 GAMMA =
  (Vec12() << 0.05, 0.05, 0.05,   0.01, 0.0005, 0.0005, 0.0005,   0.1,   0.1, 0.1, 0.1, 0.1).finished();

// Fuga sigma-modification hacia el prior [1/s], POR BLOQUE. En hw TODOS los
// priors de friccion/carga son cero (residuales): la fuga ancla los residuos
// a cero cuando no hay senal — anti-deriva frente a stiction/Stribeck reales
// que el modelo tanh no representa.
static const Vec12 SIGMA_LEAK =
  (Vec12() << 0.1, 0.1, 0.1,   0.02, 0.02, 0.02, 0.02,   0.1,   0.1, 0.1, 0.1, 0.1).finished();

// Proyeccion a la region admisible. alpha estrecho (masas pesadas con
// balanza); theta_load hasta 200 g a ~17.5 cm; friccion RESIDUAL con cotas
// ESTRECHAS alrededor de cero (la friccion base ya la pone el modelo de
// corriente; en tests 1-2 las cotas amplias [-0.06,0.15] dejaban que el
// bombeo inyectara hasta 0.15 N·m de rele — segunda barrera tras Gamma fria).
// dm/dmcx con margen negativo AMPLIO (tests 12-13: vivian clavados en
// -0.02/-0.005 el 91-93% del tiempo — el desajuste DC modelo/actuador pide
// restar mas efecto de masa del que las cotas de sim permitian).
static const Vec12 A_MIN =
  (Vec12() << 0.80, 0.80, 0.80,   -0.05, -0.012, -0.010, -0.005,   -0.02,  -0.04, -0.04, -0.04, -0.04).finished();
static const Vec12 A_MAX =
  (Vec12() << 1.25, 1.25, 1.25,    0.20,  0.035,  0.010,  0.020,    0.05,   0.08,  0.08,  0.08,  0.08).finished();
// ═══════════════════════════════════════════════════════════════════════════

// ============================================================
// Utilidades de conversión (idénticas a hw_SMC_joint_node)
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

// ============================================================
// Trayectoria de referencia (multiseno seguro, identica al gz 12p)
// ============================================================

struct Reference { Vec4 q, dq, ddq; };

// Escala de amplitudes del multiseno (param traj_scale, fijada al arrancar).
// Los centros no cambian: con 0 la referencia es reposo en los centros.
static double g_traj_scale = 1.0;

// Multiseno seguro con envolvente quintica 0->1 en T_RAMP: q_d(0) = centros
// en REPOSO (empalme exacto con el asentamiento). Centros/amplitudes con
// z_efector >= 83 mm garantizado y limites articulares con holgura (a escala
// 1.0; escalas menores solo encogen la excursion → siguen siendo seguras).
static Reference desiredTrajectory(double t)
{
  static const Vec4 C  = (Vec4() <<  0.00, -0.55,  0.10,  0.50).finished();
  static const Vec4 A1 = (Vec4() <<  0.60,  0.25,  0.25,  0.25).finished();
  static const Vec4 W1 = (Vec4() <<  0.6,   0.9,   1.3,   1.7 ).finished();
  static const Vec4 A2 = (Vec4() <<  0.20,  0.09,  0.09,  0.09).finished();
  static const Vec4 W2 = (Vec4() <<  1.5,   1.9,   0.7,   1.1 ).finished();

  double sg = 1.0, dsg = 0.0, ddsg = 0.0;
  if (t < T_RAMP) {
    const double x = t / T_RAMP;
    sg   = ((6.0 * x - 15.0) * x + 10.0) * x * x * x;
    dsg  = ((30.0 * x - 60.0) * x + 30.0) * x * x / T_RAMP;
    ddsg = ((120.0 * x - 180.0) * x + 60.0) * x / (T_RAMP * T_RAMP);
  }

  Reference ref;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    const double osc   = g_traj_scale * ( A1[i] * std::sin(W1[i] * t) + A2[i] * std::sin(W2[i] * t));
    const double dosc  = g_traj_scale * ( A1[i] * W1[i] * std::cos(W1[i] * t)
                                        + A2[i] * W2[i] * std::cos(W2[i] * t));
    const double ddosc = g_traj_scale * (-A1[i] * W1[i] * W1[i] * std::sin(W1[i] * t)
                                        - A2[i] * W2[i] * W2[i] * std::sin(W2[i] * t));
    ref.q[i]   = C[i] + sg * osc;
    ref.dq[i]  = dsg * osc + sg * dosc;
    ref.ddq[i] = ddsg * osc + 2.0 * dsg * dosc + sg * ddosc;
  }
  return ref;
}

// Homing quintico: de la postura medida (q0, v0, acc 0) en t=0 hasta el
// REPOSO en los centros del multiseno (q_d(0), 0, 0) en t=T_HOME.
static Reference homingTrajectory(double t, const Vec4 & q0, const Vec4 & v0)
{
  const Reference end = desiredTrajectory(0.0);
  const double T = T_HOME;
  Reference ref;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    const double h  = end.q[i] - q0[i];
    const double vf = end.dq[i];
    const double af = end.ddq[i];
    const double c0 = q0[i];
    const double c1 = v0[i];
    const double c3 = ( 20.0*h - (8.0*vf + 12.0*v0[i])*T + af*T*T) / (2.0*T*T*T);
    const double c4 = (-30.0*h + (14.0*vf + 16.0*v0[i])*T - 2.0*af*T*T) / (2.0*T*T*T*T);
    const double c5 = ( 12.0*h -  6.0*(vf + v0[i])*T + af*T*T) / (2.0*T*T*T*T*T);
    ref.q[i]   = c0 + c1*t + c3*t*t*t + c4*t*t*t*t + c5*t*t*t*t*t;
    ref.dq[i]  = c1 + 3.0*c3*t*t + 4.0*c4*t*t*t + 5.0*c5*t*t*t*t;
    ref.ddq[i] = 6.0*c3*t + 12.0*c4*t*t + 20.0*c5*t*t*t;
  }
  return ref;
}

// Polinomio quintico general: interpola (q,dq,ddq) desde el estado de borde
// (q0,v0,a0) en t=0 hasta (qf,vf,af) en t=T (identico a hw_fl_control_node).
// quinticReturn (retorno final) es un caso particular de este blend.
static Reference quinticBlend(double t, double T,
                              const Vec4& q0, const Vec4& v0, const Vec4& a0,
                              const Vec4& qf, const Vec4& vf, const Vec4& af)
{
  if (T <= 0.0) { Reference r; r.q = qf; r.dq = vf; r.ddq = af; return r; }

  const double T2=T*T, T3=T2*T, T4=T3*T, T5=T4*T;
  const Vec4 c0 = q0;
  const Vec4 c1 = v0;
  const Vec4 c2 = 0.5 * a0;
  const Vec4 c3 = (20.0*(qf-q0) - (8.0*vf+12.0*v0)*T - (3.0*a0-af)*T2) / (2.0*T3);
  const Vec4 c4 = (30.0*(q0-qf) + (14.0*vf+16.0*v0)*T + (3.0*a0-2.0*af)*T2) / (2.0*T4);
  const Vec4 c5 = (12.0*(qf-q0) - (6.0*vf+6.0*v0)*T - (a0-af)*T2) / (2.0*T5);

  const double t2=t*t, t3=t2*t, t4=t3*t, t5=t4*t;
  Reference r;
  r.q   = c0 + c1*t  + c2*t2  + c3*t3  + c4*t4  + c5*t5;
  r.dq  = c1 + 2.0*c2*t + 3.0*c3*t2 + 4.0*c4*t3 + 5.0*c5*t4;
  r.ddq = 2.0*c2 + 6.0*c3*t + 12.0*c4*t2 + 20.0*c5*t3;
  return r;
}

// Retorno final: parte del estado de la referencia en el instante en que
// termina RUN (start = ultima referencia de desiredTrajectory) y llega en
// reposo (vf=af=0) a qf (q_inicial) — evita el frenazo/caida por corte de
// torque en medio del multiseno.
static Reference quinticReturn(double t, double T, const Reference& start, const Vec4& qf)
{
  return quinticBlend(t, T, start.q, start.dq, start.ddq, qf, Vec4::Zero(), Vec4::Zero());
}

// ── Regresor Slotine-Li: identidad de polarizacion sobre el regresor ────────
//   R(q,v,a)*pi = M(q)a + C(q,v)v + g(q) para cualquier pi (10 por cuerpo).
static Eigen::MatrixXd slotineLiRegressor(
    const pinocchio::Model & model, pinocchio::Data & data,
    const Eigen::VectorXd & q,   const Eigen::VectorXd & dq,
    const Eigen::VectorXd & dqr, const Eigen::VectorXd & ddqr)
{
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(model.nv);
  const Eigen::MatrixXd R_g     = pinocchio::computeJointTorqueRegressor(model, data, q, zero,     zero);
  const Eigen::MatrixXd R_Mddqr = pinocchio::computeJointTorqueRegressor(model, data, q, zero,     ddqr);
  const Eigen::MatrixXd R_sum   = pinocchio::computeJointTorqueRegressor(model, data, q, dq + dqr, zero);
  const Eigen::MatrixXd R_dq    = pinocchio::computeJointTorqueRegressor(model, data, q, dq,       zero);
  const Eigen::MatrixXd R_dqr   = pinocchio::computeJointTorqueRegressor(model, data, q, dqr,      zero);
  return R_Mddqr + 0.5 * (R_sum - R_dq - R_dqr + R_g);
}

// ── Fases de arranque ────────────────────────────────────────────────────────
enum class Phase { HOMING, SETTLE, RUN, RETURN };

// ============================================================
// Nodo ROS 2
// ============================================================

class HWMRACJoint12pNode : public rclcpp::Node
{
public:
  explicit HWMRACJoint12pNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("hw_mrac_joint_12p_node", opts),
    hw_active_(false), q_initial_captured_(false)
  {
    // ── Parámetros propios ──────────────────────────────────────────────────
    this->declare_parameter<std::string>("port_name",  "/dev/ttyUSB0");
    this->declare_parameter<bool>       ("adaptive",    true);
    this->declare_parameter<double>     ("gain_scale",  1.0);
    this->declare_parameter<double>     ("t_run",       30.0);
    this->declare_parameter<int>        ("test_num",    1);
    this->declare_parameter<double>     ("phi",         0.3);
    this->declare_parameter<double>     ("adapt_deadzone", 0.3);
    this->declare_parameter<double>     ("traj_scale",  1.0);

    using dvec = std::vector<double>;
    this->declare_parameter<dvec>("motor_alpha",            dvec{208.5, 208.5, 208.5, 208.5});
    this->declare_parameter<dvec>("motor_Fv",               dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<dvec>("motor_Fc",               dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<dvec>("motor_I_offset",         dvec{0.0,   0.0,   0.0,   0.0  });
    this->declare_parameter<double>("motor_epsilon_friction", 0.05);
    this->declare_parameter<double>("friction_fc_scale", 0.95);
    // Filtro alfa-beta: 0.35/0.06 → ancho de banda ~8 Hz (con 0.2/0.02 el
    // retardo de fase del estimador dejaba sin margen al kd de la ley en la
    // banda de 10 Hz — leccion del test 5)
    this->declare_parameter<double>("ab_alpha", 0.35);
    this->declare_parameter<double>("ab_beta",  0.06);
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

    port_name_  = this->get_parameter("port_name").as_string();
    adaptive_   = this->get_parameter("adaptive").as_bool();
    gain_scale_ = this->get_parameter("gain_scale").as_double();
    t_run_      = this->get_parameter("t_run").as_double();
    const int test_num = this->get_parameter("test_num").as_int();
    phi_        = this->get_parameter("phi").as_double();
    adapt_dz_   = this->get_parameter("adapt_deadzone").as_double();
    g_traj_scale = std::min(std::max(
      this->get_parameter("traj_scale").as_double(), 0.0), 1.0);
    mode_str_   = adaptive_ ? "adaptive" : "fixed";

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

    // Prior: alphas en 1, TODO lo demas (carga y friccion residual) en 0 —
    // en a_hat = a_prior la ley es "torque computado + modelo identificado".
    a_prior_.setZero();
    a_prior_.segment<3>(IDX_ALPHA).setOnes();
    a_hat_ = a_prior_;

    RCLCPP_INFO(get_logger(),
      "puerto=%s  modo=%s  gain=%.2f  traj_scale=%.2f  t_run=%.1fs  test=%d",
      port_name_.c_str(), mode_str_.c_str(), gain_scale_, g_traj_scale, t_run_, test_num);
    RCLCPP_INFO(get_logger(),
      "MRAC 12p hw — phi=%.2f  tau_max=%.2f N·m  T_home=%.1f s  deadzone(s)=%.2f  "
      "clamp e=%.2f rad / edq=%.1f rad/s  (M(q)-escalado, gan. hw-SMC)",
      phi_, tau_max_, T_HOME, adapt_dz_, E_CLAMP, DE_CLAMP);
    RCLCPP_INFO(get_logger(),
      "Lambda=[%.0f %.0f %.0f %.0f]  Kv=[%.0f %.0f %.0f %.0f]  Ks=[%.0f %.0f %.0f %.0f]",
      LAMBDA_Q[0], LAMBDA_Q[1], LAMBDA_Q[2], LAMBDA_Q[3],
      K_V[0],      K_V[1],      K_V[2],      K_V[3],
      K_S[0],      K_S[1],      K_S[2],      K_S[3]);
    RCLCPP_INFO(get_logger(),
      "Gamma: alpha=[%.2f x3]  load=[%.2f %.3f x3]  Fv1=%.1f  Fc=[%.1f x4]  |  sigma load=%.2f resto=%.2f",
      GAMMA[0], GAMMA[3], GAMMA[4], GAMMA[7], GAMMA[8], SIGMA_LEAK[3], SIGMA_LEAK[0]);
    RCLCPP_INFO(get_logger(),
      "motor α=[%.1f %.1f %.1f %.1f]  Fv=[%.2f %.2f %.2f %.2f]  ε=%.3f  Fc_scale=%.2f",
      motor_alpha_(0), motor_alpha_(1), motor_alpha_(2), motor_alpha_(3),
      motor_Fv_(0), motor_Fv_(1), motor_Fv_(2), motor_Fv_(3), motor_epsilon_, fc_scale_);
    RCLCPP_INFO(get_logger(),
      "hw: cmd_lim=[%d %d %d %d]  meas_peak=%d  lim_reg=%d  |  α-β: %.3f/%.4f  |  lazo=%.0f Hz",
      current_cmd_limit_[0], current_cmd_limit_[1], current_cmd_limit_[2], current_cmd_limit_[3],
      static_cast<int>(current_measured_peak_), static_cast<int>(current_limit_register_),
      ab_alpha_, ab_beta_, loop_rate_hz_);

    // ── Pinocchio ────────────────────────────────────────────────────────────
    const std::string urdf = std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf, model_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio URDF: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const std::string jname = "joint" + std::to_string(i + 1);
      jid_[i]    = model_.getJointId(jname);
      pi_nom_[i] = model_.inertias[jid_[i]].toDynamicParameters();
    }
    RCLCPP_INFO(get_logger(), "Pinocchio: nv=%d", model_.nv);

    // ── CSV ──────────────────────────────────────────────────────────────────
    std::filesystem::create_directories(std::string(PACKAGE_DATA_DIR) + "/lab7/real/mrac12p");
    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab7/real/mrac12p/hw_mrac_joint_12p_"
                + mode_str_ + "_" + std::to_string(test_num) + ".csv";
    csv_.open(csv_path_);
    if (csv_.is_open()) {
      csv_ << "t,q1,q2,q3,q4,dq1,dq2,dq3,dq4,"
              "q1_des,q2_des,q3_des,q4_des,dq1_des,dq2_des,dq3_des,dq4_des,"
              "s1,s2,s3,s4,"
              "tau1,tau2,tau3,tau4,"
              "sat1,sat2,sat3,sat4,"
              "a1_hat,a2_hat,a3_hat,"
              "dm_hat,dmcx_hat,dmcy_hat,dmcz_hat,"
              "fv1_hat,fc1_hat,fc2_hat,fc3_hat,fc4_hat,"
              "dq1_filt,dq2_filt,dq3_filt,dq4_filt,"
              "curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,"
              "curr_meas1,curr_meas2,curr_meas3,curr_meas4\n";
      RCLCPP_INFO(get_logger(), "CSV: %s", csv_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "No se pudo crear CSV: %s", csv_path_.c_str());
    }

    // ── Publisher de monitoreo ───────────────────────────────────────────────
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/hw/joint_states", 10);

    // ── SDK ──────────────────────────────────────────────────────────────────
    if (!init_hardware()) {
      RCLCPP_FATAL(get_logger(), "Fallo hardware init. Abortando.");
      throw std::runtime_error("Hardware init failed");
    }

    // ── Timer de control ─────────────────────────────────────────────────────
    start_time_ = std::chrono::high_resolution_clock::now();
    const auto period = std::chrono::microseconds(
      static_cast<int64_t>(std::lround(1e6 / loop_rate_hz_)));
    timer_ = this->create_wall_timer(period, [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Control activo a %.0f Hz (Ts=%.1f ms). Ctrl+C para detener.",
      loop_rate_hz_, 1e3 * Ts_);
  }

  ~HWMRACJoint12pNode()
  {
    if (timer_) timer_->cancel();
    shutdown_hardware();
    if (csv_.is_open()) { csv_.flush(); csv_.close(); }
    RCLCPP_INFO(get_logger(), "Nodo finalizado.");
  }

private:
  // ─────────────────────────────────────────────────────────────────────────
  //  Hardware (identico a hw_SMC_joint_node)
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
  //  Mapeo torque→corriente con el modelo identificado del motor.
  //  Fv usa la velocidad medida (termino suave); Fc usa la velocidad DESEADA
  //  (senal limpia, aporta el empuje de despegue). clipped[i] marca las
  //  articulaciones cuya corriente comandada saturo (anti-windup).
  // ─────────────────────────────────────────────────────────────────────────

  std::array<int16_t, NUM_JOINTS> torque_to_current(const Vec4& tau, const Vec4& dq,
                                                    const Vec4& dq_des,
                                                    std::array<bool, NUM_JOINTS>& clipped)
  {
    std::array<int16_t, NUM_JOINTS> cmd{};
    for (int i = 0; i < NUM_JOINTS; ++i) {
      const double I_model = motor_alpha_(i) * tau(i)
                           + motor_Fv_(i)    * dq(i)
                           + fc_scale_ * motor_Fc_(i) * std::tanh(dq_des(i) / motor_epsilon_)
                           + motor_I_offset_(i);
      const double I_cmd = current_sign_(i) * encoder_sign_(i) * I_model;
      const double lim   = static_cast<double>(current_cmd_limit_[i]);
      clipped[i] = std::abs(I_cmd) > lim;
      cmd[i] = static_cast<int16_t>(std::lround(std::min(std::max(I_cmd, -lim), lim)));
    }
    return cmd;
  }

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

    // 1. Lectura de estado
    Vec4 q, dq;
    std::array<int16_t, NUM_JOINTS> cur_meas{};
    if (!read_state(q, dq, cur_meas)) {
      emergency_stop("SyncRead fallido");
      return;
    }

    // 1b. Filtro α-β: estimacion conjunta de posicion y velocidad
    if (!ab_initialized_) {
      q_hat_  = q;
      dq_hat_ = dq;
      ab_initialized_ = true;
    } else {
      const Vec4 q_pred  = q_hat_ + Ts_ * dq_hat_;
      const Vec4 dq_pred = dq_hat_;
      const Vec4 r       = q - q_pred;
      q_hat_  = q_pred  + ab_alpha_ * r;
      dq_hat_ = dq_pred + (ab_beta_ / Ts_) * r;
    }

    // 2. Captura posicion inicial (primer tick) para el homing
    if (!q_initial_captured_) {
      q_initial_  = q;
      dq_initial_ = dq;
      q_initial_captured_ = true;
      const Reference r0 = desiredTrajectory(0.0);
      RCLCPP_INFO(get_logger(),
        "Homing (%.1f s): q0=[%.3f %.3f %.3f %.3f] -> q_d(0)=[%.2f %.2f %.2f %.2f]",
        T_HOME, q(0), q(1), q(2), q(3), r0.q[0], r0.q[1], r0.q[2], r0.q[3]);
    }

    // 3. Maquina de fases: HOMING -> SETTLE -> RUN
    Reference ref;
    if (phase_ == Phase::HOMING) {
      if (t < T_HOME) {
        ref = homingTrajectory(t, q_initial_, dq_initial_);
      } else {
        phase_ = Phase::SETTLE;
        RCLCPP_INFO(get_logger(),
          "Homing completado -> asentamiento en q_d(0) (tol %.2f rad, max %.1f s).",
          SETTLE_TOL, SETTLE_TMAX);
      }
    }
    if (phase_ == Phase::SETTLE) {
      ref = desiredTrajectory(0.0);
      const double e_max = (q - ref.q).cwiseAbs().maxCoeff();
      if (e_max < SETTLE_TOL || (t - T_HOME) >= SETTLE_TMAX) {
        phase_    = Phase::RUN;
        t_run0_   = t;
        RCLCPP_INFO(get_logger(),
          "Asentado (max|e|=%.3f rad tras %.1f s). Inicia multiseno + adaptacion. CSV desde t=0.",
          e_max, t - T_HOME);
      }
    }
    double t_run_elapsed = 0.0;
    if (phase_ == Phase::RUN) {
      t_run_elapsed = t - t_run0_;
      ref   = desiredTrajectory(t_run_elapsed);
      if (t_run_ > 0.0 && t_run_elapsed >= t_run_) {
        RCLCPP_INFO(get_logger(),
          "RUN completado (%.1f s) -> retorno quintico a q_inicial (dur %.1f s).",
          t_run_, RETURN_TIME_S);
        return_start_ref_ = ref;
        phase_            = Phase::RETURN;
        t_return0_        = t;
      }
    }
    if (phase_ == Phase::RETURN) {
      const double t_ret = t - t_return0_;
      if (t_ret < RETURN_TIME_S) {
        ref = quinticReturn(t_ret, RETURN_TIME_S, return_start_ref_, q_initial_);
      } else if (t_ret < RETURN_TIME_S + HOLD_TIME_S) {
        ref.q = q_initial_;
        ref.dq.setZero();
        ref.ddq.setZero();
      } else {
        RCLCPP_INFO(get_logger(), "Retorno a q_inicial completado. Deteniendo (t=%.1f s).", t);
        timer_->cancel();
        shutdown_hardware();
        if (csv_.is_open()) { csv_.flush(); csv_.close(); }
        rclcpp::shutdown();
        return;
      }
    }

    // 4. Verificar limites: referencia y estado medido
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (ref.q(i) < joint_lower_(i) + 0.02 || ref.q(i) > joint_upper_(i) - 0.02) {
        emergency_stop("Referencia fuera de limites articulares");
        return;
      }
      if (q(i) < joint_lower_(i) || q(i) > joint_upper_(i)) {
        emergency_stop("Articulacion " + std::to_string(i+1) + " fuera de limites: "
                       + std::to_string(q(i)) + " rad");
        return;
      }
    }

    // 5. Ley de control adaptativo — usa dq_hat_ (filtrada) en errores,
    //    regresor y ley; la superficie con dq cruda re-inyectaria el ruido
    //    de cuantizacion del encoder por Lambda y K_V.
    //    Los errores van RECORTADOS (anti-rele, tests 3-4): cerca de la
    //    referencia la ley no cambia; con error grande la demanda de torque
    //    queda acotada y el retorno es a velocidad limitada.
    const Vec4 e_q   = q       - ref.q;
    const Vec4 e_dq  = dq_hat_ - ref.dq;
    const Vec4 e_c   = e_q.cwiseMax(-E_CLAMP).cwiseMin(E_CLAMP);
    const Vec4 edq_c = e_dq.cwiseMax(-DE_CLAMP).cwiseMin(DE_CLAMP);
    const Vec4 s_q  = edq_c + LAMBDA_Q.cwiseProduct(e_c);
    const Vec4 dqr  = ref.dq  - LAMBDA_Q.cwiseProduct(e_c);
    // Termino de velocidad del ffwd de aceleracion recortado (anti-columpio
    // cruzado via M_ij — leccion test 9)
    const Vec4 ddqr = ref.ddq - LAMBDA_Q.cwiseProduct(edq_c)
                        .cwiseMax(-DDQR_CLAMP).cwiseMin(DDQR_CLAMP);

    const Vec4 tanh_dqd = ((ref.dq / motor_epsilon_).array().tanh()).matrix();

    Eigen::VectorXd q_pin    = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin   = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dqr_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd ddqr_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NUM_JOINTS>()    = q;
    dq_pin.head<NUM_JOINTS>()   = dq_hat_;
    dqr_pin.head<NUM_JOINTS>()  = dqr;
    ddqr_pin.head<NUM_JOINTS>() = ddqr;

    // Matriz de inercia nominal M(q) (CRBA) para la realimentacion escalada
    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();
    const Eigen::Matrix4d M = data_.M.topLeftCorner<NUM_JOINTS, NUM_JOINTS>();

    // Termino rigido nominal: M(q)*ddq_r + C(q,dq)*dq + g = rnea(q, dq, ddq_r).
    // Convencion NLE a dq MEDIDA (como FL/SMC hw): la variante C(q,dq)*dq_r de
    // Slotine-Li difiere solo en -C*s (<=0.03 N·m RMS medido en tests 3-4) y
    // costaba 5 RNEA en vez de 1. Sin friccion: esa vive en el modelo de
    // corriente (torque_to_current).
    const Vec4 tau_nom =
      pinocchio::rnea(model_, data_, q_pin, dq_pin, ddqr_pin).head<NUM_JOINTS>();

    // Regresor 4x12
    const Eigen::MatrixXd Y_SL =
      slotineLiRegressor(model_, data_, q_pin, dq_pin, dqr_pin, ddqr_pin);
    MatY Y = MatY::Zero();
    for (int i = 0; i < 3; ++i) {
      const int col0 = 10 * (static_cast<int>(jid_[i]) - 1);
      Y.col(IDX_ALPHA + i) = (Y_SL.middleCols(col0, 10) * pi_nom_[i]).head<NUM_JOINTS>();
    }
    const int col0_load = 10 * (static_cast<int>(jid_[3]) - 1);
    Y.block<NUM_JOINTS, 4>(0, IDX_LOAD) = Y_SL.block(0, col0_load, NUM_JOINTS, 4);
    Y(0, IDX_FV1) = dq_hat_(0);
    for (int i = 0; i < NUM_JOINTS; ++i) {
      Y(i, IDX_FC + i) = tanh_dqd(i);
    }

    // Realimentacion diag(M)-escalada con capa limite: estrictamente
    // disipativa por eje (tau_fb_i * s_i < 0); la M completa importaba el s
    // de otros ejes via M_ij y lanzaba entre articulaciones (leccion test 9)
    Vec4 sat_s;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      sat_s[i] = std::max(-1.0, std::min(1.0, s_q[i] / phi_));
    }
    const Vec4 tau_unsat = tau_nom + Y * (a_hat_ - a_prior_)
                         - M.diagonal().cwiseProduct(
                             gain_scale_ * (K_V.cwiseProduct(s_q) + K_S.cwiseProduct(sat_s)));
    const Vec4 tau = tau_unsat.cwiseMin(tau_max_).cwiseMax(-tau_max_);

    // 6. Torque → corriente y escritura
    std::array<bool, NUM_JOINTS> cur_clip{};
    const auto cur_cmd = torque_to_current(tau, dq_hat_, ref.dq, cur_clip);
    if (!send_currents(cur_cmd)) {
      emergency_stop("SyncWrite fallido");
      return;
    }

    // 7. Verificar corriente medida (seguridad)
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (std::abs(cur_meas[i]) > current_measured_peak_) {
        emergency_stop("Corriente medida insegura en J" + std::to_string(i + 1)
                       + ": " + std::to_string(cur_meas[i]) + " ticks");
        return;
      }
    }

    // 7b. Watchdog de sobrecarga: EMA (tau ~1.5 s) de |cmd|/limite por junta.
    //     Una demanda sostenida >80% del limite es la antesala de la
    //     proteccion interna del servo (corta torque en movimiento, err=128
    //     — asi cayo el brazo en el test 5): abortar limpio ANTES.
    {
      const double k_ema = Ts_ / 1.5;
      for (int i = 0; i < NUM_JOINTS; ++i) {
        const double duty = std::abs(static_cast<double>(cur_cmd[i]))
                          / static_cast<double>(current_cmd_limit_[i]);
        duty_ema_[i] = (1.0 - k_ema) * duty_ema_[i] + k_ema * duty;
        if (duty_ema_[i] > 0.80) {
          emergency_stop("Sobrecarga sostenida en J" + std::to_string(i + 1)
                         + " (EMA corriente " + std::to_string(duty_ema_[i])
                         + " del limite)");
          return;
        }
      }
    }

    // 8. Adaptacion (Euler + proyeccion + anti-windup + zona muerta +
    //    sigma-mod por bloque). Congelada fuera de RUN y cuando el torque O
    //    la corriente saturan (el actuador no aplica la ley → windup). La
    //    zona muerta descarta el ciclo limite residual y el ruido: solo el
    //    exceso de |s| sobre adapt_deadzone actualiza parametros.
    bool sat_any[NUM_JOINTS];
    bool frozen = false;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      sat_any[i] = (std::abs(tau_unsat(i)) > tau_max_) || cur_clip[i];
      frozen     = frozen || sat_any[i];
    }
    // Compuerta por calidad de tracking: solo se adapta cerca de la
    // referencia (lejos, el error es de stiction-parking/recuperacion y el
    // gradiente del regresor es basura — leccion del test 8).
    frozen = frozen || (e_q.cwiseAbs().maxCoeff() > ADAPT_EMAX);
    if (adaptive_ && phase_ == Phase::RUN && !frozen) {
      Vec4 s_dz;
      for (int i = 0; i < NUM_JOINTS; ++i) {
        const double ex = std::abs(s_q[i]) - adapt_dz_;
        s_dz[i] = (ex > 0.0) ? (s_q[i] > 0.0 ? ex : -ex) : 0.0;
      }
      const Vec12 a_dot = -GAMMA.cwiseProduct(
        Y.transpose() * s_dz + SIGMA_LEAK.cwiseProduct(a_hat_ - a_prior_));
      a_hat_ += Ts_ * a_dot;
      a_hat_ = a_hat_.cwiseMax(A_MIN).cwiseMin(A_MAX);
    }

    // 9. Publicar JointState de monitoreo
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

    // 10. CSV — solo fase RUN (t=0 al iniciar el multiseno; excluye el
    //     retorno final a q_inicial y la pausa en reposo, como en
    //     hw_fl_control_node), compatible con plots_MRAC_joint_12p.m +
    //     columnas extra de hardware al final
    if (csv_.is_open() && phase_ == Phase::RUN) {
      csv_ << std::fixed << std::setprecision(6) << t_run_elapsed
           << ',' << q(0)       << ',' << q(1)       << ',' << q(2)       << ',' << q(3)
           << ',' << dq(0)      << ',' << dq(1)      << ',' << dq(2)      << ',' << dq(3)
           << ',' << ref.q(0)   << ',' << ref.q(1)   << ',' << ref.q(2)   << ',' << ref.q(3)
           << ',' << ref.dq(0)  << ',' << ref.dq(1)  << ',' << ref.dq(2)  << ',' << ref.dq(3)
           << ',' << s_q(0)     << ',' << s_q(1)     << ',' << s_q(2)     << ',' << s_q(3)
           << ',' << tau(0)     << ',' << tau(1)     << ',' << tau(2)     << ',' << tau(3)
           << ',' << (sat_any[0] ? 1 : 0) << ',' << (sat_any[1] ? 1 : 0)
           << ',' << (sat_any[2] ? 1 : 0) << ',' << (sat_any[3] ? 1 : 0)
           << ',' << a_hat_[0]  << ',' << a_hat_[1]  << ',' << a_hat_[2]
           << ',' << a_hat_[3]  << ',' << a_hat_[4]  << ',' << a_hat_[5]  << ',' << a_hat_[6]
           << ',' << a_hat_[7]  << ',' << a_hat_[8]  << ',' << a_hat_[9]  << ',' << a_hat_[10] << ',' << a_hat_[11]
           << ',' << dq_hat_(0) << ',' << dq_hat_(1) << ',' << dq_hat_(2) << ',' << dq_hat_(3)
           << ',' << cur_cmd[0] << ',' << cur_cmd[1]  << ',' << cur_cmd[2] << ',' << cur_cmd[3]
           << ',' << cur_meas[0]<< ',' << cur_meas[1] << ',' << cur_meas[2]<< ',' << cur_meas[3]
           << '\n';
    }

    // 11. Log periodico por consola (~1 s)
    if (++log_cnt_ % static_cast<int>(std::lround(loop_rate_hz_)) == 0) {
      if (csv_.is_open()) csv_.flush();
      const char* ph = phase_ == Phase::RUN    ? "RUN "
                     : phase_ == Phase::RETURN ? "RET "
                     : phase_ == Phase::SETTLE ? "SETT" : "HOME";
      RCLCPP_INFO(get_logger(),
        "[%s] t=%.1fs |s|=%.3f |e|=%.3f  alpha=[%.3f %.3f %.3f] dm=%.3f dmcx=%.4f "
        "Fv1r=%.3f Fcr=[%.3f %.3f %.3f %.3f] i=[%d %d %d %d]",
        ph, t, s_q.norm(), e_q.norm(),
        a_hat_[0], a_hat_[1], a_hat_[2], a_hat_[3], a_hat_[4],
        a_hat_[7], a_hat_[8], a_hat_[9], a_hat_[10], a_hat_[11],
        cur_cmd[0], cur_cmd[1], cur_cmd[2], cur_cmd[3]);
    }

    // 12. Deteccion de overrun del lazo
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

  pinocchio::Model model_;
  pinocchio::Data  data_;
  std::array<pinocchio::JointIndex, NUM_JOINTS> jid_;
  std::array<Vec10, NUM_JOINTS> pi_nom_;

  std::string port_name_;
  bool        adaptive_;
  double      gain_scale_, t_run_, phi_, adapt_dz_;
  std::string mode_str_;

  Vec12 a_prior_;   // [1,1,1, 0...0] — carga y friccion residual parten de cero
  Vec12 a_hat_;     // estimacion actual [alpha1..3, theta_load, Fv1_res, Fc_res]

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
  Vec4   duty_ema_ = Vec4::Zero();  // watchdog: EMA de |cmd|/limite por junta

  dynamixel::PortHandler*   port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead>  grp_read_;
  std::unique_ptr<dynamixel::GroupSyncWrite> grp_wcur_;

  bool  hw_active_;
  bool  q_initial_captured_;
  Vec4  q_initial_, dq_initial_;
  Phase phase_{Phase::HOMING};
  double t_run0_{0.0};
  Reference return_start_ref_;  // referencia al terminar RUN, frontera del retorno
  double t_return0_{0.0};
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
    // params-file AL FINAL → el YAML del motor tiene prioridad sobre los -p del CLI
    args.push_back("--params-file");
    args.push_back(cfg);
    opts.arguments(args);
    opts.use_global_arguments(false);
    RCLCPP_INFO(rclcpp::get_logger("hw_mrac_joint_12p_node"),
      "motorXM430W350T_params auto-cargado: %s", cfg.c_str());
  } else {
    RCLCPP_WARN(rclcpp::get_logger("hw_mrac_joint_12p_node"),
      "motorXM430W350T_params no encontrado: %s — usando defaults del código.", cfg.c_str());
  }

  try {
    rclcpp::spin(std::make_shared<HWMRACJoint12pNode>(opts));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("hw_mrac_joint_12p_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
