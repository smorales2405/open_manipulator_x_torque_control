// ============================================================================
//  gz_MRAC_joint_12p_node.cpp
//  Control Adaptativo (MRAC / Slotine-Li) articular — 12 PARAMETROS
//  OpenMANIPULATOR-X (Gazebo) — campana sim-to-real (previa a hardware)
//
//  Variante del gz_MRAC_joint_node.cpp (Lab 7 Act 2, 8 parametros) orientada
//  a emular el despliegue en el robot real: adapta la escala de inercia de
//  los eslabones proximales, la CARGA en el efector (exceso inercial del
//  ultimo cuerpo), la friccion viscosa de J1 y la FRICCION DE COULOMB por
//  junta, que en el robot real domina la friccion de J2..J4 (identificacion
//  config/motorXM430W350T_params.yaml: un solo robot, una temperatura).
//
//  Parametros adaptados (12):
//    a_hat = [ alpha1..alpha3 , dm, dmcx, dmcy, dmcz , Fv1 , Fc1..Fc4 ]^T
//
//    alpha_k    (k=1..3): escala de inercia del cuerpo movido por jointk
//               (linkk+1) — masa + CoM + tensor completo, nominal = 1.0.
//    theta_load = [dm, dmcx, dmcy, dmcz]: EXCESO inercial del ultimo cuerpo
//               (link5 + lo que sujete el gripper) respecto al URDF — dm [kg]
//               y d(m·c) [kg·m] en el frame de joint4. Nominal = 0 (sin
//               carga). Una carga RIGIDA en el efector es exactamente
//               equivalente a un delta en los parametros inerciales del
//               link5, asi que vive en el span del regresor (columnas
//               [m, m·cx, m·cy, m·cz] del bloque de link5). SUSTITUYE a
//               alpha4: la escala uniforme 1-D no representa una carga (test
//               11: cilindro de 100 g = +45% de masa del link5 pero +90% de
//               primer momento -> alpha4 clavada en su tope 1.25 el 83% del
//               tiempo y descuelgue sostenido de 0.22 rad en q4); mantener
//               alpha4 Y theta_load seria colinealidad (alpha4 vive dentro
//               del span de estas columnas). El bloque I de la carga (~2 mN·m
//               a las aceleraciones del multiseno) se desprecia.
//               Verdad para el cilindro de sim_init_config.yaml (125 g,
//               Ø38 x 13 mm sostenido por su diametro, offset x = 0,
//               EE a 126 mm de joint4, frames alineados):
//               theta_load = [0.125, 0.125*0.126, 0, 0] = [0.125, 0.0158, 0, 0].
//    Fv1        friccion viscosa de joint1 [N·m·s/rad], nominal FV_NOM(1).
//               Fv de J2..J4 NO se adapta: su valor identificado es 0 en toda
//               la familia OMX, sus columnas no llevan senal (en t5-t12
//               vivieron clavados en 0) y un Fv_hat > 0 espurio con dq medida
//               es realimentacion POSITIVA de velocidad (riesgo sin beneficio).
//               El feedforward nominal FIJO de J4 (0.005*dq4) se conserva en
//               tau_nom, solo deja de adaptarse.
//    Fc_j       friccion de Coulomb de jointj [N·m], nominal = FC_NOM.
//
//  Nominales identificados (motorXM430W350T_params.yaml -> dominio de torque:
//  Fv = motor_Fv/motor_alpha, Fc = motor_Fc/motor_alpha; son los mismos
//  valores de damping/friction del Xacro a escala 1.0):
//    FV_NOM = [0.0367, 0.0000, 0.0000, 0.0050]   [N·m·s/rad]
//    FC_NOM = [0.0146, 0.0830, 0.1143, 0.0413]   [N·m]
//
//  Modelo de friccion del controlador (identico al feedforward del
//  gz_SMC_joint_node.cpp del Lab 6, validado en sim con eRMS 12-20 mrad):
//    tau_fric_j = Fv_j*dq_j + Fc_j*tanh(dq_d_j/eps),   eps = 0.05 rad/s
//  Viscosa sobre la velocidad MEDIDA dq (cancelacion exacta del damping de la
//  planta; dq - dq_r = s, la diferencia con la convencion dq_r es de orden s)
//  y Coulomb suavizado EN LAZO ABIERTO sobre la velocidad DESEADA dq_d:
//  dq_r = dq_d - Lambda*e depende del error, y una Fc_hat sobre/subestimada
//  realimentaba la oscilacion (test 2: Fc1_hat estimo 0.095 vs 0.015 real ->
//  friccion negativa neta = inyeccion de energia -> zumbido autoalimentado).
//  Con buen tracking dq ~ dq_d y ambas aproximan igual a Fc*sgn(dq).
//  La planta Gazebo (DART) implementa la friccion seca por restriccion sobre
//  dq: el modelo del controlador NO coincide exactamente con la planta,
//  igual que ocurrira en el robot real (Stribeck, stiction) -> se espera
//  adaptacion con error acotado, no convergencia parametrica exacta.
//
//  Regresor completo (4x12):
//    Y = [ Y_alpha1..3 | Y_load(4 col) | dq1 (fila 1) | diag(tanh(dq_d/eps)) ]
//  (Y_alpha via pinocchio::computeJointTorqueRegressor contraido con los
//   parametros dinamicos nominales de cada cuerpo; Y_load = columnas
//   [m, m·cx, m·cy, m·cz] del bloque de link5 del mismo regresor, SIN
//   contraer — el orden es el de pinocchio::Inertia::toDynamicParameters().)
//
//  Ley de control adaptativo (forma de desviacion respecto al prior, con la
//  realimentacion PREMULTIPLICADA por M(q) como en gz_SMC_joint_node.cpp):
//    tau_nom = tau_rigido_SlotineLi(q,dq,dqr,ddqr)
//              + Fv0 .* dq + Fc0 .* tanh(dq_d/eps)
//    tau     = tau_nom + Y*(a_hat - a_prior)
//              - M(q) * ( K_V .* s + K_S .* sat(s/phi) )
//    tau_sat = clamp(tau, -TAU_MAX, TAU_MAX)
//  En a_hat = a_prior -> tau = tau_nom - M*(...) (mejor modelo fijo + SMC).
//
//  POR QUE M(q)*(...) y no el -K_D*s crudo original: las inercias reflejadas
//  difieren ~10-100x entre ejes (M22 ~ 0.010, M44 ~ 0.001 kg·m2) y varian con
//  q, asi que NO EXISTE un K_D constante en N·m·s/rad valido para todos: en el
//  test 4 (reloj de simulacion ya corregido) K_D4 = 0.10 equivalia a una
//  ganancia efectiva K_D4/M44 ~ 90 1/s >> (0.2~0.3)/Ts -> ciclo limite de
//  22 Hz en q4, y q2/q3 saturaban 61-66% con TV(tau) ~ 1600. Con M(q) por
//  delante, K_V/K_S/Lambda son ACELERACIONES [1/s, rad/s2]: la ganancia
//  efectiva es identica en los 4 ejes y el criterio discreto se cumple por
//  diseno. Se heredan la estructura y las ganancias exactas del SMC (rho=sat,
//  phi=0.15) que en el Lab 6 Act 1 dieron eRMS 12-20 mrad, 0% de saturacion
//  y TV(tau) < 32. La capa limite K_S*sat(s/phi) ademas domina el residuo de
//  friccion de DART que el tanh no representa (en q4 equivale a ~10-20
//  rad/s2), descargando a la adaptacion: sin ella, Fc4_hat se clavaba en
//  A_MAX (test 3 y 4) intentando modelar ese residuo.
//
//  Lazo de control: 200 Hz en tiempo SIMULADO (Ts = 5 ms), alineado con el
//  nodo SMC. /joint_states llega a ~100 Hz sim: los ticks intermedios reusan
//  la ultima medicion (ZOH), pero refrescan referencia y feedforward — el
//  SMC del Lab 6 opero asi (60% de sus filas con medicion repetida) sin
//  degradacion visible.
//
//  Ley de adaptacion (proyeccion + sigma-modification):
//    a_hat_dot = -Gamma .* ( Y^T*s + sigma*(a_hat - a_prior) )   (si adaptive)
//    a_hat     = clamp(a_hat, A_MIN, A_MAX)   (alpha en [0.8,1.25], Fv,Fc >= 0)
//  Clamps por bloque segun la incertidumbre real: alpha estrecho (masas
//  pesadas con balanza), friccion amplio (~2.5x nominal, lo peor identificado).
//  La fuga sigma (robustez estandar) acota la deriva parametrica causada por
//  dinamica no modelada: la friccion de DART es un pulso de breakaway que
//  tanh() no representa, y sin fuga el gradiente bombeaba Fv4/Fc4 hasta
//  A_MAX aun sin saturacion de torque (test 3). Costo: sesgo leve hacia el
//  prior cuando el valor real difiere.
//
//  ANTI-WINDUP: la adaptacion se CONGELA mientras cualquier |tau| comandado
//  supere TAU_MAX (el torque aplicado no es el de la ley -> integrar
//  a_hat_dot seria windup: en corridas previas los parametros se clavaban en
//  A_MAX en 10-30 ms) y tambien durante la fase de homing.
//
//  HOMING + ASENTAMIENTO: el controlador de esfuerzo comanda 0 N·m hasta que
//  este nodo corre, asi que el brazo llega COLAPSADO por gravedad. Fases:
//    1) HOMING : quintica de T_HOME s desde (q,dq) medidos hasta reposo en
//                los centros del multiseno (q_d(0), 0, 0).
//    2) SETTLE : mantiene q_d(0) hasta max|e_q| < SETTLE_TOL (timeout
//                SETTLE_TMAX). Corrige el atraso residual del homing con
//                ganancias suaves (en test 2, q4 entrego 0.42 rad tarde y el
//                transitorio termino en colision).
//    3) RUN    : multiseno con envolvente de arranque + adaptacion activa.
//                El CSV y t_sim cuentan desde aqui (t = 0).
//
//  RELOJ: el nodo fuerza use_sim_time=true y su timer corre sobre /clock de
//  Gazebo (el launch ya lo puentea con ros_gz_bridge). Asi la trayectoria se
//  ejecuta a la velocidad disenada aunque la simulacion no corra a tiempo
//  real. (En el test 3 el RTF era ~0.5 y el nodo, con reloj de pared,
//  ejecutaba la trayectoria al DOBLE de velocidad: aceleraciones 4x y
//  saturacion cronica en q2/q3.) t_sim y el CSV quedan en SEGUNDOS SIMULADOS;
//  con RTF<1 la corrida tarda proporcionalmente mas en reloj de pared.
//
//  a_prior (= a_hat(0) = referencia de la forma de desviacion):
//    friction_prior:=true  (defecto) -> [1,1,1, 0,0,0,0, FV_NOM(1), FC_NOM]
//        mejor modelo fijo identificado; baseline justo para adaptive:=false.
//    friction_prior:=false           -> [1,1,1, 0,0,0,0, 0, 0,0,0,0]
//        "robot nunca identificado": la adaptacion debe descubrir la
//        friccion desde cero (escenario C4).
//    theta_load parte SIEMPRE de cero: "sin carga" es el estado nominal.
//
//  Campana de comparaciones sugerida (escalas mass/damping/friction en
//  config/sim_init_config.yaml — a diferencia del Act2, friction_scale va en
//  1.0 o perturbado, NO en 0.0: el robot real tiene Coulomb):
//    C1 gemelo digital   : 1.0/1.0/1.0, adaptive true vs false.
//                          Esperado: params cerca del prior (test anti-deriva).
//    C2 error de friccion: 1.0/{0.5,1.5}/{0.5,1.5}, adaptive true vs false.
//                          Esperado: Fc_hat -> friction_scale*FC_NOM; el caso
//                          fijo muestra error en las inversiones de velocidad.
//    C3 "otro robot"     : conjunto asimetrico, ej. 1.1/1.3/0.7.
//                          Test de identificabilidad conjunta de los 3 bloques.
//    C4 sin identificar  : 1.0/1.0/1.0, adaptive:=true friction_prior:=false.
//                          ¿La adaptacion en linea reemplaza la identificacion
//                          offline? Comparar contra el fijo de C1.
//    C5 carga en efector : 1.0/1.0/1.0 + spawn_load: true (cilindro 125 g).
//                          Esperado: dm -> 0.125 kg, dmcx -> 0.0158 kg·m y
//                          tracking nivel C1 (con alpha4 era imposible:
//                          capacidad 0.035-0.13 N·m vs los ~0.15-0.30 N·m que
//                          pide la carga). SIN carga, theta_load debe quedarse
//                          en ~0: es el nuevo test anti-deriva (C1'). Sesgos
//                          esperados sin carga: d(m·cy) vaga (columna casi sin
//                          excitacion, fuera del plano sagital) y el residuo
//                          de stiction de DART en q4 puede filtrarse como
//                          d(m·c) ~ 0.001-0.002 kg·m — acotado por sigma-mod.
//
//  Trayectoria articular de referencia — MULTISENO con envolvente de arranque
//  (excitacion persistente con inversiones frecuentes de velocidad,
//  necesarias para separar Fv de Fc). Centros/amplitudes RE-disenados con
//  restriccion cartesiana (verificado con Pinocchio sobre 300 s): altura del
//  efector z >= 83 mm en todo t — la version anterior (centros -0.30/0.30/
//  0.80, amplitudes 0.45/0.15) penetraba la base con z_min = -16 mm, causa
//  directa de las colisiones observadas. tau_ff <= 0.63 N·m (margen ~2x) y
//  limites articulares con holgura:
//    q1_d =  0.00 + 0.60*sin(0.6t) + 0.20*sin(1.5t)
//    q2_d = -0.55 + 0.25*sin(0.9t) + 0.09*sin(1.9t)
//    q3_d =  0.10 + 0.25*sin(1.3t) + 0.09*sin(0.7t)
//    q4_d =  0.50 + 0.25*sin(1.7t) + 0.09*sin(1.1t)
//  La parte oscilatoria va multiplicada por una envolvente quintica 0->1 en
//  T_RAMP s, asi q_d arranca EXACTAMENTE en reposo en los centros (empalme
//  suave con la fase de asentamiento).
//  Arranque en q_d(0): use_fixed_init: true, q_init: [0.0, -0.55, 0.10, 0.50].
//
//  Suscriptor : /joint_states                    (sensor_msgs/JointState)
//  Publicador : /arm_effort_controller/commands   (std_msgs/Float64MultiArray)
//
//  Parametros ROS 2 (--ros-args -p nombre:=valor):
//    test_num        [int]     1     — identificador del CSV generado
//    t_sim           [double]  0.0   — duracion en segundos SIMULADOS (0 = ilimitado)
//    adaptive        [bool]    true  — true: adapta | false: a_hat fijo en a_prior
//    friction_prior  [bool]    true  — true: prior = friccion identificada
//                                      false: prior de friccion en cero (C4)
//
//  CSV generado: data/lab7/sim/mrac12p/gz_mrac_joint_12p_<modo>_<test_num>.csv
//    <modo> = "adaptive" | "fixed" [+ "_noprior" si friction_prior:=false]
//  Columnas: t, q1..q4, dq1..dq4, q1_des..q4_des, dq1_des..dq4_des,
//            s1..s4, tau1..tau4, sat1..sat4,
//            a1_hat..a3_hat, dm_hat, dmcx_hat, dmcy_hat, dmcz_hat,
//            fv1_hat, fc1_hat..fc4_hat
//
//  Ejemplos de uso:
//
//    # C1/C2/C3 — adaptativo con prior identificado:
//    ros2 run open_manipulator_x_torque_control gz_mrac_joint_12p_node
//      --ros-args -p adaptive:=true -p test_num:=1 -p t_sim:=30.0
//
//    # Baseline no adaptativo (mejor modelo fijo identificado):
//    ros2 run open_manipulator_x_torque_control gz_mrac_joint_12p_node
//      --ros-args -p adaptive:=false -p test_num:=2 -p t_sim:=30.0
//
//    # C4 — adaptativo partiendo sin conocimiento de friccion:
//    ros2 run open_manipulator_x_torque_control gz_mrac_joint_12p_node
//      --ros-args -p adaptive:=true -p friction_prior:=false -p test_num:=4 -p t_sim:=30.0
//
// ============================================================================

#include <array>
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
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/regressor.hpp>

using namespace std::chrono_literals;

#ifndef PACKAGE_DATA_DIR
#define PACKAGE_DATA_DIR "."
#endif
#ifndef PACKAGE_URDF_DIR
#define PACKAGE_URDF_DIR "."
#endif

static constexpr double PI      = M_PI;
static constexpr int    NARM    = 4;       // articulaciones controladas
static constexpr int    NP      = 12;      // parametros: [alpha1..3, theta_load(4), Fv1, Fc1..4]

// Indices de bloque dentro de a_hat:
static constexpr int IDX_ALPHA = 0;   // [0..2]  alpha1..alpha3
static constexpr int IDX_LOAD  = 3;   // [3..6]  dm, dmcx, dmcy, dmcz (exceso del link5)
static constexpr int IDX_FV1   = 7;   // [7]     Fv1
static constexpr int IDX_FC    = 8;   // [8..11] Fc1..Fc4
static constexpr double TAU_MAX = 1.2;     // [N·m] limite de torque por articulacion
static constexpr double DT      = 0.005;   // [s] periodo de control (200 Hz, como el nodo SMC)
static constexpr double T_HOME  = 5.0;     // [s] duracion del homing quintico inicial
static constexpr double T_RAMP  = 2.0;     // [s] envolvente 0->1 del multiseno
static constexpr double SETTLE_TOL  = 0.08;  // [rad] max|e| para dar por asentado
static constexpr double SETTLE_TMAX = 5.0;   // [s] timeout de la fase de asentamiento

// [rad/s] suavizado tanh de Coulomb del feedforward de friccion. Igual que
// FRIC_EPS del nodo SMC: como se alimenta con la velocidad DESEADA dq_d
// (senal limpia, sin ruido de medicion), una capa estrecha es segura y
// compensa el Coulomb casi como sgn(dq_d). Es ademas el mismo eps de la
// identificacion (motor_epsilon_friction = 0.05).
static constexpr double EPS_FRICTION = 0.05;

using Vec4  = Eigen::Vector4d;
using Vec12 = Eigen::Matrix<double, NP, 1>;
using Vec10 = Eigen::Matrix<double, 10, 1>;
using MatY  = Eigen::Matrix<double, NARM, NP>;

// Friccion nominal identificada por junta (URDF/Xacro escala 1.0, derivada de
// config/motorXM430W350T_params.yaml en dominio de torque). FV_NOM completa se
// usa como feedforward FIJO en tau_nom; como parametro ADAPTADO solo entra
// Fv1 (J2..J4 tienen Fv identificado = 0 en toda la familia OMX).
static const Vec4 FV_NOM = (Vec4() << 0.0367, 0.0000, 0.0000, 0.0050).finished();
static const Vec4 FC_NOM = (Vec4() << 0.0146, 0.0830, 0.1143, 0.0413).finished();

// ═══════════════════════════════════════════════════════════════════════════
//  GANANCIAS MRAC 12p — realimentacion M(q)-escalada (estructura del nodo SMC)
//  Indice articular: [joint1, joint2, joint3, joint4]
//  Indice parametro: [alpha1..3, dm, dmcx, dmcy, dmcz, Fv1, Fc1..4]
// ═══════════════════════════════════════════════════════════════════════════
// LAMBDA_Q / K_V / K_S / PHI_BL: HEREDADOS SIN CAMBIO de gz_SMC_joint_node.cpp
// (rho=sat), validados en el Lab 6 Act 1 (eRMS 12-20 mrad, 0% sat, TV < 32).
// Al ir premultiplicadas por M(q), son aceleraciones: mismo efecto en los 4
// ejes, sin depender de la inercia reflejada de cada uno.
//  - Criterio discreto (Ts = 5 ms): ganancia efectiva dentro de la capa
//    K_V + K_S/PHI_BL <= (0.2~0.3)/Ts. Aqui: 28/38/38/50 [1/s], bordes OK.
//  - K_S solo domina la incertidumbre acotada (residuo de Coulomb DART,
//    errores de escala): K_S grande degenera sat() en sign() (chattering).
//  - joint4: residuo de friccion / M44 ~ 10-20 rad/s2 -> K_S4 y Lambda4 altos.
static const Vec4 LAMBDA_Q = {10.0, 10.0, 10.0, 15.0};   // superficie de seguimiento [1/s]
static const Vec4 K_V      = { 8.0,  8.0,  8.0, 10.0};   // alcance exponencial sobre s [1/s]
static const Vec4 K_S      = { 3.0,  4.5,  4.5,  6.0};   // conmutacion sat(s/phi) [rad/s2]
static constexpr double PHI_BL = 0.15;                   // capa limite de sat() [rad/s]

// Tasas de adaptacion por bloque: pequena en alpha (masas bien conocidas,
// pesadas con balanza), mayor en friccion (lo peor identificado). Si Fc_hat
// converge lento en C2/C4, subir el bloque Fc hasta ~5.0.
// theta_load: ENFRIADO tras el test 14 — con Gamma_dm = 0.3 la columna de
// gravedad (~2.5 N·m/kg) daba convergencia en 0.2 s, demasiado caliente para
// la cresta mal condicionada que forman las columnas m y m·cx (casi
// colineales sobre esta trayectoria: ambas son gravedad con la misma firma):
// dm y dmcx oscilaban en contrafase entre sus topes (periodo 10-15 s) y q3
// empeoraba. Con 0.05/0.002 la convergencia pasa a ~2-3 s bien amortiguada.
static const Vec12 GAMMA =
  (Vec12() << 0.3, 0.3, 0.3,   0.05, 0.002, 0.002, 0.002,   2.0,   2.0, 2.0, 2.0, 2.0).finished();

// Fuga sigma-modification hacia el prior [1/s], POR BLOQUE: acota la deriva
// parametrica por dinamica no modelada (friccion de pulso de DART). En el
// bloque de carga la fuga es debil (0.02): su prior es CERO, y con carga real
// una fuga fuerte pelea contra el valor verdadero (parte de la caza del test
// 14); las cotas fisicas estrechas ya contienen la deriva. Subir un bloque si
// deriva al tope; bajarlo si el sesgo hacia el prior es excesivo (C2/C4).
static const Vec12 SIGMA_LEAK =
  (Vec12() << 0.1, 0.1, 0.1,   0.02, 0.02, 0.02, 0.02,   0.1,   0.1, 0.1, 0.1, 0.1).finished();

// Proyeccion a la region fisica admisible (anti-deriva parametrica).
// alpha estrecho: no dejar que alpha absorba errores de friccion.
// theta_load: hasta 200 g de carga a ~17.5 cm (dm 0.20 kg, dmcx 0.035 kg·m);
// margen negativo pequeno para que el error de modelo no se acumule en el
// borde. dmcy casi sin excitacion -> cota simetrica estrecha.
static const Vec12 A_MIN =
  (Vec12() << 0.80, 0.80, 0.80,   -0.02, -0.005, -0.010, -0.005,   0.0,   0.0, 0.0, 0.0, 0.0).finished();
static const Vec12 A_MAX =
  (Vec12() << 1.25, 1.25, 1.25,    0.20,  0.035,  0.010,  0.020,   0.15,  0.30, 0.30, 0.30, 0.30).finished();
// ═══════════════════════════════════════════════════════════════════════════

// ── Trayectoria de referencia articular (multiseno seguro + envolvente) ──────
//   Parte oscilatoria multiplicada por una envolvente quintica sigma(t): 0->1
//   en T_RAMP s, de modo que q_d(0) = centros con dq_d(0) = ddq_d(0) = 0
//   (empalme exacto con la fase de asentamiento). Centros/amplitudes con
//   z_efector >= 83 mm garantizado (ver cabecera).
struct Reference {
  Vec4 q, dq, ddq;
};

static Reference desiredTrajectory(double t)
{
  static const Vec4 C  = (Vec4() <<  0.00, -0.55,  0.10,  0.50).finished();
  static const Vec4 A1 = (Vec4() <<  0.60,  0.25,  0.25,  0.25).finished();
  static const Vec4 W1 = (Vec4() <<  0.6,   0.9,   1.3,   1.7 ).finished();
  static const Vec4 A2 = (Vec4() <<  0.20,  0.09,  0.09,  0.09).finished();
  static const Vec4 W2 = (Vec4() <<  1.5,   1.9,   0.7,   1.1 ).finished();

  // Envolvente quintica: sigma = 10x^3 - 15x^4 + 6x^5, x = t/T_RAMP
  double sg = 1.0, dsg = 0.0, ddsg = 0.0;
  if (t < T_RAMP) {
    const double x = t / T_RAMP;
    sg   = ((6.0 * x - 15.0) * x + 10.0) * x * x * x;
    dsg  = ((30.0 * x - 60.0) * x + 30.0) * x * x / T_RAMP;
    ddsg = ((120.0 * x - 180.0) * x + 60.0) * x / (T_RAMP * T_RAMP);
  }

  Reference ref;
  for (int i = 0; i < NARM; ++i) {
    const double osc   =  A1[i] * std::sin(W1[i] * t) + A2[i] * std::sin(W2[i] * t);
    const double dosc  =  A1[i] * W1[i] * std::cos(W1[i] * t)
                        + A2[i] * W2[i] * std::cos(W2[i] * t);
    const double ddosc = -A1[i] * W1[i] * W1[i] * std::sin(W1[i] * t)
                        - A2[i] * W2[i] * W2[i] * std::sin(W2[i] * t);
    ref.q[i]   = C[i] + sg * osc;
    ref.dq[i]  = dsg * osc + sg * dosc;
    ref.ddq[i] = ddsg * osc + 2.0 * dsg * dosc + sg * ddosc;
  }
  return ref;
}

// ── Fases de arranque ─────────────────────────────────────────────────────────
enum class Phase { HOMING, SETTLE, RUN };

// ── Homing quintico: de la postura medida (q0,v0, acc 0) en t=0 hasta el ────
//   REPOSO en los centros del multiseno (q_d(0), 0, 0) en t=T_HOME (gracias a
//   la envolvente, desiredTrajectory(0) ya es reposo). Coeficientes estandar
//   del polinomio de 5to orden con condiciones de frontera completas.
static Reference homingTrajectory(double t, const Vec4 & q0, const Vec4 & v0)
{
  const Reference end = desiredTrajectory(0.0);
  const double T = T_HOME;
  Reference ref;
  for (int i = 0; i < NARM; ++i) {
    const double h  = end.q[i] - q0[i];
    const double vf = end.dq[i];
    const double af = end.ddq[i];
    const double c0 = q0[i];
    const double c1 = v0[i];
    const double c2 = 0.0;                                                     // acc inicial = 0
    const double c3 = ( 20.0*h - (8.0*vf + 12.0*v0[i])*T + af*T*T) / (2.0*T*T*T);
    const double c4 = (-30.0*h + (14.0*vf + 16.0*v0[i])*T - 2.0*af*T*T) / (2.0*T*T*T*T);
    const double c5 = ( 12.0*h -  6.0*(vf + v0[i])*T + af*T*T) / (2.0*T*T*T*T*T);
    ref.q[i]   = c0 + c1*t + c2*t*t + c3*t*t*t + c4*t*t*t*t + c5*t*t*t*t*t;
    ref.dq[i]  = c1 + 2.0*c2*t + 3.0*c3*t*t + 4.0*c4*t*t*t + 5.0*c5*t*t*t*t;
    ref.ddq[i] = 2.0*c2 + 6.0*c3*t + 12.0*c4*t*t + 20.0*c5*t*t*t;
  }
  return ref;
}

// ── Termino Slotine-Li rigido:  M(q)*ddq_r + C(q,dq)*dq_r + g(q) via RNEA ────
//   Identidades (rnea(q,v,a) = M(q)a + C(q,v)v + g(q)):
//     g(q)        = rnea(q, 0,   0)
//     M(q)*ddq_r  = rnea(q, 0,   ddq_r) - g
//     C(q,dq)*dq_r= 1/2 [ rnea(q, dq+dq_r, 0) - rnea(q, dq, 0) - rnea(q, dq_r, 0) + g ]
static Eigen::VectorXd slotineLiTorque(
    const pinocchio::Model & model, pinocchio::Data & data,
    const Eigen::VectorXd & q,   const Eigen::VectorXd & dq,
    const Eigen::VectorXd & dqr, const Eigen::VectorXd & ddqr)
{
  const Eigen::VectorXd zero  = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd g     = pinocchio::rnea(model, data, q, zero,      zero);
  const Eigen::VectorXd Mddqr = pinocchio::rnea(model, data, q, zero,      ddqr);  // M*ddq_r + g
  const Eigen::VectorXd r_sum = pinocchio::rnea(model, data, q, dq + dqr,  zero);
  const Eigen::VectorXd r_dq  = pinocchio::rnea(model, data, q, dq,        zero);
  const Eigen::VectorXd r_dqr = pinocchio::rnea(model, data, q, dqr,       zero);
  const Eigen::VectorXd Cqr   = 0.5 * (r_sum - r_dq - r_dqr + g);                 // C(q,dq)*dq_r
  return Mddqr + Cqr;                                                            // M*ddq_r + C*dq_r + g
}

// ── Regresor Slotine-Li:  misma identidad de polarizacion, aplicada al ──────
//   regresor R(q,v,a) en vez de RNEA. R(q,v,a)*pi = M(q)a+C(q,v)v+g(q) para
//   cualquier vector de parametros dinamicos pi (uno por cuerpo, 10 c/u).
//   Cada llamada a computeJointTorqueRegressor sobrescribe data.jointTorqueRegressor,
//   por eso cada resultado se copia (Eigen::MatrixXd) antes de la siguiente.
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

// ── Nodo principal ────────────────────────────────────────────────────────────
class MRACJoint12pSimNode : public rclcpp::Node
{
public:
  MRACJoint12pSimNode()
  : Node("gz_mrac_joint_12p_node"), t_(0.0)
  {
    // ── Reloj de simulacion: el timer avanza con /clock de Gazebo ────────────
    //    (robusto a RTF < 1; el launch ya puentea /clock con ros_gz_bridge)
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // ── Parametros ────────────────────────────────────────────────────────────
    this->declare_parameter<int>   ("test_num",       1);
    this->declare_parameter<double>("t_sim",          0.0);
    this->declare_parameter<bool>  ("adaptive",       true);
    this->declare_parameter<bool>  ("friction_prior", true);

    const int test_num = this->get_parameter("test_num").as_int();
    t_sim_             = this->get_parameter("t_sim").as_double();
    adaptive_          = this->get_parameter("adaptive").as_bool();
    friction_prior_    = this->get_parameter("friction_prior").as_bool();

    mode_str_ = adaptive_ ? "adaptive" : "fixed";
    if (!friction_prior_) { mode_str_ += "_noprior"; }

    // Prior de parametros (= a_hat(0) y referencia de la forma de desviacion).
    // fv_prior_ es el feedforward viscoso FIJO de las 4 juntas; como parametro
    // adaptado solo entra Fv1. theta_load parte de cero (sin carga).
    fv_prior_ = friction_prior_ ? FV_NOM : Vec4::Zero().eval();
    a_prior_.setZero();
    a_prior_.segment<3>(IDX_ALPHA).setOnes();
    a_prior_[IDX_FV1]              = fv_prior_[0];
    a_prior_.segment<NARM>(IDX_FC) = friction_prior_ ? FC_NOM : Vec4::Zero().eval();
    a_hat_ = a_prior_;

    // ── Modelo Pinocchio (brazo nominal, escala 1.0) ────────────────────────────
    const std::string urdf_path =
      std::string(PACKAGE_URDF_DIR) + "/open_manipulator_x.urdf";
    try {
      pinocchio::urdf::buildModel(urdf_path, model_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Pinocchio buildModel: %s", e.what());
      throw;
    }
    data_ = pinocchio::Data(model_);

    precompute_regressor_bases();

    RCLCPP_INFO(this->get_logger(),
      "MRAC articular 12p — modo=%s  friction_prior=%s  tau_max=%.2f N·m",
      mode_str_.c_str(), friction_prior_ ? "identificado" : "cero", TAU_MAX);
    RCLCPP_INFO(this->get_logger(),
      "Lambda=[%.1f %.1f %.1f %.1f]  Kv=[%.1f %.1f %.1f %.1f]  "
      "Ks=[%.1f %.1f %.1f %.1f]  phi=%.2f  (realimentacion M(q)-escalada, gan. del SMC)",
      LAMBDA_Q[0], LAMBDA_Q[1], LAMBDA_Q[2], LAMBDA_Q[3],
      K_V[0],      K_V[1],      K_V[2],      K_V[3],
      K_S[0],      K_S[1],      K_S[2],      K_S[3],  PHI_BL);
    RCLCPP_INFO(this->get_logger(),
      "Gamma: alpha=[%.2f %.2f %.2f]  load=[%.2f %.3f %.3f %.3f]  Fv1=%.2f  Fc=[%.2f %.2f %.2f %.2f]",
      GAMMA[0], GAMMA[1], GAMMA[2],
      GAMMA[3], GAMMA[4], GAMMA[5],  GAMMA[6],  GAMMA[7],
      GAMMA[8], GAMMA[9], GAMMA[10], GAMMA[11]);
    RCLCPP_INFO(this->get_logger(),
      "a_hat(0): alpha=[%.2f %.2f %.2f]  load=[%.3f %.4f %.4f %.4f]  Fv1=%.4f  Fc=[%.4f %.4f %.4f %.4f]",
      a_prior_[0], a_prior_[1], a_prior_[2],
      a_prior_[3], a_prior_[4], a_prior_[5],  a_prior_[6],  a_prior_[7],
      a_prior_[8], a_prior_[9], a_prior_[10], a_prior_[11]);
    if (t_sim_ > 0.0) {
      RCLCPP_INFO(this->get_logger(),
        "t_sim = %.1f s (contados desde el inicio del multiseno, tras homing+asentamiento)",
        t_sim_);
    } else {
      RCLCPP_INFO(this->get_logger(), "t_sim = ilimitado");
    }

    open_csv(test_num);

    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_effort_controller/commands", 10);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        last_js_ = msg;
      });

    RCLCPP_INFO(this->get_logger(),
      "use_sim_time=true: timer sobre /clock (t_sim y CSV en segundos simulados)");

    timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(DT),
      [this]() { tick(); });
  }

  ~MRACJoint12pSimNode()
  {
    if (csv_.is_open()) {
      csv_.close();
      RCLCPP_INFO(this->get_logger(), "CSV cerrado: %s", csv_path_.c_str());
    }
  }

private:
  // ── Indices de junta y parametros dinamicos nominales (para Y_alpha) ───────
  //   pi_nom_[i] = parametros dinamicos nominales (10) del cuerpo movido por
  //   jointi (linki+1), en el orden interno de Pinocchio (ver
  //   pinocchio::Inertia::toDynamicParameters()). Y_alpha.col(i), i=0..2,
  //   contrae el bloque de 10 columnas de jointi en el regresor Slotine-Li
  //   con pi_nom_[i]: escalado UNIFORME de masa + CoM + tensor completo.
  //   jid_[3] se usa para ubicar el bloque de link5 del que se toman las 4
  //   columnas de theta_load sin contraer.
  void precompute_regressor_bases()
  {
    for (int i = 0; i < NARM; ++i) {
      const std::string jname = "joint" + std::to_string(i + 1);
      jid_[i]    = model_.getJointId(jname);
      pi_nom_[i] = model_.inertias[jid_[i]].toDynamicParameters();
    }
  }

  // ── CSV ───────────────────────────────────────────────────────────────────
  void open_csv(int test_num)
  {
    std::filesystem::create_directories(
      std::string(PACKAGE_DATA_DIR) + "/lab7/sim/mrac12p");

    csv_path_ = std::string(PACKAGE_DATA_DIR) + "/lab7/sim/mrac12p/gz_mrac_joint_12p_"
                + mode_str_ + "_" + std::to_string(test_num) + ".csv";
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
         << "s1,s2,s3,s4,"
         << "tau1,tau2,tau3,tau4,"
         << "sat1,sat2,sat3,sat4,"
         << "a1_hat,a2_hat,a3_hat,"
         << "dm_hat,dmcx_hat,dmcy_hat,dmcz_hat,"
         << "fv1_hat,fc1_hat,fc2_hat,fc3_hat,fc4_hat\n";
    RCLCPP_INFO(this->get_logger(), "CSV: %s", csv_path_.c_str());
  }

  // ── Lectura de estados articulares (por nombre, orden independiente) ───────
  void read_js(Vec4 & q, Vec4 & dq)
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

  // ── Tick de control a 200 Hz (tiempo simulado) ────────────────────────────
  void tick()
  {
    if (!last_js_) { return; }

    Vec4 q, dq;
    read_js(q, dq);

    // ── Maquina de fases: HOMING -> SETTLE -> RUN ───────────────────────────
    if (!home_init_) {
      q_home0_  = q;
      dq_home0_ = dq;
      home_init_ = true;
      const Reference r0 = desiredTrajectory(0.0);
      RCLCPP_INFO(this->get_logger(),
        "Homing (%.1f s): q0=[%.3f %.3f %.3f %.3f] -> q_d(0)=[%.2f %.2f %.2f %.2f]",
        T_HOME, q[0], q[1], q[2], q[3], r0.q[0], r0.q[1], r0.q[2], r0.q[3]);
    }

    Reference ref;
    if (phase_ == Phase::HOMING) {
      if (t_ < T_HOME) {
        ref = homingTrajectory(t_, q_home0_, dq_home0_);
      } else {
        phase_ = Phase::SETTLE;
        RCLCPP_INFO(this->get_logger(),
          "Homing completado -> asentamiento en q_d(0) (tol %.2f rad, max %.1f s).",
          SETTLE_TOL, SETTLE_TMAX);
      }
    }
    if (phase_ == Phase::SETTLE) {
      ref = desiredTrajectory(0.0);                       // reposo en los centros
      const double e_max = (q - ref.q).cwiseAbs().maxCoeff();
      if (e_max < SETTLE_TOL || t_settle_ >= SETTLE_TMAX) {
        phase_ = Phase::RUN;
        RCLCPP_INFO(this->get_logger(),
          "Asentado (max|e|=%.3f rad tras %.1f s). Inicia multiseno + adaptacion. CSV desde t=0.",
          e_max, t_settle_);
      }
    }
    if (phase_ == Phase::RUN) {
      ref = desiredTrajectory(t_run_);
    }

    // ── Errores, superficie y referencias auxiliares ───────────────────────
    const Vec4 e_q   = q  - ref.q;
    const Vec4 e_dq  = dq - ref.dq;
    const Vec4 s_q   = e_dq + LAMBDA_Q.asDiagonal() * e_q;          // s = dq - dq_r
    const Vec4 dqr   = ref.dq  - LAMBDA_Q.asDiagonal() * e_q;       // dq_r
    const Vec4 ddqr  = ref.ddq - LAMBDA_Q.asDiagonal() * e_dq;      // ddq_r

    // Coulomb suavizado sobre dq_d (LAZO ABIERTO: no depende del error, evita
    // que una Fc_hat sobre/subestimada realimente oscilaciones)
    const Vec4 tanh_dqd = ((ref.dq / EPS_FRICTION).array().tanh()).matrix();

    // ── Vectores de tamano nv para Pinocchio (gripper en cero) ─────────────
    Eigen::VectorXd q_pin    = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dq_pin   = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd dqr_pin  = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd ddqr_pin = Eigen::VectorXd::Zero(model_.nv);
    q_pin.head<NARM>()    = q;
    dq_pin.head<NARM>()   = dq;
    dqr_pin.head<NARM>()  = dqr;
    ddqr_pin.head<NARM>() = ddqr;

    // ── Matriz de inercia nominal M(q) (CRBA) para la realimentacion ────────
    pinocchio::crba(model_, data_, q_pin);
    data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.triangularView<Eigen::StrictlyUpper>().transpose();
    const Eigen::Matrix4d M = data_.M.topLeftCorner<NARM, NARM>();

    // ── Termino rigido nominal (RNEA) + friccion nominal del prior ──────────
    const Eigen::VectorXd tau_nom_rigid_full =
      slotineLiTorque(model_, data_, q_pin, dq_pin, dqr_pin, ddqr_pin);
    const Vec4 tau_nom = tau_nom_rigid_full.head<NARM>()
                       + fv_prior_.cwiseProduct(dq)
                       + a_prior_.segment<NARM>(IDX_FC).cwiseProduct(tanh_dqd);

    // ── Regresor Slotine-Li completo y columnas de escala de inercia ───────
    const Eigen::MatrixXd Y_SL = slotineLiRegressor(model_, data_, q_pin, dq_pin, dqr_pin, ddqr_pin);

    MatY Y = MatY::Zero();
    for (int i = 0; i < 3; ++i) {                                        // Y_alpha1..3
      const int col0 = 10 * (static_cast<int>(jid_[i]) - 1);
      Y.col(IDX_ALPHA + i) = (Y_SL.middleCols(col0, 10) * pi_nom_[i]).head<NARM>();
    }
    // Y_load: columnas [m, m·cx, m·cy, m·cz] del bloque de link5, sin contraer
    const int col0_load = 10 * (static_cast<int>(jid_[3]) - 1);
    Y.block<NARM, 4>(0, IDX_LOAD) = Y_SL.block(0, col0_load, NARM, 4);
    Y(0, IDX_FV1) = dq[0];                                               // Y_Fv1 = dq1 medida, como el SMC
    for (int i = 0; i < NARM; ++i) {
      Y(i, IDX_FC + i) = tanh_dqd[i];                                    // Y_Fc = diag(tanh(dq_d/eps))
    }

    // ── Ley de control adaptativo (desviacion del prior + realimentacion ────
    //    M(q)-escalada con capa limite, estructura del nodo SMC)
    Vec4 sat_s;
    for (int i = 0; i < NARM; ++i) {
      sat_s[i] = std::max(-1.0, std::min(1.0, s_q[i] / PHI_BL));
    }
    const Vec4 tau = tau_nom + Y * (a_hat_ - a_prior_)
                   - M * (K_V.cwiseProduct(s_q) + K_S.cwiseProduct(sat_s));
    const Vec4 tau_sat = tau.cwiseMin(TAU_MAX).cwiseMax(-TAU_MAX);

    // ── Publicar torques ───────────────────────────────────────────────────
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(tau_sat.data(), tau_sat.data() + NARM);
    torque_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[%s] t=%.2fs  |s|=%.4f  |e|=%.4f rad  alpha=[%.3f %.3f %.3f]  "
      "load=[%.3fkg %.4f %.4f %.4f]  Fv1=%.4f  Fc=[%.4f %.4f %.4f %.4f]",
      phase_ == Phase::RUN ? "RUN" : (phase_ == Phase::SETTLE ? "SETTLE" : "HOME"),
      t_, s_q.norm(), e_q.norm(),
      a_hat_[0], a_hat_[1], a_hat_[2],
      a_hat_[3], a_hat_[4], a_hat_[5],  a_hat_[6],  a_hat_[7],
      a_hat_[8], a_hat_[9], a_hat_[10], a_hat_[11]);

    // ── Registro CSV (a_hat usado en este tick; solo en fase RUN) ──────────
    if (csv_.is_open() && phase_ == Phase::RUN) {
      csv_ << std::fixed << std::setprecision(6)
           << t_run_
           << ',' << q[0]        << ',' << q[1]        << ',' << q[2]        << ',' << q[3]
           << ',' << dq[0]       << ',' << dq[1]       << ',' << dq[2]       << ',' << dq[3]
           << ',' << ref.q[0]    << ',' << ref.q[1]    << ',' << ref.q[2]    << ',' << ref.q[3]
           << ',' << ref.dq[0]   << ',' << ref.dq[1]   << ',' << ref.dq[2]   << ',' << ref.dq[3]
           << ',' << s_q[0]      << ',' << s_q[1]      << ',' << s_q[2]      << ',' << s_q[3]
           << ',' << tau_sat[0]  << ',' << tau_sat[1]  << ',' << tau_sat[2]  << ',' << tau_sat[3]
           << ',' << (std::abs(tau[0]) > TAU_MAX ? 1 : 0)
           << ',' << (std::abs(tau[1]) > TAU_MAX ? 1 : 0)
           << ',' << (std::abs(tau[2]) > TAU_MAX ? 1 : 0)
           << ',' << (std::abs(tau[3]) > TAU_MAX ? 1 : 0)
           << ',' << a_hat_[0]   << ',' << a_hat_[1]   << ',' << a_hat_[2]   << ',' << a_hat_[3]
           << ',' << a_hat_[4]   << ',' << a_hat_[5]   << ',' << a_hat_[6]   << ',' << a_hat_[7]
           << ',' << a_hat_[8]   << ',' << a_hat_[9]   << ',' << a_hat_[10]  << ',' << a_hat_[11]
           << '\n';
    }

    // ── Ley de adaptacion (Euler + proyeccion + anti-windup + sigma-mod) ───
    //    Solo en fase RUN y sin saturacion (el torque aplicado no es el de la
    //    ley -> integrar seria windup). La fuga sigma acota la deriva por
    //    dinamica no modelada (friccion de pulso de DART).
    const bool tau_clipped = (tau.array().abs() > TAU_MAX).any();
    if (adaptive_ && phase_ == Phase::RUN && !tau_clipped) {
      const Vec12 a_dot = -GAMMA.cwiseProduct(
        Y.transpose() * s_q + SIGMA_LEAK.cwiseProduct(a_hat_ - a_prior_));
      a_hat_ += DT * a_dot;
      a_hat_ = a_hat_.cwiseMax(A_MIN).cwiseMin(A_MAX);
    }

    t_ += DT;
    if (phase_ == Phase::SETTLE) { t_settle_ += DT; }
    if (phase_ == Phase::RUN)    { t_run_    += DT; }

    if (t_sim_ > 0.0 && phase_ == Phase::RUN && t_run_ >= t_sim_) {
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

  // ── Miembros ───────────────────────────────────────────────────────────────
  pinocchio::Model model_;
  pinocchio::Data  data_;

  std::array<pinocchio::JointIndex, NARM> jid_;
  std::array<Vec10, NARM> pi_nom_;

  double      t_;
  double      t_sim_;
  bool        adaptive_;
  bool        friction_prior_;
  Phase       phase_     = Phase::HOMING;
  bool        home_init_ = false;  // postura inicial ya capturada
  Vec4        q_home0_, dq_home0_; // estado medido al primer tick
  double      t_settle_  = 0.0;    // tiempo transcurrido en SETTLE
  double      t_run_     = 0.0;    // tiempo del experimento (fase RUN)
  std::string mode_str_;
  Vec12       a_prior_;   // a_hat(0) y referencia de la forma de desviacion
  Vec12       a_hat_;     // estimacion actual [alpha1..3, theta_load, Fv1, Fc1..4]
  Vec4        fv_prior_;  // feedforward viscoso FIJO por junta (solo Fv1 se adapta)

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
    rclcpp::spin(std::make_shared<MRACJoint12pSimNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gz_mrac_joint_12p_node"), "Excepcion: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
