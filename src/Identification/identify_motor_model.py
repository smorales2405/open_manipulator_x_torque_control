#!/usr/bin/env python3
"""
identify_motor_model.py
Identificación del modelo de conversión torque→corriente de los 4 motores
Dynamixel XM430-W350-T del OpenMANIPULATOR-X, por mínimos cuadrados (OLS).

Modelo por articulación (mismo que usan los nodos hw_*):

    I_meas[i] = alpha[i]·tau[i] + Fv[i]·dq[i] + Fc[i]·tanh(dq[i]/eps) + I_offset[i]

donde:
    tau[i]  = torque de cuerpo rígido por RNEA (Pinocchio + URDF), evaluado en
              (q_meas, dq, ddq).  Pinocchio NO incluye fricción, por lo que los
              términos Fv/Fc/I_offset absorben exactamente lo que RNEA omite.
    I_meas  = corriente medida del registro PRESENT_CURRENT [ticks, 2.69 mA/tick],
              llevada a convención articular: y = current_sign·encoder_sign·curr_meas.

Entrada : data/diagnostics/sinusoidal/hw_sin_torque_<LOG_ID>.csv
          (generado por hw_sinusoidal_torque_node en modo "position")
Salidas :
    1) Consola : alpha, Fv, Fc, I_offset y R² por articulación.
    2) YAML    : config/motorXM430W350T_params_posmode.yaml  (referencia modo posición;
                 NO es el modelo final — ese se ensambla a mano combinando alpha de modo
                 corriente + Fv/Fc de identify_friction.py en motorXM430W350T_params.yaml)
    3) CSV     : data/identification/identify_fit_<LOG_ID>.csv
                 (t, tau_rb, I_meas, I_pred por articulación)

Requisitos: pinocchio, numpy, scipy.
Uso       : python3 src/Identification/identify_motor_model.py
"""

import os
import csv
import datetime
import numpy as np
from scipy.signal import savgol_filter
from scipy.optimize import lsq_linear
import pinocchio as pin

# ════════════════════════════════════════════════════════════════════════════
#  Configuración
# ════════════════════════════════════════════════════════════════════════════

PKG_DIR = "/home/utec/open_manx_ws/src/open_manipulator_x_torque_control"

LOG_ID  = 11          # hw_sin_torque_<LOG_ID>.csv (ensayo con buen tracking)

# Fuente de velocidad y aceleración para construir los regresores:
#   VEL_SOURCE : 'sg'   → derivada Savitzky-Golay de la posición medida (recomendado)
#                'meas' → registro PRESENT_VELOCITY (cuantizado a 0.024 rad/s)
#                'ref'  → derivada analítica de la trayectoria (asume tracking perfecto)
#   ACC_SOURCE : 'ref'  → ddq analítico de la trayectoria (validado vs SG)
#                'sg'   → 2da derivada Savitzky-Golay de la posición medida
VEL_SOURCE = 'sg'
ACC_SOURCE = 'ref'

# Savitzky-Golay (para VEL_SOURCE/ACC_SOURCE = 'sg')
SG_WINDOW = 31        # impar; 31 @ 100 Hz ≈ 0.31 s  (<< periodo trayectoria)
SG_ORDER  = 3

# Estructura del modelo
#   SHARED_ALPHA    : una sola alpha para los 4 joints. Físicamente correcto —
#                     los 4 son el mismo motor XM430-W350 (alpha = 1/(kt·Iu) es
#                     constante del motor). Robustece joints sin gravedad (J1) o
#                     poco excitados (J3), cuya alpha individual no es identificable.
#   NONNEG_FRICTION : restringe alpha>=0, Fv>=0, Fc>=0 (lsq_linear con bounds).
#                     Evita los valores no físicos (Fv<0) que produce la colinealidad
#                     Fv·dq ↔ Fc·tanh(dq/eps) con velocidad sinusoidal suave.
SHARED_ALPHA    = True
NONNEG_FRICTION = True

# Suavizado del tanh en la fricción de Coulomb
GRID_EPSILON = False  # True: busca eps que maximiza el R² medio (tiende a sobreajustar)
EPSILON      = 0.05   # [rad/s] valor fijo si GRID_EPSILON = False
EPS_GRID     = np.arange(0.02, 0.1001, 0.005)   # rejilla de búsqueda

# Convención de signos por joint (debe coincidir con el nodo que generó los datos)
ENCODER_SIGN = np.array([1.0, 1.0, 1.0, 1.0])
CURRENT_SIGN = np.array([1.0, 1.0, 1.0, 1.0])

# Ventana de análisis: se descartan settling + transitorio inicial
SKIP_AFTER_START = 1.0   # [s] a descartar tras el inicio del control (paso de velocidad)

# Unidad de corriente Dynamixel (para reportar kt efectivo)
CURRENT_UNIT_A = 2.69e-3  # [A/tick]

URDF_PATH = os.path.join(PKG_DIR, "urdf", "open_manipulator_x.urdf")
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4"]
NJ = 4

# ════════════════════════════════════════════════════════════════════════════
#  Carga de datos
# ════════════════════════════════════════════════════════════════════════════

csv_in = os.path.join(PKG_DIR, "data", "diagnostics", "sinusoidal",
                      f"hw_sin_torque_{LOG_ID}.csv")
if not os.path.isfile(csv_in):
    raise FileNotFoundError(f"No se encontró el CSV de entrada:\n  {csv_in}")

raw = np.genfromtxt(csv_in, delimiter=",", names=True)
t        = raw["t"] - raw["t"][0]
q        = np.column_stack([raw[f"q{i}"]        for i in range(1, NJ + 1)])
dq_meas  = np.column_stack([raw[f"dq{i}"]       for i in range(1, NJ + 1)])
curr     = np.column_stack([raw[f"curr_meas{i}"] for i in range(1, NJ + 1)])

has_ref = all(f"dq_ref{i}" in raw.dtype.names for i in range(1, NJ + 1))
if has_ref:
    dq_ref  = np.column_stack([raw[f"dq_ref{i}"]  for i in range(1, NJ + 1)])
    ddq_ref = np.column_stack([raw[f"ddq_ref{i}"] for i in range(1, NJ + 1)])
else:
    dq_ref = ddq_ref = None
    if VEL_SOURCE == 'ref' or ACC_SOURCE == 'ref':
        raise RuntimeError("El CSV no tiene dq_ref/ddq_ref; use VEL/ACC_SOURCE='sg'.")

Ts = float(np.mean(np.diff(t)))
N  = len(t)
print(f"Cargado: {csv_in}")
print(f"  {N} muestras | Ts = {Ts*1000:.2f} ms | fs = {1/Ts:.1f} Hz")

# ── Ventana de análisis (descartar settling + transitorio) ───────────────────
if ddq_ref is not None:
    active = np.where(np.any(np.abs(ddq_ref) > 1e-9, axis=1))[0]
    t_start = t[active[0]] if active.size else 0.0
else:
    t_start = 0.0
mask = t >= (t_start + SKIP_AFTER_START)
n_use = int(np.sum(mask))
print(f"  Control inicia ~{t_start:.2f}s → ventana de análisis: "
      f"{n_use} muestras (t ≥ {t_start + SKIP_AFTER_START:.2f}s)")

# ── Estimación de dq y ddq según la fuente elegida ───────────────────────────
def sg(col, deriv):
    return savgol_filter(col, SG_WINDOW, SG_ORDER, deriv=deriv, delta=Ts)

if VEL_SOURCE == 'sg':
    dq = np.column_stack([sg(q[:, i], 1) for i in range(NJ)])
elif VEL_SOURCE == 'meas':
    dq = dq_meas
elif VEL_SOURCE == 'ref':
    dq = dq_ref
else:
    raise ValueError("VEL_SOURCE inválido")

if ACC_SOURCE == 'ref':
    ddq = ddq_ref
elif ACC_SOURCE == 'sg':
    ddq = np.column_stack([sg(q[:, i], 2) for i in range(NJ)])
else:
    raise ValueError("ACC_SOURCE inválido")

print(f"  Fuente velocidad = '{VEL_SOURCE}'   aceleración = '{ACC_SOURCE}'")

# ════════════════════════════════════════════════════════════════════════════
#  Modelo dinámico (Pinocchio): RNEA → tau de cuerpo rígido
# ════════════════════════════════════════════════════════════════════════════

model_full = pin.buildModelFromUrdf(URDF_PATH)
lock_ids = [jid for jid in range(1, model_full.njoints)
            if model_full.names[jid] not in ARM_JOINTS]
model = pin.buildReducedModel(model_full, lock_ids, pin.neutral(model_full))
data  = model.createData()

if model.nq != NJ:
    raise RuntimeError(f"Modelo reducido con nq={model.nq}, se esperaban {NJ} GDL")
joint_order = [model.names[j] for j in range(1, model.njoints)]
print(f"Modelo Pinocchio (reducido): {model.nq} GDL  orden={joint_order}")
if joint_order != ARM_JOINTS:
    print(f"  ADVERTENCIA: el orden de joints no es {ARM_JOINTS}")

tau_rb = np.zeros((N, NJ))
for k in range(N):
    tau_rb[k, :] = pin.rnea(model, data,
                            q[k, :].copy(), dq[k, :].copy(), ddq[k, :].copy())

# ════════════════════════════════════════════════════════════════════════════
#  Regresión por articulación  (alpha compartida / por-joint, con/sin bounds)
# ════════════════════════════════════════════════════════════════════════════

# Corriente medida en convención articular
y_all = (CURRENT_SIGN * ENCODER_SIGN) * curr   # [N×4], en ticks

def build_and_solve(eps):
    """Resuelve el modelo apilando los 4 joints en un solo sistema.
       alpha compartida (SHARED_ALPHA) o una por joint; bounds opcionales.
       Devuelve alpha[NJ], Fv[NJ], Fc[NJ], Ioff[NJ]."""
    m = mask
    n = int(np.sum(m))
    if SHARED_ALPHA:
        P     = 1 + 3 * NJ
        a_col = lambda i: 0
        f0    = lambda i: 1 + 3 * i
    else:
        P     = 4 * NJ
        a_col = lambda i: 4 * i
        f0    = lambda i: 4 * i + 1
    A = np.zeros((n * NJ, P))
    b = np.zeros(n * NJ)
    for i in range(NJ):
        r = slice(i * n, (i + 1) * n)
        A[r, a_col(i)] += tau_rb[m, i]               # alpha
        A[r, f0(i) + 0] = dq[m, i]                   # Fv
        A[r, f0(i) + 1] = np.tanh(dq[m, i] / eps)    # Fc
        A[r, f0(i) + 2] = 1.0                         # I_offset
        b[r]            = y_all[m, i]
    lo = np.full(P, -np.inf)
    hi = np.full(P,  np.inf)
    if NONNEG_FRICTION:
        for i in range(NJ):
            lo[a_col(i)]  = 0.0      # alpha >= 0
            lo[f0(i) + 0] = 0.0      # Fv    >= 0
            lo[f0(i) + 1] = 0.0      # Fc    >= 0
    x = lsq_linear(A, b, bounds=(lo, hi)).x
    alpha = np.array([x[a_col(i)]     for i in range(NJ)])
    Fv    = np.array([x[f0(i) + 0]    for i in range(NJ)])
    Fc    = np.array([x[f0(i) + 1]    for i in range(NJ)])
    Ioff  = np.array([x[f0(i) + 2]    for i in range(NJ)])
    return alpha, Fv, Fc, Ioff

def r2_joint(i, alpha_i, Fv_i, Fc_i, Io_i, eps):
    m = mask
    pred = (alpha_i * tau_rb[m, i] + Fv_i * dq[m, i]
            + Fc_i * np.tanh(dq[m, i] / eps) + Io_i)
    y = y_all[m, i]
    ss_res = float(((y - pred) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

# ── Selección de epsilon ─────────────────────────────────────────────────────
if GRID_EPSILON:
    best_eps, best_mean = EPSILON, -np.inf
    for e in EPS_GRID:
        al, fv, fc, io = build_and_solve(e)
        mr2 = np.mean([r2_joint(i, al[i], fv[i], fc[i], io[i], e) for i in range(NJ)])
        if mr2 > best_mean:
            best_mean, best_eps = mr2, e
    eps = float(best_eps)
    print(f"epsilon óptimo (rejilla): {eps:.3f} rad/s  (R² medio = {best_mean:.4f})")
else:
    eps = EPSILON
    print(f"epsilon fijo: {eps:.3f} rad/s")

print(f"Modelo: alpha {'COMPARTIDA' if SHARED_ALPHA else 'por-joint'}"
      f" | fricción {'Fv,Fc>=0' if NONNEG_FRICTION else 'sin restricción'}")

# ── Ajuste final ─────────────────────────────────────────────────────────────
alpha, Fv, Fc, Ioff = build_and_solve(eps)
R2     = np.array([r2_joint(i, alpha[i], Fv[i], Fc[i], Ioff[i], eps) for i in range(NJ)])
I_pred = np.full((N, NJ), np.nan)
for i in range(NJ):
    I_pred[mask, i] = (alpha[i] * tau_rb[mask, i] + Fv[i] * dq[mask, i]
                       + Fc[i] * np.tanh(dq[mask, i] / eps) + Ioff[i])

# ── Diagnóstico de identificabilidad por joint ───────────────────────────────
corr_tau_I = np.zeros(NJ)
condW      = np.zeros(NJ)
for i in range(NJ):
    a = tau_rb[mask, i]; c = y_all[mask, i]
    corr_tau_I[i] = np.corrcoef(a, c)[0, 1] if np.std(a) > 1e-12 else 0.0
    W = np.column_stack([a, dq[mask, i], np.tanh(dq[mask, i] / eps),
                         np.ones(int(np.sum(mask)))])
    condW[i] = float(np.linalg.cond(W))

# ════════════════════════════════════════════════════════════════════════════
#  Reporte en consola
# ════════════════════════════════════════════════════════════════════════════

print("\n" + "=" * 84)
print(f" Identificación torque→corriente  [log {LOG_ID}]")
print("=" * 84)
print(f"{'Joint':>6} {'alpha':>10} {'Fv':>9} {'Fc':>9} {'I_offset':>10} "
      f"{'R²':>8} {'corr(τ,I)':>10} {'cond(W)':>9}")
print("-" * 84)
for i in range(NJ):
    flags = []
    if abs(corr_tau_I[i]) < 0.30: flags.append("α débil")
    if alpha[i] <= 0:             flags.append("α≤0")
    flag = ("  <-- " + ", ".join(flags)) if flags else ""
    print(f"J{i+1:>5} {alpha[i]:10.3f} {Fv[i]:9.3f} {Fc[i]:9.3f} {Ioff[i]:10.3f} "
          f"{R2[i]:8.4f} {corr_tau_I[i]:10.3f} {condW[i]:9.1f}{flag}")
print("-" * 84)
kt_ef = 1.0 / (np.mean(alpha) * CURRENT_UNIT_A) if np.mean(alpha) > 0 else float("nan")
print(f"  Unidades: alpha [ticks/N·m]  Fv [ticks/(rad/s)]  Fc, I_offset [ticks]")
print(f"  epsilon = {eps:.3f} rad/s   R² medio = {np.mean(R2):.4f}   "
      f"kt_efectivo = {kt_ef:.3f} N·m/A")
print(f"  Nota: 'α débil' = |corr(τ,I)|<0.3 → torque poco excitado; su α se apoya "
      f"en la compartida.")
print("=" * 84)

# ════════════════════════════════════════════════════════════════════════════
#  Salida 1: YAML  config/motorXM430W350T_params_posmode.yaml  (referencia)
#  NO es el modelo final: el modo posición sesga alpha por el PID interno.
#  El modelo final (motorXM430W350T_params.yaml) se ensambla a mano con alpha de
#  modo corriente + Fv/Fc de identify_friction.py.
# ════════════════════════════════════════════════════════════════════════════

yaml_out = os.path.join(PKG_DIR, "config", "motorXM430W350T_params_posmode.yaml")
fmt = lambda v: "[" + ", ".join(f"{x:.4f}" for x in v) + "]"
today = datetime.date.today().isoformat()

with open(yaml_out, "w") as f:
    f.write("/**:\n")
    f.write("  ros__parameters:\n\n")
    method = ("alpha compartida" if SHARED_ALPHA else "alpha por-joint") + \
             (" + Fv,Fc>=0" if NONNEG_FRICTION else "")
    f.write("    # ── Modelo torque→corriente Dynamixel XM430-W350-T (OpenMANIPULATOR-X) ──\n")
    f.write(f"    # identify_motor_model.py — {today}  ({method})\n")
    f.write(f"    # Fuente: data/diagnostics/sinusoidal/hw_sin_torque_{LOG_ID}.csv\n")
    f.write(f"    # tau por RNEA (Pinocchio + open_manipulator_x.urdf); "
            f"vel='{VEL_SOURCE}' acc='{ACC_SOURCE}'\n")
    f.write("    # Modelo: I[i] = alpha[i]·tau[i] + Fv[i]·dq[i] + Fc[i]·tanh(dq[i]/eps) + I_offset[i]\n")
    f.write(f"    # R² por joint: " + ", ".join(f"J{i+1}={R2[i]:.3f}" for i in range(NJ)) + "\n")
    f.write(f"    # corr(tau,I): " + ", ".join(f"J{i+1}={corr_tau_I[i]:.2f}" for i in range(NJ))
            + "   (|corr|<0.3 → alpha poco fiable, se apoya en la compartida)\n")
    f.write("    #\n")
    f.write("    # Unidades:\n")
    f.write("    #   motor_alpha            [ticks/N·m]\n")
    f.write("    #   motor_Fv               [ticks/(rad/s)]\n")
    f.write("    #   motor_Fc               [ticks]\n")
    f.write("    #   motor_I_offset         [ticks]\n")
    f.write("    #   motor_epsilon_friction [rad/s]\n\n")
    f.write(f"    motor_alpha:             {fmt(alpha)}    # J1 J2 J3 J4\n")
    f.write(f"    motor_Fv:                {fmt(Fv)}\n")
    f.write(f"    motor_Fc:                {fmt(Fc)}\n")
    f.write(f"    motor_I_offset:          {fmt(Ioff)}\n")
    f.write(f"    motor_epsilon_friction:   {eps:.4f}\n")

print(f"\nYAML guardado : {yaml_out}")

# ════════════════════════════════════════════════════════════════════════════
#  Salida 2: CSV de ajuste  data/identification/identify_fit_<LOG_ID>.csv
# ════════════════════════════════════════════════════════════════════════════

fit_out = os.path.join(PKG_DIR, "data", "identification",
                       f"identify_fit_{LOG_ID}.csv")
os.makedirs(os.path.dirname(fit_out), exist_ok=True)

header = (["t"]
          + [f"tau_rb{i+1}" for i in range(NJ)]
          + [f"I_meas{i+1}" for i in range(NJ)]
          + [f"I_pred{i+1}" for i in range(NJ)])
with open(fit_out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(header)
    for k in range(N):
        if not mask[k]:
            continue
        row = ([f"{t[k]:.6f}"]
               + [f"{tau_rb[k, i]:.6f}" for i in range(NJ)]
               + [f"{y_all[k, i]:.6f}"  for i in range(NJ)]
               + [f"{I_pred[k, i]:.6f}" for i in range(NJ)])
        w.writerow(row)

print(f"CSV ajuste    : {fit_out}  ({n_use} filas)")
