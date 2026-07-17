#!/usr/bin/env python3
"""
identify_alpha.py
Identificación de alpha (torque→corriente) e I_offset de los 4 motores
Dynamixel XM430-W350-T del OpenMANIPULATOR-X — MÉTODO VALIDADO: modo
CORRIENTE dinámico (lazo cerrado de torque computado / FL).

Modelo por articulación (el mismo que usan los nodos hw_*):

    I_meas[i] = alpha[i]·tau[i] + Fv[i]·dq[i] + Fc[i]·tanh(dq[i]/eps) + I_offset[i]

donde tau es el torque COMANDADO por el controlador FL (columna tau del CSV)
e I_meas la corriente medida (PRESENT_CURRENT). En lazo cerrado con buen
tracking, tau comandado ≈ torque realmente entregado, y la regresión mide
cuántos ticks de corriente cuesta cada N·m en régimen dinámico.

REQUISITO CRÍTICO — parámetros BOOTSTRAP durante el ensayo:
El ensayo debe correrse con la conversión NOMINAL y SIN compensación:
    motor_alpha:    [208.5, 208.5, 208.5, 208.5]
    motor_Fv/Fc/I_offset: [0, 0, 0, 0]
Si el nodo ya usa un alpha identificado y compensación de fricción, la
regresión queda ANCLADA a ese modelo (circularidad) y solo devuelve una
versión filtrada del mismo. Este script DETECTA esa condición desde el
propio log (ajustando curr_cmd vs tau) y lo advierte.

POR QUÉ ESTE MÉTODO (y no el modo posición ni el régimen cuasi-estático):
    alpha = 224.8 → kt_ef = 1.65 N·m/A   ✓ (< 1.78 ideal del datasheet;
                                            la diferencia son pérdidas del reductor)
    alpha = 153   → kt_ef = 2.43 N·m/A   ✗ IMPOSIBLE (más torque por amperio
                                            que el motor ideal)
En una articulación con reductor 353.5:1, parte de la carga cuasi-estática
(sostener gravedad casi sin moverse) la soporta la fricción del engranaje /
no-retroconducción, NO la corriente del motor → alpha sale subestimada
(kt inflada e imposible). El PID interno del modo posición añade otro sesgo.
Este script VERIFICA la ventana física de kt y avisa si los datos provienen
de un régimen inválido.

Estrategia por joint (la misma validada en la identificación original):
    · joints con torque bien excitado (std(tau) y corr(tau,I) suficientes,
      típicamente J2/J3 con la trayectoria act1) → OLS completo de 4 parámetros.
    · joints poco excitados (J1 sin gravedad, J4 casi estático) → alpha fija
      = media de los identificables (mismo motor XM430 → mismo kt) y OLS
      parcial de [Fv, Fc, I_offset].

Entrada (autodetectada por columnas):
    data/lab4/real/act1/hw_fl_data_<LOG_ID>.csv        (hw_fl_control_node) ← recomendado
    data/diagnostics/sinusoidal/hw_sin_torque_<LOG_ID>.csv  (joints en mode="current")

Salidas:
    1) Consola : alpha, Fv, Fc, I_offset, R², diagnóstico y VEREDICTO
                 (OK / REVISAR / RE-EJECUTAR con instrucciones concretas).
    2) YAML    : data/identification/alpha_fit_log<LOG_ID>.yaml   (fragmento
                 para assemble_motor_params.py — alpha e I_offset; los Fv/Fc
                 de este ajuste son solo informativos, los oficiales salen de
                 identify_friction.py).
    3) CSV     : data/identification/alpha_fit_log<LOG_ID>.csv  (t, tau,
                 I_meas, I_pred por articulación, para graficar el ajuste).

Código de salida: 0 = OK/REVISAR, 2 = RE-EJECUTAR (datos no aptos para alpha).

Requisitos: numpy, scipy (no requiere Pinocchio).
Uso:
    python3 src/Identification/identify_alpha.py 30
    python3 src/Identification/identify_alpha.py --csv /ruta/al/log.csv
Protocolo completo: src/Identification/Ident_OpenManX_XM430W350T_procedure.md
"""

import os
import sys
import csv
import argparse
import datetime
import numpy as np
from scipy.optimize import lsq_linear

# ════════════════════════════════════════════════════════════════════════════
#  Configuración
# ════════════════════════════════════════════════════════════════════════════

# Raíz del paquete: dos niveles arriba de src/Identification/
PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
NJ = 4

# Convención de signos (debe coincidir con el nodo que generó los datos)
ENCODER_SIGN = np.array([1.0, 1.0, 1.0, 1.0])
CURRENT_SIGN = np.array([1.0, 1.0, 1.0, 1.0])

# Constantes físicas del XM430-W350
CURRENT_UNIT_A = 2.69e-3          # [A/tick]
KT_IDEAL       = 1.7826           # [N·m/A] constante de torque ideal (datasheet)
ALPHA_NOM      = 1.0 / (KT_IDEAL * CURRENT_UNIT_A)   # ≈ 208.5 ticks/N·m

# Ventana física aceptable para alpha:
#   alpha < ALPHA_NOM  → kt_ef > kt ideal → IMPOSIBLE (régimen cuasi-estático /
#                        modo posición: la fricción del reductor sostiene carga)
#   alpha > ALPHA_MAX  → kt_ef < ~1.3 → pérdidas >27%: sospechoso (signos,
#                        temperatura, colinealidad)
ALPHA_MIN = ALPHA_NOM * 0.99      # 1% de tolerancia numérica
ALPHA_MAX = 1.0 / (1.30 * CURRENT_UNIT_A)            # ≈ 286 ticks/N·m

# Identificabilidad por joint. Con la trayectoria act1 pasan J2 y J3;
# J1 (sin gravedad) y J4 (casi estático) usan alpha fija — mismo criterio
# que la identificación original.
TAU_STD_MIN  = 0.05               # [N·m] excitación mínima del torque comandado
CORR_MIN     = 0.30               # |corr(tau, I_meas)| mínima

# Detección de circularidad: alpha/compensación que usó EL NODO durante el
# ensayo, estimadas del propio log (curr_cmd ≈ alpha_usada·tau + comp(dq)).
BOOT_ALPHA_TOL = 0.04             # ±4% alrededor de ALPHA_NOM
BOOT_COMP_TOL  = 2.0              # [ticks] Fv/Fc/Ioff usados ≈ 0

# Saturación de corriente comandada: muestras saturadas se descartan del
# ajuste (la corriente recortada rompe la linealidad I↔tau)
CURRENT_CMD_LIMIT = 257           # [ticks]
SAT_FRAC_WARN     = 0.02          # >2% saturado → advertencia

R2_WARN  = 0.85
R2_FAIL  = 0.60
IOFF_WARN = 30.0                  # [ticks]

# ════════════════════════════════════════════════════════════════════════════
#  CLI y carga de datos
# ════════════════════════════════════════════════════════════════════════════

ap = argparse.ArgumentParser(description="Identificación de alpha e I_offset "
                             "(modo corriente dinámico).")
ap.add_argument("log_id", nargs="?", type=int, default=None,
                help="ID del log (hw_fl_data_<id>.csv o hw_sin_torque_<id>.csv)")
ap.add_argument("--csv",  default=None, help="ruta explícita al CSV de entrada")
ap.add_argument("--skip", type=float, default=4.0,
                help="segundos a descartar al inicio (rampa/transitorio; def. 4.0)")
ap.add_argument("--eps",  type=float, default=0.05,
                help="suavizado del tanh de Coulomb [rad/s] (def. 0.05)")
args = ap.parse_args()

if args.csv is None and args.log_id is None:
    ap.error("indique LOG_ID o --csv")

if args.csv is not None:
    csv_in = args.csv
else:
    cand_fl  = os.path.join(PKG_DIR, "data", "lab4", "real", "act1",
                            f"hw_fl_data_{args.log_id}.csv")
    cand_sin = os.path.join(PKG_DIR, "data", "diagnostics", "sinusoidal",
                            f"hw_sin_torque_{args.log_id}.csv")
    if os.path.isfile(cand_fl):
        csv_in = cand_fl
    elif os.path.isfile(cand_sin):
        csv_in = cand_sin
    else:
        raise FileNotFoundError(
            "No se encontró el log en ninguna de las rutas esperadas:\n"
            f"  {cand_fl}\n  {cand_sin}")

raw = np.genfromtxt(csv_in, delimiter=",", names=True)
cols = raw.dtype.names
t = raw["t"] - raw["t"][0]
N = len(t)
Ts = float(np.mean(np.diff(t)))

# Detección de formato por columnas: FL usa tau{i}; el nodo sinusoidal, tau_ref{i}
if all(f"tau{i}" in cols for i in range(1, NJ + 1)):
    src_kind, tau_col = "fl", "tau{}"
elif all(f"tau_ref{i}" in cols for i in range(1, NJ + 1)):
    src_kind, tau_col = "sin", "tau_ref{}"
else:
    raise RuntimeError("El CSV no tiene columnas tau1..4 ni tau_ref1..4 — "
                       "¿es un log de control en modo corriente?")

dq_all   = np.column_stack([raw[f"dq{i}"]        for i in range(1, NJ + 1)])
tau_all  = np.column_stack([raw[tau_col.format(i)] for i in range(1, NJ + 1)])
curr     = np.column_stack([raw[f"curr_meas{i}"] for i in range(1, NJ + 1)])
has_cmd  = all(f"curr_cmd{i}" in cols for i in range(1, NJ + 1))
curr_cmd = (np.column_stack([raw[f"curr_cmd{i}"] for i in range(1, NJ + 1)])
            if has_cmd else None)

print(f"Cargado : {csv_in}")
print(f"  {N} muestras | Ts = {Ts*1000:.2f} ms | fs = {1/Ts:.1f} Hz | formato = {src_kind}")

# ── Verificación de régimen: debe haber torque comandado en modo corriente ──
if not (np.abs(tau_all).max(axis=0) > 1e-6).any():
    print("\n" + "!" * 78)
    print(" ERROR: ningún joint tiene torque comandado (tau ≈ 0 en todo el log).")
    print(" Estos datos provienen de un ensayo SIN modo corriente (p.ej. solo modo")
    print(" posición/velocidad). El alpha ajustado sobre datos así queda sesgado por")
    print(" el PID interno y el régimen cuasi-estático → kt imposible (>1.78 N·m/A).")
    print(" → Acción: ejecute el nodo hw_fl_control_node (lazo de torque computado)")
    print("   y analice ese log. Ver Ident_OpenManX_XM430W350T_procedure.md, Paso 2.")
    print("!" * 78)
    sys.exit(2)

# ── Ventana de análisis (descartar rampa/transitorio inicial) ────────────────
act = np.where(np.any(np.abs(tau_all) > 1e-9, axis=1))[0]
t0  = t[act[0]] if act.size else 0.0
mask = t >= (t0 + args.skip)
if int(mask.sum()) < 200:
    print(f"\nERROR: solo {int(mask.sum())} muestras tras descartar {args.skip:.1f} s — "
          "log demasiado corto.")
    print("→ Acción: repita el ensayo con mayor duración (t_imp ≥ 20 s).")
    sys.exit(2)
print(f"  Ventana de análisis: t ≥ {t0 + args.skip:.2f} s → {int(mask.sum())} muestras")

# Corriente medida en convención articular
y_all = (CURRENT_SIGN * ENCODER_SIGN) * curr

eps = args.eps

# ════════════════════════════════════════════════════════════════════════════
#  Detección de circularidad: modelo que usó el nodo durante el ensayo
#  curr_cmd ≈ alpha_usada·tau + Fv_u·dq + Fc_u·tanh + Ioff_u  (exacto salvo
#  redondeo si la compensación estaba apagada). El ensayo VÁLIDO usa
#  parámetros bootstrap: alpha ≈ 208.5 y compensación nula.
# ════════════════════════════════════════════════════════════════════════════

alpha_used = np.full(NJ, np.nan)
comp_used  = np.zeros(NJ)     # max(|Fv_u|, |Fc_u|, |Ioff_u|) por joint
if curr_cmd is not None:
    for i in range(NJ):
        m = mask
        A = np.column_stack([tau_all[m, i], dq_all[m, i],
                             np.tanh(dq_all[m, i] / eps), np.ones(int(m.sum()))])
        if np.std(tau_all[m, i]) < 1e-4:
            continue
        xu, *_ = np.linalg.lstsq(A, curr_cmd[m, i], rcond=None)
        alpha_used[i] = xu[0]
        comp_used[i]  = float(np.max(np.abs(xu[1:])))

bootstrap_ok = True
if curr_cmd is not None and np.isfinite(alpha_used).any():
    a_med = float(np.nanmedian(alpha_used))
    c_max = float(np.nanmax(comp_used))
    if abs(a_med - ALPHA_NOM) > BOOT_ALPHA_TOL * ALPHA_NOM or c_max > BOOT_COMP_TOL:
        bootstrap_ok = False

# ════════════════════════════════════════════════════════════════════════════
#  Regresión por articulación con fallback de alpha compartida
# ════════════════════════════════════════════════════════════════════════════

def fit_joint(i, m, alpha_fixed=None):
    """OLS del joint i sobre la máscara m (alpha, Fv, Fc >= 0). Si alpha_fixed
       se da, ajusta solo [Fv, Fc, Ioff] sobre y - alpha_fixed·tau."""
    tanh_i = np.tanh(dq_all[m, i] / eps)
    if alpha_fixed is None:
        A = np.column_stack([tau_all[m, i], dq_all[m, i], tanh_i,
                             np.ones(int(m.sum()))])
        x = lsq_linear(A, y_all[m, i],
                       bounds=([0.0, 0.0, 0.0, -np.inf], [np.inf] * 4)).x
        return x[0], x[1], x[2], x[3]
    A = np.column_stack([dq_all[m, i], tanh_i, np.ones(int(m.sum()))])
    b = y_all[m, i] - alpha_fixed * tau_all[m, i]
    x = lsq_linear(A, b, bounds=([0.0, 0.0, -np.inf], [np.inf] * 3)).x
    return alpha_fixed, x[0], x[1], x[2]

# Máscara por joint: ventana global menos muestras con corriente saturada
mask_j, sat_frac = [], np.zeros(NJ)
for i in range(NJ):
    m = mask.copy()
    if curr_cmd is not None:
        sat = np.abs(curr_cmd[:, i]) >= (CURRENT_CMD_LIMIT - 0.5)
        sat_frac[i] = float(sat[mask].mean())
        m &= ~sat
    mask_j.append(m)

# Diagnóstico de identificabilidad
tau_std, corr_tau_I = np.zeros(NJ), np.zeros(NJ)
for i in range(NJ):
    m = mask_j[i]
    a, c = tau_all[m, i], y_all[m, i]
    tau_std[i] = float(np.std(a))
    corr_tau_I[i] = float(np.corrcoef(a, c)[0, 1]) if np.std(a) > 1e-12 else 0.0

strong = [i for i in range(NJ)
          if tau_std[i] >= TAU_STD_MIN and abs(corr_tau_I[i]) >= CORR_MIN]
weak   = [i for i in range(NJ) if i not in strong]

alpha = np.zeros(NJ); Fv = np.zeros(NJ); Fc = np.zeros(NJ); Ioff = np.zeros(NJ)
for i in strong:
    alpha[i], Fv[i], Fc[i], Ioff[i] = fit_joint(i, mask_j[i])

alpha_shared = float(np.mean(alpha[strong])) if strong else float("nan")

for i in weak:
    if strong:
        alpha[i], Fv[i], Fc[i], Ioff[i] = fit_joint(i, mask_j[i],
                                                    alpha_fixed=alpha_shared)

# R², residuo RMS e I_pred por joint
R2 = np.zeros(NJ)
resid_rms = np.zeros(NJ)
I_pred = np.full((N, NJ), np.nan)
for i in range(NJ):
    m = mask_j[i]
    pred = (alpha[i] * tau_all[m, i] + Fv[i] * dq_all[m, i]
            + Fc[i] * np.tanh(dq_all[m, i] / eps) + Ioff[i])
    yy = y_all[m, i]
    ss_tot = float(((yy - yy.mean()) ** 2).sum())
    R2[i] = 1.0 - float(((yy - pred) ** 2).sum()) / ss_tot if ss_tot > 0 else float("nan")
    resid_rms[i] = float(np.sqrt(np.mean((yy - pred) ** 2)))
    I_pred[m, i] = pred

# ════════════════════════════════════════════════════════════════════════════
#  Reporte
# ════════════════════════════════════════════════════════════════════════════

def kt_of(a):
    return 1.0 / (a * CURRENT_UNIT_A) if a > 0 else float("nan")

print("\n" + "=" * 88)
print(f" Identificación alpha / I_offset — modo corriente dinámico  [{os.path.basename(csv_in)}]")
print("=" * 88)
print(f"{'Joint':>6} {'alpha':>9} {'kt_ef':>7} {'Fv*':>8} {'Fc*':>8} {'I_offset':>9} "
      f"{'R²':>7} {'std(τ)':>8} {'corr':>6} {'sat%':>5}  fuente")
print("-" * 88)
for i in range(NJ):
    fuente = "OLS completo" if i in strong else "α compartida"
    print(f"J{i+1:>5} {alpha[i]:9.2f} {kt_of(alpha[i]):7.3f} {Fv[i]:8.3f} {Fc[i]:8.3f} "
          f"{Ioff[i]:9.3f} {R2[i]:7.4f} {tau_std[i]:8.3f} {corr_tau_I[i]:6.2f} "
          f"{100*sat_frac[i]:5.1f}  {fuente}")
print("-" * 88)
print(f"  alpha compartida = {alpha_shared:.2f} ticks/N·m → kt_efectivo = "
      f"{kt_of(alpha_shared):.3f} N·m/A  (ideal datasheet: {KT_IDEAL:.3f})")
print(f"  * Fv/Fc de ESTE ajuste son informativos: los oficiales salen del barrido")
print(f"    de velocidad (identify_friction.py). Unidades: alpha [ticks/N·m],")
print(f"    Fv [ticks/(rad/s)], Fc/I_offset [ticks]. eps = {eps:.3f} rad/s")
if curr_cmd is not None and np.isfinite(alpha_used).any():
    au = ", ".join(f"J{i+1}={alpha_used[i]:.1f}" for i in range(NJ)
                   if np.isfinite(alpha_used[i]))
    print(f"  Modelo usado por el nodo durante el ensayo (de curr_cmd): {au}"
          f"   comp_max = {np.nanmax(comp_used):.1f} ticks")

# ── Veredicto ────────────────────────────────────────────────────────────────
fails, warns = [], []

if not strong:
    fails.append(
        "Ningún joint con excitación suficiente (std(τ) ≥ %.2f N·m y |corr| ≥ %.2f).\n"
        "   → Acción: use una trayectoria dinámica (hw_fl_control_node, Paso 2 del\n"
        "     protocolo) con t_imp ≥ 20 s; verifique que el robot realmente se movió."
        % (TAU_STD_MIN, CORR_MIN))
else:
    if alpha_shared < ALPHA_MIN:
        fails.append(
            f"alpha = {alpha_shared:.1f} < {ALPHA_NOM:.1f} ticks/N·m → kt_ef = "
            f"{kt_of(alpha_shared):.2f} N·m/A > {KT_IDEAL:.2f} ideal: FÍSICAMENTE IMPOSIBLE.\n"
            "   Causa típica: datos cuasi-estáticos o de modo posición (la fricción del\n"
            "   reductor sostiene la gravedad y la corriente queda por debajo de α·τ).\n"
            "   → Acción: RE-EJECUTE el ensayo en modo corriente dinámico\n"
            "     (hw_fl_control_node con la trayectoria act1) y analice ese log.")
    if alpha_shared > ALPHA_MAX:
        warns.append(
            f"alpha = {alpha_shared:.1f} > {ALPHA_MAX:.0f} ticks/N·m → kt_ef = "
            f"{kt_of(alpha_shared):.2f} N·m/A (<1.30): pérdidas anómalas.\n"
            "   → Revise CURRENT_SIGN/ENCODER_SIGN, temperatura de motores y masas del URDF.")
    spread = (np.max(alpha[strong]) - np.min(alpha[strong])) if len(strong) > 1 else 0.0
    if alpha_shared > 0 and spread > 0.15 * alpha_shared:
        warns.append(
            f"Dispersión de alpha entre joints identificables = {spread:.1f} ticks/N·m "
            "(>15% de la media).\n"
            "   → Revise las masas del URDF o repita con una trayectoria más rica.")

if not bootstrap_ok:
    warns.append(
        "El ensayo NO se corrió con parámetros bootstrap: el nodo ya usaba un alpha\n"
        f"   identificado y/o compensación de fricción (mediana α_usada = "
        f"{np.nanmedian(alpha_used):.1f}, comp_max = {np.nanmax(comp_used):.1f} ticks).\n"
        "   El alpha re-identificado queda ANCLADO a ese modelo (circularidad): sirve\n"
        "   como verificación de consistencia, NO como identificación desde cero.\n"
        "   → Para identificar de verdad: ponga en config/motorXM430W350T_params.yaml\n"
        "     motor_alpha: [208.5,208.5,208.5,208.5] y Fv/Fc/I_offset en 0, repita el\n"
        "     ensayo y vuelva a analizar (Paso 2 del protocolo).")

if float(np.nanmean(R2)) < R2_FAIL:
    fails.append(
        f"R² medio = {np.nanmean(R2):.2f} < {R2_FAIL}: el modelo no explica la corriente.\n"
        "   → Revise signos (CURRENT_SIGN/ENCODER_SIGN), la ventana --skip y que el\n"
        "     robot se haya movido sin colisiones.")
else:
    for i in range(NJ):
        # En joints débiles (α compartida) la varianza es casi puro ruido y el R²
        # pierde sentido; ahí importa el residuo absoluto, no el R².
        if i in strong and R2[i] < R2_WARN:
            warns.append(f"R² de J{i+1} = {R2[i]:.2f} < {R2_WARN}: ajuste pobre en ese "
                         "joint (¿poca excitación o fricción no modelada?).")
        elif i not in strong and resid_rms[i] > 8.0:
            warns.append(f"J{i+1}: residuo RMS = {resid_rms[i]:.1f} ticks > 8: corriente "
                         "no explicada por el modelo en ese joint.")
for i in range(NJ):
    if sat_frac[i] > SAT_FRAC_WARN:
        warns.append(
            f"J{i+1}: {100*sat_frac[i]:.1f}% de muestras con corriente saturada "
            "(descartadas del ajuste).\n"
            "   → Si supera ~10%, repita con gain_scale menor o trayectoria más suave.")
    if abs(Ioff[i]) > IOFF_WARN:
        warns.append(f"J{i+1}: |I_offset| = {abs(Ioff[i]):.1f} ticks > {IOFF_WARN:.0f}: "
                     "sospechoso (¿bias del sensor o desbalance mecánico?).")

print("\n" + "=" * 88)
if fails:
    print(" VEREDICTO: RE-EJECUTAR — los datos NO sirven para identificar alpha")
    for f_ in fails:
        print(" ✗ " + f_)
    for w in warns:
        print(" ! " + w)
    print("=" * 88)
    sys.exit(2)
elif warns:
    print(" VEREDICTO: REVISAR — alpha utilizable, pero con observaciones")
    for w in warns:
        print(" ! " + w)
else:
    print(" VEREDICTO: OK — alpha e I_offset listos para el ensamblado")
print("   Siguiente paso: python3 src/Identification/assemble_motor_params.py "
      "--alpha-log <este log> --friction-logs <J1> <J2> <J3> <J4>")
print("=" * 88)

# ════════════════════════════════════════════════════════════════════════════
#  Salidas: fragmento YAML + CSV de ajuste
# ════════════════════════════════════════════════════════════════════════════

out_dir = os.path.join(PKG_DIR, "data", "identification")
os.makedirs(out_dir, exist_ok=True)
log_tag = args.log_id if args.log_id is not None else \
          os.path.splitext(os.path.basename(csv_in))[0]

yaml_out = os.path.join(out_dir, f"alpha_fit_log{log_tag}.yaml")
fmt = lambda v: "[" + ", ".join(f"{x:.4f}" for x in v) + "]"
with open(yaml_out, "w") as f:
    f.write("# Fragmento de identificación de alpha/I_offset (identify_alpha.py)\n")
    f.write(f"# Fecha: {datetime.date.today().isoformat()}   Fuente: {csv_in}\n")
    f.write("# Método: OLS de I_meas vs torque comandado, modo corriente dinámico (FL)\n")
    f.write(f"source_csv: {os.path.basename(csv_in)}\n")
    f.write(f"eps: {eps:.4f}\n")
    f.write(f"alpha:            {fmt(alpha)}\n")
    f.write(f"I_offset:         {fmt(Ioff)}\n")
    f.write(f"alpha_shared:     {alpha_shared:.4f}\n")
    f.write(f"kt_effective:     {kt_of(alpha_shared):.4f}\n")
    f.write(f"R2:               {fmt(R2)}\n")
    f.write(f"resid_rms_ticks:  {fmt(resid_rms)}\n")
    f.write(f"corr_tau_I:       {fmt(corr_tau_I)}\n")
    f.write(f"strong_joints:    {[i + 1 for i in strong]}\n")
    f.write(f"bootstrap_run:    {bootstrap_ok}\n")
    f.write(f"Fv_informativo:   {fmt(Fv)}\n")
    f.write(f"Fc_informativo:   {fmt(Fc)}\n")
    f.write(f"verdict: {'REVISAR' if warns else 'OK'}\n")
print(f"\nFragmento YAML : {yaml_out}")

fit_out = os.path.join(out_dir, f"alpha_fit_log{log_tag}.csv")
header = (["t"] + [f"tau{i+1}" for i in range(NJ)]
          + [f"I_meas{i+1}" for i in range(NJ)]
          + [f"I_pred{i+1}" for i in range(NJ)])
with open(fit_out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(header)
    for k in range(N):
        if not mask[k]:
            continue
        w.writerow([f"{t[k]:.6f}"]
                   + [f"{tau_all[k, i]:.6f}" for i in range(NJ)]
                   + [f"{y_all[k, i]:.6f}"  for i in range(NJ)]
                   + [f"{I_pred[k, i]:.6f}" for i in range(NJ)])
print(f"CSV de ajuste  : {fit_out}")
