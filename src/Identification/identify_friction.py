#!/usr/bin/env python3
"""
identify_friction.py
Identificación LIMPIA de la fricción articular (Fv, Fc) del OpenMANIPULATOR-X
a partir de un barrido de VELOCIDAD CONSTANTE generado por
hw_sinusoidal_torque_node en mode="velocity" (launch friction_sweep.launch.py,
config hw_friction_sweep_params.yaml).

Método: REVERSIÓN DE VELOCIDAD emparejada por posición (sin modelo ni alpha).
A la misma configuración q, la corriente se descompone en:
    I(+v, q) = +I_fric(v) + I_grav(q)      (fricción impar + gravedad par)
    I(-v, q) = -I_fric(v) + I_grav(q)
de donde, restando muestras en el MISMO bin de q:
    I_fric(v) = [ <I(+v,q)> - <I(-v,q)> ] / 2     → cancela la gravedad EXACTAMENTE
    I_grav(q) = [ <I(+v,q)> + <I(-v,q)> ] / 2     (informativo)

Se obtiene un punto I_fric por cada velocidad y se ajusta el modelo de fricción
que usan los nodos:
    I_fric(v) = Fv·v + Fc·tanh(v/eps)      (Fv, Fc >= 0)

Como la gravedad se cancela por construcción, NO requiere Pinocchio, el URDF ni
alpha — y elimina el Fv negativo espurio que produce restar α·tau_grav imperfecto.
Para J1 (eje vertical) la gravedad es nula y la corriente ya es fricción pura.

CONTROL DE CALIDAD (este script decide si hay que repetir el ensayo):
  · puntos con <2 bins de q emparejados se DESCARTAN del ajuste (frágiles)
  · outlier alto a la v mínima = fricción de arranque (breakaway/Stribeck):
    se excluye del ajuste (los Fv/Fc oficiales son del régimen CINÉTICO) y
    NO obliga a repetir
  · detecta BARRIDO TRUNCADO (duration_s corto para vel_seg_duration elegido)
  · el consejo de re-ejecución tiene en cuenta la banda y duración de segmento
    que el log YA usó (estimadas de los datos): no repite recetas ya aplicadas
Veredicto por joint: OK / OK con nota / REVISAR / RE-EJECUTAR, con el comando
concreto de relanzamiento. Código de salida 2 = repetir el ensayo.

Entrada : data/diagnostics/sinusoidal/hw_sin_torque_<LOG_ID>.csv
Salidas :
    1) Consola : Fv, Fc, residuo, curva I_fric(v) y VEREDICTO.
    2) PNG     : plots/diagnostics/friction/log<LOG_ID>_friction.png
    3) CSV     : data/identification/friction_segments_<LOG_ID>.csv  (v, I_fric, I_grav)
    4) YAML    : data/identification/friction_J<j>_log<LOG_ID>.yaml  (fragmento
                 para assemble_motor_params.py)

Uso : python3 src/Identification/identify_friction.py <LOG_ID> [--joint N]
      --joint N verifica que el log corresponda a la articulación esperada.
Protocolo completo: src/Identification/Ident_OpenManX_XM430W350T_procedure.md
"""

import os
import sys
import csv
import math
import argparse
import datetime
import numpy as np
from scipy.optimize import lsq_linear
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ════════════════════════════════════════════════════════════════════════════
#  Configuración
# ════════════════════════════════════════════════════════════════════════════

# Raíz del paquete: dos niveles arriba de src/Identification/
PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

EPSILON = 0.05         # [rad/s] suavizado del tanh (debe coincidir con el nodo)

# Selección de muestras en régimen permanente dentro de cada meseta
STEADY_TOL    = 0.30   # |dq_meas - dq_ref| / |dq_ref| máximo para "en régimen"
SETTLE_SKIP_S = 0.5    # [s] descartados tras el fin del settling

# Emparejamiento por posición (cancelación exacta de gravedad)
N_QBINS     = 8        # número de bins de q sobre el rango recorrido
MIN_BIN     = 5        # muestras mínimas por bin y por sentido para usar el bin
MIN_BINS_PT = 2        # bins emparejados mínimos para que un punto entre al ajuste
MIN_VELS    = 3        # velocidades mínimas (tras descartes) para ajustar

# Umbrales del veredicto (residuo RMS del ajuste, en ticks)
RESID_OK     = 4.0     # ≤ → OK
RESID_REVIEW = 8.0     # ≤ → REVISAR ; > → RE-EJECUTAR
RESID_BREAK  = 3.0     # residuo sin el punto de v mínima ≤ → outlier = breakaway

# Detección de barrido truncado / parámetros ya aplicados (estimados del log)
TRUNC_FRAC   = 0.6     # dwell de la última magnitud < 0.6·mediana → truncado
BAND_APPLIED = 0.40    # band_est ≤ → la banda angosta ya se usó
SEG_APPLIED  = 7.0     # seg_est ≥ → los segmentos largos ya se usaron
N_VEL_LIST   = 8       # nº de velocidades del vel_list por defecto (duration_s)

NJ = 4

# ════════════════════════════════════════════════════════════════════════════
#  CLI y carga de datos
# ════════════════════════════════════════════════════════════════════════════

ap = argparse.ArgumentParser(description="Identificación de fricción por "
                             "reversión de velocidad (Fv, Fc).")
ap.add_argument("log_id", nargs="?", type=int, default=20,
                help="ID del log hw_sin_torque_<id>.csv (def. 20)")
ap.add_argument("--joint", type=int, default=None, choices=[1, 2, 3, 4],
                help="articulación esperada en velocity mode (verificación)")
ap.add_argument("--eps", type=float, default=EPSILON,
                help=f"suavizado del tanh [rad/s] (def. {EPSILON})")
args = ap.parse_args()
LOG_ID  = args.log_id
EPSILON = args.eps

csv_in = os.path.join(PKG_DIR, "data", "diagnostics", "sinusoidal",
                      f"hw_sin_torque_{LOG_ID}.csv")
if not os.path.isfile(csv_in):
    raise FileNotFoundError(f"No se encontró el CSV de entrada:\n  {csv_in}")

raw = np.genfromtxt(csv_in, delimiter=",", names=True)
t       = raw["t"] - raw["t"][0]
q       = np.column_stack([raw[f"q{i}"]         for i in range(1, NJ + 1)])
dq_meas = np.column_stack([raw[f"dq{i}"]        for i in range(1, NJ + 1)])
dq_ref  = np.column_stack([raw[f"dq_ref{i}"]    for i in range(1, NJ + 1)])
curr    = np.column_stack([raw[f"curr_meas{i}"] for i in range(1, NJ + 1)])
Ts = float(np.mean(np.diff(t)))
print(f"Cargado: {csv_in}  ({len(t)} muestras, fs={1/Ts:.1f} Hz)")

vel_joints = [i for i in range(NJ) if np.max(np.abs(dq_ref[:, i])) > 0.01]
if not vel_joints:
    print("\nERROR: no se detectó ningún joint en velocity mode (dq_ref ≈ 0).")
    print("→ Acción: lance el ensayo con friction_sweep.launch.py, p.ej.:")
    print("  ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py \\")
    print("    friction_joint:=1 open_port:=true enable_torque:=true \\")
    print("    enable_current_commands:=true log_id:=21")
    sys.exit(2)
print(f"Joints en velocity mode: {[f'J{i+1}' for i in vel_joints]}")

if args.joint is not None and (args.joint - 1) not in vel_joints:
    print(f"\nERROR: se esperaba J{args.joint} en velocity mode, pero el log "
          f"contiene {[f'J{i+1}' for i in vel_joints]}.")
    print("→ Acción: verifique el log_id o repita el ensayo con "
          f"friction_joint:={args.joint}.")
    sys.exit(2)

active = np.where(np.any(np.abs(dq_ref) > 0.01, axis=1))[0]
t_settle_est = t[active[0]] if active.size else 2.0
t0 = t_settle_est + SETTLE_SKIP_S
win = t >= t0

# ════════════════════════════════════════════════════════════════════════════
#  Condiciones del ensayo estimadas del propio log (para el consejo de
#  re-ejecución): banda de rebote, duración de segmento y truncamiento.
# ════════════════════════════════════════════════════════════════════════════

def sweep_conditions(i):
    """Devuelve (band_est, seg_est, truncated, dwell) del joint i."""
    act = np.abs(dq_ref[:, i]) > 0.01
    qa = q[act, i]
    band_est = float(qa.max() - qa.min()) / 2.0 if qa.size else float("nan")
    mags = sorted(set(np.round(np.abs(dq_ref[act, i]), 3)))
    dwell = {m: Ts * float(np.sum(np.abs(np.abs(dq_ref[:, i]) - m) < 1e-3))
             for m in mags}
    if len(dwell) >= 2:
        d_sorted = [dwell[m] for m in mags]
        seg_est = float(np.median(d_sorted[:-1]))   # sin la última (posible corte)
        truncated = d_sorted[-1] < TRUNC_FRAC * seg_est
    else:
        seg_est = float(next(iter(dwell.values()), 0.0))
        truncated = False
    return band_est, seg_est, truncated, dwell

def relaunch_advice(j, band_est, seg_est, truncated):
    """Comando de re-ejecución que NO repite recetas ya aplicadas en el log.
       Devuelve (comando, escalated): escalated=True si banda y segmentos ya
       estaban ajustados (el problema no es de configuración del barrido).
       Si el barrido se truncó, la receta es la MISMA configuración con la
       duración completa (no escalar nada: el ajuste local no es el problema)."""
    band_applied = band_est <= BAND_APPLIED
    seg_applied  = seg_est >= SEG_APPLIED
    band_t = round(band_est, 1) if band_applied else 0.3
    if truncated:
        escalated = False
        seg_t = round(seg_est, 1)
    elif band_applied and seg_applied:
        escalated = True
        seg_t = math.ceil(seg_est - 0.01) + 2.0  # escalar: aún más tiempo por segmento
    else:
        escalated = False
        seg_t = round(seg_est, 1) if seg_applied else 8.0
    dur_t = math.ceil(t_settle_est + N_VEL_LIST * seg_t + 2.0)
    cmd = ("  ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py \\\n"
           f"    friction_joint:={j} open_port:=true enable_torque:=true \\\n"
           f"    enable_current_commands:=true log_id:=<NUEVO_LOG_ID> \\\n"
           f"    vel_band:={band_t} vel_seg_duration:={seg_t} duration_s:={dur_t}.0")
    return cmd, escalated

# ════════════════════════════════════════════════════════════════════════════
#  Fricción por reversión de velocidad emparejada por posición
# ════════════════════════════════════════════════════════════════════════════

def friction_points(i):
    """Devuelve (v, I_fric, I_grav, n_bins, mags_sin_par):
       un punto por magnitud de velocidad + cobertura del emparejamiento."""
    qi   = q[win, i]
    dqr  = dq_ref[win, i]
    dqm  = dq_meas[win, i]
    cur  = curr[win, i]
    mags = sorted(set(np.round(np.abs(dqr[np.abs(dqr) > 0.01]), 3)))
    qbins = np.linspace(qi.min(), qi.max(), N_QBINS + 1)

    V, FR, GR, NB, missing = [], [], [], [], []
    for mg in mags:
        pos = (np.abs(dqr - mg) < 1e-3) & (np.abs(dqm - mg) < STEADY_TOL * mg)
        neg = (np.abs(dqr + mg) < 1e-3) & (np.abs(dqm + mg) < STEADY_TOL * mg)
        fr_bins, gr_bins = [], []
        for b in range(N_QBINS):
            inb = (qi >= qbins[b]) & (qi < qbins[b + 1])
            p = inb & pos
            n = inb & neg
            if p.sum() >= MIN_BIN and n.sum() >= MIN_BIN:
                Ip, In = np.mean(cur[p]), np.mean(cur[n])
                fr_bins.append((Ip - In) / 2.0)   # fricción (gravedad cancelada)
                gr_bins.append((Ip + In) / 2.0)   # gravedad
        if fr_bins:
            V.append(mg)
            FR.append(np.median(fr_bins))          # mediana → robusto a outliers
            GR.append(np.median(gr_bins))
            NB.append(len(fr_bins))
        else:
            missing.append(mg)
    return np.array(V), np.array(FR), np.array(GR), np.array(NB), missing

def fit_fv_fc(v, fr):
    """Ajusta I_fric = Fv·v + Fc·tanh(v/eps) con Fv, Fc >= 0.
       Sin término de offset: el bias de corriente es PAR y se cancela en la
       reversión (queda en I_grav). Además tanh(v/eps)≈1 aquí → un offset sería
       colineal con Fc. El I_offset del nodo proviene de identify_alpha.py."""
    W = np.column_stack([v, np.tanh(v / EPSILON)])
    theta = lsq_linear(W, fr, bounds=([0.0, 0.0], [np.inf, np.inf])).x
    resid = float(np.sqrt(np.mean((fr - W @ theta) ** 2)))
    return theta[0], theta[1], resid

results = {}
seg_rows = []
for i in vel_joints:
    v, fr, gr, nb, missing = friction_points(i)
    band_est, seg_est, truncated, dwell = sweep_conditions(i)
    use = nb >= MIN_BINS_PT
    r = dict(band_est=band_est, seg_est=seg_est, truncated=truncated,
             v=v, fr=fr, gr=gr, nb=nb, missing=missing,
             dropped=[(vv, ff, n) for vv, ff, n in zip(v[~use], fr[~use], nb[~use])])
    v_fit, fr_fit = v[use], fr[use]
    if len(v_fit) < MIN_VELS:
        r.update(ok=False, n=len(v_fit))
        results[i] = r
        continue
    Fv, Fc, resid_rms = fit_fv_fc(v_fit, fr_fit)
    # Fricción de arranque (breakaway/Stribeck): si al excluir el punto de v
    # mínima el residuo se desploma Y ese punto queda POR ENCIMA del ajuste
    # cinético, el outlier es arranque real → los Fv/Fc oficiales son los del
    # régimen cinético (sin ese punto), que es el que compensan los nodos.
    breakaway = False
    Fv_full, Fc_full, resid_full = Fv, Fc, resid_rms
    if len(v_fit) > MIN_VELS and resid_rms > RESID_OK:
        Fv_a, Fc_a, resid_a = fit_fv_fc(v_fit[1:], fr_fit[1:])
        pred0 = Fv_a * v_fit[0] + Fc_a * np.tanh(v_fit[0] / EPSILON)
        if resid_a <= RESID_BREAK and fr_fit[0] > pred0:
            breakaway = True
            Fv, Fc, resid_rms = Fv_a, Fc_a, resid_a
    r.update(ok=True, n=len(v_fit), Fv=Fv, Fc=Fc, resid=resid_rms,
             breakaway=breakaway, Fv_full=Fv_full, Fc_full=Fc_full,
             resid_full=resid_full, v_fit=v_fit, fr_fit=fr_fit)
    results[i] = r
    for vv, ff, gg in zip(v, fr, gr):
        seg_rows.append([i + 1, vv, ff, gg])

# ════════════════════════════════════════════════════════════════════════════
#  Reporte
# ════════════════════════════════════════════════════════════════════════════

print("\n" + "=" * 76)
print(f" Fricción por reversión de velocidad (gravedad cancelada)  [log {LOG_ID}]")
print("=" * 76)
print(f"{'Joint':>6} {'Fv':>12} {'Fc':>12} {'resid_RMS':>11} {'n_vel':>7}")
print("-" * 76)
for i in vel_joints:
    r = results[i]
    if not r["ok"]:
        print(f"J{i+1:>5} {'—':>12} {'—':>12} {'—':>11} {r['n']:7d}  (insuficiente)")
        continue
    tag = "  (cinético, sin breakaway)" if r["breakaway"] else ""
    print(f"J{i+1:>5} {r['Fv']:12.3f} {r['Fc']:12.3f} {r['resid']:11.2f} {r['n']:7d}{tag}")
print("-" * 76)
print(f"  Unidades: Fv [ticks/(rad/s)]  Fc [ticks]   (eps={EPSILON} rad/s)")
print(f"  Modelo: I_fric = Fv·v + Fc·tanh(v/eps)   (Fv, Fc >= 0)")
for i in vel_joints:
    r = results[i]
    curve = "  ".join(f"{vv:.2f}:{ff:.1f}" for vv, ff in zip(r["v"], r["fr"]))
    if curve:
        print(f"  J{i+1} I_fric(v): {curve}")
        print(f"  J{i+1} bins emparejados por velocidad: "
              + "  ".join(f"{vv:.2f}:{n}" for vv, n in zip(r["v"], r["nb"])))
    for vv, ff, n in r["dropped"]:
        print(f"  J{i+1} punto v={vv:.2f} DESCARTADO del ajuste ({n} bin emparejado: "
              f"frágil; I_fric={ff:.1f})")
    if r["missing"]:
        print(f"  J{i+1} velocidades SIN emparejamiento válido: "
              + ", ".join(f"{m:.2f}" for m in r["missing"]) + " rad/s")
    print(f"  J{i+1} condiciones del ensayo (estimadas): banda ±{r['band_est']:.2f} rad, "
          f"{r['seg_est']:.1f} s/velocidad"
          + ("  → BARRIDO TRUNCADO (duration_s corto)" if r["truncated"] else ""))
print("=" * 76)

# ── Veredicto por joint ──────────────────────────────────────────────────────

need_rerun = False
verdicts = {}
print("\n" + "=" * 76)
for i in vel_joints:
    r = results[i]
    j = i + 1
    cmd, escalated = relaunch_advice(j, r["band_est"], r["seg_est"], r["truncated"])
    trunc_msg = None
    if r["truncated"]:
        dur = math.ceil(t_settle_est + N_VEL_LIST * max(r["seg_est"], 5.0) + 2.0)
        trunc_msg = (f"BARRIDO TRUNCADO: el log solo cubre hasta "
                     f"{r['v'].max() if len(r['v']) else 0:.2f} rad/s — duration_s fue "
                     f"menor que t_settle + {N_VEL_LIST}·vel_seg_duration. Re-lance con "
                     f"duration_s:={dur}.0 (o actualice el launch, que ya la calcula solo).")

    if not r["ok"]:
        need_rerun = True
        verdicts[i] = "RE-EJECUTAR"
        print(f" VEREDICTO J{j}: RE-EJECUTAR — solo {r['n']} velocidades útiles tras "
              f"descartes (mínimo {MIN_VELS}).")
        print("   El rebote no cubre ambos sentidos en las mismas posiciones. Repita:")
        print(cmd)
        if trunc_msg:
            print(f"   ! {trunc_msg}")
        continue

    notes = []
    if trunc_msg:
        notes.append(trunc_msg)
    if np.any(r["fr_fit"] < 0):
        notes.append("puntos I_fric NEGATIVOS en el ajuste → revise "
                     "current_sign/encoder_sign antes de usar estos valores")
    if r["missing"]:
        vmiss = ", ".join(f"{m:.2f}" for m in r["missing"])
        notes.append(f"sin dato en v = {vmiss} rad/s (rebote insuficiente ahí)")

    if r["breakaway"]:
        verdicts[i] = "OK_NOTA"
        print(f" VEREDICTO J{j}: OK con nota — Fv={r['Fv']:.2f}, Fc={r['Fc']:.2f} "
              f"(resid={r['resid']:.2f} ticks, régimen cinético)")
        print(f"   El punto de v mínima ({r['v_fit'][0]:.2f} rad/s → {r['fr_fit'][0]:.1f} "
              f"ticks) es fricción de ARRANQUE (breakaway/Stribeck): real pero fuera del")
        print(f"   modelo Coulomb cinético → se EXCLUYE del ajuste oficial. Con él: "
              f"Fv={r['Fv_full']:.2f}, Fc={r['Fc_full']:.2f} (resid={r['resid_full']:.2f})."
              f" No es necesario repetir.")
    elif r["resid"] <= RESID_OK and not np.any(r["fr_fit"] < 0):
        verdicts[i] = "OK"
        print(f" VEREDICTO J{j}: OK — Fv={r['Fv']:.2f}, Fc={r['Fc']:.2f} "
              f"(resid={r['resid']:.2f} ticks)")
    elif r["resid"] <= RESID_REVIEW:
        verdicts[i] = "REVISAR"
        print(f" VEREDICTO J{j}: REVISAR — Fv={r['Fv']:.2f}, Fc={r['Fc']:.2f} pero "
              f"resid={r['resid']:.2f} ticks (> {RESID_OK}).")
        if escalated:
            print("   Banda y duración de segmento YA estaban ajustadas: la dispersión es")
            print("   del robot (mecánica: cableado rozando, backlash, temperatura). Puede")
            print("   aceptar estos valores centrales o intentar con más tiempo por segmento:")
        else:
            print("   Valores utilizables como estimación central; para mejorar, repita:")
        print(cmd)
    else:
        need_rerun = True
        verdicts[i] = "RE-EJECUTAR"
        print(f" VEREDICTO J{j}: RE-EJECUTAR — resid={r['resid']:.2f} ticks "
              f"(> {RESID_REVIEW}): dispersión excesiva.")
        if escalated:
            print("   Banda y duración de segmento YA estaban ajustadas → el problema no es")
            print("   la configuración del barrido: revise mecánica (cableado, montaje,")
            print("   backlash), temperatura de motores y signos. Luego repita con:")
        else:
            print("   Curva I_fric(v) ruidosa (típico: rebote pobre a baja velocidad). Repita:")
        print(cmd)

    # Truncamiento: aunque el ajuste local sea bueno, faltan las velocidades
    # altas → sin plateau confirmado no se separa bien Fv de Fc.
    if r["truncated"] and verdicts[i] in ("OK", "OK_NOTA"):
        verdicts[i] = "REVISAR"
        print("   ↓ Degradado a REVISAR por barrido truncado: faltan las velocidades altas")
        print("     para confirmar el plateau (separación Fv↔Fc). Re-lance completo:")
        print(cmd)
    for nmsg in notes:
        print(f"   ! {nmsg}")
print("=" * 76)

# ════════════════════════════════════════════════════════════════════════════
#  Gráfica I_fric vs v + ajuste
# ════════════════════════════════════════════════════════════════════════════

plotted = {i: r for i, r in results.items() if r["ok"]}
if plotted:
    plot_dir = os.path.join(PKG_DIR, "plots", "diagnostics", "friction")
    os.makedirs(plot_dir, exist_ok=True)
    fig, axs = plt.subplots(1, len(plotted), figsize=(5.2 * len(plotted), 4.2),
                            squeeze=False)
    for ax, (i, r) in zip(axs[0], plotted.items()):
        ax.plot(r["v_fit"], r["fr_fit"], "o", color="#0072BD",
                label="medido (reversión)")
        if r["dropped"]:
            ax.plot([d[0] for d in r["dropped"]], [d[1] for d in r["dropped"]],
                    "x", color="0.5", ms=8, label="descartado (<2 bins)")
        if r["breakaway"]:
            ax.plot(r["v_fit"][0], r["fr_fit"][0], "s", mfc="none", mec="#A2142F",
                    ms=11, label="breakaway (excluido)")
        xs = np.linspace(0, r["v_fit"].max(), 200)
        ys = r["Fv"] * xs + r["Fc"] * np.tanh(xs / EPSILON)
        ax.plot(xs, ys, "-", color="#D95319", lw=1.8,
                label=f"ajuste (resid={r['resid']:.1f})")
        ax.axhline(0, ls=":", c="0.5", lw=0.8)
        ax.set_xlabel(r"$|\dot{q}|$ [rad/s]")
        ax.set_ylabel(r"$I_{fric}$ [ticks]")
        ax.set_title(f"J{i+1}:  Fv={r['Fv']:.2f}  Fc={r['Fc']:.2f}")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)
    fig.suptitle(f"Fricción por reversión de velocidad — log {LOG_ID}", fontweight="bold")
    fig.tight_layout()
    png = os.path.join(plot_dir, f"log{LOG_ID}_friction.png")
    fig.savefig(png, dpi=200)
    print(f"\nGráfica guardada : {png}")

# ════════════════════════════════════════════════════════════════════════════
#  Salidas: CSV de puntos + fragmento YAML por joint
# ════════════════════════════════════════════════════════════════════════════

out_dir = os.path.join(PKG_DIR, "data", "identification")
os.makedirs(out_dir, exist_ok=True)

if seg_rows:
    seg_out = os.path.join(out_dir, f"friction_segments_{LOG_ID}.csv")
    with open(seg_out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["joint", "v", "I_fric", "I_grav"])
        w.writerows(seg_rows)
    print(f"CSV puntos       : {seg_out}  ({len(seg_rows)} puntos)")

for i, r in plotted.items():
    j = i + 1
    verdict = verdicts[i]
    frag = os.path.join(out_dir, f"friction_J{j}_log{LOG_ID}.yaml")
    with open(frag, "w") as f:
        f.write("# Fragmento de identificación de fricción (identify_friction.py)\n")
        f.write(f"# Fecha: {datetime.date.today().isoformat()}   Fuente: {csv_in}\n")
        f.write("# Método: reversión de velocidad emparejada por posición\n")
        f.write(f"joint: {j}\n")
        f.write(f"source_csv: {os.path.basename(csv_in)}\n")
        f.write(f"eps: {EPSILON:.4f}\n")
        f.write(f"Fv: {r['Fv']:.4f}\n")
        f.write(f"Fc: {r['Fc']:.4f}\n")
        f.write(f"resid_rms: {r['resid']:.4f}\n")
        f.write(f"n_vel: {r['n']}\n")
        f.write(f"verdict: {verdict}\n")
        f.write(f"breakaway_excluded: {r['breakaway']}\n")
        f.write(f"band_est: {r['band_est']:.3f}\n")
        f.write(f"seg_est: {r['seg_est']:.2f}\n")
        f.write(f"truncated: {r['truncated']}\n")
        f.write(f"v_points:      [{', '.join(f'{x:.3f}' for x in r['v'])}]\n")
        f.write(f"I_fric_points: [{', '.join(f'{x:.3f}' for x in r['fr'])}]\n")
    print(f"Fragmento YAML   : {frag}  (veredicto: {verdict})")

sys.exit(2 if need_rerun else 0)
