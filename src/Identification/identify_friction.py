#!/usr/bin/env python3
"""
identify_friction.py
Identificación LIMPIA de la fricción articular (Fv, Fc, I_offset) del
OpenMANIPULATOR-X a partir de un barrido de VELOCIDAD CONSTANTE generado por
hw_sinusoidal_torque_node en mode="velocity" (config hw_friction_sweep_params.yaml).

Método: REVERSIÓN DE VELOCIDAD emparejada por posición (sin modelo ni alpha).
A la misma configuración q, la corriente se descompone en:
    I(+v, q) = +I_fric(v) + I_grav(q)      (fricción impar + gravedad par)
    I(-v, q) = -I_fric(v) + I_grav(q)
de donde, restando muestras en el MISMO bin de q:
    I_fric(v) = [ <I(+v,q)> - <I(-v,q)> ] / 2     → cancela la gravedad EXACTAMENTE
    I_grav(q) = [ <I(+v,q)> + <I(-v,q)> ] / 2     (informativo)

Se obtiene un punto I_fric por cada velocidad y se ajusta el modelo de fricción
que usan los nodos:
    I_fric(v) = Fv·v + Fc·tanh(v/eps) + I_offset      (Fv, Fc >= 0)

Como la gravedad se cancela por construcción, NO requiere Pinocchio, el URDF ni
alpha — y elimina el Fv negativo espurio que produce restar α·tau_grav imperfecto.
Para J1 (eje vertical) la gravedad es nula y la corriente ya es fricción pura.

Entrada : data/diagnostics/sinusoidal/hw_sin_torque_<LOG_ID>.csv
Salidas :
    1) Consola : Fv, Fc, I_offset y R² del joint en velocity mode.
    2) PNG     : plots/diagnostics/friction/log<LOG_ID>_friction.png  (I_fric vs v + ajuste)
    3) CSV     : data/identification/friction_segments_<LOG_ID>.csv   (v, I_fric, I_grav)

Uso : python3 src/Identification/identify_friction.py [LOG_ID]
"""

import os
import sys
import csv
import numpy as np
from scipy.optimize import lsq_linear
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ════════════════════════════════════════════════════════════════════════════
#  Configuración
# ════════════════════════════════════════════════════════════════════════════

PKG_DIR = "/home/utec/open_manx_ws/src/open_manipulator_x_torque_control"
# log del barrido de velocidad; admite override por CLI:  identify_friction.py 21
LOG_ID  = int(sys.argv[1]) if len(sys.argv) > 1 else 20

EPSILON = 0.05         # [rad/s] suavizado del tanh (debe coincidir con el nodo)

# Selección de muestras en régimen permanente dentro de cada meseta
STEADY_TOL    = 0.30   # |dq_meas - dq_ref| / |dq_ref| máximo para "en régimen"
SETTLE_SKIP_S = 0.5    # [s] descartados tras el fin del settling

# Emparejamiento por posición (cancelación exacta de gravedad)
N_QBINS  = 8           # número de bins de q sobre el rango recorrido
MIN_BIN  = 5           # muestras mínimas por bin y por sentido para usar el bin
MIN_VELS = 3           # velocidades mínimas con dato para ajustar

NJ = 4

# ════════════════════════════════════════════════════════════════════════════
#  Carga de datos
# ════════════════════════════════════════════════════════════════════════════

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
    raise RuntimeError("No se detectó ningún joint en velocity mode (dq_ref ≈ 0).")
print(f"Joints en velocity mode: {[f'J{i+1}' for i in vel_joints]}")

active = np.where(np.any(np.abs(dq_ref) > 0.01, axis=1))[0]
t0 = t[active[0]] + SETTLE_SKIP_S if active.size else 0.0
win = t >= t0

# ════════════════════════════════════════════════════════════════════════════
#  Fricción por reversión de velocidad emparejada por posición
# ════════════════════════════════════════════════════════════════════════════

def friction_points(i):
    """Devuelve arrays (v, I_fric, I_grav) — un punto por magnitud de velocidad."""
    qi   = q[win, i]
    dqr  = dq_ref[win, i]
    dqm  = dq_meas[win, i]
    cur  = curr[win, i]
    mags = sorted(set(np.round(np.abs(dqr[np.abs(dqr) > 0.01]), 3)))
    qbins = np.linspace(qi.min(), qi.max(), N_QBINS + 1)

    V, FR, GR = [], [], []
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
    return np.array(V), np.array(FR), np.array(GR)

results = {}
seg_rows = []
for i in vel_joints:
    v, fr, gr = friction_points(i)
    if len(v) < MIN_VELS:
        print(f"  J{i+1}: solo {len(v)} velocidades con emparejamiento q — insuficiente.")
        continue
    # Ajuste  I_fric = Fv·v + Fc·tanh(v/eps)   con Fv, Fc >= 0
    # Sin término de offset: el bias de corriente es PAR y se cancela en la
    # reversión (queda en I_grav). Además tanh(v/eps)≈1 aquí → un offset sería
    # colineal con Fc. El I_offset del nodo proviene de la identificación de α.
    W = np.column_stack([v, np.tanh(v / EPSILON)])
    theta = lsq_linear(W, fr, bounds=([0.0, 0.0], [np.inf, np.inf])).x
    Fv, Fc = theta
    pred = W @ theta
    resid_rms = float(np.sqrt(np.mean((fr - pred) ** 2)))   # [ticks] dispersión del ajuste
    results[i] = dict(Fv=Fv, Fc=Fc, resid=resid_rms, v=v, fr=fr, gr=gr, n=len(v))
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
    if i not in results:
        continue
    r = results[i]
    print(f"J{i+1:>5} {r['Fv']:12.3f} {r['Fc']:12.3f} {r['resid']:11.2f} {r['n']:7d}")
print("-" * 76)
print(f"  Unidades: Fv [ticks/(rad/s)]  Fc [ticks]   (eps={EPSILON} rad/s)")
print(f"  Modelo: I_fric = Fv·v + Fc·tanh(v/eps)   (Fv, Fc >= 0)")
for i in vel_joints:
    if i in results:
        r = results[i]
        curve = "  ".join(f"{vv:.2f}:{ff:.1f}" for vv, ff in zip(r["v"], r["fr"]))
        print(f"  J{i+1} I_fric(v): {curve}")
print("=" * 76)

# ════════════════════════════════════════════════════════════════════════════
#  Gráfica I_fric vs v + ajuste
# ════════════════════════════════════════════════════════════════════════════

if results:
    plot_dir = os.path.join(PKG_DIR, "plots", "diagnostics", "friction")
    os.makedirs(plot_dir, exist_ok=True)
    fig, axs = plt.subplots(1, len(results), figsize=(5.2 * len(results), 4.2),
                            squeeze=False)
    for ax, (i, r) in zip(axs[0], results.items()):
        ax.plot(r["v"], r["fr"], "o", color="#0072BD", label="medido (reversión)")
        xs = np.linspace(0, r["v"].max(), 200)
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

if seg_rows:
    seg_out = os.path.join(PKG_DIR, "data", "identification",
                           f"friction_segments_{LOG_ID}.csv")
    os.makedirs(os.path.dirname(seg_out), exist_ok=True)
    with open(seg_out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["joint", "v", "I_fric", "I_grav"])
        w.writerows(seg_rows)
    print(f"CSV puntos       : {seg_out}  ({len(seg_rows)} puntos)")
