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
al final imprime un VEREDICTO por joint — OK / OK con nota / REVISAR /
RE-EJECUTAR — con el comando de relanzamiento concreto. Criterios:
  · nº de velocidades con emparejamiento válido (mínimo 3)
  · residuo RMS del ajuste (≤4 ticks bueno; >8 repetir)
  · outlier a la velocidad más baja = fricción de arranque (breakaway/Stribeck):
    real pero fuera del modelo Coulomb cinético → NO obliga a repetir
  · puntos I_fric negativos → revisar convención de signos
Caso típico (J2): resid alto por rebote pobre a baja velocidad → repetir con
vel_band:=0.3 (más reversiones por segmento) y/o vel_seg_duration:=8.0.

Entrada : data/diagnostics/sinusoidal/hw_sin_torque_<LOG_ID>.csv
Salidas :
    1) Consola : Fv, Fc, residuo, curva I_fric(v) y VEREDICTO.
    2) PNG     : plots/diagnostics/friction/log<LOG_ID>_friction.png
    3) CSV     : data/identification/friction_segments_<LOG_ID>.csv  (v, I_fric, I_grav)
    4) YAML    : data/identification/friction_J<j>_log<LOG_ID>.yaml  (fragmento
                 para assemble_motor_params.py)

Código de salida: 0 = OK/REVISAR, 2 = RE-EJECUTAR (repetir el ensayo).

Uso : python3 src/Identification/identify_friction.py <LOG_ID> [--joint N]
      --joint N verifica que el log corresponda a la articulación esperada.
Protocolo completo: src/Identification/Ident_OpenManX_XM430W350T_procedure.md
"""

import os
import sys
import csv
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
N_QBINS  = 8           # número de bins de q sobre el rango recorrido
MIN_BIN  = 5           # muestras mínimas por bin y por sentido para usar el bin
MIN_VELS = 3           # velocidades mínimas con dato para ajustar

# Umbrales del veredicto (residuo RMS del ajuste, en ticks)
RESID_OK     = 4.0     # ≤ → OK
RESID_REVIEW = 8.0     # ≤ → OK con nota / REVISAR ; > → RE-EJECUTAR
RESID_BREAK  = 3.0     # residuo sin el punto de v mínima ≤ → outlier = breakaway

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
t0 = t[active[0]] + SETTLE_SKIP_S if active.size else 0.0
win = t >= t0

# ════════════════════════════════════════════════════════════════════════════
#  Fricción por reversión de velocidad emparejada por posición
# ════════════════════════════════════════════════════════════════════════════

def friction_points(i):
    """Devuelve (v, I_fric, I_grav, n_bins, mags_perdidas):
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
    if len(v) < MIN_VELS:
        results[i] = dict(ok=False, n=len(v), missing=missing)
        continue
    Fv, Fc, resid_rms = fit_fv_fc(v, fr)
    # Ajuste alternativo sin la velocidad más baja: si el residuo se desploma,
    # el punto de v mínima es fricción de arranque (breakaway), no ruido.
    if len(v) > MIN_VELS:
        Fv_alt, Fc_alt, resid_alt = fit_fv_fc(v[1:], fr[1:])
    else:
        Fv_alt, Fc_alt, resid_alt = Fv, Fc, resid_rms
    results[i] = dict(ok=True, Fv=Fv, Fc=Fc, resid=resid_rms,
                      Fv_alt=Fv_alt, Fc_alt=Fc_alt, resid_alt=resid_alt,
                      v=v, fr=fr, gr=gr, nb=nb, missing=missing, n=len(v))
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
    print(f"J{i+1:>5} {r['Fv']:12.3f} {r['Fc']:12.3f} {r['resid']:11.2f} {r['n']:7d}")
print("-" * 76)
print(f"  Unidades: Fv [ticks/(rad/s)]  Fc [ticks]   (eps={EPSILON} rad/s)")
print(f"  Modelo: I_fric = Fv·v + Fc·tanh(v/eps)   (Fv, Fc >= 0)")
for i in vel_joints:
    r = results[i]
    if r["ok"]:
        curve = "  ".join(f"{vv:.2f}:{ff:.1f}" for vv, ff in zip(r["v"], r["fr"]))
        print(f"  J{i+1} I_fric(v): {curve}")
        print(f"  J{i+1} bins emparejados por velocidad: "
              + "  ".join(f"{vv:.2f}:{n}" for vv, n in zip(r["v"], r["nb"])))
    if r.get("missing"):
        print(f"  J{i+1} velocidades SIN emparejamiento válido: "
              + ", ".join(f"{m:.2f}" for m in r["missing"]) + " rad/s")
print("=" * 76)

# ── Veredicto por joint ──────────────────────────────────────────────────────

RELAUNCH = ("  ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py \\\n"
            "    friction_joint:={j} open_port:=true enable_torque:=true \\\n"
            "    enable_current_commands:=true log_id:=<NUEVO_LOG_ID>{extra}")

need_rerun = False
print("\n" + "=" * 76)
for i in vel_joints:
    r = results[i]
    j = i + 1
    if not r["ok"]:
        need_rerun = True
        print(f" VEREDICTO J{j}: RE-EJECUTAR — solo {r['n']} velocidades con "
              f"emparejamiento (mínimo {MIN_VELS}).")
        print("   El rebote no cubre ambos sentidos en las mismas posiciones. Repita con")
        print("   segmentos más largos y banda más angosta:")
        print(RELAUNCH.format(j=j, extra=" \\\n    vel_seg_duration:=8.0 vel_band:=0.3"))
        continue

    notes = []
    if np.any(r["fr"] < 0):
        notes.append("puntos I_fric NEGATIVOS → revise current_sign/encoder_sign "
                     "antes de usar estos valores")
    if r["missing"]:
        vmiss = ", ".join(f"{m:.2f}" for m in r["missing"])
        notes.append(f"sin dato en v = {vmiss} rad/s (rebote insuficiente ahí)")
    if float(np.mean(r["nb"])) < 2.0:
        notes.append("menos de 2 bins de q emparejados en promedio → cobertura pobre; "
                     "considere vel_band:=0.3")

    if r["resid"] <= RESID_OK and not np.any(r["fr"] < 0):
        print(f" VEREDICTO J{j}: OK — Fv={r['Fv']:.2f}, Fc={r['Fc']:.2f} "
              f"(resid={r['resid']:.2f} ticks)")
    elif r["resid"] <= RESID_REVIEW and r["resid_alt"] <= RESID_BREAK:
        print(f" VEREDICTO J{j}: OK con nota — Fv={r['Fv']:.2f}, Fc={r['Fc']:.2f} "
              f"(resid={r['resid']:.2f} ticks)")
        print(f"   El residuo lo domina el punto de v mínima ({r['v'][0]:.2f} rad/s → "
              f"{r['fr'][0]:.1f} ticks): fricción de ARRANQUE (breakaway/Stribeck), real"
              f" pero fuera del modelo Coulomb cinético. Sin ese punto: Fv={r['Fv_alt']:.2f},"
              f" Fc={r['Fc_alt']:.2f} (resid={r['resid_alt']:.2f}). No es necesario repetir.")
    elif r["resid"] <= RESID_REVIEW:
        print(f" VEREDICTO J{j}: REVISAR — Fv={r['Fv']:.2f}, Fc={r['Fc']:.2f} pero "
              f"resid={r['resid']:.2f} ticks (> {RESID_OK}).")
        print("   Valores utilizables como estimación central; para mejorar, repita con")
        print("   banda angosta (más reversiones a baja velocidad):")
        print(RELAUNCH.format(j=j, extra=" \\\n    vel_band:=0.3"))
    else:
        need_rerun = True
        print(f" VEREDICTO J{j}: RE-EJECUTAR — resid={r['resid']:.2f} ticks "
              f"(> {RESID_REVIEW}): dispersión excesiva.")
        print("   Curva I_fric(v) ruidosa (típico: rebote pobre a baja velocidad). Repita:")
        print(RELAUNCH.format(j=j, extra=" \\\n    vel_band:=0.3 vel_seg_duration:=8.0"))
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
    if r["resid"] <= RESID_OK and not np.any(r["fr"] < 0):
        verdict = "OK"
    elif r["resid"] <= RESID_REVIEW:
        verdict = "OK_NOTA" if r["resid_alt"] <= RESID_BREAK else "REVISAR"
    else:
        verdict = "RE-EJECUTAR"
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
        f.write(f"v_points:      [{', '.join(f'{x:.3f}' for x in r['v'])}]\n")
        f.write(f"I_fric_points: [{', '.join(f'{x:.3f}' for x in r['fr'])}]\n")
    print(f"Fragmento YAML   : {frag}  (veredicto: {verdict})")

sys.exit(2 if need_rerun else 0)
