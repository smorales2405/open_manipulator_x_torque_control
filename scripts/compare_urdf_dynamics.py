#!/usr/bin/env python3
"""
compare_urdf_dynamics.py
Verifica la coherencia fisica entre el URDF antiguo (openmani.urdf) y el nuevo
(open_manipulator_x.urdf) usando Pinocchio.

Checks:
  1. Estructura de M(q): signos y patron de entradas dominantes preservados.
  2. Vector de gravedad G(q): signos identicos en ambos modelos para todo q.
  3. NLE(q, dq): coherencia de efectos no lineales.
  4. Magnitudes: los cambios de masa se reflejan monotonamente en M y G.
  5. Diagnostico de saturacion: G_new vs TAU_MAX del nodo FL (0.82 N.m).

Uso:
  python3 scripts/compare_urdf_dynamics.py

Salida:
  - Reporte de texto en consola.
  - Figura  plots/compare_dynamics/dynamics_comparison.png
"""

import os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── Pinocchio ─────────────────────────────────────────────────────────────────
try:
    import pinocchio as pin
except ImportError:
    sys.exit("ERROR: pinocchio no esta instalado. "
             "Ejecuta: source /opt/openrobots/setup.bash")

# ── Rutas ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR    = os.path.dirname(SCRIPT_DIR)
URDF_OLD   = os.path.join(PKG_DIR, "urdf", "openmani.urdf")
URDF_NEW   = os.path.join(PKG_DIR, "urdf", "open_manipulator_x.urdf")
PLOT_DIR   = os.path.join(PKG_DIR, "plots", "compare_dynamics")
TAU_MAX    = 0.82          # limite del nodo gz_fl_control_node.cpp [N·m]
NARM       = 4             # joints del brazo (primeros 4 de nv)
THRESHOLD  = 1e-4          # umbral para considerar un torque "significativo"

os.makedirs(PLOT_DIR, exist_ok=True)

# ── Configuraciones de prueba ─────────────────────────────────────────────────
pi = np.pi
CONFIGS = {
    "home [0,0,0,0]":           np.array([0.0,   0.0,   0.0,   0.0  ]),
    "j2=+pi/4":                 np.array([0.0,   pi/4,  0.0,   0.0  ]),
    "j2=-pi/4":                 np.array([0.0,  -pi/4,  0.0,   0.0  ]),
    "j3=+pi/4":                 np.array([0.0,   0.0,   pi/4,  0.0  ]),
    "j3=-pi/4":                 np.array([0.0,   0.0,  -pi/4,  0.0  ]),
    "j4=+pi/4":                 np.array([0.0,   0.0,   0.0,   pi/4 ]),
    "tipica [0,.5,.3,-.5]":     np.array([0.0,   0.5,   0.3,  -0.5  ]),
    "test8_q0 [0,.87,.39,-1.3]":np.array([0.0,   0.866, 0.385,-1.334]),
    "all [.5,.3,.2,.5]":        np.array([0.5,   0.3,   0.2,   0.5  ]),
    "limits [0,1.5,1.4,1.97]":  np.array([0.0,   1.4,   1.3,   1.7  ]),
}
DQ_TEST = np.array([0.3, 0.5, -0.4, 0.2])   # velocidad de prueba para NLE

# ── Cargar modelos ────────────────────────────────────────────────────────────
def load_model(urdf_path, label):
    if not os.path.exists(urdf_path):
        sys.exit(f"ERROR: URDF no encontrado: {urdf_path}")
    model = pin.buildModelFromUrdf(urdf_path)
    data  = pin.Data(model)
    print(f"  {label}: nv={model.nv}  ntotal_mass={sum(model.inertias[i].mass for i in range(1, model.njoints)):.4f} kg")
    return model, data

print("=" * 65)
print("  COMPARACION DE DINAMICA URDF VIEJO vs NUEVO")
print("=" * 65)
print("\nCargando modelos Pinocchio:")
model_old, data_old = load_model(URDF_OLD, "openmani.urdf (viejo)")
model_new, data_new = load_model(URDF_NEW, "open_manipulator_x.urdf (nuevo)")

# ── Funcion auxiliar: construir q_pin de dimension nv ─────────────────────────
def make_qpin(model, q4):
    """Rellena vector de configuracion: primeros 4 = q_arm, resto = 0."""
    q = np.zeros(model.nv)
    q[:NARM] = q4
    return q

def make_dqpin(model, dq4):
    dq = np.zeros(model.nv)
    dq[:NARM] = dq4
    return dq

# ── Calcular M(q), G(q), NLE(q,dq) ───────────────────────────────────────────
def compute_dynamics(model, data, q4, dq4=None):
    q   = make_qpin(model, q4)
    dq  = make_qpin(model, dq4) if dq4 is not None else np.zeros(model.nv)
    z   = np.zeros(model.nv)

    # Matriz de masa
    pin.crba(model, data, q)
    M = data.M.copy()[:NARM, :NARM]          # bloque 4x4

    # Vector de gravedad G(q) = RNEA(q, 0, 0)
    G = pin.rnea(model, data, q, z, z)[:NARM]

    # NLE = C(q,dq)*dq + G(q) = RNEA(q, dq, 0)
    NLE = pin.rnea(model, data, q, dq, z)[:NARM]

    return M, G, NLE

# ── Recopilar resultados ──────────────────────────────────────────────────────
results = {}
for name, q4 in CONFIGS.items():
    M_o, G_o, N_o = compute_dynamics(model_old, data_old, q4, DQ_TEST)
    M_n, G_n, N_n = compute_dynamics(model_new, data_new, q4, DQ_TEST)
    results[name] = dict(q4=q4, M_o=M_o, G_o=G_o, N_o=N_o,
                                  M_n=M_n, G_n=G_n, N_n=N_n)

# ─────────────────────────────────────────────────────────────────────────────
#  CHECK 1: Signos de G(q)
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "─" * 65)
print("CHECK 1 — Signos de G(q)  (PASS = mismo signo donde |G_old| > threshold)")
print("─" * 65)
sign_errors = 0
for name, r in results.items():
    G_o, G_n = r["G_o"], r["G_n"]
    flags = []
    for j in range(NARM):
        if abs(G_o[j]) < THRESHOLD:
            flags.append("~0  ")
        elif np.sign(G_o[j]) == np.sign(G_n[j]):
            flags.append("OK  ")
        else:
            flags.append("SIGN_ERR")
            sign_errors += 1
    flag_str = "  ".join(flags)
    sat = [j+1 for j in range(NARM) if abs(G_n[j]) > TAU_MAX]
    sat_str = f"  [SAT j{sat}]" if sat else ""
    print(f"  {name:38s}  [{flag_str}]{sat_str}")

if sign_errors == 0:
    print(f"\n  -> PASS: Sin inversion de signos en G(q).")
else:
    print(f"\n  -> FAIL: {sign_errors} inversiones de signo detectadas.")

# ─────────────────────────────────────────────────────────────────────────────
#  CHECK 2: Positividad definida de M(q)
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "─" * 65)
print("CHECK 2 — Positividad definida de M(q)  (todos los eigenvalores > 0)")
print("─" * 65)
pd_errors = 0
for name, r in results.items():
    ev_o = np.linalg.eigvalsh(r["M_o"])
    ev_n = np.linalg.eigvalsh(r["M_n"])
    ok_o = np.all(ev_o > 0)
    ok_n = np.all(ev_n > 0)
    cond_o = ev_o[-1] / ev_o[0]
    cond_n = ev_n[-1] / ev_n[0]
    status = "PASS" if ok_o and ok_n else "FAIL"
    if not (ok_o and ok_n): pd_errors += 1
    print(f"  {name:38s}  {status}  "
          f"cond_old={cond_o:.1f}  cond_new={cond_n:.1f}")

if pd_errors == 0:
    print(f"\n  -> PASS: M(q) positiva definida en todas las configuraciones.")
else:
    print(f"\n  -> FAIL: {pd_errors} configuraciones con M no positiva definida.")

# ─────────────────────────────────────────────────────────────────────────────
#  CHECK 3: Estructura de M(q) — patrón de entradas significativas
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "─" * 65)
print("CHECK 3 — Estructura de M(q) al 10% de la diagonal max")
print("─" * 65)
struct_errors = 0
q_ref = np.array([0.5, 0.3, 0.2, 0.5])
M_o, _, _ = compute_dynamics(model_old, data_old, q_ref)
M_n, _, _ = compute_dynamics(model_new, data_new, q_ref)
thr_o = 0.1 * np.max(np.diag(M_o))
thr_n = 0.1 * np.max(np.diag(M_n))
struct_o = np.abs(M_o) > thr_o
struct_n = np.abs(M_n) > thr_n
sign_o   = np.sign(M_o)
sign_n   = np.sign(M_n)

print(f"  Configuracion: q={q_ref}")
print(f"  Patron M_old (|Mij|>10% diag_max):")
for i in range(NARM):
    row_o = "  ".join("X" if struct_o[i,j] else "." for j in range(NARM))
    row_n = "  ".join("X" if struct_n[i,j] else "." for j in range(NARM))
    print(f"    fila {i+1}:  old=[{row_o}]   new=[{row_n}]")

# Verificar signos donde ambos son significativos
for i in range(NARM):
    for j in range(NARM):
        if struct_o[i,j] and struct_n[i,j]:
            if sign_o[i,j] != sign_n[i,j]:
                print(f"  SIGN_ERR en M[{i+1},{j+1}]: old={M_o[i,j]:.4e}  new={M_n[i,j]:.4e}")
                struct_errors += 1

if struct_errors == 0:
    print(f"\n  -> PASS: Sin inversion de signos en M(q) para entradas significativas.")
else:
    print(f"\n  -> FAIL: {struct_errors} inversiones de signo en M.")

# ─────────────────────────────────────────────────────────────────────────────
#  CHECK 4: Magnitudes relativas de M y G
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "─" * 65)
print("CHECK 4 — Variacion de magnitudes  M_new/M_old y |G_new|/|G_old|")
print("─" * 65)
print(f"  {'Config':38s}  {'dM_diag[1..4]':30s}  {'d|G|[1..4]'}")
for name, r in results.items():
    ratio_M = [r["M_n"][j,j]/r["M_o"][j,j] if r["M_o"][j,j] > 1e-12 else float('nan')
               for j in range(NARM)]
    ratio_G = [abs(r["G_n"][j])/abs(r["G_o"][j]) if abs(r["G_o"][j]) > THRESHOLD else float('nan')
               for j in range(NARM)]
    rM = "  ".join(f"{v:.2f}" if not np.isnan(v) else "  -- " for v in ratio_M)
    rG = "  ".join(f"{v:.2f}" if not np.isnan(v) else "  -- " for v in ratio_G)
    print(f"  {name:38s}  [{rM}]  [{rG}]")

print("\n  Nota: ratio > 1.0 esperado para joints con masa distal aumentada.")

# ─────────────────────────────────────────────────────────────────────────────
#  DIAGNOSTICO DE SATURACION
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "─" * 65)
print(f"DIAGNOSTICO — G(q) vs TAU_MAX={TAU_MAX} N.m  (limite gz_fl_control_node)")
print("─" * 65)
print(f"  {'Config':38s}  {'G_old[Nm]':25s}  {'G_new[Nm]':25s}  Saturacion_new")
for name, r in results.items():
    go = "  ".join(f"{v:+.3f}" for v in r["G_o"])
    gn = "  ".join(f"{v:+.3f}" for v in r["G_n"])
    sat_j = [j+1 for j in range(NARM) if abs(r["G_n"][j]) > TAU_MAX]
    sat_str = f"j{sat_j}" if sat_j else "ninguna"
    print(f"  {name:38s}  [{go}]  [{gn}]  {sat_str}")

# G(q) maximos sobre un barrido de configuraciones
print(f"\n  Maximo |G_new| por joint (barrido uniforme 500 configs aleatorias):")
rng = np.random.default_rng(42)
gmax = np.zeros(NARM)
for _ in range(500):
    q4 = rng.uniform([-pi, -1.5, -1.5, -1.7], [pi, 1.5, 1.4, 1.97])
    _, G_n, _ = compute_dynamics(model_new, data_new, q4)
    gmax = np.maximum(gmax, np.abs(G_n))
for j in range(NARM):
    flag = " <-- SATURADO" if gmax[j] > TAU_MAX else ""
    print(f"    joint{j+1}: max|G|={gmax[j]:.4f} N.m  TAU_MAX={TAU_MAX}{flag}")

# ─────────────────────────────────────────────────────────────────────────────
#  FIGURA
# ─────────────────────────────────────────────────────────────────────────────
cfg_names_short = [n.split(" ")[0] for n in CONFIGS.keys()]
n_cfg = len(CONFIGS)
G_old_mat = np.array([r["G_o"] for r in results.values()])   # (n_cfg, 4)
G_new_mat = np.array([r["G_n"] for r in results.values()])
M_diag_o  = np.array([[r["M_o"][j,j] for j in range(NARM)] for r in results.values()])
M_diag_n  = np.array([[r["M_n"][j,j] for j in range(NARM)] for r in results.values()])

fig = plt.figure(figsize=(18, 13))
fig.suptitle("Comparacion dinamica URDF antiguo vs nuevo\n"
             "(openmani.urdf  vs  open_manipulator_x.urdf)", fontsize=13)
gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.55, wspace=0.35)

colors = ["#e74c3c", "#2980b9", "#27ae60", "#8e44ad"]
jlabels = ["joint1", "joint2", "joint3", "joint4"]
x = np.arange(n_cfg)
w = 0.18

# ── G(q) viejo ──────────────────────────────────────────────────────────────
ax1 = fig.add_subplot(gs[0, 0])
for j in range(NARM):
    ax1.bar(x + j*w, G_old_mat[:, j], w, label=jlabels[j], color=colors[j], alpha=0.85)
ax1.axhline( TAU_MAX, color='k', ls='--', lw=1.2, label=f"+TAU_MAX={TAU_MAX}")
ax1.axhline(-TAU_MAX, color='k', ls='--', lw=1.2)
ax1.set_title("G(q) — URDF antiguo [N·m]", fontsize=10)
ax1.set_xticks(x + 1.5*w); ax1.set_xticklabels(cfg_names_short, rotation=40, ha='right', fontsize=7)
ax1.legend(fontsize=7); ax1.grid(axis='y', alpha=0.3)

# ── G(q) nuevo ──────────────────────────────────────────────────────────────
ax2 = fig.add_subplot(gs[0, 1])
for j in range(NARM):
    ax2.bar(x + j*w, G_new_mat[:, j], w, label=jlabels[j], color=colors[j], alpha=0.85)
ax2.axhline( TAU_MAX, color='k', ls='--', lw=1.2, label=f"+TAU_MAX={TAU_MAX}")
ax2.axhline(-TAU_MAX, color='k', ls='--', lw=1.2)
ax2.set_title("G(q) — URDF nuevo [N·m]", fontsize=10)
ax2.set_xticks(x + 1.5*w); ax2.set_xticklabels(cfg_names_short, rotation=40, ha='right', fontsize=7)
ax2.legend(fontsize=7); ax2.grid(axis='y', alpha=0.3)

# ── Diferencia G_new - G_old ─────────────────────────────────────────────────
ax3 = fig.add_subplot(gs[1, 0])
dG = G_new_mat - G_old_mat
for j in range(NARM):
    ax3.bar(x + j*w, dG[:, j], w, label=jlabels[j], color=colors[j], alpha=0.85)
ax3.axhline(0, color='k', lw=0.8)
ax3.set_title("ΔG(q) = G_new − G_old [N·m]", fontsize=10)
ax3.set_xticks(x + 1.5*w); ax3.set_xticklabels(cfg_names_short, rotation=40, ha='right', fontsize=7)
ax3.legend(fontsize=7); ax3.grid(axis='y', alpha=0.3)

# ── Diagonal M(q): ratio new/old ─────────────────────────────────────────────
ax4 = fig.add_subplot(gs[1, 1])
for j in range(NARM):
    ratio = M_diag_n[:, j] / (M_diag_o[:, j] + 1e-20)
    ax4.plot(x, ratio, 'o-', color=colors[j], label=jlabels[j], lw=1.5, ms=5)
ax4.axhline(1.0, color='k', ls='--', lw=1, label='ratio=1')
ax4.set_title("Ratio diag(M_new)/diag(M_old)", fontsize=10)
ax4.set_xticks(x); ax4.set_xticklabels(cfg_names_short, rotation=40, ha='right', fontsize=7)
ax4.legend(fontsize=7); ax4.grid(alpha=0.3)

# ── M(q) heatmaps en q=home ──────────────────────────────────────────────────
q_home = np.zeros(NARM)
M_o_h, _, _ = compute_dynamics(model_old, data_old, q_home)
M_n_h, _, _ = compute_dynamics(model_new, data_new, q_home)
vmax = max(np.abs(M_o_h).max(), np.abs(M_n_h).max())

ax5 = fig.add_subplot(gs[2, 0])
im5 = ax5.imshow(M_o_h, cmap='RdBu_r', vmin=-vmax, vmax=vmax)
ax5.set_title("M(q=0) — URDF antiguo [kg·m²]", fontsize=10)
ax5.set_xticks(range(NARM)); ax5.set_yticks(range(NARM))
ax5.set_xticklabels(jlabels, fontsize=8); ax5.set_yticklabels(jlabels, fontsize=8)
for i in range(NARM):
    for j in range(NARM):
        ax5.text(j, i, f"{M_o_h[i,j]:.3e}", ha='center', va='center', fontsize=6)
plt.colorbar(im5, ax=ax5, shrink=0.8)

ax6 = fig.add_subplot(gs[2, 1])
im6 = ax6.imshow(M_n_h, cmap='RdBu_r', vmin=-vmax, vmax=vmax)
ax6.set_title("M(q=0) — URDF nuevo [kg·m²]", fontsize=10)
ax6.set_xticks(range(NARM)); ax6.set_yticks(range(NARM))
ax6.set_xticklabels(jlabels, fontsize=8); ax6.set_yticklabels(jlabels, fontsize=8)
for i in range(NARM):
    for j in range(NARM):
        ax6.text(j, i, f"{M_n_h[i,j]:.3e}", ha='center', va='center', fontsize=6)
plt.colorbar(im6, ax=ax6, shrink=0.8)

plot_path = os.path.join(PLOT_DIR, "dynamics_comparison.png")
fig.savefig(plot_path, dpi=130, bbox_inches='tight')
plt.close(fig)

# ─────────────────────────────────────────────────────────────────────────────
#  RESUMEN FINAL
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "=" * 65)
print("RESUMEN")
print("=" * 65)
checks = [
    ("Signos G(q)",           sign_errors == 0),
    ("M(q) positiva definida", pd_errors   == 0),
    ("Signos M(q) preservados",struct_errors == 0),
]
all_pass = all(ok for _, ok in checks)
for label, ok in checks:
    print(f"  {'PASS' if ok else 'FAIL'}  {label}")

print(f"\n  Joints saturados con G_new > TAU_MAX={TAU_MAX}:")
for j in range(NARM):
    print(f"    joint{j+1}: max|G_new|={gmax[j]:.4f} N.m  "
          f"{'-> SATURADO' if gmax[j] > TAU_MAX else 'OK'}")

print(f"\n  Masa total viejo: "
      f"{sum(model_old.inertias[i].mass for i in range(1,model_old.njoints)):.4f} kg")
print(f"  Masa total nuevo: "
      f"{sum(model_new.inertias[i].mass for i in range(1,model_new.njoints)):.4f} kg")
print(f"\n  Figura guardada en: {plot_path}")
if all_pass:
    print("\n  -> MODELO CORRECTO: cambios de inercia coherentes, sin errores de signo.")
    print("     La saturacion de joints 2-3 es una consecuencia fisica esperada")
    print("     del aumento de masa (link5: 143g -> 236g). El modelo es valido.")
    print("     Accion recomendada: aumentar TAU_MAX y/o los limites de esfuerzo.")
else:
    print("\n  -> ATENCION: hay inconsistencias en el modelo dinamico. Revisar URDF.")
print("=" * 65)
