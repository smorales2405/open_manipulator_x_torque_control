#!/usr/bin/env python3
"""
assemble_motor_params.py
Ensambla el modelo torque→corriente FINAL del OpenMANIPULATOR-X combinando
cada parámetro de su método de identificación MÁS ADECUADO:

    · alpha, I_offset → identify_alpha.py     (modo corriente dinámico, FL)
    · Fv, Fc          → identify_friction.py  (barrido de velocidad + reversión,
                                               un log por articulación)

y escribe config/motorXM430W350T_params_identified.yaml con el MISMO formato
que config/motorXM430W350T_params.yaml (incluyendo el bloque de configuración
de hardware, copiado del archivo actual). El archivo generado NO reemplaza al
oficial: revíselo y cópielo a mano cuando esté conforme:

    cp config/motorXM430W350T_params_identified.yaml config/motorXM430W350T_params.yaml

Entradas (fragmentos generados en data/identification/):
    alpha_fit_log<A>.yaml            (identify_alpha.py)
    friction_J1_log<F1>.yaml ... friction_J4_log<F4>.yaml  (identify_friction.py)

Uso:
    python3 src/Identification/assemble_motor_params.py \\
        --alpha-log 30 --friction-logs 21 25 23 24

    (--friction-logs recibe 4 IDs en orden J1 J2 J3 J4; pueden repetirse logs
     re-grabados, p.ej. el 25 que reemplazó al 22 para J2)

Código de salida: 0 = ensamblado escrito, 2 = fragmentos faltantes/inválidos.
Protocolo completo: src/Identification/Ident_OpenManX_XM430W350T_procedure.md
"""

import os
import sys
import argparse
import datetime
import yaml

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
FRAG_DIR = os.path.join(PKG_DIR, "data", "identification")
NJ = 4

# Ventana física de alpha (ver identify_alpha.py)
CURRENT_UNIT_A = 2.69e-3
KT_IDEAL       = 1.7826
ALPHA_MIN      = 0.99 / (KT_IDEAL * CURRENT_UNIT_A)

# Bloque de hardware por defecto (si no existe motorXM430W350T_params.yaml)
HW_DEFAULTS = {
    "joint_zero_tick":        [2048, 2048, 2048, 2048],
    "encoder_sign":           [1.0, 1.0, 1.0, 1.0],
    "current_sign":           [1.0, 1.0, 1.0, 1.0],
    "joint_lower":            [-2.356194, -1.919862, -1.919862, -1.8],
    "joint_upper":            [2.356194, 1.745329, 1.570796, 2.1],
    "current_limit_register": 350,
    "current_cmd_limit":      [257, 257, 257, 257],
    "current_measured_peak":  313,
}

ap = argparse.ArgumentParser(description="Ensambla motorXM430W350T_params "
                             "desde los fragmentos de identificación.")
ap.add_argument("--alpha-log", required=True,
                help="ID del log de identify_alpha.py (alpha_fit_log<ID>.yaml)")
ap.add_argument("--friction-logs", required=True, nargs=4, metavar=("J1", "J2", "J3", "J4"),
                help="4 IDs de logs de identify_friction.py, en orden J1 J2 J3 J4")
ap.add_argument("--eps", type=float, default=0.05,
                help="motor_epsilon_friction del YAML final (def. 0.05)")
ap.add_argument("--out", default=os.path.join(PKG_DIR, "config",
                "motorXM430W350T_params_identified.yaml"),
                help="ruta del YAML ensamblado")
args = ap.parse_args()

problems, notes = [], []

# ── Fragmento de alpha ───────────────────────────────────────────────────────
alpha_path = os.path.join(FRAG_DIR, f"alpha_fit_log{args.alpha_log}.yaml")
if not os.path.isfile(alpha_path):
    print(f"ERROR: no existe {alpha_path}")
    print("→ Acción: ejecute primero  python3 src/Identification/identify_alpha.py "
          f"{args.alpha_log}")
    sys.exit(2)
with open(alpha_path) as f:
    afrag = yaml.safe_load(f)

alpha = [float(x) for x in afrag["alpha"]]
ioff  = [float(x) for x in afrag["I_offset"]]
kt_ef = float(afrag.get("kt_effective", 0.0))

if any(a < ALPHA_MIN for a in alpha):
    problems.append(
        f"alpha = {alpha} contiene valores < {ALPHA_MIN:.1f} ticks/N·m (kt > ideal):\n"
        "  físicamente imposible — repita el ensayo de alpha (Paso 2 del protocolo).")
if str(afrag.get("bootstrap_run", "True")) == "False":
    notes.append("el ensayo de alpha NO fue bootstrap (circularidad): el valor queda "
                 "anclado al modelo previo. Válido solo como verificación.")
if str(afrag.get("verdict", "OK")) not in ("OK", "REVISAR"):
    problems.append(f"veredicto del fragmento de alpha = {afrag.get('verdict')!r}: "
                    "repita ese ensayo antes de ensamblar.")

# ── Fragmentos de fricción (uno por joint) ───────────────────────────────────
Fv, Fc, fr_src = [0.0] * NJ, [0.0] * NJ, [""] * NJ
for j in range(1, NJ + 1):
    log = args.friction_logs[j - 1]
    p = os.path.join(FRAG_DIR, f"friction_J{j}_log{log}.yaml")
    if not os.path.isfile(p):
        import glob
        avail = sorted(os.path.basename(x)
                       for x in glob.glob(os.path.join(FRAG_DIR, f"friction_J{j}_*.yaml")))
        print(f"ERROR: no existe {p}")
        print(f"  Fragmentos disponibles para J{j}: {avail if avail else 'ninguno'}")
        print(f"→ Acción: ejecute  python3 src/Identification/identify_friction.py {log} "
              f"--joint {j}\n  (o corrija el orden/IDs de --friction-logs: J1 J2 J3 J4)")
        sys.exit(2)
    with open(p) as f:
        frag = yaml.safe_load(f)
    if int(frag["joint"]) != j:
        print(f"ERROR: {os.path.basename(p)} es del joint {frag['joint']}, no de J{j}.")
        sys.exit(2)
    Fv[j - 1] = float(frag["Fv"])
    Fc[j - 1] = float(frag["Fc"])
    fr_src[j - 1] = f"log {log}"
    v = str(frag.get("verdict", "OK"))
    if v == "RE-EJECUTAR":
        problems.append(f"fricción de J{j} (log {log}) tiene veredicto RE-EJECUTAR: "
                        "repita ese barrido antes de ensamblar.")
    elif v == "REVISAR":
        notes.append(f"fricción de J{j} (log {log}) con veredicto REVISAR "
                     f"(resid={float(frag.get('resid_rms', 0)):.1f} ticks): valor central "
                     "utilizable; re-grabar con vel_band:=0.3 lo afinaría.")
    elif v == "OK_NOTA":
        notes.append(f"fricción de J{j} (log {log}): residuo dominado por breakaway a "
                     "v mínima (normal, no requiere acción).")

if problems:
    print("=" * 78)
    print(" ENSAMBLADO ABORTADO — corrija antes de continuar:")
    for p_ in problems:
        print(" ✗ " + p_)
    print("=" * 78)
    sys.exit(2)

# ── Bloque de hardware: copiar del YAML oficial actual si existe ─────────────
official = os.path.join(PKG_DIR, "config", "motorXM430W350T_params.yaml")
hw = dict(HW_DEFAULTS)
old = None
if os.path.isfile(official):
    with open(official) as f:
        doc = yaml.safe_load(f)
    params = doc.get("/**", {}).get("ros__parameters", {})
    for k in HW_DEFAULTS:
        if k in params:
            hw[k] = params[k]
    old = params

# ── Escribir el YAML ensamblado ──────────────────────────────────────────────
fmt = lambda v, w=8, d=3: "[" + ", ".join(f"{float(x):{w}.{d}f}" for x in v) + "]"
fmt_i = lambda v: "[" + ", ".join(str(int(x)) for x in v) + "]"
today = datetime.date.today().isoformat()

lines = []
lines.append("/**:")
lines.append("  ros__parameters:")
lines.append("")
lines.append("    # ════════════════════════════════════════════════════════════════════════")
lines.append("    #  Modelo torque→corriente Dynamixel XM430-W350-T (OpenMANIPULATOR-X)")
lines.append(f"    #  ENSAMBLADO por assemble_motor_params.py — {today}")
lines.append("    #")
lines.append("    #  Modelo (por articulación):")
lines.append("    #    I[i] = alpha[i]·tau[i] + Fv[i]·dq[i] + Fc[i]·tanh(dq[i]/eps) + I_offset[i]")
lines.append("    #")
lines.append("    #  Cada parámetro proviene del método MÁS ADECUADO para identificarlo:")
lines.append(f"    #   · alpha, I_offset → modo CORRIENTE dinámico (identify_alpha.py, "
             f"log {args.alpha_log}).")
lines.append(f"    #                       kt_efectivo ≈ {kt_ef:.2f} N·m/A (ideal datasheet 1.78).")
lines.append("    #   · Fv, Fc          → barrido de VELOCIDAD CONSTANTE + reversión")
lines.append("    #                       emparejada por posición (identify_friction.py,")
lines.append(f"    #                       logs J1..J4: "
             + ", ".join(fr_src) + ").")
lines.append("    #")
lines.append("    #  Protocolo: src/Identification/Ident_OpenManX_XM430W350T_procedure.md")
lines.append("    #")
lines.append("    #  Unidades:")
lines.append("    #    motor_alpha            [ticks/N·m]")
lines.append("    #    motor_Fv               [ticks/(rad/s)]")
lines.append("    #    motor_Fc               [ticks]")
lines.append("    #    motor_I_offset         [ticks]")
lines.append("    #    motor_epsilon_friction [rad/s]")
lines.append("    # ════════════════════════════════════════════════════════════════════════")
lines.append("    #                              J1        J2        J3        J4")
lines.append(f"    motor_alpha:             {fmt(alpha)}   # modo corriente")
lines.append(f"    motor_Fv:                {fmt(Fv)}   # barrido velocidad")
lines.append(f"    motor_Fc:                {fmt(Fc)}   # barrido velocidad")
lines.append(f"    motor_I_offset:          {fmt(ioff)}   # modo corriente")
lines.append("    # Suavizado del tanh de Coulomb. Los nodos hw (hw_fl, hw_io, hw_tvlqr)")
lines.append("    # alimentan el tanh con la velocidad DESEADA (sin ruido): eps pequeno es")
lines.append("    # seguro y acorta la zona muerta en las inversiones de direccion")
lines.append("    # (micro-plateaus). Si un nodo usa la velocidad MEDIDA aqui, subir hacia")
lines.append("    # 0.30 (con dq medida el tanh conmutaria con el ruido cerca de velocidad")
lines.append("    # cero).")
lines.append("    # Solo afecta la COMPENSACION, no los valores identificados de Fc/Fv.")
lines.append(f"    motor_epsilon_friction:    {args.eps:.4f}")
lines.append("")
lines.append("    # ── Configuración de hardware (señales, encoder, límites) ────────────────")
lines.append("    # joint_zero_tick  : posición cero del encoder [ticks] por joint")
lines.append("    # encoder_sign     : dirección positiva del encoder (+1 o -1)")
lines.append("    # current_sign     : polaridad de la corriente respecto al encoder (+1 o -1)")
lines.append("    # joint_lower/upper: límites articulares [rad]")
lines.append("    # current_limit_register : registro Current Limit de Dynamixel (addr 38)")
lines.append("    # current_cmd_limit      : corriente comandada máxima por joint [ticks]")
lines.append("    # current_measured_peak  : umbral de parada de emergencia [ticks]")
lines.append("    # NOTA: tau_max NO se define aquí; cada nodo lo fija internamente (1.2 N·m).")
lines.append(f"    joint_zero_tick:          {fmt_i(hw['joint_zero_tick'])}")
lines.append(f"    encoder_sign:             {fmt(hw['encoder_sign'], 4, 1)}")
lines.append(f"    current_sign:             {fmt(hw['current_sign'], 4, 1)}")
lines.append(f"    joint_lower:              {fmt(hw['joint_lower'], 9, 6)}")
lines.append(f"    joint_upper:              {fmt(hw['joint_upper'], 9, 6)}")
lines.append(f"    current_limit_register:   {int(hw['current_limit_register'])}")
lines.append(f"    current_cmd_limit:        {fmt_i(hw['current_cmd_limit'])}")
lines.append(f"    current_measured_peak:    {int(hw['current_measured_peak'])}")
lines.append("")

with open(args.out, "w") as f:
    f.write("\n".join(lines))

# ── Reporte y comparación con el YAML oficial actual ────────────────────────
print("=" * 78)
print(f" Modelo ensamblado → {args.out}")
print("=" * 78)
rows = [("motor_alpha", alpha), ("motor_Fv", Fv), ("motor_Fc", Fc),
        ("motor_I_offset", ioff)]
if old:
    print(f" {'parámetro':<16} {'':>4} {'actual (oficial)':>18} {'nuevo (ensamblado)':>20}")
    print("-" * 78)
    for name, new in rows:
        cur = old.get(name, ["?"] * NJ)
        for j in range(NJ):
            c = f"{float(cur[j]):10.3f}" if cur[j] != "?" else "         ?"
            print(f" {name if j == 0 else '':<16} J{j+1:<3} {c:>18} {float(new[j]):20.3f}")
    print("-" * 78)
else:
    for name, new in rows:
        print(f" {name:<16} {fmt(new)}")
print(f" kt_efectivo = {kt_ef:.3f} N·m/A   eps = {args.eps:.3f} rad/s")
for n in notes:
    print(" ! " + n)
print()
print(" Revise el archivo y, cuando esté conforme, actívelo con:")
print(f"   cp '{args.out}' \\")
print(f"      '{official}'")
print(" (los nodos hw leen el config del árbol fuente: no hace falta recompilar)")
print("=" * 78)
