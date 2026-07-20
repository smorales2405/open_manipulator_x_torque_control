# Protocolo de identificación torque→corriente — OpenMANIPULATOR-X (XM430-W350-T)

Este protocolo permite identificar, **en un robot OpenMANIPULATOR-X nuevo**, los
parámetros del modelo torque→corriente de los 4 motores principales (IDs 11–14)
y dejarlos listos en `config/motorXM430W350T_params.yaml`:

```
I[i] = alpha[i]·tau[i] + Fv[i]·dq[i] + Fc[i]·tanh(dq[i]/eps) + I_offset[i]
```

| Parámetro | Significado | Unidades |
|---|---|---|
| `motor_alpha` | ganancia torque→corriente (α = 1/(kt_ef·Iu), Iu = 2.69 mA/tick) | ticks/N·m |
| `motor_Fv` | fricción viscosa | ticks/(rad/s) |
| `motor_Fc` | fricción de Coulomb | ticks |
| `motor_I_offset` | bias de corriente | ticks |
| `motor_epsilon_friction` | suavizado del tanh de Coulomb | rad/s |

**Pipeline completo** (2 tipos de ensayo + 3 scripts):

| Ensayo en el robot | Script de análisis | Produce |
|---|---|---|
| Barrido de velocidad constante (×4, uno por joint) — `friction_sweep.launch.py` | `identify_friction.py` | `Fv`, `Fc` por joint |
| Control FL dinámico con parámetros *bootstrap* — `hw_fl_control_node` | `identify_alpha.py` | `alpha`, `I_offset` |
| — | `assemble_motor_params.py` | YAML final ensamblado |

Cada script imprime un **VEREDICTO** (OK / OK con nota / REVISAR / RE-EJECUTAR)
con el comando concreto de re-ejecución cuando los datos no bastan. Código de
salida 2 = hay que repetir el ensayo.

---

## 1. Fundamento: un método por parámetro

**Regla de oro (verificación física):** la constante de torque efectiva
`kt_ef = 1/(alpha·Iu)` **no puede superar 1.78 N·m/A** (kt ideal del datasheet
XM430-W350: no se obtiene más torque por amperio que el motor ideal). Un α sano
queda en **208.5–286 ticks/N·m** (kt_ef entre 1.78 y ~1.30; en este robot:
α ≈ 225, kt_ef ≈ 1.65, es decir ~7 % de pérdidas del reductor).

Por qué cada parámetro exige SU método:

- **α e I_offset — modo corriente dinámico (lazo FL).** En el pasado se intentó
  identificar α con datos de modo posición y con la parte par (gravedad) del
  barrido: dio **α ≈ 153 → kt_ef = 2.43 N·m/A, físicamente IMPOSIBLE**. La causa:
  con un reductor 353.5:1, en régimen cuasi-estático parte de la carga de
  gravedad la sostiene la **fricción del engranaje (no-retroconducción)**, no la
  corriente del motor → la corriente medida queda por debajo de α·τ y α sale
  subestimada. Además el PID interno del modo posición sesga la corriente. En
  cambio, con el robot **en movimiento bajo control de corriente** (torque
  computado/FL), la relación corriente↔torque comandado es la del régimen en que
  los controladores realmente operan: **α = 224.8 → kt_ef = 1.65 ✓**.

- **Fv y Fc — barrido de velocidad constante + reversión.** A velocidad
  constante (ddq = 0) y comparando corrientes a la MISMA posición en ambos
  sentidos, la gravedad (par en q) se cancela EXACTAMENTE y queda la fricción
  (impar). No requiere URDF ni α, y elimina la colinealidad Fv↔Fc que arruina
  el ajuste con trayectorias suaves.

---

## 2. Requisitos previos

1. **Workspace compilado** (desde `/home/utec/open_manx_ws`):

   ```bash
   CMAKE_BUILD_PARALLEL_LEVEL=2 nice -n 15 colcon build --symlink-install \
     --executor sequential --packages-select open_manipulator_x_torque_control
   source install/setup.bash
   ```

2. **Robot montado y despejado**: base firmemente sujeta, sin objetos en el
   volumen de trabajo, gripper montado (las masas del URDF lo incluyen).
   Alimentación 12 V estable y U2D2/USB conectado (`/dev/ttyUSB0`).

3. **Latencia del FTDI a 1 ms** (necesario para lazos ≥100 Hz; repetir tras
   reconectar el USB):

   ```bash
   echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
   cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # debe imprimir 1
   ```

4. **Permisos del puerto**: usuario en el grupo `dialout`
   (`sudo usermod -aG dialout $USER` + re-login) o `sudo chmod 666 /dev/ttyUSB0`.

5. **Servos**: IDs 11–14, Protocol 2.0, baudrate 1 000 000 (configuración de
   fábrica del OpenMANIPULATOR-X).

6. **Python**: `numpy`, `scipy`, `matplotlib`, `PyYAML` (los trae ROS 2 Humble).
   No se necesita MATLAB ni Pinocchio para este protocolo.

7. **Seguridad**: el launch de fricción arranca con los flags en `false`
   (dry-run); el robot solo se mueve pasando `open_port:=true enable_torque:=true
   enable_current_commands:=true`. Todos los nodos se detienen ante corriente
   medida > `current_measured_peak` (313 ticks) y con `Ctrl+C` (torque off).
   Tenga siempre a mano el corte de alimentación.

> Los CSV se escriben en el **árbol fuente** del paquete
> (`data/diagnostics/sinusoidal/`, `data/lab4/real/act1/`), y los nodos leen
> `config/motorXM430W350T_params.yaml` **del árbol fuente al lanzarse**: editar
> ese YAML **no requiere recompilar**.

---

## 3. Paso 1 — Fricción (Fv, Fc): un barrido por articulación

El joint elegido entra en *velocity mode* y recorre magnitudes
0.05…0.50 rad/s en ambos sentidos, rebotando dentro de una banda segura
(±`vel_band` alrededor de su posición de reposo); los demás joints se mantienen
fijos en *position mode*. Duración ≈ 43 s por ensayo.

**3.1 Ejecutar los 4 barridos** (convención de logs: J1→21, J2→22, J3→23, J4→24):

```bash
# dry-run opcional (verifica parámetros sin mover el robot): omitir los flags true
ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py \
  friction_joint:=1 open_port:=true enable_torque:=true \
  enable_current_commands:=true log_id:=21

# repetir cambiando friction_joint y log_id:
#   friction_joint:=2 log_id:=22
#   friction_joint:=3 log_id:=23
#   friction_joint:=4 log_id:=24
```

**3.2 Analizar cada log** (el flag `--joint` verifica que el log corresponda a
la articulación esperada):

```bash
python3 src/Identification/identify_friction.py 21 --joint 1
python3 src/Identification/identify_friction.py 22 --joint 2
python3 src/Identification/identify_friction.py 23 --joint 3
python3 src/Identification/identify_friction.py 24 --joint 4
```

Cada corrida imprime la curva `I_fric(v)`, el ajuste `Fv·v + Fc·tanh(v/eps)`, la
gráfica en `plots/diagnostics/friction/` y un **VEREDICTO**:

| Veredicto | Significado | Acción |
|---|---|---|
| **OK** | resid ≤ 4 ticks | nada |
| **OK con nota** | el punto de v mínima es fricción de **arranque** (breakaway/Stribeck): real pero fuera del modelo Coulomb cinético → se **excluye** del ajuste oficial (los nodos compensan el régimen cinético) | nada — no repetir |
| **REVISAR** | resid 4–8 ticks, o ajuste bueno pero **barrido truncado** (faltan las velocidades altas para confirmar el plateau) | valor central utilizable; el script imprime el comando exacto para afinar |
| **RE-EJECUTAR** (exit 2) | resid > 8 ticks o < 3 velocidades útiles | repetir con el comando que imprime el script |

Robustez automática del análisis: los puntos con **menos de 2 bins de q
emparejados** se descartan del ajuste (frágiles, p.ej. un 0.20 rad/s con un
solo bin puede salir hasta negativo); el script estima del propio log la
**banda** y la **duración de segmento** realmente usadas, detecta **barridos
truncados** (`duration_s` corto) y adapta el consejo de re-ejecución para no
repetir recetas ya aplicadas.

**3.3 Cuándo y cómo repetir un barrido.** El caso típico es un joint con carga
de gravedad (J2) y datos dispersos a baja velocidad: pocos rebotes → poco
muestreo bidireccional. Repetir con banda angosta (más reversiones por segmento)
y/o segmentos más largos, usando un `log_id` NUEVO para no pisar el anterior:

```bash
ros2 launch open_manipulator_x_torque_control friction_sweep.launch.py \
  friction_joint:=2 open_port:=true enable_torque:=true \
  enable_current_commands:=true log_id:=25 vel_band:=0.3 vel_seg_duration:=8.0

python3 src/Identification/identify_friction.py 25 --joint 2
```

> `duration_s` es `auto` en el launch: se calcula como
> `t_settle + len(vel_list)·vel_seg_duration + 1 s`, de modo que alargar
> `vel_seg_duration` nunca trunca el barrido. Solo pásela explícita si quiere
> recortarlo a propósito. (Con `ros2 run` directo, el nodo avisa al arrancar si
> `duration_s` no alcanza para todo el `vel_list`.)

La escalera de re-ejecución que aplica el script (y que puede seguir a mano):

1. banda ancha (±0.5) → `vel_band:=0.3` (más reversiones a baja velocidad);
2. banda ya angosta → `vel_seg_duration:=8.0` (más muestreo por velocidad);
3. banda y segmentos ya ajustados y sigue disperso → el problema no es el
   barrido: revisar mecánica (cableado rozando, montaje, backlash), temperatura
   de motores y signos; puede intentarse `vel_seg_duration:=10.0` o aceptar el
   valor central (REVISAR).

*Referencia real de este robot:* J2 con `vel_band` 0.5 dio resid 10.6 (punto
0.10 rad/s → 37 ticks, espurio); re-grabado con `vel_band:=0.3` bajó a 5.84 con
la curva asentada en ~18 ticks, quedando solo el punto de arranque a 0.10 rad/s
(31.6 ticks) — veredicto *OK con nota*, Fc ≈ 18.7 confirmado.

Al terminar tendrá 4 fragmentos `data/identification/friction_J<j>_log<id>.yaml`
(anote qué log quedó como definitivo para cada joint).

---

## 4. Paso 2 — α e I_offset: ensayo FL con parámetros *bootstrap*

**4.1 Poner el YAML en modo bootstrap.** El ensayo DEBE correrse con la
conversión nominal y SIN compensación de fricción; si el nodo ya usa un α
identificado, la regresión queda anclada a ese modelo (circularidad) y el
resultado solo sirve como verificación. Edite `config/motorXM430W350T_params.yaml`
(guarde antes una copia si ya tenía valores) dejando:

```yaml
    #                              J1        J2        J3        J4
    motor_alpha:             [208.500,  208.500,  208.500,  208.500]   # nominal = 1/(1.7826·0.00269)
    motor_Fv:                [  0.000,    0.000,    0.000,    0.000]
    motor_Fc:                [  0.000,    0.000,    0.000,    0.000]
    motor_I_offset:          [  0.000,    0.000,    0.000,    0.000]
```

(el resto del archivo — `motor_epsilon_friction`, bloque de hardware — se deja
igual; no hace falta recompilar).

**4.2 Ejecutar el ensayo FL** (trayectoria articular act1: senos en J1–J3 a
1 rad/s, J4 constante — J2/J3 quedan bien excitados en torque):

```bash
# primera vez en un robot nuevo: ganancias a la mitad por seguridad
ros2 run open_manipulator_x_torque_control hw_fl_control_node \
  --ros-args -p gain_scale:=0.5 -p t_run:=30.0 -p log_id:=30
```

Con bootstrap el tracking será mediocre (stick-slip visible, sin compensación de
fricción): **es esperable y NO afecta la identificación** — la regresión usa el
torque comandado y la corriente medida, no el error de seguimiento. Si el robot
se mueve sin golpes ni paradas de emergencia, el log sirve.

**4.3 Analizar:**

```bash
python3 src/Identification/identify_alpha.py 30
```

El script ajusta por joint `I_meas = α·τ + Fv·dq + Fc·tanh(dq/eps) + I_offset`
(OLS completo en los joints bien excitados — típicamente J2/J3 —; en los débiles
fija α a la media de los identificables, mismo motor → mismo kt) y **verifica**:

- **Ventana física de kt**: α < 208.5 (kt_ef > 1.78) → **RE-EJECUTAR** con
  explicación (datos cuasi-estáticos / modo posición). α > 286 → revisar signos
  y URDF.
- **Bootstrap**: estima del propio log qué α/compensación usó el nodo
  (pendiente de `curr_cmd` vs `tau`); si no era nominal-sin-compensación,
  advierte circularidad.
- **Excitación** (std(τ), corr(τ,I)), **saturación** de corriente (descarta esas
  muestras) y **R²** por joint.

Resultado esperado en un XM430-W350 sano: α ≈ 210–240 ticks/N·m
(kt_ef ≈ 1.55–1.75), R² ≥ 0.85 en los joints excitados. Se genera el fragmento
`data/identification/alpha_fit_log30.yaml`.

**Cuándo repetir:** veredicto RE-EJECUTAR (kt imposible, sin excitación, log
corto) o saturación > 10 %. Ajustes típicos: subir `t_run` a 40–60 s (más
datos), `gain_scale:=0.7` si hubo mucha saturación con 1.0, verificar que la
trayectoria realmente se ejecutó (¿parada de emergencia a mitad?).

---

## 5. Paso 3 — Ensamblar el YAML final

```bash
python3 src/Identification/assemble_motor_params.py \
  --alpha-log 30 --friction-logs 21 25 23 24
```

(`--friction-logs` en orden **J1 J2 J3 J4**, usando el log definitivo de cada
joint — en el ejemplo, el 25 re-grabado reemplaza al 22 de J2.)

El script valida los veredictos de todos los fragmentos (aborta si alguno quedó
en RE-EJECUTAR), re-verifica la ventana de kt, copia el bloque de hardware del
YAML oficial actual y escribe `config/motorXM430W350T_params_identified.yaml`
con una tabla comparativa actual↔nuevo. Revíselo y actívelo:

```bash
cp config/motorXM430W350T_params_identified.yaml config/motorXM430W350T_params.yaml
```

---

## 6. Paso 4 — Validación en el robot

Repita el ensayo FL, ahora con el modelo completo y ganancias nominales:

```bash
ros2 run open_manipulator_x_torque_control hw_fl_control_node \
  --ros-args -p gain_scale:=1.0 -p t_run:=20.0 -p log_id:=31
```

y grafique con `src/Feedback Linearization/MATLAB/plots_FL_control.m`
(`log_id = 31`). Referencia de un robot sano con este modelo (este robot,
kp = [400 400 600 3000], 200 Hz): errores RMS articulares del orden de
**0.005–0.03 rad** por joint tras la rampa, sin saturación de torque y sin
stick-slip visible en las inversiones. Si el tracking empeoró respecto al
bootstrap, algo está mal: revise la tabla comparativa del ensamblado y la
sección de problemas.

Opcionalmente, re-ejecute `identify_alpha.py 31` sobre este log: debe reportar
la advertencia de circularidad (correcto: ya no es bootstrap) y un α consistente
(±10 %) con el identificado — sirve como verificación de regresión.

---

## 7. Solución de problemas

| Síntoma | Causa probable | Acción |
|---|---|---|
| `kt_ef > 1.78` (α < 208.5) en identify_alpha | datos cuasi-estáticos o de modo posición | ensayo FL dinámico (Paso 2); no usar `identify_motor_model.py` para α |
| Advertencia "NO se corrió con parámetros bootstrap" | YAML con α/fricción ya identificados | poner bloque bootstrap (4.1) y repetir |
| `No se pudo abrir /dev/ttyUSB0` | otro proceso usa el puerto / permisos | cerrar otros nodos (ros2_control, otro launch); grupo dialout |
| Parada `Corriente insegura J*` | colisión, trayectoria fuera de rango, cableado | despejar el robot; verificar `joint_zero_tick` (¿robot ensamblado en la pose cero?) |
| Warnings de overrun del lazo (>Ts) | latency_timer ≠ 1 ms | requisito 2.3; o bajar `loop_rate_hz` a 100 |
| J2 con resid alto en fricción | rebote pobre a baja velocidad | re-grabar con `vel_band:=0.3` (± más `vel_seg_duration:=8.0`) |
| Barrido sin velocidades altas ("BARRIDO TRUNCADO") | `duration_s` explícita menor que `t_settle + n_vel·vel_seg_duration` | usar `duration_s` auto del launch (o pasarla ≥ la suma); el nodo también lo advierte al arrancar |
| Dispersión persiste con banda 0.3 y segmentos 8 s | causa mecánica, no de configuración | revisar cableado/montaje/backlash, temperatura; `vel_seg_duration:=10.0` o aceptar valor central |
| Puntos `I_fric` negativos | `current_sign`/`encoder_sign` invertidos | corregir signos en el YAML y repetir el barrido |
| El joint en velocity no se mueve / se sale de banda | `A_pos` fuera de límites o banda excesiva | revisar `hw_friction_sweep_params.yaml` (A_pos, vel_band vs joint_lower/upper) |
| CSV no aparece | se busca en el árbol instalado | los datos van al árbol FUENTE: `data/…` del repositorio |

---

## 8. Archivos del pipeline (referencia)

```
src/Identification/
├── Ident_OpenManX_XM430W350T_procedure.md   ← este protocolo
├── hw_sinusoidal_torque_node.cpp    nodo de excitación (current/position/velocity)
├── identify_friction.py             Fv, Fc   (barrido de velocidad + reversión)
├── identify_alpha.py                alpha, I_offset (modo corriente dinámico FL)
├── assemble_motor_params.py         ensambla el YAML final
├── identify_motor_model.py          referencia modo posición (NO usar su α)
└── MATLAB/sinusoidal_torque_analysis.m

launch/friction_sweep.launch.py      barrido de fricción por articulación
config/hw_friction_sweep_params.yaml parámetros del barrido (vel_list, banda…)
config/hw_sinusoidal_torque_params.yaml  parámetros generales del nodo de excitación
config/motorXM430W350T_params.yaml   ← DESTINO FINAL del modelo

data/diagnostics/sinusoidal/hw_sin_torque_<id>.csv   logs de barridos
data/lab4/real/act1/hw_fl_data_<id>.csv              logs del ensayo FL (alpha)
data/identification/*.yaml                           fragmentos (entrada del ensamblador)
config/motorXM430W350T_params_identified.yaml        YAML ensamblado (revisar y copiar)
```
