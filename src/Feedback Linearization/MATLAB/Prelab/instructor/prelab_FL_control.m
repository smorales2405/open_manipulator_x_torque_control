%% prelab_FL_control.m
% Prelab — Control por Feedback Linearization (Computed Torque) articular
% OpenMANIPULATOR-X — simulacion MATLAB equivalente al nodo gz_fl_control_node.cpp
%
% Replica la estructura del nodo (misma trayectoria, ganancias, lazo a 200 Hz
% y saturacion), sustituyendo Gazebo por una planta integrada con OMDyn
% (dinamica rigida generada del URDF) mas friccion articular viscosa + Coulomb:
%
%   Ley de control (identica al nodo, con feedforward de friccion SIEMPRE
%   activo — equivale a friction_ffwd:=true en gz_fl_control_node):
%     v       = ddq_des + Kp.*(q_des - q) + Kd.*(dq_des - dq)
%     tau     = M(q)*v + phib(q,dq)              <- [M,phib] = OMDyn(q,dq)
%               + b.*dq + fc.*tanh(dq_des/eps)
%     tau_sat = clamp(tau, -TAU_MAX, TAU_MAX)
%
%   Planta simulada (el papel que cumple Gazebo frente al nodo):
%     M(q)*ddq + phib(q,dq) + b.*dq + fc.*tanh(dq/eps) = tau_sat
%     integrada con RK4 a Ts/N_SUB, con ZOH del torque durante Ts = 1/200 s.
%
%   Referencia: sinusoide articular (w = 1 rad/s), identica al nodo:
%     q1_des =  (pi/4)*sin(t)
%     q2_des = -0.5 + 0.5*sin(t)
%     q3_des =  0.3 - 0.5*sin(t)
%     q4_des =  pi/4
%
% SIN RAMPA QUINTICA: en Gazebo/hardware la rampa existe porque el robot
% arranca en una pose arbitraria y hay que empalmar sin salto de referencia.
% En simulacion la condicion inicial se elige: el robot parte YA en
% q_des(0) = [0; -0.5; 0.3; pi/4] y en reposo, y no se cae porque la FL
% compensa la gravedad desde el primer tick. Como la sinusoide arranca con
% velocidad no nula (dq_des(0) = [pi/4; 0.5; -0.5; 0] rad/s), hay un
% transitorio breve al inicio que el controlador absorbe; las metricas
% excluyen t < T_REG por eso.
%
% Visualizacion 3D del robot (cadena del URDF + STLs de meshes/), dos opciones:
%   ANIM_MODE = 'post' — OPCION 1: simula primero todo el horizonte y luego
%               anima la trayectoria con dt = 1/200 s entre movimientos
%               (tiempo real; si el render no alcanza 200 fps se saltan
%               frames para no perder el ritmo del reloj).
%   ANIM_MODE = 'live' — OPCION 2: renderiza DURANTE la simulacion, como si
%               fuera Gazebo (el lazo espera al reloj de pared; el render va
%               decimado a ~50 fps para no cargar el lazo).
%   ANIM_MODE = 'off'  — sin animacion (solo figuras de seguimiento).
%
% Figuras de seguimiento: estilo plots_FL_control.m (posiciones, errores y
% torques) + tabla de metricas en consola (sin el transitorio inicial).
%
% CARPETA AUTOCONTENIDA (portable): copiar esta carpeta completa a cualquier
% maquina y ejecutar el script desde ahi. Contenido:
%   prelab_FL_control.m    este script
%   OMDyn.mexa64           MEX de OMDyn precompilado para Linux
%   OMDyn.mexw64           MEX de OMDyn precompilado para Windows
%   meshes/*.stl           mallas del robot para la vista 3D
% (distribucion solo binarios, igual que Lab 5 student/: sin OMDyn.m ni
%  build_omdyn_mex.m; Mac requeriria compilar OMDyn.mexmaci64 aparte)

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
T_SIM       = 20.0;    % [s] duracion de la simulacion (t_sim del nodo)
ANIM_MODE   = 'live';  % 'post' | 'live' | 'off'  (ver cabecera)

EXPORT_FIGS = false;   % true = guardar PNG (300 dpi) en plots/ junto al script

% ═══════════════════════════════════════════════════════════════════════════
%  GANANCIAS DEL CONTROLADOR — identicas a gz_fl_control_node.cpp
%  Indice: [joint1, joint2, joint3, joint4]   (kd = 1.4*sqrt(kp), zeta = 0.7)
% ═══════════════════════════════════════════════════════════════════════════
KP      = [400.0; 400.0; 600.0; 3000.0];
KD      = [ 28.0;  28.0;  34.0;   77.0];
TAU_MAX = 1.2;         % [N·m] igual que tau_max del robot real
% ═══════════════════════════════════════════════════════════════════════════

Ts          = 1/200;   % [s] periodo del lazo de control (200 Hz, como el nodo)
T_REG       = 1.0;     % [s] ventana excluida de las metricas: transitorio
                       %     inicial (el robot parte en reposo y dq_des(0)~=0)
FRIC_EPS    = 0.05;    % [rad/s] suavizado del tanh en el feedforward de Coulomb

% ── Friccion articular de la planta (valores asumidos para el prelab) ─────────
% La planta la simula SIEMPRE (es lo que hace Gazebo con el <dynamics> del
% URDF) y el controlador la compensa SIEMPRE via feedforward.
FRIC_DAMPING = 0.001 * ones(4,1);   % b  [N·m·s/rad] igual en las 4 articulaciones
FRIC_COULOMB = 0.05  * ones(4,1);   % fc [N·m]       igual en las 4 articulaciones
EPS_PLANT    = 0.01;   % [rad/s] suavizado del sign() de Coulomb en la planta
N_SUB        = 5;      % subpasos RK4 por tick (h = Ts/N_SUB = 1 ms)

% Limites articulares del URDF (solo verificacion post-simulacion)
Q_LOWER = [-2.3562; -1.9199; -1.9199; -1.8];
Q_UPPER = [ 2.3562;  1.7453;  1.5708;  2.1];

%% ── Rutas y dependencias (autocontenidas en esta carpeta) ────────────────────
this_dir = fileparts(mfilename('fullpath'));
mesh_dir = fullfile(this_dir, 'meshes');
addpath(this_dir);   % OMDyn resoluble aunque se ejecute desde otro cwd

assert(~isempty(which('OMDyn')), ...
    'OMDyn.%s no encontrado junto al script para esta plataforma.', mexext);
assert(isfolder(mesh_dir), 'No se encontro la carpeta meshes/ junto al script.');

fprintf('Modelo OMDyn: %s\n', which('OMDyn'));
fprintf('Kp=[%.1f %.1f %.1f %.1f]  Kd=[%.1f %.1f %.1f %.1f]\n', KP, KD);
fprintf(['Feedforward de friccion SIEMPRE activo: damping=[%.4f %.4f %.4f %.4f]  ' ...
         'coulomb=[%.4f %.4f %.4f %.4f]\n'], FRIC_DAMPING, FRIC_COULOMB);
fprintf('Tiempo de simulacion: %.1f s  |  lazo: %.0f Hz  |  animacion: %s\n\n', ...
        T_SIM, 1/Ts, ANIM_MODE);

%% ── Seccion 1: Referencia precomputada (sinusoide articular) ─────────────────
% El nodo la evalua tick a tick; aqui se precomputa (mismas formulas) para
% reutilizarla en el lazo, el trazado 3D y las figuras.
Nsim  = round(T_SIM / Ts);
t_log = (0:Nsim-1)' * Ts;

QD = zeros(Nsim, 4); DQD = zeros(Nsim, 4); DDQD = zeros(Nsim, 4);
for k = 1:Nsim
    [qd, dqd, ddqd] = trayectoria_deseada(t_log(k));
    QD(k,:) = qd'; DQD(k,:) = dqd'; DDQD(k,:) = ddqd';
end

% Condicion inicial de la planta: EN la posicion inicial de la trayectoria y
% en reposo (sin rampa quintica — ver cabecera). q_des(0) = [0; -0.5; 0.3; pi/4].
q0  = QD(1,:)';
dq0 = zeros(4,1);
fprintf('Pose inicial = q_des(0) = [%.3f %.3f %.3f %.3f] rad (en reposo)\n\n', q0);

% Trayectoria cartesiana deseada del EE (para la vista 3D)
Y_DES = zeros(Nsim, 4);
for k = 1:Nsim
    Y_DES(k,:) = fkin_local(QD(k,:)')';
end

%% ── Seccion 2: Vista 3D (solo modo 'live': se construye antes del lazo) ──────
H = [];
if strcmp(ANIM_MODE, 'live')
    H = build_robot3d(mesh_dir, q0, Y_DES);
    ANIM_DECIM = 4;            % render cada 4 ticks (~50 fps)
end

%% ── Seccion 3: Lazo de control a 200 Hz (tick del nodo) ──────────────────────
Q = zeros(Nsim, 4); DQ = zeros(Nsim, 4); TAU = zeros(Nsim, 4);

x = [q0; dq0];
t0_wall = tic;
for k = 1:Nsim
    % ── Leer estado (en el nodo: /joint_states) ───────────────────────────
    q  = x(1:4);
    dq = x(5:8);

    % ── Referencia del tick ───────────────────────────────────────────────
    qd = QD(k,:)'; dqd = DQD(k,:)'; ddqd = DDQD(k,:)';

    % ── Feedback linearization (identico al nodo) ─────────────────────────
    % tau = M(q) * (ddq_des + Kp*(q_des-q) + Kd*(dq_des-dq)) + phib(q,dq)
    [M, phib] = OMDyn(q, dq);
    e   = qd  - q;
    de  = dqd - dq;
    v   = ddqd + KP.*e + KD.*de;
    tau = M*v + phib;

    % Feedforward de friccion (siempre activo): viscosa con la velocidad
    % medida; Coulomb con la velocidad DESEADA (senal limpia, criterio del hw)
    tau = tau + FRIC_DAMPING.*dq + FRIC_COULOMB.*tanh(dqd / FRIC_EPS);

    % ── Saturacion de torque ──────────────────────────────────────────────
    tau_sat = min(max(tau, -TAU_MAX), TAU_MAX);

    % ── Registro (columnas del CSV del nodo) ──────────────────────────────
    Q(k,:) = q'; DQ(k,:) = dq'; TAU(k,:) = tau_sat';

    % ── Planta (el papel de Gazebo): RK4 con ZOH del torque ──────────────
    h = Ts / N_SUB;
    for s = 1:N_SUB
        x = rk4_step(x, tau_sat, h, FRIC_DAMPING, FRIC_COULOMB, EPS_PLANT);
    end

    % ── Animacion en vivo + ritmo de tiempo real (solo 'live') ───────────
    if strcmp(ANIM_MODE, 'live')
        % Render decimado (~50 fps) y solo si el lazo no va retrasado: si el
        % render no da abasto se salta ese frame y se mantiene el tiempo real.
        if mod(k-1, ANIM_DECIM) == 0 && toc(t0_wall) <= t_log(k) + Ts
            update_robot3d(H, q, t_log(k));
            drawnow;
        end
        sobra = t_log(k) + Ts - toc(t0_wall);
        if sobra > 0, pause(sobra); end   % espera al reloj de pared
    end

    if mod(k, round(1/Ts)) == 0
        fprintf('t=%5.2f s  |e|=%.5f rad  tau=[%+.3f %+.3f %+.3f %+.3f] Nm\n', ...
                t_log(k), norm(e), tau_sat);
    end
end
fprintf('Simulacion completada (%.1f s).\n\n', T_SIM);

% Verificacion de limites articulares del URDF (Gazebo los impone; aqui se avisa)
for i = 1:4
    if any(Q(:,i) < Q_LOWER(i)) || any(Q(:,i) > Q_UPPER(i))
        warning('q%d salio de los limites del URDF [%.3f, %.3f] rad.', ...
                i, Q_LOWER(i), Q_UPPER(i));
    end
end

%% ── Seccion 4: Animacion 3D posterior (OPCION 1: dt = 1/200 s por frame) ─────
if strcmp(ANIM_MODE, 'post')
    H = build_robot3d(mesh_dir, q0, Y_DES);
    drawnow;
    t0_wall = tic;
    k = 1;
    while k <= Nsim
        update_robot3d(H, Q(k,:)', t_log(k));
        drawnow;
        % siguiente frame segun el reloj: mantiene dt=1/200 s de movimiento en
        % tiempo real aunque el render no alcance 200 fps (salta frames)
        k = max(k + 1, 1 + floor(toc(t0_wall) / Ts));
    end
    update_robot3d(H, Q(end,:)', t_log(end));
    drawnow;
end

%% ── Seccion 5: Figuras de seguimiento (mismas que plots_FL_control.m) ────────
e_q = Q - QD;

% Metricas por articulacion, excluyendo el transitorio inicial (T_REG)
m_reg = t_log >= T_REG;
if ~any(m_reg)
    warning('No hay muestras con t >= %.1f s; revisar T_REG.', T_REG);
end
fprintf('%s\n', repmat('═', 1, 76));
fprintf(' Metricas FL articular  [Prelab MATLAB]  (t >= %.1f s, sin transitorio inicial)\n', ...
        T_REG);
fprintf('%s\n', repmat('═', 1, 76));
fprintf('%-6s  %-12s  %-12s  %-14s  %-14s  %-8s\n', ...
        'Joint', 'e_max[rad]', 'e_RMS[rad]', 'max|tau|[N·m]', 'tau_RMS[N·m]', 'Sat[%]');
fprintf('%s\n', repmat('-', 1, 76));
for i = 1:4
    e_i   = e_q(m_reg, i);
    tau_i = TAU(m_reg, i);
    fprintf('q%-5d  %12.5f  %12.5f  %14.4f  %14.4f  %8.2f\n', i, ...
            max(abs(e_i)), sqrt(mean(e_i.^2)), ...
            max(abs(tau_i)), sqrt(mean(tau_i.^2)), ...
            100 * mean(abs(tau_i) >= TAU_MAX - 1e-6));
end
fprintf('%s\n\n', repmat('═', 1, 76));

lw         = 1.6;
fs         = 11;
fs_title   = 12;
color_ref  = [0.8500 0.3250 0.0980];   % naranja — referencia
color_meas = [0.0000 0.4470 0.7410];   % azul    — simulacion
color_tau  = [0.4660 0.6740 0.1880];   % verde   — torque
jointNames = {'Articulacion 1', 'Articulacion 2', ...
              'Articulacion 3', 'Articulacion 4'};
xlims      = [t_log(1), t_log(end)];
mode_label = 'Prelab MATLAB';

% ── Figura 1 — Posiciones articulares ────────────────────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 1100 700]);
tl1  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs1 = gobjects(1, 4);
h_ref1 = []; h_mea1 = [];
for i = 1:4
    axs1(i) = nexttile(tl1);
    h2 = plot(t_log, Q(:,i),  '-',  'Color', color_meas, 'LineWidth', lw); hold on;
    h1 = plot(t_log, QD(:,i), '--', 'Color', color_ref,  'LineWidth', lw);
    if i == 1; h_ref1 = h1; h_mea1 = h2; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
lgd1 = legend(axs1(1), [h_ref1, h_mea1], {'Referencia', 'Simulacion'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd1.Layout.Tile = 'north';
title(tl1, sprintf('[%s] Seguimiento de posiciones articulares', mode_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

% ── Figura 2 — Errores de seguimiento articular ──────────────────────────────
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [120 120 1100 560]);
tl2 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:4
    nexttile(tl2);
    plot(t_log, e_q(:,i), '-', 'Color', color_meas, 'LineWidth', lw);
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$e_{q_%d}$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
title(tl2, sprintf('[%s] FL Control - Errores de Seguimiento Articular', mode_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

% ── Figura 3 — Torques de control ────────────────────────────────────────────
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [140 140 1100 700]);
for i = 1:4
    subplot(2,2,i);
    plot(t_log, TAU(:,i), '-', 'Color', color_tau, 'LineWidth', lw);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N\\cdot m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(['Torque de control - ' jointNames{i}], 'FontSize', fs_title);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
sgtitle(sprintf('[%s] FL Control - Torques de control calculados', mode_label), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    output_dir = fullfile(this_dir, 'plots');
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end
    exportgraphics(figure(1), fullfile(output_dir, 'tracking_plot_q.png'),     'Resolution', 300);
    exportgraphics(figure(2), fullfile(output_dir, 'tracking_plot_error.png'), 'Resolution', 300);
    exportgraphics(figure(3), fullfile(output_dir, 'torques_plot.png'),        'Resolution', 300);
    if ~isempty(H) && isvalid(H.fig)
        exportgraphics(H.fig, fullfile(output_dir, 'robot3d_final.png'), 'Resolution', 300);
    end
    fprintf('Graficas guardadas en: %s\n', output_dir);
end

%% ═════════════════════════ Funciones locales ═════════════════════════════════

% ── Trayectoria sinusoidal articular (identica a desiredTrajectory del nodo) ──
function [qd, dqd, ddqd] = trayectoria_deseada(t)
    w = 1.0;
    qd = [ (pi/4)*sin(w*t);
           -0.5 + 0.5*sin(w*t);
            0.3 - 0.5*sin(w*t);
            pi/4              ];
    dqd = [ (pi/4)*w*cos(w*t);
             0.5*w*cos(w*t);
            -0.5*w*cos(w*t);
             0.0              ];
    ddqd = [ -(pi/4)*w^2*sin(w*t);
             -0.5*w^2*sin(w*t);
              0.5*w^2*sin(w*t);
              0.0              ];
end

% ── Dinamica de la planta: M*ddq + phib + friccion articular = tau ────────────
function dx = dinamica_planta(x, tau, b, fc, eps_pl)
    q  = x(1:4);
    dq = x(5:8);
    [M, phib] = OMDyn(q, dq);
    ddq = M \ (tau - phib - b.*dq - fc.*tanh(dq / eps_pl));
    dx = [dq; ddq];
end

% ── Paso RK4 con torque constante (ZOH) ───────────────────────────────────────
function x = rk4_step(x, tau, h, b, fc, eps_pl)
    k1 = dinamica_planta(x,            tau, b, fc, eps_pl);
    k2 = dinamica_planta(x + 0.5*h*k1, tau, b, fc, eps_pl);
    k3 = dinamica_planta(x + 0.5*h*k2, tau, b, fc, eps_pl);
    k4 = dinamica_planta(x + h*k3,     tau, b, fc, eps_pl);
    x  = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

% ── Cinematica directa analitica (identica a fkin del nodo) ───────────────────
function y = fkin_local(q)
    x_base = 0.012;  z_base = 0.017 + 0.0595;
    x23 = 0.024;  z23 = 0.128;
    l34 = 0.124;  l4e = 0.126;
    r = x23*cos(q(2)) + z23*sin(q(2)) ...
      + l34*cos(q(2)+q(3)) + l4e*cos(q(2)+q(3)+q(4));
    z = z_base + (-x23*sin(q(2)) + z23*cos(q(2))) ...
      - l34*sin(q(2)+q(3)) - l4e*sin(q(2)+q(3)+q(4));
    y = [x_base + r*cos(q(1)); r*sin(q(1)); z; q(2)+q(3)+q(4)];
end

% ── Vista 3D del robot: STLs del URDF sobre una jerarquia de hgtransform ──────
% Cadena (urdf/open_manipulator_x.urdf):
%   link1 (fijo) --joint1 z @ [0.012 0 0.017]--> link2 (malla +0.019 z)
%         --joint2 y @ [0 0 0.0595]--> link3
%         --joint3 y @ [0.024 0 0.128]--> link4
%         --joint4 y @ [0.124 0 0]--> link5 + palmas del gripper @ [0.0817 ±0.021 0]
function H = build_robot3d(mesh_dir, q0, y_des_path)
    H.fig = figure(4); clf(H.fig);
    set(H.fig, 'Color', 'w', 'Name', 'OpenMANIPULATOR-X — Prelab FL', ...
        'Position', [80 80 900 700]);
    H.ax = axes(H.fig); hold(H.ax, 'on');
    grid(H.ax, 'on'); box(H.ax, 'on'); axis(H.ax, 'equal');
    xlim(H.ax, [-0.15 0.45]); ylim(H.ax, [-0.25 0.25]); zlim(H.ax, [-0.01 0.3]);
    xlabel(H.ax, 'x [m]'); ylabel(H.ax, 'y [m]'); zlabel(H.ax, 'z [m]');
    view(H.ax, 45, 25);
    title(H.ax, 'Feedback Linearization articular — OpenMANIPULATOR-X');

    % piso de referencia
    patch(H.ax, 'XData', [-0.25 0.45 0.45 -0.25], ...
                'YData', [-0.35 -0.35 0.35 0.35], 'ZData', zeros(1,4), ...
          'FaceColor', [0.94 0.94 0.94], 'EdgeColor', [0.85 0.85 0.85], ...
          'HandleVisibility', 'off');

    % jerarquia: ax -> T1 -> T2 -> T3 -> T4 (cada Matrix es LOCAL a su padre)
    H.T1 = hgtransform('Parent', H.ax);
    H.T2 = hgtransform('Parent', H.T1);
    H.T3 = hgtransform('Parent', H.T2);
    H.T4 = hgtransform('Parent', H.T3);

    c_link = [0.45 0.45 0.48];
    c_grip = [0.25 0.25 0.28];
    add_stl(H.ax, mesh_dir, 'link1.stl', [0 0 0],     c_link);
    add_stl(H.T1, mesh_dir, 'link2.stl', [0 0 0.019], c_link);
    add_stl(H.T2, mesh_dir, 'link3.stl', [0 0 0],     c_link);
    add_stl(H.T3, mesh_dir, 'link4.stl', [0 0 0],     c_link);
    add_stl(H.T4, mesh_dir, 'link5.stl', [0 0 0],     c_link);
    add_stl(H.T4, mesh_dir, 'gripper_left_palm.stl',  [0.0817  0.021 0], c_grip);
    add_stl(H.T4, mesh_dir, 'gripper_right_palm.stl', [0.0817 -0.021 0], c_grip);

    % punto EE (l4e = 0.126 en el marco de link5), trayectorias y reloj
    line('Parent', H.T4, 'XData', 0.126, 'YData', 0, 'ZData', 0, ...
         'Marker', 'o', 'MarkerSize', 5, 'Color', [0 0.447 0.741], ...
         'MarkerFaceColor', [0 0.447 0.741], 'HandleVisibility', 'off');
    plot3(H.ax, y_des_path(:,1), y_des_path(:,2), y_des_path(:,3), '--', ...
          'Color', [0.85 0.325 0.098], 'LineWidth', 1.2, ...
          'DisplayName', 'EE deseado');
    H.trace = animatedline(H.ax, 'Color', [0 0.447 0.741], 'LineWidth', 1.6, ...
                           'DisplayName', 'EE simulado');
    legend(H.ax, 'Location', 'northeast');
    H.txt = text(H.ax, -0.23, -0.33, 0.48, 't = 0.00 s', 'FontSize', 12, ...
                 'FontWeight', 'bold');

    camlight(H.ax, 'headlight');
    lighting(H.ax, 'gouraud');
    update_robot3d(H, q0, 0);
end

function add_stl(parent, mesh_dir, fname, offset, col)
    TR = stlread(fullfile(mesh_dir, fname));
    V  = TR.Points * 1e-3 + offset(:)';   % STL en mm (scale 0.001 del URDF)
    patch('Parent', parent, 'Faces', TR.ConnectivityList, 'Vertices', V, ...
          'FaceColor', col, 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
          'AmbientStrength', 0.45, 'HandleVisibility', 'off');
end

function update_robot3d(H, q, t)
    H.T1.Matrix = makehgtform('translate', [0.012 0 0.017]) * makehgtform('zrotate', q(1));
    H.T2.Matrix = makehgtform('translate', [0 0 0.0595])    * makehgtform('yrotate', q(2));
    H.T3.Matrix = makehgtform('translate', [0.024 0 0.128]) * makehgtform('yrotate', q(3));
    H.T4.Matrix = makehgtform('translate', [0.124 0 0])     * makehgtform('yrotate', q(4));
    y = fkin_local(q);
    addpoints(H.trace, y(1), y(2), y(3));
    set(H.txt, 'String', sprintf('t = %.2f s', t));
end
