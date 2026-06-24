%% sinusoidal_torque_analysis.m
% Visualización del experimento hw_sinusoidal_torque_node.
%
% Detecta automáticamente qué joints están en current mode (tau_ref ≠ 0)
% y cuáles en position mode (pos_ref ≠ 0) para adaptar las gráficas.
%
% Figuras:
%   Fig 1 — Posiciones articulares:    pos_ref vs q
%   Fig 2 — Velocidades articulares:   dq_ref  vs dq  (analítico vs medido)
%   Fig 3 — Aceleraciones articulares: ddq_ref vs ddq (analítico vs diferencias de dq)
%   Fig 4 — Torques de referencia (current mode) / error de seguimiento (position mode)
%   Fig 5 — Corriente medida vs corriente comandada
%
%   dq_ref  y ddq_ref  se leen del CSV (derivadas analíticas de la trayectoria sinusoidal).
%   ddq     se calcula por diferencias finitas centradas de dq  (gradient).
%   Figs 2 y 3 permiten evaluar si el tracking es bueno y si ddq por diferencias
%   tiene demasiado ruido para ser útil en identificación.
%
%   Compatibilidad con CSV de versiones anteriores (sin dq_ref/ddq_ref).
%
% Configurar log_id y ejecutar.

clear; clc; close all;

%% ── Configuración ────────────────────────────────────────────────────────
log_id      = 10;            % hw_sin_torque_<log_id>.csv
EXPORT_FIGS = true;        % true = guardar PNG + EPS

pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

%% ── Carga de datos ───────────────────────────────────────────────────────
csv_file = fullfile(pkg_dir, 'data', 'diagnostics', 'sinusoidal', ...
                   sprintf('hw_sin_torque_%d.csv', log_id));
if ~isfile(csv_file)
    error('Archivo no encontrado:\n  %s', csv_file);
end
T = readtable(csv_file);
fprintf('Cargado: %s  (%d muestras)\n', csv_file, height(T));

t       = T.t - T.t(1);
Ts      = mean(diff(t));

q       = [T.q1,    T.q2,    T.q3,    T.q4   ];
dq      = [T.dq1,   T.dq2,   T.dq3,   T.dq4  ];
tau_ref = [T.tau_ref1, T.tau_ref2, T.tau_ref3, T.tau_ref4];

% Compatibilidad con CSV nuevo (pos_ref) y antiguo (pos_cmd_rad)
if ismember('pos_ref1', T.Properties.VariableNames)
    pos_ref = [T.pos_ref1, T.pos_ref2, T.pos_ref3, T.pos_ref4];
else
    pos_ref = [T.pos_cmd_rad1, T.pos_cmd_rad2, T.pos_cmd_rad3, T.pos_cmd_rad4];
end

i_cmd   = [T.curr_cmd1,  T.curr_cmd2,  T.curr_cmd3,  T.curr_cmd4 ];
i_meas  = [T.curr_meas1, T.curr_meas2, T.curr_meas3, T.curr_meas4];

% dq_ref y ddq_ref analíticos (generados por el nodo a partir de la trayectoria)
has_vel_ref = ismember('dq_ref1', T.Properties.VariableNames);
if has_vel_ref
    dq_ref  = [T.dq_ref1,  T.dq_ref2,  T.dq_ref3,  T.dq_ref4 ];
    ddq_ref = [T.ddq_ref1, T.ddq_ref2, T.ddq_ref3, T.ddq_ref4];
else
    dq_ref  = zeros(size(dq));
    ddq_ref = zeros(size(dq));
    warning(['CSV sin columnas dq_ref / ddq_ref (CSV de version anterior). ' ...
             'Figs 2 y 3 solo muestran datos medidos.']);
end

% ddq medido: diferencias finitas centradas (gradient) sobre dq
ddq = zeros(size(dq));
for i = 1:4
    ddq(:,i) = gradient(dq(:,i), Ts);
end

% Detectar modo de cada joint
%   position mode → pos_ref tiene variación apreciable durante la fase de control
%   current mode  → pos_ref ≈ 0 durante la fase de control
is_pos_mode = max(abs(pos_ref)) > 1e-4;   % [1×4 logical]
is_cur_mode = ~is_pos_mode;

%% ── Estilos comunes ──────────────────────────────────────────────────────
lw       = 1.5;
fs       = 11;
fst      = 12;
c_meas   = [0.0000 0.4470 0.7410];   % azul    — medido
c_ref    = [0.8500 0.3250 0.0980];   % naranja — referencia
c_cmd    = [0.4660 0.6740 0.1880];   % verde   — comandado (modelo)
c_err    = [0.6350 0.0780 0.1840];   % rojo    — error
xlims    = [t(1), t(end)];

mode_str = cell(1,4);
for i = 1:4
    mode_str{i} = sprintf('J%d [%s]', i, ternary(is_pos_mode(i), 'position', 'current'));
end

%% ── Figura 1 — Seguimiento de posiciones articulares ────────────────────
figure(1); clf;
set(gcf,'Color','w','Position',[40 40 1100 700]);
tl1  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs1 = gobjects(1, 4);
h_ref1 = []; h_mea1 = [];

for i = 1:4
    axs1(i) = nexttile(tl1);
    if is_pos_mode(i)
        h2 = plot(t, q(:,i),       '-',  'Color', c_meas, 'LineWidth', lw); hold on;
        h1 = plot(t, pos_ref(:,i), '--', 'Color', c_ref,  'LineWidth', lw);
        if isempty(h_ref1); h_ref1 = h1; h_mea1 = h2; end
    else
        plot(t, q(:,i), '-', 'Color', c_meas, 'LineWidth', lw);
    end
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(mode_str{i}, 'FontSize', fst);
    grid on; box on; xlim(xlims);
    set(gca, 'FontSize', fs);
end

if ~isempty(h_ref1)
    lgd1 = legend(axs1(find(is_pos_mode, 1)), [h_ref1, h_mea1], ...
                  {'Referencia ($q_{ref}$)', 'Medicion ($q$)'}, ...
                  'Interpreter', 'latex', ...
                  'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
    lgd1.Layout.Tile = 'north';
end
title(tl1, 'Fig 1 — Seguimiento de posiciones articulares', ...
      'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 2 — Seguimiento de velocidades articulares (dq_ref vs dq) ────
figure(2); clf;
set(gcf,'Color','w','Position',[60 60 1100 700]);
tl2  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs2 = gobjects(1, 4);
h_ref2 = []; h_mea2 = [];

for i = 1:4
    axs2(i) = nexttile(tl2);
    h2 = plot(t, dq(:,i), '-', 'Color', c_meas, 'LineWidth', lw); hold on;
    if is_pos_mode(i) && has_vel_ref
        h1 = plot(t, dq_ref(:,i), '--', 'Color', c_ref, 'LineWidth', lw);
        if isempty(h_ref2); h_ref2 = h1; h_mea2 = h2; end
    end
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\dot{q}_%d$ [rad/s]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(mode_str{i}, 'FontSize', fst);
    grid on; box on; xlim(xlims);
    set(gca, 'FontSize', fs);
end

if ~isempty(h_ref2)
    lgd2 = legend(axs2(find(is_pos_mode, 1)), [h_ref2, h_mea2], ...
                  {'Referencia ($\dot{q}_{ref}$)', 'Medicion ($\dot{q}$)'}, ...
                  'Interpreter', 'latex', ...
                  'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
    lgd2.Layout.Tile = 'north';
end
title(tl2, 'Fig 2 — Seguimiento de velocidades articulares', ...
      'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 3 — Seguimiento de aceleraciones articulares (ddq_ref vs ddq)
figure(3); clf;
set(gcf,'Color','w','Position',[80 80 1100 700]);
tl3  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs3 = gobjects(1, 4);
h_ref3 = []; h_mea3 = [];

for i = 1:4
    axs3(i) = nexttile(tl3);
    h2 = plot(t, ddq(:,i), '-', 'Color', c_meas, 'LineWidth', lw); hold on;
    if is_pos_mode(i) && has_vel_ref
        h1 = plot(t, ddq_ref(:,i), '--', 'Color', c_ref, 'LineWidth', lw);
        if isempty(h_ref3); h_ref3 = h1; h_mea3 = h2; end
    end
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\ddot{q}_%d$ [rad/s$^2$]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(mode_str{i}, 'FontSize', fst);
    grid on; box on; xlim(xlims);
    set(gca, 'FontSize', fs);
end

if ~isempty(h_ref3)
    lgd3 = legend(axs3(find(is_pos_mode, 1)), [h_ref3, h_mea3], ...
                  {'Referencia ($\ddot{q}_{ref}$, analítica)', ...
                   'Diferencias ($\ddot{q}_{diff}$, de $\dot{q}$)'}, ...
                  'Interpreter', 'latex', ...
                  'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
    lgd3.Layout.Tile = 'north';
end
title(tl3, 'Fig 3 — Seguimiento de aceleraciones articulares  (ddq: gradient de dq)', ...
      'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 4 — Torques (current mode) / Error de seguimiento (position mode)
figure(4); clf;
set(gcf,'Color','w','Position',[100 100 1100 700]);
for i = 1:4
    subplot(2,2,i);
    if is_cur_mode(i)
        plot(t, tau_ref(:,i), '-', 'Color', c_ref, 'LineWidth', lw);
        yline(0, ':', 'LineWidth', 0.8);
        ylabel(sprintf('$\\tau_{ref,%d}$ [N$\\cdot$m]', i), 'Interpreter', 'latex', 'FontSize', fs);
        title(sprintf('%s — torque referencia', mode_str{i}), 'FontSize', fst);
    else
        e_pos = q(:,i) - pos_ref(:,i);
        plot(t, e_pos, '-', 'Color', c_err, 'LineWidth', lw);
        yline(0, ':', 'LineWidth', 0.8);
        rmse = sqrt(mean(e_pos.^2));
        text(0.02, 0.95, sprintf('RMSE = %.4f rad', rmse), 'Units', 'normalized', ...
             'FontSize', fs-1, 'VerticalAlignment', 'top');
        ylabel(sprintf('$e_{q_%d}$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
        title(sprintf('%s — error de seguimiento', mode_str{i}), 'FontSize', fst);
    end
    xlabel('Tiempo [s]', 'FontSize', fs);
    grid on; box on; xlim(xlims);
    set(gca, 'FontSize', fs);
end
sgtitle('Fig 4 — Torques (current) / Error de seguimiento (position)', ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 5 — Corriente medida vs corriente comandada ──────────────────
figure(5); clf;
set(gcf,'Color','w','Position',[120 120 1100 700]);
for i = 1:4
    subplot(2,2,i);
    plot(t, i_meas(:,i), '-',  'Color', c_meas, 'LineWidth', lw); hold on;
    if is_cur_mode(i)
        plot(t, i_cmd(:,i), '--', 'Color', c_cmd, 'LineWidth', lw);
        legend({'$I_{meas}$', '$I_{cmd}$ (modelo)'}, ...
               'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs-1);
    end
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('Corriente J%d [ticks]', i), 'FontSize', fs);
    title(mode_str{i}, 'FontSize', fst);
    grid on; box on; xlim(xlims);
    set(gca, 'FontSize', fs);
end
sgtitle('Fig 5 — Corriente medida vs comandada', 'FontSize', 14, 'FontWeight', 'bold');

%% ── Métricas de resumen ──────────────────────────────────────────────────
fprintf('\n── Métricas ──────────────────────────────────────────────────────────\n');
for i = 1:4
    modo = ternary(is_pos_mode(i), 'position', 'current');
    if is_cur_mode(i)
        rmse_I = sqrt(mean((i_meas(:,i) - i_cmd(:,i)).^2));
        fprintf('J%d [%s]  |tau|_max=%.3f N·m   RMSE_I=%.2f ticks\n', ...
                i, modo, max(abs(tau_ref(:,i))), rmse_I);
    else
        e_pos   = q(:,i) - pos_ref(:,i);
        rmse_q  = sqrt(mean(e_pos.^2));
        if has_vel_ref
            e_vel   = dq(:,i) - dq_ref(:,i);
            rmse_dq = sqrt(mean(e_vel.^2));
            fprintf('J%d [%s]  RMSE_q=%.4f rad   RMSE_dq=%.4f rad/s   |I_meas|_max=%d ticks\n', ...
                    i, modo, rmse_q, rmse_dq, max(abs(i_meas(:,i))));
        else
            fprintf('J%d [%s]  RMSE_q=%.4f rad   |I_meas|_max=%d ticks\n', ...
                    i, modo, rmse_q, max(abs(i_meas(:,i))));
        end
    end
end
fprintf('──────────────────────────────────────────────────────────────────────\n');

%% ── Exportación ─────────────────────────────────────────────────────────
if EXPORT_FIGS
    out_dir = fullfile(pkg_dir, 'plots', 'diagnostics', 'sinusoidal', ...
                       sprintf('log%d', log_id));
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end
    figs   = {1,            2,              3,              4,               5          };
    fnames = {'posiciones', 'vel_tracking', 'acc_tracking', 'torques_error', 'corrientes'};
    for k = 1:numel(figs)
        f = figure(figs{k});
        exportgraphics(f, fullfile(out_dir, [fnames{k} '.png']), 'Resolution', 300);
    end
    fprintf('Guardado en: %s\n', out_dir);
end

%% ── Utilidad ─────────────────────────────────────────────────────────────
function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end
