%% sinusoidal_torque_analysis.m
% Visualización del experimento hw_sinusoidal_torque_node.
%
% Detecta automáticamente qué joints están en current mode (tau_ref ≠ 0)
% y cuáles en position mode (pos_ref ≠ 0) para adaptar las gráficas.
%
% Figuras:
%   Fig 1 — Posiciones articulares: referencia sinusoidal vs medición
%   Fig 2 — Velocidades articulares medidas
%   Fig 3 — Torques de referencia (current mode) / error de seguimiento (position mode)
%   Fig 4 — Corriente medida vs corriente comandada
%
% Configurar log_id y ejecutar.

clear; clc; close all;

%% ── Configuración ────────────────────────────────────────────────────────
log_id      = 8;            % hw_sin_torque_<log_id>.csv
EXPORT_FIGS = false;        % true = guardar PNG + EPS

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

% Detectar modo de cada joint:
%   position mode → pos_ref tiene valor no nulo en la fase de control
%   current mode  → tau_ref tiene variación apreciable
is_pos_mode = max(abs(pos_ref)) > 1e-4;      % [1×4 logical]
is_cur_mode = ~is_pos_mode;

%% ── Estilos ──────────────────────────────────────────────────────────────
lw       = 1.5;
fs       = 11;
fst      = 12;
c_meas   = [0.0000 0.4470 0.7410];   % azul    — medido
c_ref    = [0.8500 0.3250 0.0980];   % naranja — referencia/setpoint
c_cmd    = [0.4660 0.6740 0.1880];   % verde   — comandado (modelo)
c_err    = [0.6350 0.0780 0.1840];   % rojo    — error
xlims    = [t(1), t(end)];

mode_str = cell(1,4);
for i = 1:4
    mode_str{i} = sprintf('J%d [%s]', i, ternary(is_pos_mode(i), 'position', 'current'));
end

%% ── Figura 1 — Seguimiento de posiciones articulares ────────────────────
figure(1); clf;
set(gcf,'Color','w','Position',[50 50 1100 700]);
tl1  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs1 = gobjects(1, 4);
h_ref1 = []; h_mea1 = [];

for i = 1:4
    axs1(i) = nexttile(tl1);
    if is_pos_mode(i)
        h2 = plot(t, q(:,i),      '-',  'Color', c_meas, 'LineWidth', lw); hold on;
        h1 = plot(t, pos_ref(:,i),'--', 'Color', c_ref,  'LineWidth', lw);
        if isempty(h_ref1); h_ref1 = h1; h_mea1 = h2; end
    else
        plot(t, q(:,i), '-', 'Color', c_meas, 'LineWidth', lw);
    end
    yline(0,':','LineWidth',0.8);
    xlabel('Tiempo [s]','FontSize',fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter','latex', 'FontSize', fs);
    title(mode_str{i},'FontSize',fst);
    grid on; box on; xlim(xlims);
    set(gca,'FontSize',fs);
end

if ~isempty(h_ref1)
    lgd1 = legend(axs1(find(is_pos_mode,1)), [h_ref1, h_mea1], ...
                  {'Referencia', 'Medicion'}, ...
                  'Orientation','horizontal','FontSize',fs,'Location','northoutside');
    lgd1.Layout.Tile = 'north';
end
title(tl1, 'Seguimiento de posiciones articulares', ...
      'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 2 — Velocidades articulares ──────────────────────────────────
figure(2); clf;
set(gcf,'Color','w','Position',[70 70 1100 700]);
for i = 1:4
    subplot(2,2,i);
    plot(t, dq(:,i), '-', 'Color', c_meas, 'LineWidth', lw);
    yline(0,':','LineWidth',0.8);
    xlabel('Tiempo [s]','FontSize',fs);
    ylabel(sprintf('$\\dot{q}_%d$ [rad/s]', i), 'Interpreter','latex', 'FontSize', fs);
    title(mode_str{i},'FontSize',fst);
    grid on; box on; xlim(xlims);
    set(gca,'FontSize',fs);
end
sgtitle('Velocidades articulares medidas','FontSize',14,'FontWeight','bold');

%% ── Figura 3 — Torques (current mode) / Error de seguimiento (position mode)
figure(3); clf;
set(gcf,'Color','w','Position',[90 90 1100 700]);
for i = 1:4
    subplot(2,2,i);
    if is_cur_mode(i)
        plot(t, tau_ref(:,i), '-', 'Color', c_ref, 'LineWidth', lw);
        yline(0,':','LineWidth',0.8);
        ylabel(sprintf('$\\tau_{ref,%d}$ [N$\\cdot$m]', i), 'Interpreter','latex', 'FontSize', fs);
        title(sprintf('%s — torque referencia', mode_str{i}),'FontSize',fst);
    else
        e_pos = q(:,i) - pos_ref(:,i);   % error respecto a referencia dinámica
        plot(t, e_pos, '-', 'Color', c_err, 'LineWidth', lw);
        yline(0,':','LineWidth',0.8);
        rmse = sqrt(mean(e_pos.^2));
        text(0.02,0.95, sprintf('RMSE = %.4f rad', rmse), 'Units','normalized', ...
             'FontSize',fs-1,'VerticalAlignment','top');
        ylabel(sprintf('$e_{q_%d}$ [rad]', i), 'Interpreter','latex', 'FontSize', fs);
        title(sprintf('%s — error de seguimiento', mode_str{i}),'FontSize',fst);
    end
    xlabel('Tiempo [s]','FontSize',fs);
    grid on; box on; xlim(xlims);
    set(gca,'FontSize',fs);
end
sgtitle('Torques enviados (current) / Error de seguimiento (position)',...
        'FontSize',14,'FontWeight','bold');

%% ── Figura 4 — Corriente medida vs corriente comandada ──────────────────
figure(4); clf;
set(gcf,'Color','w','Position',[110 110 1100 700]);
for i = 1:4
    subplot(2,2,i);
    plot(t, i_meas(:,i), '-',  'Color', c_meas, 'LineWidth', lw); hold on;
    if is_cur_mode(i)
        plot(t, i_cmd(:,i), '--', 'Color', c_cmd, 'LineWidth', lw);
        legend({'$I_{meas}$','$I_{cmd}$ (modelo)'}, ...
               'Interpreter','latex','Location','best','FontSize',fs-1);
    end
    yline(0,':','LineWidth',0.8);
    xlabel('Tiempo [s]','FontSize',fs);
    ylabel(sprintf('Corriente J%d [ticks]', i),'FontSize',fs);
    title(mode_str{i},'FontSize',fst);
    grid on; box on; xlim(xlims);
    set(gca,'FontSize',fs);
end
sgtitle('Corriente medida vs comandada','FontSize',14,'FontWeight','bold');

%% ── Métricas de resumen ──────────────────────────────────────────────────
fprintf('\n── Métricas ──────────────────────────────────────────────────────\n');
fprintf('%-6s  %-10s  %-14s  %-14s\n', 'Joint', 'Modo', 'Métrica 1', 'Métrica 2');
for i = 1:4
    modo = ternary(is_pos_mode(i), 'position', 'current');
    if is_cur_mode(i)
        rmse_I = sqrt(mean((i_meas(:,i) - i_cmd(:,i)).^2));
        fprintf('J%-5d  %-10s  |tau|max=%.3f N·m  RMSE_I=%.2f ticks\n', ...
                i, modo, max(abs(tau_ref(:,i))), rmse_I);
    else
        e_pos = q(:,i) - pos_ref(:,i);
        fprintf('J%-5d  %-10s  RMSE_q=%.4f rad  |I_meas|max=%d ticks\n', ...
                i, modo, sqrt(mean(e_pos.^2)), max(abs(i_meas(:,i))));
    end
end

%% ── Exportación ─────────────────────────────────────────────────────────
if EXPORT_FIGS
    out_dir = fullfile(pkg_dir, 'plots', 'diagnostics', 'sinusoidal', ...
                       sprintf('log%d', log_id));
    if ~exist(out_dir,'dir'), mkdir(out_dir); end
    figs   = {1, 2, 3, 4};
    fnames = {'posiciones','velocidades','torques_error','corrientes'};
    for k = 1:numel(figs)
        f = figure(figs{k});
        exportgraphics(f, fullfile(out_dir, [fnames{k} '.png']), 'Resolution',300);
        exportgraphics(f, fullfile(out_dir, [fnames{k} '.eps']), ...
                       'ContentType','vector','Resolution',600);
    end
    fprintf('Guardado en: %s\n', out_dir);
end

%% ── Utilidad ─────────────────────────────────────────────────────────────
function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end
