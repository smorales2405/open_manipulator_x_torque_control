%% plots_FL_control.m
% Graficas para Actividad 1 — Control FL (Feedback Linearization)
% OpenMANIPULATOR-X, Laboratorio 4
%
%   Figura 1 — Seguimiento de posiciones articulares  q1..q4
%   Figura 2 — Errores de seguimiento articular e_q1..e_q4
%   Figura 3 — Seguimiento de velocidades articulares dq1..dq4
%   Figura 4 — Torques de control tau1..tau4
%
%   Metricas reportadas en consola (por articulacion, sin la rampa quintica
%   inicial):
%     e_q,max [rad]  |  e_q,RMS [rad]  |  max|tau| [N·m]  |  tau_RMS [N·m]  |  Sat [%]
%
%   El nodo hw_fl/gz_fl ya excluye del CSV el retorno quintico final a
%   q_inicial y la pausa en reposo (solo registra rampa + trayectoria), por
%   lo que no se requiere recortar nada al final aqui.
%
% Configurar las variables de la seccion "Configuracion" y ejecutar.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
mode        = 'real';   % 'sim'  = simulacion Gazebo
                       % 'real' = implementacion hardware real

test_num    = 7;       % Numero de log

EXPORT_FIGS = false;    % true  = guardar PNG (300 dpi) y EPS vectorial (600 dpi)
                       % false = solo visualizar

RAMP_TIME_S = 3.0;     % [s] duracion de la rampa quintica inicial — debe
                       % coincidir con RAMP_TIME_S de los nodos gz/hw_fl;
                       % las metricas excluyen t < RAMP_TIME_S

TAU_MAX     = 1.2;     % [N·m] limite de torque (tau_max en los nodos);
                       % usado para calcular Sat [%]

% Directorio raiz del paquete ROS 2
pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

%% ── Rutas dinamicas ──────────────────────────────────────────────────────────
switch mode
    case 'sim'
        csvFile    = fullfile(pkg_dir, 'data', 'lab4', 'sim', 'act1', ...
                              sprintf('gz_fl_data_%d.csv', test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'lab4', 'sim', 'act1', ...
                              sprintf('test%d', test_num));
        mode_label = 'Simulacion';
    case 'real'
        csvFile    = fullfile(pkg_dir, 'data', 'lab4', 'real', 'act1', ...
                              sprintf('hw_fl_data_%d.csv', test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'lab4', 'real', 'act1', ...
                              sprintf('test%d', test_num));
        mode_label = 'Implementacion';
    otherwise
        error('mode debe ser ''sim'' o ''real''.');
end

%% ── Carga de datos ───────────────────────────────────────────────────────────
if ~isfile(csvFile)
    error('Archivo no encontrado:\n  %s\nVerificar mode y test_num.', csvFile);
end
T = readtable(csvFile);
fprintf('[%s] Cargado: %s  (%d muestras)\n', mode_label, csvFile, height(T));

t      = T.t - T.t(1);

q      = [T.q1,      T.q2,      T.q3,      T.q4     ];
q_des  = [T.q1_des,  T.q2_des,  T.q3_des,  T.q4_des ];

dq     = [T.dq1,     T.dq2,     T.dq3,     T.dq4    ];
dq_des = [T.dq1_des, T.dq2_des, T.dq3_des, T.dq4_des];

% Velocidad filtrada (columna nueva; puede no existir en CSVs anteriores)
has_dq_filt = ismember('dq1_filt', T.Properties.VariableNames);
if has_dq_filt
    dq_filt = [T.dq1_filt, T.dq2_filt, T.dq3_filt, T.dq4_filt];
end

tau    = [T.tau1,    T.tau2,    T.tau3,    T.tau4   ];

e_q    = q - q_des;

%% ── Metricas (por articulacion, excluyendo la rampa quintica) ────────────────
m_reg = t >= RAMP_TIME_S;   % regimen: se descarta la transicion inicial
if ~any(m_reg)
    warning('No hay muestras con t >= %.1f s; revisar RAMP_TIME_S.', RAMP_TIME_S);
end

fprintf('\n%s\n', repmat('═', 1, 76));
fprintf(' Metricas FL articular  [%s | test=%d]  (t >= %.1f s, sin rampa)\n', ...
        mode_label, test_num, RAMP_TIME_S);
fprintf('%s\n', repmat('═', 1, 76));
fprintf('%-6s  %-12s  %-12s  %-14s  %-14s  %-8s\n', ...
        'Joint', 'e_max[rad]', 'e_RMS[rad]', 'max|tau|[N·m]', 'tau_RMS[N·m]', 'Sat[%]');
fprintf('%s\n', repmat('-', 1, 76));
for i = 1:4
    e_i   = e_q(m_reg, i);
    tau_i = tau(m_reg, i);
    fprintf('q%-5d  %12.5f  %12.5f  %14.4f  %14.4f  %8.2f\n', i, ...
            max(abs(e_i)), sqrt(mean(e_i.^2)), ...
            max(abs(tau_i)), sqrt(mean(tau_i.^2)), ...
            100 * mean(abs(tau_i) >= TAU_MAX - 1e-6));
end
fprintf('%s\n\n', repmat('═', 1, 76));

%% ── Estilo ───────────────────────────────────────────────────────────────────
lw         = 1.6;
fs         = 11;
fs_title   = 12;
color_ref  = [0.8500 0.3250 0.0980];   % naranja — referencia
color_meas = [0.0000 0.4470 0.7410];   % azul    — medicion (raw)
color_filt = [0.4660 0.6740 0.1880];   % verde   — medicion filtrada
color_tau  = [0.4660 0.6740 0.1880];   % verde   — torque
jointNames = {'Articulacion 1', 'Articulacion 2', ...
              'Articulacion 3', 'Articulacion 4'};
xlims = [t(1), t(end)];

%% ── Figura 1 — Posiciones articulares ────────────────────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 1100 700]);
tl1  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs1 = gobjects(1, 4);
h_ref1 = []; h_mea1 = [];

for i = 1:4
    axs1(i) = nexttile(tl1);
    h2 = plot(t, q(:,i),     '-',  'Color', color_meas, 'LineWidth', lw); hold on;
    h1 = plot(t, q_des(:,i), '--', 'Color', color_ref,  'LineWidth', lw);
    if i == 1; h_ref1 = h1; h_mea1 = h2; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

lgd1 = legend(axs1(1), [h_ref1, h_mea1], {'Referencia', 'Medicion'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd1.Layout.Tile = 'north';
title(tl1, sprintf('[%s] Seguimiento de posiciones articulares', mode_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 2 — Errores de seguimiento articular ──────────────────────────────
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [120 120 1100 560]);
tl2  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs2 = gobjects(1, 4);

for i = 1:4
    axs2(i) = nexttile(tl2);
    plot(t, e_q(:,i), '-', 'Color', color_meas, 'LineWidth', lw);
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$e_{q_%d}$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title([jointNames{i}], 'FontSize', fs_title);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl2, sprintf('[%s] FL Control - Errores de Seguimiento Articular', mode_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 3 — Velocidades articulares ───────────────────────────────────────
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [130 130 1100 700]);
tl3  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs3 = gobjects(1, 4);
h_ref3 = []; h_mea3 = [];

for i = 1:4
    axs3(i) = nexttile(tl3);
    % Usar velocidad filtrada si esta disponible, sino la cruda (CSVs anteriores)
    dq_plot = dq(:,i);
    if has_dq_filt
        dq_plot = dq_filt(:,i);
    end
    h2 = plot(t, dq_plot,      '-',  'Color', color_meas, 'LineWidth', lw); hold on;
    h1 = plot(t, dq_des(:,i),  '--', 'Color', color_ref,  'LineWidth', lw);
    if i == 1; h_ref3 = h1; h_mea3 = h2; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\dot{q}_%d$ [rad/s]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

lgd3 = legend(axs3(1), [h_ref3, h_mea3], {'Referencia', 'Medicion'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd3.Layout.Tile = 'north';
title(tl3, sprintf('[%s] Seguimiento de velocidades articulares', mode_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 4 — Torques de control ────────────────────────────────────────────
figure(4); clf;
set(gcf, 'Color', 'w', 'Position', [140 140 1100 700]);

for i = 1:4
    subplot(2,2,i);
    plot(t, tau(:,i), '-', 'Color', color_tau, 'LineWidth', lw); hold on;
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N\\cdot m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(['Torque de control - ' jointNames{i}], 'FontSize', fs_title);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

sgtitle(sprintf('[%s] FL Control - Torques de control calculados', mode_label), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    % PNG (raster, 300 dpi)
    exportgraphics(figure(1), fullfile(output_dir, 'tracking_plot_q.png'),     'Resolution', 300);
    exportgraphics(figure(2), fullfile(output_dir, 'tracking_plot_error.png'), 'Resolution', 300);
    exportgraphics(figure(3), fullfile(output_dir, 'tracking_plot_dq.png'),    'Resolution', 300);
    exportgraphics(figure(4), fullfile(output_dir, 'torques_plot.png'),        'Resolution', 300);

    % EPS (vectorial, 600 dpi)
    exportgraphics(figure(1), fullfile(output_dir, 'tracking_plot_q.eps'),     'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(2), fullfile(output_dir, 'tracking_plot_error.eps'), 'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(3), fullfile(output_dir, 'tracking_plot_dq.eps'),    'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(4), fullfile(output_dir, 'torques_plot.eps'),        'ContentType', 'vector', 'Resolution', 600);

    fprintf('Graficas guardadas en: %s\n', output_dir);
end
