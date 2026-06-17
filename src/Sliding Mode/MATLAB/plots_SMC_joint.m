%% plots_SMC_q.m
% Graficas para Actividad 1 — Control por Modo Deslizante en espacio articular
% OpenMANIPULATOR-X, Laboratorio 6
%
%   Figura 1 — Seguimiento de posiciones articulares  q1..q4
%   Figura 2 — Error de seguimiento articular         e_q1..e_q4
%   Figura 3 — Seguimiento de velocidades articulares dq1..dq4
%   Figura 4 — Superficies deslizantes                s1..s4
%   Figura 5 — Torques de control                     tau1..tau4
%
%   Metricas reportadas en consola (Tabla 3 de la guia, por articulacion):
%     e_q,max [rad]  |  e_q,RMS [rad]  |  max|tau| [N·m]  |  tau_RMS [N·m]  |  Sat [%]  |  TV(tau)
%
% Configurar la seccion "Configuracion" y ejecutar.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
mode        = 'sim';    % 'sim'  = simulacion Gazebo (gz_SMC_q_node)
                        % 'real' = implementacion hardware (hw_smc_q_node)

rho_func    = 'sat';   % Funcion de conmutacion usada: 'sign' | 'sat'

test_num    = 1;        % Identificador del ensayo (test_num usado al lanzar el nodo)

EXPORT_FIGS = false;    % true  = guardar PNG (300 dpi) y EPS vectorial (600 dpi)
                        % false = solo visualizar

TAU_MAX     = 1.2;      % [N·m] limite de torque (debe coincidir con el nodo)

% Directorio raiz del paquete ROS 2
pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

%% ── Rutas dinamicas ──────────────────────────────────────────────────────────
switch mode
    case 'sim'
        csv_file   = fullfile(pkg_dir, 'data', 'lab6', 'sim', 'act1', ...
                              sprintf('gz_smc_joint_%s_%d.csv', rho_func, test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'lab6', 'sim', 'act1', ...
                              sprintf('test%d_%s', test_num, rho_func));
        mode_label = 'Simulacion';
    case 'real'
        csv_file   = fullfile(pkg_dir, 'data', 'lab6', 'real', 'act1', ...
                              sprintf('hw_smc_art_%s_%d.csv', rho_func, test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'lab6', 'real', 'act1', ...
                              sprintf('test%d_%s', test_num, rho_func));
        mode_label = 'Implementacion';
    otherwise
        error('mode debe ser ''sim'' o ''real''.');
end

%% ── Carga de datos ───────────────────────────────────────────────────────────
if ~isfile(csv_file)
    error('Archivo no encontrado:\n  %s\nVerificar mode, rho_func y test_num.', csv_file);
end
T = readtable(csv_file);
fprintf('[%s | rho=%s | test=%d]  Cargado: %s  (%d muestras)\n', ...
        mode_label, rho_func, test_num, csv_file, height(T));

t      = T.t - T.t(1);

q      = [T.q1,      T.q2,      T.q3,      T.q4     ];
q_des  = [T.q1_des,  T.q2_des,  T.q3_des,  T.q4_des ];

dq     = [T.dq1,     T.dq2,     T.dq3,     T.dq4    ];
dq_des = [T.dq1_des, T.dq2_des, T.dq3_des, T.dq4_des];

s_q    = [T.s1,      T.s2,      T.s3,      T.s4     ];

tau    = [T.tau1,    T.tau2,    T.tau3,    T.tau4   ];
sat    = [T.sat1,    T.sat2,    T.sat3,    T.sat4   ];   % 0/1 por muestra

e_q    = q - q_des;

%% ── Metricas (Tabla 3 de la guia) ───────────────────────────────────────────
fprintf('\n%s\n', repmat('═', 1, 88));
fprintf(' Metricas SMC articular  [%s | rho=%s | test=%d]\n', ...
        mode_label, rho_func, test_num);
fprintf('%s\n', repmat('═', 1, 88));
fprintf('%-6s  %-12s  %-12s  %-14s  %-14s  %-10s  %-10s\n', ...
        'Joint', 'e_max[rad]', 'e_RMS[rad]', 'max|tau|[N·m]', 'tau_RMS[N·m]', 'Sat[%]', 'TV(tau)');
fprintf('%s\n', repmat('-', 1, 88));
for i = 1:4
    e_max_i   = max(abs(e_q(:,i)));
    e_rms_i   = sqrt(mean(e_q(:,i).^2));
    tau_max_i = max(abs(tau(:,i)));
    tau_rms_i = sqrt(mean(tau(:,i).^2));
    sat_pct_i = 100 * mean(sat(:,i));
    tv_i      = sum(abs(diff(tau(:,i))));
    fprintf('q%-5d  %12.5f  %12.5f  %14.4f  %14.4f  %10.2f  %10.4f\n', ...
            i, e_max_i, e_rms_i, tau_max_i, tau_rms_i, sat_pct_i, tv_i);
end
fprintf('%s\n\n', repmat('═', 1, 88));

%% ── Estilo comun ─────────────────────────────────────────────────────────────
lw         = 1.6;
fs         = 11;
fs_title   = 14;
color_ref  = [0.8500 0.3250 0.0980];   % naranja — referencia
color_meas = [0.0000 0.4470 0.7410];   % azul    — medicion
color_surf = [0.4940 0.1840 0.5560];   % violeta — superficies deslizantes
c_tau      = lines(4);                 % un color distintivo por articulacion
jointNames = {'Articulacion 1', 'Articulacion 2', ...
              'Articulacion 3', 'Articulacion 4'};
xlims      = [t(1), t(end)];

% Etiqueta de la funcion de conmutacion para los titulos (texto plano)
rho_tex = struct('sign', 'sign(s)', 'sat', 'sat(s/phi)');
if isfield(rho_tex, rho_func)
    rho_label = rho_tex.(rho_func);
else
    rho_label = rho_func;
end
sub_label = sprintf('[%s | rho = %s]', mode_label, rho_label);

%% ── Figura 1 — Seguimiento de posiciones articulares ─────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [50 620 1100 680]);
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
    title(jointNames{i}, 'FontSize', fs_title - 2);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

lgd1 = legend(axs1(1), [h_ref1, h_mea1], {'Referencia', 'Medicion'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd1.Layout.Tile = 'north';
title(tl1, sprintf('SMC Articular — Seguimiento de Posiciones  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Figura 2 — Error de seguimiento articular ────────────────────────────────
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [70 390 1100 620]);
tl2 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl2);
    plot(t, e_q(:,i), '-', 'Color', color_meas, 'LineWidth', lw); hold on;
    yline(0, ':', 'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$e_{q_%d}$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title - 2);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl2, sprintf('SMC Articular — Error de Seguimiento Articular  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Figura 3 — Seguimiento de velocidades articulares ────────────────────────
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [90 160 1100 680]);
tl3  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs3 = gobjects(1, 4);
h_ref3 = []; h_mea3 = [];

for i = 1:4
    axs3(i) = nexttile(tl3);
    h2 = plot(t, dq(:,i),     '-',  'Color', color_meas, 'LineWidth', lw); hold on;
    h1 = plot(t, dq_des(:,i), '--', 'Color', color_ref,  'LineWidth', lw);
    if i == 1; h_ref3 = h1; h_mea3 = h2; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\dot{q}_%d$ [rad/s]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title - 2);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

lgd3 = legend(axs3(1), [h_ref3, h_mea3], {'Referencia', 'Medicion'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd3.Layout.Tile = 'north';
title(tl3, sprintf('SMC Articular — Seguimiento de Velocidades  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Figura 4 — Superficies deslizantes ───────────────────────────────────────
figure(4); clf;
set(gcf, 'Color', 'w', 'Position', [110 -80 1100 680]);
tl4 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl4);
    plot(t, s_q(:,i), '-', 'Color', color_surf, 'LineWidth', lw); hold on;
    yline(0, ':', 'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$s_{q_%d}$ [rad/s]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title - 2);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl4, sprintf('SMC Articular — Superficies Deslizantes  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Figura 5 — Torques de control ────────────────────────────────────────────
figure(5); clf;
set(gcf, 'Color', 'w', 'Position', [130 -330 1100 680]);
tl5 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl5);
    plot(t, tau(:,i), '-', 'Color', c_tau(i,:), 'LineWidth', lw); hold on;
    yline( 0,        ':',   'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]);
    yline( TAU_MAX, '--k',  'LineWidth', 1.0, 'Label', sprintf('+%.1f N·m', TAU_MAX));
    yline(-TAU_MAX, '--k',  'LineWidth', 1.0, 'Label', sprintf('\x2212%.1f N·m', TAU_MAX));
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N{\\cdot}m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title - 2);
    ylim([-TAU_MAX * 1.25, TAU_MAX * 1.25]);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl5, sprintf('SMC Articular — Torques de Control  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    % PNG (raster, 300 dpi)
    exportgraphics(figure(1), fullfile(output_dir, 'plot_q_tracking.png'),  'Resolution', 300);
    exportgraphics(figure(2), fullfile(output_dir, 'plot_q_error.png'),     'Resolution', 300);
    exportgraphics(figure(3), fullfile(output_dir, 'plot_dq_tracking.png'), 'Resolution', 300);
    exportgraphics(figure(4), fullfile(output_dir, 'plot_surfaces.png'),    'Resolution', 300);
    exportgraphics(figure(5), fullfile(output_dir, 'plot_torques.png'),     'Resolution', 300);

    % EPS (vectorial, 600 dpi)
    exportgraphics(figure(1), fullfile(output_dir, 'plot_q_tracking.eps'),  'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(2), fullfile(output_dir, 'plot_q_error.eps'),     'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(3), fullfile(output_dir, 'plot_dq_tracking.eps'), 'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(4), fullfile(output_dir, 'plot_surfaces.eps'),    'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(5), fullfile(output_dir, 'plot_torques.eps'),     'ContentType', 'vector', 'Resolution', 600);

    fprintf('Graficas guardadas en:\n  %s\n', output_dir);
end
