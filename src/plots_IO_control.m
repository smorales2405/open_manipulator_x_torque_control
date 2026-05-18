%% plots_IO_control.m
% Graficas del controlador Input-Output Linearization en espacio cartesiano.
%
%   Figura 1 — Posiciones articulares q1..q4  [rad]
%   Figura 2 — Seguimiento cartesiano: x, y, z, phi  vs referencia
%   Figura 3 — Torques de control tau1..tau4  [N·m]
%   Figura 4 — Velocidades cartesianas: xdot, ydot, zdot, phidot  vs referencia
%
% Carga automaticamente data/act2/io_data_<test_num>.csv (junto al script).

clear; clc; close all;

%% 1. Carga de datos
script_dir = fileparts(mfilename('fullpath'));
data_dir   = fullfile(script_dir, '..', 'data', 'act2');

test_num    = 1;      % <-- Cambiar aqui para seleccionar la prueba
EXPORT_FIGS = false;  % true = exportar PNG y EPS en data/plots/act2/test<N>/

filename = sprintf('io_data_%d.csv', test_num);
fprintf('Cargando: %s\n', filename);

filepath = fullfile(data_dir, filename);
if ~isfile(filepath)
    error('No se encontro %s.\nEjecutar io_control_node primero.', filepath);
end

T = readtable(filepath);
fprintf('Cargado: %s  (%d muestras)\n', filename, height(T));

t      = T.t - T.t(1);
q      = [T.q1, T.q2, T.q3, T.q4];
y      = [T.x,        T.y,        T.z,        T.phi       ];
y_des  = [T.x_des,    T.y_des,    T.z_des,    T.phi_des   ];
ydot   = [T.xdot,     T.ydot,     T.zdot,     T.phidot    ];
ydot_des = [T.xdot_des, T.ydot_des, T.zdot_des, T.phidot_des];
tau    = [T.tau1, T.tau2, T.tau3, T.tau4];

%% 2. Estilo
lw     = 1.6;
fs     = 11;
c_real = [0.0000 0.4470 0.7410];   % azul    — medicion/simulacion
c_ref  = [0.8500 0.3250 0.0980];   % naranja — referencia
c_tau  = lines(4);
xlims  = [t(1), t(end)];

%% 3. Figura 1 — Posiciones articulares
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 620 1100 520]);
tl1 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl1);
    plot(t, q(:,i), '-', 'Color', c_real, 'LineWidth', lw);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl1, 'IO Control - Posiciones Articulares', ...
      'FontSize', 14, 'FontWeight', 'bold');

%% 4. Figura 2 — Seguimiento cartesiano (unico titulo y leyenda)
ylabels = {'$x$ [m]', '$y$ [m]', '$z$ [m]', '$\phi$ [rad]'};

figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [130 100 1100 540]);
tl2  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs2 = gobjects(1, 4);
h_ref_h = [];
h_sim_h = [];

for i = 1:4
    axs2(i) = nexttile(tl2);
    h1 = plot(t, y_des(:,i), '--', 'Color', c_ref,  'LineWidth', lw); hold on;
    h2 = plot(t, y(:,i),     '-',  'Color', c_real, 'LineWidth', lw);
    if i == 1
        h_ref_h = h1;
        h_sim_h = h2;
    end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(ylabels{i}, 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

lgd2 = legend(axs2(1), [h_ref_h, h_sim_h], {'Referencia', 'Simulacion'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd2.Layout.Tile = 'north';
title(tl2, 'IO Control - Seguimiento Cartesiano', ...
      'FontSize', 14, 'FontWeight', 'bold');

%% 5. Figura 3 — Velocidades cartesianas
ydot_labels = {'$\dot{x}$ [m/s]', '$\dot{y}$ [m/s]', '$\dot{z}$ [m/s]', '$\dot{\phi}$ [rad/s]'};

figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [160 -380 1100 540]);
tl4  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs4 = gobjects(1, 4);
h_ref4 = [];
h_sim4 = [];

for i = 1:4
    axs4(i) = nexttile(tl4);
    h1 = plot(t, ydot_des(:,i), '--', 'Color', c_ref,  'LineWidth', lw); hold on;
    h2 = plot(t, ydot(:,i),     '-',  'Color', c_real, 'LineWidth', lw);
    if i == 1
        h_ref4 = h1;
        h_sim4 = h2;
    end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(ydot_labels{i}, 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

lgd4 = legend(axs4(1), [h_ref4, h_sim4], {'Referencia', 'Simulacion'}, ...
    'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd4.Layout.Tile = 'north';
title(tl4, 'IO Control - Velocidades Cartesianas', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% 6. Figura 4 — Torques de control
figure(4); clf;
set(gcf, 'Color', 'w', 'Position', [160 -380 1100 500]);
tl3 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl3);
    plot(t, tau(:,i), '-', 'Color', c_tau(i,:), 'LineWidth', lw);
    yline(0, ':', 'LineWidth', 0.8);
    yline( 0.82, '--k', 'LineWidth', 0.8);   % limite superior
    yline(-0.82, '--k', 'LineWidth', 0.8);   % limite inferior
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N{\\cdot}m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl3, 'IO Control - Torques de Control', ...
      'FontSize', 14, 'FontWeight', 'bold');



%% 7. Exportacion de figuras
if EXPORT_FIGS
    out_dir = fullfile(data_dir, 'plots', sprintf('test%d', test_num));
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    % PNG (raster, 300 dpi)
    exportgraphics(figure(1), fullfile(out_dir, 'plot_q.png'),                  'Resolution', 300);
    exportgraphics(figure(2), fullfile(out_dir, 'plot_tracking_cartesian.png'),  'Resolution', 300);
    exportgraphics(figure(3), fullfile(out_dir, 'plot_ydot_cartesian.png'),      'Resolution', 300);
    exportgraphics(figure(4), fullfile(out_dir, 'plot_tau.png'),                 'Resolution', 300);


    % EPS (vectorial)
    exportgraphics(figure(1), fullfile(out_dir, 'plot_q.eps'),                  'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(2), fullfile(out_dir, 'plot_tracking_cartesian.eps'),  'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(3), fullfile(out_dir, 'plot_ydot_cartesian.eps'),      'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(4), fullfile(out_dir, 'plot_tau.eps'),                 'ContentType', 'vector', 'Resolution', 600);

    fprintf('Figuras exportadas en: %s\n', out_dir);
end