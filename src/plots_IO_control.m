%% plots_IO_control.m
% Graficas del controlador Input-Output Linearization en espacio cartesiano.
%
%   Figura 1 — Posiciones articulares q1..q4  [rad]
%   Figura 2 — Seguimiento cartesiano: x, y, z, phi  vs referencia
%   Figura 3 — Torques de control tau1..tau4  [N·m]
%   Figura 4 — Velocidades cartesianas: xdot, ydot, zdot, phidot  vs referencia
%
% Carga automaticamente fl_xyz_data.csv desde data/ (junto al script).

clear; clc; close all;

%% 1. Carga de datos
script_dir = fileparts(mfilename('fullpath'));
data_dir   = fullfile(script_dir, '..', 'data');
filename   = 'fl_xyz_new_data.csv';

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

%% 5. Figura 3 — Torques de control
figure(3); clf;
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

%% 6. Figura 4 — Velocidades cartesianas
ydot_labels = {'$\dot{x}$ [m/s]', '$\dot{y}$ [m/s]', '$\dot{z}$ [m/s]', '$\dot{\phi}$ [rad/s]'};

figure(4); clf;
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

%% 7. Exportacion opcional para informe

test_num = 1;

% Carpeta de salida
output_dir = fullfile('../data/plots/cartesian/', sprintf('test%d', test_num));

% Crear carpeta si no existe
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% Guardar figuras
exportgraphics(figure(1), fullfile(output_dir, 'plot_q.png'), ...
    'Resolution', 300);

exportgraphics(figure(2), fullfile(output_dir, 'plot_tracking_cartesian.png'), ...
    'Resolution', 300);

exportgraphics(figure(3), fullfile(output_dir, 'torques_plot.png'), ...
    'Resolution', 300);

exportgraphics(figure(4), fullfile(output_dir, 'plot_ydot_cartesian.png'), ...
    'Resolution', 300);