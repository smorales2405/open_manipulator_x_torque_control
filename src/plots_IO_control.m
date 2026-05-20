%% plots_IO_control.m
% Graficas para Actividad 2 — Control IO (Input-Output Linearization)
% OpenMANIPULATOR-X, Laboratorio 4
%
%   Figura 1 — Seguimiento cartesiano: x, y, z, phi  vs referencia
%   Figura 2 — Errores de seguimiento cartesiano e_x, e_y, e_z, e_phi
%   Figura 3 — Seguimiento de velocidades cartesianas: xdot, ydot, zdot, phidot
%   Figura 4 — Posiciones articulares q1..q4
%   Figura 5 — Torques de control tau1..tau4
%
% Configurar las dos variables de la seccion "Configuracion" y ejecutar.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
mode        = 'sim';   % 'sim'  = simulacion Gazebo
                       % 'real' = implementacion hardware real

test_num    = 1;       % Numero de log

EXPORT_FIGS = false;    % true  = guardar PNG (300 dpi) y EPS vectorial (600 dpi)
                       % false = solo visualizar

% Directorio raiz del paquete ROS 2
pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

%% ── Rutas dinamicas ──────────────────────────────────────────────────────────
switch mode
    case 'sim'
        csvFile    = fullfile(pkg_dir, 'data', 'sim', 'act2', ...
                              sprintf('gz_io_data_%d.csv', test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'sim', 'act2', ...
                              sprintf('test%d', test_num));
        mode_label = 'Simulacion';
    case 'real'
        csvFile    = fullfile(pkg_dir, 'data', 'real', 'act2', ...
                              sprintf('hw_io_data_%d.csv', test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'real', 'act2', ...
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

t = T.t - T.t(1);

q = [T.q1, T.q2, T.q3, T.q4];

y     = [T.x,     T.y,     T.z,     T.phi    ];
y_des = [T.x_des, T.y_des, T.z_des, T.phi_des];

ydot     = [T.xdot,     T.ydot,     T.zdot,     T.phidot    ];
ydot_des = [T.xdot_des, T.ydot_des, T.zdot_des, T.phidot_des];

tau = [T.tau1, T.tau2, T.tau3, T.tau4];

%% ── Estilo ───────────────────────────────────────────────────────────────────
lw        = 1.6;
fs        = 11;
fs_ttl    = 14;
c_meas    = [0.0000 0.4470 0.7410];   % azul    — medicion
c_ref     = [0.8500 0.3250 0.0980];   % naranja — referencia
c_tau     = lines(4);
xlims     = [t(1), t(end)];
jointNames = {'Articulacion 1', 'Articulacion 2', ...
              'Articulacion 3', 'Articulacion 4'};

%% ── Figura 1 — Seguimiento cartesiano ───────────────────────────────────────
ylabels_cart = {'$x$ [m]', '$y$ [m]', '$z$ [m]', '$\phi$ [rad]'};
titles_cart  = {'Posicion x', 'Posicion y', 'Posicion z', 'Orientacion \phi'};

figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 620 1100 540]);
tl1  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs1 = gobjects(1, 4);
h_ref_h = [];
h_mea_h = [];

for i = 1:4
    axs1(i) = nexttile(tl1);
    h1 = plot(t, y_des(:,i), '--', 'Color', c_ref,  'LineWidth', lw); hold on;
    h2 = plot(t, y(:,i),     '-',  'Color', c_meas, 'LineWidth', lw);
    if i == 1
        h_ref_h = h1;
        h_mea_h = h2;
    end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(ylabels_cart{i}, 'Interpreter', 'latex', 'FontSize', fs);
    title(titles_cart{i}, 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

lgd1 = legend(axs1(1), [h_ref_h, h_mea_h], {'Referencia', 'Medicion'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd1.Layout.Tile = 'north';

title(tl1, sprintf('[%s] IO Control - Seguimiento Cartesiano', mode_label), ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');

%% ── Figura 2 — Errores de seguimiento cartesiano ─────────────────────────────
e_y      = y - y_des;
e_labels = {'$e_x$ [m]', '$e_y$ [m]', '$e_z$ [m]', '$e_\phi$ [rad]'};

figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [120 480 1100 540]);
tl2  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl2);
    plot(t, e_y(:,i), '-', 'Color', c_meas, 'LineWidth', lw);
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(e_labels{i}, 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl2, sprintf('[%s] IO Control - Errores de Seguimiento Cartesiano', mode_label), ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');

%% ── Figura 3 — Velocidades cartesianas ──────────────────────────────────────
ylabels_vel = {'$\dot{x}$ [m/s]', '$\dot{y}$ [m/s]', ...
               '$\dot{z}$ [m/s]', '$\dot{\phi}$ [rad/s]'};
titles_vel  = {'Velocidad $\dot{x}$', 'Velocidad $\dot{y}$', ...
               'Velocidad $\dot{z}$', 'Velocidad $\dot{\phi}$'};

figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [130 100 1100 540]);
tl3  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs3 = gobjects(1, 4);
h_ref_v = [];
h_mea_v = [];

for i = 1:4
    axs3(i) = nexttile(tl3);
    h1 = plot(t, ydot_des(:,i), '--', 'Color', c_ref,  'LineWidth', lw); hold on;
    h2 = plot(t, ydot(:,i),     '-',  'Color', c_meas, 'LineWidth', lw);
    if i == 1
        h_ref_v = h1;
        h_mea_v = h2;
    end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(ylabels_vel{i}, 'Interpreter', 'latex', 'FontSize', fs);
    title(titles_vel{i}, 'FontSize', fs, 'Interpreter', 'latex');
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

lgd3 = legend(axs3(1), [h_ref_v, h_mea_v], {'Referencia', 'Medicion'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd3.Layout.Tile = 'north';

title(tl3, sprintf('[%s] IO Control - Seguimiento de Velocidades Cartesianas', mode_label), ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');

%% ── Figura 4 — Posiciones articulares ────────────────────────────────────────
figure(4); clf;
set(gcf, 'Color', 'w', 'Position', [160 -380 1100 520]);
tl4 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl4);
    plot(t, q(:,i), '-', 'Color', c_meas, 'LineWidth', lw);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl4, sprintf('[%s] IO Control - Posiciones Articulares', mode_label), ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');

%% ── Figura 5 — Torques de control ────────────────────────────────────────────
figure(5); clf;
set(gcf, 'Color', 'w', 'Position', [190 -920 1100 500]);
tl5 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl5);
    plot(t, tau(:,i), '-', 'Color', c_tau(i,:), 'LineWidth', lw); hold on;
    yline(0,     ':',  'LineWidth', 0.8);
    yline( 0.82, '--k', 'LineWidth', 0.9, 'Label', '+0.82 N·m');
    yline(-0.82, '--k', 'LineWidth', 0.9, 'Label', '-0.82 N·m');
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N{\\cdot}m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl5, sprintf('[%s] IO Control - Torques de Control', mode_label), ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    % PNG (raster, 300 dpi)
    exportgraphics(figure(1), fullfile(output_dir, 'plot_tracking_cartesian.png'),  'Resolution', 300);
    exportgraphics(figure(2), fullfile(output_dir, 'plot_tracking_error.png'),      'Resolution', 300);
    exportgraphics(figure(3), fullfile(output_dir, 'plot_tracking_velocity.png'),   'Resolution', 300);
    exportgraphics(figure(4), fullfile(output_dir, 'plot_q.png'),                   'Resolution', 300);
    exportgraphics(figure(5), fullfile(output_dir, 'torques_plot.png'),             'Resolution', 300);

    % EPS (vectorial, 600 dpi)
    exportgraphics(figure(1), fullfile(output_dir, 'plot_tracking_cartesian.eps'),  'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(2), fullfile(output_dir, 'plot_tracking_error.eps'),      'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(3), fullfile(output_dir, 'plot_tracking_velocity.eps'),   'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(4), fullfile(output_dir, 'plot_q.eps'),                   'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(5), fullfile(output_dir, 'torques_plot.eps'),             'ContentType', 'vector', 'Resolution', 600);

    fprintf('Graficas guardadas en: %s\n', output_dir);
end
