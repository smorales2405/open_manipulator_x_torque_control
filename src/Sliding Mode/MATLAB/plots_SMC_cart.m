%% plots_SMC_cart.m
% Graficas para Actividad 2 — Control por Modo Deslizante en espacio cartesiano
% OpenMANIPULATOR-X, Laboratorio 6
%
%   Figura 1 — Seguimiento cartesiano: x, y, z, phi  vs referencia
%   Figura 2 — Errores de seguimiento cartesiano e_x, e_y, e_z, e_phi
%   Figura 3 — Seguimiento de velocidades cartesianas: xdot, ydot, zdot, phidot
%   Figura 4 — Posiciones articulares q1..q4
%   Figura 5 — Superficies deslizantes s1..s4
%   Figura 6 — Torques de control tau1..tau4
%
%   Metricas reportadas en consola (Tabla 5 de la guia):
%     Por salida cartesiana: e_max [m o rad]  |  e_RMS [m o rad]
%     Por articulacion:      max|tau| [N·m]   |  tau_RMS [N·m]  |  Sat [%]  |  TV(tau)
%     Global:                max kappa(J_y)
%
% Configurar la seccion "Configuracion" y ejecutar.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
mode        = 'sim';    % 'sim'  = simulacion Gazebo (gz_smc_cart_node)
                        % 'real' = implementacion hardware (hw_smc_cart_node)

rho_func    = 'sign';   % Funcion de conmutacion: 'sign' | 'sat'

test_num    = 1;        % Identificador del ensayo (test_num usado al lanzar el nodo)

EXPORT_FIGS = false;    % true  = guardar PNG (300 dpi) y EPS vectorial (600 dpi)
                        % false = solo visualizar

TAU_MAX     = 1.2;      % [N·m] limite de torque (debe coincidir con el nodo)

% Directorio raiz del paquete ROS 2
pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

%% ── Rutas dinamicas ──────────────────────────────────────────────────────────
switch mode
    case 'sim'
        csv_file   = fullfile(pkg_dir, 'data', 'lab6', 'sim', 'act2', ...
                              sprintf('gz_smc_cart_%s_%d.csv', rho_func, test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'lab6', 'sim', 'act2', ...
                              sprintf('test%d_%s', test_num, rho_func));
        mode_label = 'Simulacion';
    case 'real'
        csv_file   = fullfile(pkg_dir, 'data', 'lab6', 'real', 'act2', ...
                              sprintf('hw_smc_cart_%s_%d.csv', rho_func, test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'lab6', 'real', 'act2', ...
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

t = T.t - T.t(1);

% Estados articulares
q = [T.q1, T.q2, T.q3, T.q4];

% Salida cartesiana medida y deseada
y     = [T.x,     T.y,     T.z,     T.phi    ];
y_des = [T.x_des, T.y_des, T.z_des, T.phi_des];

% Velocidades cartesianas medida y deseada
ydot     = [T.xdot,     T.ydot,     T.zdot,     T.phidot    ];
ydot_des = [T.xdot_des, T.ydot_des, T.zdot_des, T.phidot_des];

% Superficies deslizantes cartesianas
s_y = [T.s1, T.s2, T.s3, T.s4];

% Torques y banderas de saturacion
tau = [T.tau1, T.tau2, T.tau3, T.tau4];
sat = [T.sat1, T.sat2, T.sat3, T.sat4];

% Condicionamiento del Jacobiano
cond_J = T.cond_J;

% Errores cartesianos
e_y = y - y_des;

%% ── Metricas (Tabla 5 de la guia) ───────────────────────────────────────────
cartNames  = {'x [m]', 'y [m]', 'z [m]', 'phi [rad]'};
cartSymbol = {'e_x', 'e_y', 'e_z', 'e_phi'};

fprintf('\n%s\n', repmat('═', 1, 78));
fprintf(' Metricas SMC Cartesiano  [%s | rho=%s | test=%d]\n', ...
        mode_label, rho_func, test_num);
fprintf('%s\n', repmat('═', 1, 78));

fprintf('\n  Errores de seguimiento cartesiano:\n');
fprintf('  %-10s  %-14s  %-14s\n', 'Salida', 'e_max[m/rad]', 'e_RMS[m/rad]');
fprintf('  %s\n', repmat('-', 1, 42));
for i = 1:4
    e_max_i = max(abs(e_y(:,i)));
    e_rms_i = sqrt(mean(e_y(:,i).^2));
    fprintf('  %-10s  %14.6f  %14.6f\n', cartNames{i}, e_max_i, e_rms_i);
end

fprintf('\n  Metricas de torque articular:\n');
fprintf('  %-6s  %-14s  %-14s  %-10s  %-10s\n', 'Joint', 'max|tau|[N·m]', 'tau_RMS[N·m]', 'Sat[%]', 'TV(tau)');
fprintf('  %s\n', repmat('-', 1, 62));
for i = 1:4
    tau_max_i = max(abs(tau(:,i)));
    tau_rms_i = sqrt(mean(tau(:,i).^2));
    sat_pct_i = 100 * mean(sat(:,i));
    tv_i      = sum(abs(diff(tau(:,i))));
    fprintf('  tau%-3d  %14.4f  %14.4f  %10.2f  %10.4f\n', i, tau_max_i, tau_rms_i, sat_pct_i, tv_i);
end

fprintf('\n  Condicionamiento del Jacobiano:\n');
fprintf('  max kappa(J_y) = %.2f\n', max(cond_J));

fprintf('%s\n\n', repmat('═', 1, 78));

%% ── Estilo comun ─────────────────────────────────────────────────────────────
lw         = 1.6;
fs         = 11;
fs_title   = 14;
color_ref  = [0.8500 0.3250 0.0980];   % naranja — referencia
color_meas = [0.0000 0.4470 0.7410];   % azul    — medicion
c_tau      = lines(4);
xlims      = [t(1), t(end)];

jointNames = {'Articulacion 1', 'Articulacion 2', ...
              'Articulacion 3', 'Articulacion 4'};

ylabels_cart = {'$x$ [m]', '$y$ [m]', '$z$ [m]', '$\phi$ [rad]'};
titles_cart  = {'Posicion $x$', 'Posicion $y$', 'Posicion $z$', 'Orientacion $\phi$'};

% Etiqueta de la funcion de conmutacion para los titulos
rho_tex = struct('sign', 'sign(s)', 'sat', 'sat(s/\phi)');
if isfield(rho_tex, rho_func)
    rho_label = rho_tex.(rho_func);
else
    rho_label = rho_func;
end
sub_label = sprintf('[%s | \\rho = %s]', mode_label, rho_label);

%% ── Figura 1 — Seguimiento cartesiano ───────────────────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [50 620 1100 540]);
tl1  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs1 = gobjects(1, 4);
h_ref1 = []; h_mea1 = [];

for i = 1:4
    axs1(i) = nexttile(tl1);
    h2 = plot(t, y(:,i),     '-',  'Color', color_meas, 'LineWidth', lw); hold on;
    h1 = plot(t, y_des(:,i), '--', 'Color', color_ref,  'LineWidth', lw);
    if i == 1; h_ref1 = h1; h_mea1 = h2; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(ylabels_cart{i}, 'Interpreter', 'latex', 'FontSize', fs);
    title(titles_cart{i}, 'FontSize', fs, 'Interpreter', 'latex');
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

lgd1 = legend(axs1(1), [h_ref1, h_mea1], {'Referencia', 'Medicion'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd1.Layout.Tile = 'north';
title(tl1, sprintf('SMC Cartesiano — Seguimiento Cartesiano  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold', 'Interpreter', 'tex');

%% ── Figura 2 — Errores de seguimiento cartesiano ────────────────────────────
ylabels_err = {'$e_x$ [m]', '$e_y$ [m]', '$e_z$ [m]', '$e_\phi$ [rad]'};

figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [70 480 1100 540]);
tl2 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl2);
    plot(t, e_y(:,i), '-', 'Color', color_meas, 'LineWidth', lw); hold on;
    yline(0, ':', 'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(ylabels_err{i}, 'Interpreter', 'latex', 'FontSize', fs);
    title(titles_cart{i}, 'FontSize', fs, 'Interpreter', 'latex');
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl2, sprintf('SMC Cartesiano — Errores de Seguimiento  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold', 'Interpreter', 'tex');

%% ── Figura 3 — Seguimiento de velocidades cartesianas ───────────────────────
ylabels_vel = {'$\dot{x}$ [m/s]', '$\dot{y}$ [m/s]', ...
               '$\dot{z}$ [m/s]', '$\dot{\phi}$ [rad/s]'};
titles_vel  = {'Velocidad $\dot{x}$', 'Velocidad $\dot{y}$', ...
               'Velocidad $\dot{z}$', 'Velocidad $\dot{\phi}$'};

figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [90 100 1100 540]);
tl3  = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs3 = gobjects(1, 4);
h_ref3 = []; h_mea3 = [];

for i = 1:4
    axs3(i) = nexttile(tl3);
    h2 = plot(t, ydot(:,i),     '-',  'Color', color_meas, 'LineWidth', lw); hold on;
    h1 = plot(t, ydot_des(:,i), '--', 'Color', color_ref,  'LineWidth', lw);
    if i == 1; h_ref3 = h1; h_mea3 = h2; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(ylabels_vel{i}, 'Interpreter', 'latex', 'FontSize', fs);
    title(titles_vel{i}, 'FontSize', fs, 'Interpreter', 'latex');
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

lgd3 = legend(axs3(1), [h_ref3, h_mea3], {'Referencia', 'Medicion'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd3.Layout.Tile = 'north';
title(tl3, sprintf('SMC Cartesiano — Velocidades Cartesianas  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold', 'Interpreter', 'tex');

%% ── Figura 4 — Posiciones articulares q1..q4 ────────────────────────────────
figure(4); clf;
set(gcf, 'Color', 'w', 'Position', [110 -340 1100 520]);
tl4 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl4);
    plot(t, q(:,i), '-', 'Color', color_meas, 'LineWidth', lw);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl4, sprintf('SMC Cartesiano — Posiciones Articulares  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold', 'Interpreter', 'tex');

%% ── Figura 5 — Superficies deslizantes s1..s4 ───────────────────────────────
ylabels_s = {'$s_x$ [m/s]', '$s_y$ [m/s]', '$s_z$ [m/s]', '$s_\phi$ [rad/s]'};
titles_s  = {'Superficie $s_x$', 'Superficie $s_y$', ...
             'Superficie $s_z$', 'Superficie $s_\phi$'};

figure(5); clf;
set(gcf, 'Color', 'w', 'Position', [130 -680 1100 520]);
tl5 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl5);
    plot(t, s_y(:,i), '-', 'Color', c_tau(i,:), 'LineWidth', lw); hold on;
    yline(0, ':', 'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(ylabels_s{i}, 'Interpreter', 'latex', 'FontSize', fs);
    title(titles_s{i}, 'FontSize', fs, 'Interpreter', 'latex');
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl5, sprintf('SMC Cartesiano — Superficies Deslizantes  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold', 'Interpreter', 'tex');

%% ── Figura 6 — Torques de control tau1..tau4 ────────────────────────────────
figure(6); clf;
set(gcf, 'Color', 'w', 'Position', [130 -840 1100 520]);
tl6 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl6);
    plot(t, tau(:,i), '-', 'Color', c_tau(i,:), 'LineWidth', lw); hold on;
    yline(0,          ':',   'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]);
    yline( TAU_MAX, '--k',   'LineWidth', 1.0, 'Label', sprintf('+%.1f N·m', TAU_MAX));
    yline(-TAU_MAX, '--k',   'LineWidth', 1.0, 'Label', sprintf('\x2212%.1f N·m', TAU_MAX));
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N{\\cdot}m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs);
    ylim([-TAU_MAX * 1.25, TAU_MAX * 1.25]);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl6, sprintf('SMC Cartesiano — Torques de Control  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold', 'Interpreter', 'tex');

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    % PNG (raster, 300 dpi)
    exportgraphics(figure(1), fullfile(output_dir, 'plot_cart_tracking.png'),  'Resolution', 300);
    exportgraphics(figure(2), fullfile(output_dir, 'plot_cart_error.png'),     'Resolution', 300);
    exportgraphics(figure(3), fullfile(output_dir, 'plot_cart_velocity.png'),  'Resolution', 300);
    exportgraphics(figure(4), fullfile(output_dir, 'plot_q.png'),              'Resolution', 300);
    exportgraphics(figure(5), fullfile(output_dir, 'plot_sliding_surfaces.png'), 'Resolution', 300);
    exportgraphics(figure(6), fullfile(output_dir, 'plot_torques.png'),        'Resolution', 300);

    % EPS (vectorial, 600 dpi)
    exportgraphics(figure(1), fullfile(output_dir, 'plot_cart_tracking.eps'),  'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(2), fullfile(output_dir, 'plot_cart_error.eps'),     'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(3), fullfile(output_dir, 'plot_cart_velocity.eps'),  'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(4), fullfile(output_dir, 'plot_q.eps'),              'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(5), fullfile(output_dir, 'plot_sliding_surfaces.eps'), 'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(6), fullfile(output_dir, 'plot_torques.eps'),        'ContentType', 'vector', 'Resolution', 600);

    fprintf('Graficas guardadas en:\n  %s\n', output_dir);
end
