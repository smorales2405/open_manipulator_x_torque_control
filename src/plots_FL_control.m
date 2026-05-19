%% plots_FL_control.m
% Graficas para Actividad 1 — Control FL (Feedback Linearization)
% OpenMANIPULATOR-X, Laboratorio 4
%
%   Figura 1 — Seguimiento de posiciones articulares  q1..q4
%   Figura 2 — Seguimiento de velocidades articulares dq1..dq4
%   Figura 3 — Torques de control tau1..tau4
%
% Configurar las dos variables de la seccion "Configuracion" y ejecutar.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
mode     = 'sim';   % 'sim'  = simulacion Gazebo
                    % 'real' = implementacion hardware real
test_num = 1;       % Numero de log (coincide con log_id del nodo C++)

% Directorio raiz del paquete ROS 2
pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

%% ── Rutas dinamicas ──────────────────────────────────────────────────────────
switch mode
    case 'sim'
        csvFile    = fullfile(pkg_dir, 'data', 'sim', 'act1', ...
                              sprintf('gz_fl_data_%d.csv', test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'sim', 'act1', ...
                              sprintf('test%d', test_num));
        mode_label = 'Simulacion';
    case 'real'
        csvFile    = fullfile(pkg_dir, 'data', 'real', 'act1', ...
                              sprintf('hw_fl_data_%d.csv', test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'real', 'act1', ...
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

tau    = [T.tau1,    T.tau2,    T.tau3,    T.tau4   ];

%% ── Estilo ───────────────────────────────────────────────────────────────────
lw         = 1.6;
fs         = 11;
fs_title   = 12;
color_ref  = [0.8500 0.3250 0.0980];   % naranja — referencia
color_meas = [0.0000 0.4470 0.7410];   % azul    — medicion
color_tau  = [0.4660 0.6740 0.1880];   % verde   — torque
jointNames = {'Articulacion 1', 'Articulacion 2', ...
              'Articulacion 3', 'Articulacion 4'};
xlims = [t(1), t(end)];

%% ── Figura 1 — Posiciones articulares ────────────────────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 1100 700]);

for i = 1:4
    subplot(2,2,i);
    plot(t, q_des(:,i), '--', 'Color', color_ref,  'LineWidth', lw); hold on;
    plot(t, q(:,i),     '-',  'Color', color_meas, 'LineWidth', lw);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(['Seguimiento de posicion - ' jointNames{i}], 'FontSize', fs_title);
    legend({sprintf('$q_{%d,des}$', i), sprintf('$q_{%d,med}$', i)}, ...
           'Interpreter', 'latex', 'Location', 'best');
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

sgtitle(sprintf('[%s] Seguimiento de posiciones articulares', mode_label), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 2 — Velocidades articulares ───────────────────────────────────────
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [120 120 1100 700]);

for i = 1:4
    subplot(2,2,i);
    plot(t, dq_des(:,i), '--', 'Color', color_ref,  'LineWidth', lw); hold on;
    plot(t, dq(:,i),     '-',  'Color', color_meas, 'LineWidth', lw);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\dot{q}_%d$ [rad/s]', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(['Seguimiento de velocidad - ' jointNames{i}], 'FontSize', fs_title);
    legend({sprintf('$\\dot{q}_{%d,des}$', i), sprintf('$\\dot{q}_{%d,med}$', i)}, ...
           'Interpreter', 'latex', 'Location', 'best');
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

sgtitle(sprintf('[%s] Seguimiento de velocidades articulares', mode_label), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 3 — Torques de control ────────────────────────────────────────────
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [140 140 1100 700]);

for i = 1:4
    subplot(2,2,i);
    plot(t, tau(:,i), '-', 'Color', color_tau, 'LineWidth', lw); hold on;
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N\\cdot m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(['Torque de control - ' jointNames{i}], 'FontSize', fs_title);
    legend({sprintf('$\\tau_%d$', i)}, 'Interpreter', 'latex', 'Location', 'best');
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

sgtitle(sprintf('[%s] Torques de control calculados', mode_label), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Exportacion ──────────────────────────────────────────────────────────────
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

exportgraphics(figure(1), fullfile(output_dir, 'tracking_plot_q.jpg'),  'Resolution', 300);
exportgraphics(figure(2), fullfile(output_dir, 'tracking_plot_dq.jpg'), 'Resolution', 300);
exportgraphics(figure(3), fullfile(output_dir, 'torques_plot.jpg'),     'Resolution', 300);

fprintf('Graficas guardadas en: %s\n', output_dir);
