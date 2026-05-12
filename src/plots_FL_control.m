%% plots_FL_control.m
% Grafica las senales del controlador Feedback Linearization:
%   Figura 1 — Posiciones articulares:  q vs q_des   [rad]
%   Figura 2 — Velocidades articulares: dq vs dq_des [rad/s]
%   Figura 3 — Torques de control:      tau1..tau4   [N·m]
%
% Uso:
%   - Dejar filename = '' para cargar el CSV fl_data_*.csv mas reciente.
%   - O asignar:  filename = 'fl_data_20260512_120000.csv';
%   - El script localiza data/ automaticamente junto a si mismo.

clear; clc; close all;

%% 1. Carga de datos
script_dir = fileparts(mfilename('fullpath'));
data_dir   = fullfile(script_dir, '..', 'data');

filename = 'fl_data_20260512_141209.csv';   % '' = archivo mas reciente

if isempty(filename)
    files = dir(fullfile(data_dir, 'fl_data_*.csv'));
    if isempty(files)
        error('No hay archivos fl_data_*.csv en %s.\nEjecutar fl_control_node primero.', data_dir);
    end
    [~, idx] = max([files.datenum]);
    filename  = files(idx).name;
    fprintf('Cargando: %s\n', filename);
end

T = readtable(fullfile(data_dir, filename));

t      = T.t - T.t(1);
q      = [T.q1,      T.q2,      T.q3,      T.q4     ];
dq     = [T.dq1,     T.dq2,     T.dq3,     T.dq4    ];
q_des  = [T.q1_des,  T.q2_des,  T.q3_des,  T.q4_des ];
dq_des = [T.dq1_des, T.dq2_des, T.dq3_des, T.dq4_des];
tau    = [T.tau1,    T.tau2,    T.tau3,    T.tau4   ];

%% 2. Estilo
lw     = 1.6;
fs     = 11;
fs_t   = 12;
c_real = [0.0000 0.4470 0.7410];   % azul    — medicion
c_ref  = [0.8500 0.3250 0.0980];   % naranja — referencia
c_tau  = lines(4);
xlims  = [t(1), t(end)];
jnames = {'Articulacion 1','Articulacion 2','Articulacion 3','Articulacion 4'};

%% 3. Figura 1 — Posiciones articulares (medidas vs deseadas)
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 600 1100 500]);

for i = 1:4
    subplot(2, 2, i);
    plot(t, q_des(:,i),  '--', 'Color', c_ref,  'LineWidth', lw); hold on;
    plot(t, q(:,i),      '-',  'Color', c_real, 'LineWidth', lw);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(['Seguimiento de posicion - ' jnames{i}], 'FontSize', fs_t);
    legend({sprintf('$q_{%d,des}$', i), sprintf('$q_{%d,med}$', i)}, ...
           'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs-1);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end
sgtitle('Posiciones articulares — FL Control', 'FontSize', 14, 'FontWeight', 'bold');

%% 4. Figura 2 — Velocidades articulares (medidas vs deseadas)
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [130 130 1100 500]);

for i = 1:4
    subplot(2, 2, i);
    plot(t, dq_des(:,i), '--', 'Color', c_ref,  'LineWidth', lw); hold on;
    plot(t, dq(:,i),     '-',  'Color', c_real, 'LineWidth', lw);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\dot{q}_%d$ [rad/s]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(['Seguimiento de velocidad - ' jnames{i}], 'FontSize', fs_t);
    legend({sprintf('$\\dot{q}_{%d,des}$', i), sprintf('$\\dot{q}_{%d,med}$', i)}, ...
           'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs-1);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end
sgtitle('Velocidades articulares — FL Control', 'FontSize', 14, 'FontWeight', 'bold');

%% 5. Figura 3 — Torques de control
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [160 -340 1100 500]);

for i = 1:4
    subplot(2, 2, i);
    plot(t, tau(:,i), '-', 'Color', c_tau(i,:), 'LineWidth', lw);
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N{\\cdot}m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(['Torque de control - ' jnames{i}], 'FontSize', fs_t);
    legend({sprintf('$\\tau_%d$', i)}, ...
           'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs-1);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end
sgtitle('Torques de control — FL Control', 'FontSize', 14, 'FontWeight', 'bold');
