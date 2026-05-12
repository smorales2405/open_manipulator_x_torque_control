%% plots_torque_control.m
% Grafica las senales registradas por torque_test_node:
%   Figura 1 — Posiciones articulares  q1..q4   [rad]
%   Figura 2 — Velocidades articulares dq1..dq4 [rad/s]
%   Figura 3 — Comandos de torque      tau1..tau4 [N·m]
%
% Uso:
%   Ejecutar desde cualquier directorio; el script localiza automaticamente
%   la carpeta data/ junto a si mismo.
%   - Dejar filename = '' para cargar el CSV mas reciente.
%   - O asignar filename = 'torque_data_20260512_120000.csv'.

clear; clc; close all;

%% 1. Ruta de datos (relativa al script, independiente del cwd)
script_dir = fileparts(mfilename('fullpath'));
data_dir   = fullfile(script_dir, '..', 'data');

filename = '';   % '' = archivo mas reciente

if isempty(filename)
    files = dir(fullfile(data_dir, 'torque_data_*.csv'));
    if isempty(files)
        error('No hay archivos en %s.\nEjecutar torque_test_node primero.', data_dir);
    end
    [~, idx] = max([files.datenum]);
    filename  = files(idx).name;
    fprintf('Cargando: %s\n', filename);
end

T = readtable(fullfile(data_dir, filename));

t   = T.t - T.t(1);
q   = [T.q1,   T.q2,   T.q3,   T.q4  ];
dq  = [T.dq1,  T.dq2,  T.dq3,  T.dq4 ];
tau = [T.tau1, T.tau2, T.tau3, T.tau4 ];

%% 2. Estilo
lw     = 1.6;
fs     = 11;
fs_t   = 12;
colors = lines(4);
jnames = {'Articulacion 1', 'Articulacion 2', 'Articulacion 3', 'Articulacion 4'};
xlims  = [t(1), t(end)];

%% 3. Figura 1 — Posiciones articulares
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 500 1100 420]);
for i = 1:4
    subplot(1, 4, i);
    plot(t, q(:,i), '-', 'Color', colors(i,:), 'LineWidth', lw);
    xlabel('t [s]',                                      'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i),   'Interpreter', 'latex', 'FontSize', fs);
    title(jnames{i},                                     'FontSize', fs_t);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end
sgtitle('Posiciones articulares', 'FontSize', 14, 'FontWeight', 'bold');

%% 4. Figura 2 — Velocidades articulares
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [130 170 1100 420]);
for i = 1:4
    subplot(1, 4, i);
    plot(t, dq(:,i), '-', 'Color', colors(i,:), 'LineWidth', lw);
    xlabel('t [s]',                                                   'FontSize', fs);
    ylabel(sprintf('$\\dot{q}_%d$ [rad/s]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jnames{i},                                                  'FontSize', fs_t);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end
sgtitle('Velocidades articulares', 'FontSize', 14, 'FontWeight', 'bold');

%% 5. Figura 3 — Torques de control
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [160 -160 1100 420]);
for i = 1:4
    subplot(1, 4, i);
    plot(t, tau(:,i), '-', 'Color', colors(i,:), 'LineWidth', lw);
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('t [s]',                                                              'FontSize', fs);
    ylabel(sprintf('$\\tau_%d$ [N$\\cdot$m]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jnames{i},                                                             'FontSize', fs_t);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end
sgtitle('Comandos de torque', 'FontSize', 14, 'FontWeight', 'bold');
