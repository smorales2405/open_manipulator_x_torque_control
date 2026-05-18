%% plots_FL_control.m
% Grafica las senales del controlador Feedback Linearization:
%   Figura 1 — Posiciones articulares:  q vs q_des   [rad]
%   Figura 2 — Velocidades articulares: dq vs dq_des [rad/s]
%   Figura 3 — Torques de control:      tau1..tau4   [N·m]
%
% Uso:
%   - Cambiar test_num para seleccionar la prueba (carga data/act1/fl_data_<N>.csv).
%   - El script localiza data/act1/ automaticamente junto a si mismo.

clear; clc; close all;

%% 1. Carga de datos
script_dir = fileparts(mfilename('fullpath'));
data_dir   = fullfile(script_dir, '..', 'data', 'act1');

test_num    = 5;      % <-- Cambiar aqui para seleccionar la prueba
EXPORT_FIGS = false;  % true = exportar PNG y EPS en data/plots/act1/test<N>/

filename = sprintf('fl_data_%d.csv', test_num);
fprintf('Cargando: %s\n', filename);

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

%% 3. Figura 1 — Posiciones articulares (simulación vs referencia)
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 600 1100 560]);

tl1 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs1 = gobjects(1,4);
h_ref = gobjects(1,1);
h_sim = gobjects(1,1);

for i = 1:4
    axs1(i) = nexttile(tl1);
    ax = axs1(i);
    h1 = plot(t, q_des(:,i), '--', 'Color', c_ref,  'LineWidth', lw); hold on;
    h2 = plot(t, q(:,i),     '-',  'Color', c_real, 'LineWidth', lw);

    if i == 1
        h_ref = h1;
        h_sim = h2;
    end

    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(ax, 'FontSize', fs);
    xlim(xlims);
end

lgd1 = legend(axs1(1), [h_ref h_sim], {'Referencia', 'Simulación'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, ...
              'Location', 'northoutside');
lgd1.Layout.Tile = 'north';
%title(tl1, 'FL Control - Posiciones Articulares', 'FontSize', 14, 'FontWeight', 'bold');

%% 4. Figura 2 — Velocidades articulares (simulación vs referencia)
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [130 130 1100 560]);

tl2 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs2 = gobjects(1,4);
h_ref = gobjects(1,1);
h_sim = gobjects(1,1);

for i = 1:4
    axs2(i) = nexttile(tl2);
    ax = axs2(i);
    h1 = plot(t, dq_des(:,i), '--', 'Color', c_ref,  'LineWidth', lw); hold on;
    h2 = plot(t, dq(:,i),     '-',  'Color', c_real, 'LineWidth', lw);

    if i == 1
        h_ref = h1;
        h_sim = h2;
    end

    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(['$\dot{q}_{' num2str(i) '}$ [rad/s]'], ...
       'Interpreter', 'latex', 'FontSize', fs);
    %title(['Seguimiento de velocidad - ' jnames{i}], 'FontSize', fs_t);
    grid on; box on;
    set(ax, 'FontSize', fs);
    xlim(xlims);
end

lgd2 = legend(axs2(1), [h_ref h_sim], {'Referencia', 'Simulación'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, ...
              'Location', 'northoutside');
lgd2.Layout.Tile = 'north';
%title(tl2, 'Velocidades articulares — FL Control', 'FontSize', 14, 'FontWeight', 'bold');

%% 5. Figura 3 — Errores de seguimiento articular
e_q = q - q_des;

figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [130 130 1100 560]);
tl3 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axs3 = gobjects(1, 4);

for i = 1:4
    axs3(i) = nexttile(tl3);
    plot(t, e_q(:,i), '-', 'Color', c_real, 'LineWidth', lw);
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$e_{q_%d}$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl3, 'FL Control - Errores de Seguimiento Articular', ...
      'FontSize', 14, 'FontWeight', 'bold');

%% 6. Figura 4 — Torques de control
figure(4); clf;
set(gcf, 'Color', 'w', 'Position', [160 -340 1100 500]);

for i = 1:4
    subplot(2, 2, i);
    plot(t, tau(:,i), '-', 'Color', c_tau(i,:), 'LineWidth', lw);
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N{\\cdot}m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

%% 7. Exportacion de figuras
if EXPORT_FIGS
    out_dir = fullfile(data_dir, 'plots', sprintf('test%d', test_num));
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    % PNG (raster, 300 dpi)
    exportgraphics(figure(1), fullfile(out_dir, 'plot_q.png'),    'Resolution', 300);
    exportgraphics(figure(2), fullfile(out_dir, 'plot_dq.png'),   'Resolution', 300);
    exportgraphics(figure(3), fullfile(out_dir, 'plot_eq.png'),   'Resolution', 300);
    exportgraphics(figure(4), fullfile(out_dir, 'plot_tau.png'),  'Resolution', 300);

    % EPS (vectorial)
    exportgraphics(figure(1), fullfile(out_dir, 'plot_q.eps'),    'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(2), fullfile(out_dir, 'plot_dq.eps'),   'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(3), fullfile(out_dir, 'plot_eq.eps'),   'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(4), fullfile(out_dir, 'plot_tau.eps'),  'ContentType', 'vector', 'Resolution', 600);

    fprintf('Figuras exportadas en: %s\n', out_dir);
end