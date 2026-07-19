%% plots_MRAC_joint_12p.m
% Graficas para el MRAC articular de 12 PARAMETROS
%   [alpha1..3, theta_load (dm, dmcx, dmcy, dmcz), Fv1, Fc1..4]
% OpenMANIPULATOR-X, campana sim-to-real (nodo gz_mrac_joint_12p_node)
%
%   Figura 1 — Seguimiento de posiciones articulares  q1..q4
%   Figura 2 — Error de seguimiento articular         e_q1..e_q4
%   Figura 3 — Seguimiento de velocidades articulares dq1..dq4
%   Figura 4 — Superficies de seguimiento             s1..s4
%   Figura 5 — Torques de control                     tau1..tau4
%   Figura 6 — Parametros estimados (2x2): alpha_hat, theta_load_hat
%               (dm/dmc en ejes izq/der), Fv1_hat, Fc_hat — con lineas de
%               referencia en los valores VERDADEROS de la planta (escalas de
%               config/sim_init_config.yaml y carga LOAD_TRUE)
%
%   Metricas reportadas en consola (por articulacion):
%     e_q,max [rad] | e_q,RMS [rad] | max|tau| [N·m] | tau_RMS [N·m] | Sat [%] | TV(tau)
%   y valores finales de los parametros estimados.
%
% Configurar la seccion "Configuracion" y ejecutar.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
mode        = 'sim';        % 'sim' = simulacion Gazebo (gz_mrac_joint_12p_node)

ctrl_mode   = 'fixed';   % Caso simulado (debe coincidir con los parametros del nodo):
                            %   'adaptive'         adaptive:=true  friction_prior:=true
                            %   'fixed'            adaptive:=false friction_prior:=true
                            %   'adaptive_noprior' adaptive:=true  friction_prior:=false (C4)
                            %   'fixed_noprior'    adaptive:=false friction_prior:=false

test_num    = 15;            % Identificador del ensayo (test_num usado al lanzar el nodo)

EXPORT_FIGS = true;        % true  = guardar PNG (300 dpi) y EPS vectorial (600 dpi)
                            % false = solo visualizar

TAU_MAX     = 1.2;          % [N·m] limite de torque (debe coincidir con el nodo)

% Escalas de la PLANTA usadas en config/sim_init_config.yaml al generar este
% ensayo — definen los valores verdaderos de los parametros (lineas de
% referencia en la Figura 6):
MASS_SCALE_TRUE     = 1.0;  % mass_inertia_scale -> alpha1..3 verdadero
DAMPING_SCALE_TRUE  = 1.0;  % damping_scale      -> Fv1 verdadero = escala*FV_NOM(1)
FRICTION_SCALE_TRUE = 1.0;  % friction_scale     -> Fc verdadero = escala*FC_NOM

% Carga VERDADERA en el efector (theta_load = exceso inercial del link5):
%   sin carga (spawn_load: false)          -> [0, 0, 0, 0]
%   cilindro 100 g de sim_init_config.yaml -> [0.100, 0.0141, 0, 0]
%   (dm [kg], d(m·c) [kg·m] en el frame de joint4; EE a 126 mm + offset 15 mm)
LOAD_TRUE = [0.100, 0.0141, 0, 0];

% Friccion nominal identificada (URDF/Xacro escala 1.0, mismos valores del nodo)
FV_NOM = [0.0367, 0.0000, 0.0000, 0.0050];   % [N·m·s/rad]
FC_NOM = [0.0146, 0.0830, 0.1143, 0.0413];   % [N·m]

% Directorio raiz del paquete ROS 2
pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

%% ── Rutas dinamicas ──────────────────────────────────────────────────────────
switch mode
    case 'sim'
        csv_file   = fullfile(pkg_dir, 'data', 'lab7', 'sim', 'mrac12p', ...
                              sprintf('gz_mrac_joint_12p_%s_%d.csv', ctrl_mode, test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'lab7', 'sim', 'mrac12p', ...
                              sprintf('test%d_%s', test_num, ctrl_mode));
        mode_label = 'Simulacion';
    otherwise
        error('mode debe ser ''sim''.');
end

%% ── Carga de datos ───────────────────────────────────────────────────────────
if ~isfile(csv_file)
    error('Archivo no encontrado:\n  %s\nVerificar mode, ctrl_mode y test_num.', csv_file);
end
T = readtable(csv_file);
fprintf('[%s | modo=%s | test=%d]  Cargado: %s  (%d muestras)\n', ...
        mode_label, ctrl_mode, test_num, csv_file, height(T));

t      = T.t - T.t(1);

q      = [T.q1,      T.q2,      T.q3,      T.q4     ];
q_des  = [T.q1_des,  T.q2_des,  T.q3_des,  T.q4_des ];

dq     = [T.dq1,     T.dq2,     T.dq3,     T.dq4    ];
dq_des = [T.dq1_des, T.dq2_des, T.dq3_des, T.dq4_des];

s_q    = [T.s1,      T.s2,      T.s3,      T.s4     ];

tau    = [T.tau1,    T.tau2,    T.tau3,    T.tau4   ];
sat    = [T.sat1,    T.sat2,    T.sat3,    T.sat4   ];   % 0/1 por muestra

a_hat    = [T.a1_hat,  T.a2_hat,   T.a3_hat  ];
load_hat = [T.dm_hat,  T.dmcx_hat, T.dmcy_hat, T.dmcz_hat];
fv1_hat  =  T.fv1_hat;
fc_hat   = [T.fc1_hat, T.fc2_hat,  T.fc3_hat,  T.fc4_hat];

e_q    = q - q_des;

%% ── Metricas (por articulacion) ──────────────────────────────────────────────
fprintf('\n%s\n', repmat('═', 1, 88));
fprintf(' Metricas MRAC articular 12p  [%s | modo=%s | test=%d]\n', ...
        mode_label, ctrl_mode, test_num);
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
fprintf('%s\n', repmat('-', 1, 88));
fprintf(' Parametros estimados finales (verdadero entre parentesis):\n');
fprintf('   alpha = [%.3f %.3f %.3f]                     (%.2f)\n', ...
        a_hat(end,1), a_hat(end,2), a_hat(end,3), MASS_SCALE_TRUE);
fprintf('   carga = dm=%.3f kg  dmc=[%.4f %.4f %.4f] kg·m\n', ...
        load_hat(end,1), load_hat(end,2), load_hat(end,3), load_hat(end,4));
fprintf('           (verdad: dm=%.3f, dmc=[%.4f %.4f %.4f])\n', ...
        LOAD_TRUE(1), LOAD_TRUE(2), LOAD_TRUE(3), LOAD_TRUE(4));
fprintf('   Fv1   = %.4f N·m·s/rad                       (%.4f)\n', ...
        fv1_hat(end), DAMPING_SCALE_TRUE * FV_NOM(1));
fprintf('   Fc    = [%.4f %.4f %.4f %.4f] N·m        (escala %.2f x nominal)\n', ...
        fc_hat(end,1), fc_hat(end,2), fc_hat(end,3), fc_hat(end,4), FRICTION_SCALE_TRUE);
fprintf('%s\n\n', repmat('═', 1, 88));

%% ── Estilo comun ─────────────────────────────────────────────────────────────
lw         = 1.6;
fs         = 11;
fs_title   = 14;
color_ref  = [0.8500 0.3250 0.0980];   % naranja — referencia
color_meas = [0.0000 0.4470 0.7410];   % azul    — medicion
color_surf = [0.4940 0.1840 0.5560];   % violeta — superficies de seguimiento
c_tau      = lines(4);                 % un color distintivo por articulacion
jointNames = {'Articulacion 1', 'Articulacion 2', ...
              'Articulacion 3', 'Articulacion 4'};
xlims      = [t(1), t(end)];

% Etiqueta del modo para los titulos
mode_tex = struct('adaptive',         'adaptativo', ...
                  'fixed',            'no adaptativo', ...
                  'adaptive_noprior', 'adaptativo, sin prior de friccion', ...
                  'fixed_noprior',    'no adaptativo, sin prior de friccion');
if isfield(mode_tex, ctrl_mode)
    ctrl_label = mode_tex.(ctrl_mode);
else
    ctrl_label = ctrl_mode;
end
sub_label = sprintf('[%s | %s]', mode_label, ctrl_label);

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
title(tl1, sprintf('MRAC 12p — Seguimiento de Posiciones  %s', sub_label), ...
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

title(tl2, sprintf('MRAC 12p — Error de Seguimiento Articular  %s', sub_label), ...
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
title(tl3, sprintf('MRAC 12p — Seguimiento de Velocidades  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Figura 4 — Superficies de seguimiento ────────────────────────────────────
figure(4); clf;
set(gcf, 'Color', 'w', 'Position', [110 -80 1100 680]);
tl4 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl4);
    plot(t, s_q(:,i), '-', 'Color', color_surf, 'LineWidth', lw); hold on;
    yline(0, ':', 'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$s_%d$ [rad/s]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title - 2);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim(xlims);
end

title(tl4, sprintf('MRAC 12p — Superficies de Seguimiento  %s', sub_label), ...
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

title(tl5, sprintf('MRAC 12p — Torques de Control  %s', sub_label), ...
      'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Figura 6 — Parametros estimados ──────────────────────────────────────────
figure(6); clf;
set(gcf, 'Color', 'w', 'Position', [150 -560 1250 720]);
tl6 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% (1) Escala de inercia estimada (alpha1..3)
nexttile(tl6);
h_a = gobjects(1, 3);
for i = 1:3
    h_a(i) = plot(t, a_hat(:,i), '-', 'Color', c_tau(i,:), 'LineWidth', lw); hold on;
end
if MASS_SCALE_TRUE ~= 1.0
    yline(MASS_SCALE_TRUE, '--k', 'LineWidth', 1.0, ...
          'Label', sprintf('planta real (%.2f)', MASS_SCALE_TRUE));
end
yline(1.0, ':', 'LineWidth', 0.9, 'Color', [0.5 0.5 0.5], 'Label', 'nominal (1.0)');
xlabel('Tiempo [s]', 'FontSize', fs);
ylabel('$\hat{\alpha}_k$', 'Interpreter', 'latex', 'FontSize', fs);
title('Escala de inercia estimada (links proximales)', 'FontSize', fs_title - 2);
legend(h_a, {'$\hat{\alpha}_1$', '$\hat{\alpha}_2$', '$\hat{\alpha}_3$'}, ...
       'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs);
grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);

% (2) Carga estimada en el efector — dm en eje izquierdo, d(m·c) en el derecho
nexttile(tl6);
yyaxis left;
h_dm = plot(t, load_hat(:,1), '-', 'Color', c_tau(1,:), 'LineWidth', lw); hold on;
yline(LOAD_TRUE(1), '--', 'Color', c_tau(1,:), 'LineWidth', 0.9);
ylabel('$\Delta\hat{m}$ [kg]', 'Interpreter', 'latex', 'FontSize', fs);
set(gca, 'YColor', c_tau(1,:));
yyaxis right;
h_mc = gobjects(1, 3);
for i = 1:3
    h_mc(i) = plot(t, load_hat(:,i+1), '-', 'Color', c_tau(i+1,:), 'LineWidth', lw); hold on;
    yline(LOAD_TRUE(i+1), '--', 'Color', c_tau(i+1,:), 'LineWidth', 0.9);
end
ylabel('$\Delta\widehat{m c}$ [kg$\cdot$m]', 'Interpreter', 'latex', 'FontSize', fs);
set(gca, 'YColor', [0.3 0.3 0.3]);
xlabel('Tiempo [s]', 'FontSize', fs);
title('Carga estimada en el efector (-- verdadero)', 'FontSize', fs_title - 2);
legend([h_dm, h_mc], {'$\Delta\hat{m}$', '$\Delta\widehat{mc}_x$', ...
       '$\Delta\widehat{mc}_y$', '$\Delta\widehat{mc}_z$'}, ...
       'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs);
grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);

% (3) Friccion viscosa estimada (solo Fv1 se adapta)
nexttile(tl6);
h_fv = plot(t, fv1_hat, '-', 'Color', c_tau(1,:), 'LineWidth', lw); hold on;
yline(DAMPING_SCALE_TRUE * FV_NOM(1), '--', 'Color', c_tau(1,:), 'LineWidth', 0.9);
xlabel('Tiempo [s]', 'FontSize', fs);
ylabel('$\hat{F}_{v,1}$ [N$\cdot$m$\cdot$s/rad]', 'Interpreter', 'latex', 'FontSize', fs);
title('Friccion viscosa estimada, J1 (-- verdadero)', 'FontSize', fs_title - 2);
legend(h_fv, {'$\hat{F}_{v,1}$'}, ...
       'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs);
grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);

% (4) Friccion de Coulomb estimada (Fc1..4) — lineas discontinuas: verdadero
nexttile(tl6);
h_fc = gobjects(1, 4);
for i = 1:4
    h_fc(i) = plot(t, fc_hat(:,i), '-', 'Color', c_tau(i,:), 'LineWidth', lw); hold on;
end
for i = 1:4
    yline(FRICTION_SCALE_TRUE * FC_NOM(i), '--', 'Color', c_tau(i,:), 'LineWidth', 0.9);
end
xlabel('Tiempo [s]', 'FontSize', fs);
ylabel('$\hat{F}_{c,i}$ [N$\cdot$m]', 'Interpreter', 'latex', 'FontSize', fs);
title('Friccion de Coulomb estimada (-- verdadero)', 'FontSize', fs_title - 2);
legend(h_fc, {'$\hat{F}_{c,1}$', '$\hat{F}_{c,2}$', '$\hat{F}_{c,3}$', '$\hat{F}_{c,4}$'}, ...
       'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs);
grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);

title(tl6, sprintf('MRAC 12p — Parametros Estimados  %s', sub_label), ...
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
    exportgraphics(figure(6), fullfile(output_dir, 'plot_params.png'),      'Resolution', 300);

    % EPS (vectorial, 600 dpi)
    exportgraphics(figure(1), fullfile(output_dir, 'plot_q_tracking.eps'),  'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(2), fullfile(output_dir, 'plot_q_error.eps'),     'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(3), fullfile(output_dir, 'plot_dq_tracking.eps'), 'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(4), fullfile(output_dir, 'plot_surfaces.eps'),    'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(5), fullfile(output_dir, 'plot_torques.eps'),     'ContentType', 'vector', 'Resolution', 600);
    exportgraphics(figure(6), fullfile(output_dir, 'plot_params.eps'),      'ContentType', 'vector', 'Resolution', 600);

    fprintf('Graficas guardadas en:\n  %s\n', output_dir);
end
