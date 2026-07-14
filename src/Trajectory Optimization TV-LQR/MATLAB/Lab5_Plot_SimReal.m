%% Lab5_Plot_SimReal.m
% Graficas de seguimiento TV-LQR — Laboratorio 5
% OpenMANIPULATOR-X
%
% Figuras generadas:
%   Figura 1 — Seguimiento de posiciones articulares q1..q4
%   Figura 2 — Error de seguimiento articular e_q1..e_q4
%   Figura 3 — Velocidades articulares dq1..dq4
%   Figura 4 — Trayectoria cartesiana: x, y, z, phi vs referencia (4x1)
%   Figura 5 — Torques de control tau1..tau4 vs u_ref
%   Figura 6 — Trayectoria cartesiana del efector final (3D)
%
% Configurar las variables de la seccion "Configuracion" y ejecutar.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
mode        = 'real';   % 'sim'  = simulacion Gazebo
                       % 'real' = implementacion hardware real

test_num    = 4;       % Numero de log

EXPORT_FIGS = true;   % true  = guardar PNG (300 dpi) y EPS vectorial (600 dpi)
                       % false = solo visualizar

% Directorio raiz del paquete ROS 2
pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

% Directorio de referencias (para y_ref y u_ref del grafico de torques)
ref_dir = fullfile(pkg_dir, 'src', 'Trajectory Optimization TV-LQR', 'references');

%% ── Rutas dinamicas ──────────────────────────────────────────────────────────
switch mode
    case 'sim'
        csvFile    = fullfile(pkg_dir, 'data', 'lab5', 'sim', ...
                              sprintf('data_log_sim_lab5_%d.csv', test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'lab5', 'sim', ...
                              sprintf('test%d', test_num));
        mode_label = 'Simulacion';
    case 'real'
        csvFile    = fullfile(pkg_dir, 'data', 'lab5', 'real', ...
                              sprintf('data_log_real_lab5_%d.csv', test_num));
        output_dir = fullfile(pkg_dir, 'plots', 'lab5', 'real', ...
                              sprintf('test%d', test_num));
        mode_label = 'Hardware';
    otherwise
        error('mode debe ser ''sim'' o ''real''.');
end

%% ── Carga de datos ───────────────────────────────────────────────────────────
if ~isfile(csvFile)
    error('Archivo no encontrado:\n  %s\nVerificar mode y test_num.', csvFile);
end
T = readtable(csvFile);
fprintf('[%s] Cargado: %s  (%d muestras)\n', mode_label, csvFile, height(T));

t   = T.t - T.t(1);
q   = [T.q1,  T.q2,  T.q3,  T.q4];
dq  = [T.dq1, T.dq2, T.dq3, T.dq4];
tau = [T.tau1, T.tau2, T.tau3, T.tau4];

q_ref  = [T.q1_ref,  T.q2_ref,  T.q3_ref,  T.q4_ref];
dq_ref = [T.dq1_ref, T.dq2_ref, T.dq3_ref, T.dq4_ref];

% Trayectoria cartesiana: sim la tiene en CSV; real se calcula con fkin
if strcmp(mode, 'sim') && ismember('x', T.Properties.VariableNames)
    y_cart     = [T.x,     T.y,     T.z,     T.phi];
    y_cart_ref = [T.x_ref, T.y_ref, T.z_ref, T.phi_ref];
else
    % Calcular cinematica directa si open_manx_fkin esta en el path
    if exist('open_manx_fkin', 'file')
        y_cart     = zeros(height(T), 4);
        y_cart_ref = zeros(height(T), 4);
        for i = 1:height(T)
            y_cart(i,:)     = open_manx_fkin(q(i,:)')';
            y_cart_ref(i,:) = open_manx_fkin(q_ref(i,:)')';
        end
    else
        y_cart     = [];
        y_cart_ref = [];
        warning('open_manx_fkin.m no encontrado: Figuras 4 y 6 no disponibles.');
    end
end

% Cargar u_ref desde archivo de referencia (si existe)
u_ref_file = fullfile(ref_dir, 'u_ref.txt');
u_ref_loaded = false;
if isfile(u_ref_file)
    u_ref_mat = readmatrix(u_ref_file);
    u_ref_loaded = true;
end

%% ── Estilo ───────────────────────────────────────────────────────────────────
lw        = 1.6;
fs        = 11;
fs_title  = 12;
c_ref     = [0.8500 0.3250 0.0980];   % naranja  — referencia
c_meas    = [0.0000 0.4470 0.7410];   % azul     — medicion
c_tau     = [0.4660 0.6740 0.1880];   % verde    — torque aplicado
c_uref    = [0.9290 0.6940 0.1250];   % amarillo — torque de referencia
xlims     = [t(1), t(end)];
jointNames = {'Articulacion 1', 'Articulacion 2', ...
              'Articulacion 3', 'Articulacion 4'};

%% ── Figura 1 — Posiciones articulares ────────────────────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 1100 700]);

for i = 1:4
    subplot(2,2,i);
    plot(t, q(:,i),     '-',  'Color', c_meas, 'LineWidth', lw); hold on;
    plot(t, q_ref(:,i), '--', 'Color', c_ref,  'LineWidth', lw);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    legend({sprintf('$q_{%d}$', i), sprintf('$q_{ref,%d}$', i)}, ...
           'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs-1);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
sgtitle(sprintf('[%s — TV-LQR] Seguimiento de posiciones articulares', mode_label), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 2 — Error de seguimiento articular ────────────────────────────────
e_q = q - q_ref;
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [120 120 1100 560]);
tl2 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl2);
    plot(t, e_q(:,i), '-', 'Color', c_meas, 'LineWidth', lw);
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$e_{q_%d}$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end

title(tl2, sprintf('[%s — TV-LQR] Error de seguimiento articular', mode_label), ...
      'FontSize', 14, 'FontWeight', 'bold');

fprintf('\nError maximo articular [rad]:  [%.5f %.5f %.5f %.5f]\n', max(abs(e_q)));
fprintf('Error RMS articular   [rad]:  [%.5f %.5f %.5f %.5f]\n', sqrt(mean(e_q.^2)));

%% ── Figura 3 — Velocidades articulares ───────────────────────────────────────
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [140 140 1100 700]);

for i = 1:4
    subplot(2,2,i);
    plot(t, dq(:,i),     '-',  'Color', c_meas, 'LineWidth', lw); hold on;
    plot(t, dq_ref(:,i), '--', 'Color', c_ref,  'LineWidth', lw);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\dot{q}_%d$ [rad/s]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    legend({sprintf('$\\dot{q}_{%d}$', i), sprintf('$\\dot{q}_{ref,%d}$', i)}, ...
           'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs-1);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
sgtitle(sprintf('[%s — TV-LQR] Seguimiento de velocidades articulares', mode_label), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 4 — Trayectoria cartesiana: x, y, z, phi ─────────────────────────
if ~isempty(y_cart)
    figure(4); clf;
    set(gcf, 'Color', 'w', 'Position', [155 155 1100 700]);

    cart_labels = {'$x$ [m]', '$y$ [m]', '$z$ [m]', '$\phi$ [rad]'};

    tl4  = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    axs4 = gobjects(1, 4);
    h_meas = []; h_ref = [];

    for i = 1:4
        axs4(i) = nexttile(tl4);
        h1 = plot(t, y_cart(:,i),     '-',  'Color', c_meas, 'LineWidth', lw); hold on;
        h2 = plot(t, y_cart_ref(:,i), '--', 'Color', c_ref,  'LineWidth', lw);
        if i == 1, h_meas = h1; h_ref = h2; end
        ylabel(cart_labels{i}, 'Interpreter', 'latex', 'FontSize', fs);
        grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
        if i < 4
            set(gca, 'XTickLabel', []);
        else
            xlabel('Tiempo [s]', 'FontSize', fs);
        end
    end

    lgd4 = legend(axs4(1), [h_meas, h_ref], {'Simulado/Real', 'Referencia'}, ...
                  'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
    lgd4.Box = 'on';
    try, lgd4.Layout.Tile = 'north'; catch, end

    title(tl4, sprintf('[%s — TV-LQR] Trayectoria cartesiana: x, y, z, \\phi', mode_label), ...
          'FontSize', fs_title, 'FontWeight', 'bold');
end

%% ── Figura 5 — Torques de control ────────────────────────────────────────────
figure(5); clf;
set(gcf, 'Color', 'w', 'Position', [160 160 1100 700]);

for i = 1:4
    subplot(2,2,i);
    plot(t, tau(:,i), '-', 'Color', c_tau, 'LineWidth', lw); hold on;
    if u_ref_loaded
        % u_ref tiene N filas; interpolar al tiempo del log
        t_uref = (0.05:0.05:size(u_ref_mat,1)*0.05)';
        u_interp = interp1(t_uref, u_ref_mat(:,i), min(t, t_uref(end)), 'previous', 'extrap');
        plot(t, u_interp, '--', 'Color', c_uref, 'LineWidth', lw);
        legend({'$\tau_{aplicado}$', '$u_{ref}$'}, ...
               'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs-1);
    else
        yline(0, ':', 'LineWidth', 0.8);
    end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N\\cdot m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(['Torque - ' jointNames{i}], 'FontSize', fs_title);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
end
sgtitle(sprintf('[%s — TV-LQR] Torques de control aplicados', mode_label), ...
        'FontSize', 14, 'FontWeight', 'bold');

tau_max_abs = max(abs(tau));
sat_pct     = 100*mean(abs(tau) >= (0.82 - 1e-3));
fprintf('Max |tau| [N.m]:           [%.4f %.4f %.4f %.4f]\n', tau_max_abs);
fprintf('Saturacion [%%]:            [%.2f %.2f %.2f %.2f]\n', sat_pct);

%% ── Figura 6 — Trayectoria cartesiana 3D ─────────────────────────────────────
if ~isempty(y_cart)
    figure(6); clf;
    set(gcf, 'Color', 'w', 'Position', [180 80 900 700]);
    hold on; grid on; box on;

    plot3(y_cart(:,1),     y_cart(:,2),     y_cart(:,3), ...
          '-',  'Color', c_meas, 'LineWidth', 2.0);
    plot3(y_cart_ref(:,1), y_cart_ref(:,2), y_cart_ref(:,3), ...
          '--o','Color', c_ref,  'LineWidth', 1.4, 'MarkerSize', 4);

    % Punto final de referencia
    plot3(y_cart_ref(end,1), y_cart_ref(end,2), y_cart_ref(end,3), ...
          'kx', 'MarkerSize', 12, 'LineWidth', 2.0);

    legend({'Trayectoria simulada/real', 'Referencia optimizada', 'Punto final $y_f$'}, ...
           'Interpreter', 'latex', 'Location', 'northeastoutside', 'FontSize', fs);
    xlabel('x [m]', 'FontSize', fs);
    ylabel('y [m]', 'FontSize', fs);
    zlabel('z [m]', 'FontSize', fs);
    view(45, 30);
    set(gca, 'FontSize', fs);
    title(sprintf('[%s — TV-LQR] Trayectoria cartesiana del efector final', mode_label), ...
          'FontSize', fs_title, 'FontWeight', 'bold');

    % Error cartesiano final
    e_cart_final = norm(y_cart(end,1:3) - y_cart_ref(end,1:3));
    fprintf('Error cartesiano final ||p(tf)-pf|| [m]: %.6f\n', e_cart_final);
end

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end

    fig_names = {'tracking_plot_q', 'tracking_plot_error', ...
                 'tracking_plot_dq', 'cartesian_xyz_plot', ...
                 'torques_plot',     'cartesian_3d_plot'};
    figs_to_export = [1 2 3 5];
    if ~isempty(y_cart)
        figs_to_export = [1 2 3 4 5 6];
    end

    for fi = figs_to_export
        base = fullfile(output_dir, fig_names{fi});
        exportgraphics(figure(fi), [base '.png'], 'Resolution', 300);
        exportgraphics(figure(fi), [base '.eps'], ...
                       'ContentType', 'vector', 'Resolution', 600);
    end
    fprintf('Graficas guardadas en: %s\n', output_dir);
end
