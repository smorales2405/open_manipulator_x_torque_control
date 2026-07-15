%% plots_SMC_comp_rob_cart.m
% Graficas COMPARATIVAS de ROBUSTEZ — Control SMC Cartesiano
% OpenMANIPULATOR-X, Laboratorio 6 — Actividad 2
%
%   Compara dos ensayos: caso nominal vs caso con perturbacion
%     robustness_test = 'params'  -> con y sin incertidumbre parametrica
%     robustness_test = 'load'    -> con y sin carga en el efector final
%
%   Figura 1 — Seguimiento cartesiano: x, y, z, phi  vs referencia
%   Figura 2 — Errores de seguimiento cartesiano e_x, e_y, e_z, e_phi
%   Figura 3 — Torques de control                tau1..tau4
%
%   Metricas comparativas reportadas en consola:
%     Por salida cartesiana: e_max [m/rad]  |  e_RMS [m/rad]
%     Por articulacion:      max|tau| [N·m] |  tau_RMS [N·m]  |  Sat [%]  |  TV(tau)
%
%   Solo se usa rho = 'sat'.
%
% Configurar la seccion "Configuracion" y ejecutar.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
mode            = 'sim';     % 'sim' = simulacion Gazebo | 'real' = hardware
robustness_test = 'params';  % 'params' = incertidumbre parametrica
                              % 'load'   = carga en el efector final

% test_num para cada caso (seleccionar el CSV correspondiente)
test_num_nominal = 1;   % ensayo sin perturbacion (nominal)
test_num_perturb = 2;   % ensayo con perturbacion

EXPORT_FIGS = false;    % true = guardar PNG (300 dpi) y EPS vectorial (600 dpi)

TAU_MAX = 1.2;          % [N·m] limite de torque (debe coincidir con el nodo)

pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

%% ── Etiquetas segun tipo de ensayo ──────────────────────────────────────────
switch robustness_test
    case 'params'
        label_nominal = 'Nominal';
        label_perturb = 'Con incertidumbre param.';
        rob_disp      = 'Incertidumbre Parametrica';
        rob_tag       = 'params';
    case 'load'
        label_nominal = 'Sin carga';
        label_perturb = 'Con carga';
        rob_disp      = 'Carga en Efector Final';
        rob_tag       = 'load';
    otherwise
        error('robustness_test debe ser ''params'' o ''load''.');
end

%% ── Rutas dinamicas ──────────────────────────────────────────────────────────
switch mode
    case 'sim'
        data_dir   = fullfile(pkg_dir, 'data',  'lab6', 'sim',  'act2');
        output_dir = fullfile(pkg_dir, 'plots', 'lab6', 'sim',  'act2', 'comp_rob');
        mode_label = 'Simulacion';
        prefix     = 'gz_smc_cart';
    case 'real'
        data_dir   = fullfile(pkg_dir, 'data',  'lab6', 'real', 'act2');
        output_dir = fullfile(pkg_dir, 'plots', 'lab6', 'real', 'act2', 'comp_rob');
        mode_label = 'Implementacion';
        prefix     = 'hw_smc_cart';
    otherwise
        error('mode debe ser ''sim'' o ''real''.');
end

%% ── Carga de datos ───────────────────────────────────────────────────────────
test_nums   = [test_num_nominal, test_num_perturb];
case_labels = {label_nominal, label_perturb};

data = struct();
for k = 1:2
    tnum  = test_nums(k);
    fpath = fullfile(data_dir, sprintf('%s_sat_%d.csv', prefix, tnum));
    if ~isfile(fpath)
        error('Archivo no encontrado:\n  %s\nVerificar mode y test_num.', fpath);
    end
    Tk = readtable(fpath);
    fprintf('[%s | %s | test=%d]  Cargado: %d muestras\n', ...
            mode_label, case_labels{k}, tnum, height(Tk));

    data(k).t     = Tk.t - Tk.t(1);
    data(k).y     = [Tk.x,        Tk.y,        Tk.z,        Tk.phi       ];
    data(k).y_des = [Tk.x_des,    Tk.y_des,    Tk.z_des,    Tk.phi_des   ];
    data(k).e_y   = data(k).y - data(k).y_des;
    data(k).tau   = [Tk.tau1,     Tk.tau2,     Tk.tau3,     Tk.tau4      ];
    data(k).sat   = abs(data(k).tau) >= 0.99 * TAU_MAX;   % criterio de la guia
end

%% ── Metricas comparativas ────────────────────────────────────────────────────
cartNames = {'x [m]', 'y [m]', 'z [m]', 'phi [rad]'};

fprintf('\n%s\n', repmat('═', 1, 86));
fprintf(' Robustez SMC Cartesiano — %s  [%s]\n', rob_disp, mode_label);
fprintf('%s\n', repmat('═', 1, 86));
for k = 1:2
    fprintf('\n  Caso: %s  (test=%d)\n', case_labels{k}, test_nums(k));
    fprintf('  %-12s  %-14s  %-14s\n', 'Salida', 'e_max[m/rad]', 'e_RMS[m/rad]');
    fprintf('  %s\n', repmat('-', 1, 44));
    for i = 1:4
        e_max_i = max(abs(data(k).e_y(:,i)));
        e_rms_i = sqrt(mean(data(k).e_y(:,i).^2));
        fprintf('  %-12s  %14.6f  %14.6f\n', cartNames{i}, e_max_i, e_rms_i);
    end
    fprintf('  Torques:\n');
    fprintf('  %-6s  %-14s  %-14s  %-8s  %-10s\n', 'Joint','max|tau|[N·m]','tau_RMS[N·m]','Sat[%]','TV(tau)');
    fprintf('  %s\n', repmat('-', 1, 60));
    for i = 1:4
        tau_max_i = max(abs(data(k).tau(:,i)));
        tau_rms_i = sqrt(mean(data(k).tau(:,i).^2));
        sat_pct_i = 100 * mean(data(k).sat(:,i));
        tv_i      = sum(abs(diff(data(k).tau(:,i))));
        fprintf('  tau%-3d  %14.4f  %14.4f  %8.2f  %10.4f\n', i, tau_max_i, tau_rms_i, sat_pct_i, tv_i);
    end
end
fprintf('%s\n\n', repmat('═', 1, 86));

%% ── Estilo comun ─────────────────────────────────────────────────────────────
lw       = 1.5;
fs       = 11;
fs_title = 14;

colors = [0.0000 0.4470 0.7410;   % azul   — nominal / sin carga
          0.8500 0.3250 0.0980];  % naranja — perturbado / con carga

lstyles      = {'-', '--'};
color_ref    = [0.4 0.4 0.4];   % gris — referencia
ylabels_cart = {'$x$ [m]', '$y$ [m]', '$z$ [m]', '$\phi$ [rad]'};
titles_cart  = {'$x$', '$y$', '$z$', '$\phi$'};
ylabels_err  = {'$e_x$ [m]', '$e_y$ [m]', '$e_z$ [m]', '$e_\phi$ [rad]'};
jointNames   = {'Articulacion 1','Articulacion 2','Articulacion 3','Articulacion 4'};
sgtitle_str  = sprintf('SMC Cartesiano — Robustez: %s  [%s]', rob_disp, mode_label);

%% ── Figura 1 — Seguimiento cartesiano ────────────────────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [50 620 1100 580]);
tl1 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

h_leg1 = gobjects(1, 3);   % ref + nominal + perturbado
for i = 1:4
    nexttile(tl1);
    hr = plot(data(1).t, data(1).y_des(:,i), '-', ...
              'Color', color_ref, 'LineWidth', 1.2); hold on;
    for k = 1:2
        hk = plot(data(k).t, data(k).y(:,i), lstyles{k}, ...
                  'Color', colors(k,:), 'LineWidth', lw);
        if i == 1; h_leg1(k+1) = hk; end
    end
    if i == 1; h_leg1(1) = hr; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(ylabels_cart{i}, 'Interpreter', 'latex', 'FontSize', fs);
    title(titles_cart{i}, 'FontSize', fs, 'Interpreter', 'latex');
    grid on; box on; set(gca, 'FontSize', fs);
    xlim([data(1).t(1), data(1).t(end)]);
end

lgd1 = legend(nexttile(tl1, 1), h_leg1, [{'Referencia'}, case_labels], ...
    'Orientation', 'horizontal', 'FontSize', fs, 'Interpreter', 'none');
lgd1.Layout.Tile = 'north';
title(tl1, sgtitle_str, 'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Figura 2 — Errores de seguimiento cartesiano ────────────────────────────
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [70 390 1100 580]);
tl2 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

h_leg2 = gobjects(1, 2);
for i = 1:4
    nexttile(tl2);
    yline(0, ':', 'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]); hold on;
    for k = 1:2
        hk = plot(data(k).t, data(k).e_y(:,i), lstyles{k}, ...
                  'Color', colors(k,:), 'LineWidth', lw);
        if i == 1; h_leg2(k) = hk; end
    end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(ylabels_err{i}, 'Interpreter', 'latex', 'FontSize', fs);
    title(titles_cart{i}, 'FontSize', fs, 'Interpreter', 'latex');
    grid on; box on; set(gca, 'FontSize', fs);
    xlim([data(1).t(1), data(1).t(end)]);
end

lgd2 = legend(nexttile(tl2, 1), h_leg2, case_labels, ...
    'Orientation', 'horizontal', 'FontSize', fs, 'Interpreter', 'none');
lgd2.Layout.Tile = 'north';
title(tl2, strrep(sgtitle_str, 'Robustez', 'Robustez — Errores'), ...
      'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Figura 3 — Torques de control ────────────────────────────────────────────
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [90 160 1100 580]);
tl3 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

h_leg3 = gobjects(1, 2);
for i = 1:4
    nexttile(tl3);
    yline( 0,       ':',   'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]); hold on;
    yline( TAU_MAX, '--k', 'LineWidth', 1.0, 'Label', sprintf('+%.1f N·m', TAU_MAX));
    yline(-TAU_MAX, '--k', 'LineWidth', 1.0, 'Label', sprintf('\x2212%.1f N·m', TAU_MAX));
    for k = 1:2
        hk = plot(data(k).t, data(k).tau(:,i), lstyles{k}, ...
                  'Color', colors(k,:), 'LineWidth', lw);
        if i == 1; h_leg3(k) = hk; end
    end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N{\\cdot}m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs);
    ylim([-TAU_MAX * 1.25, TAU_MAX * 1.25]);
    grid on; box on; set(gca, 'FontSize', fs);
    xlim([data(1).t(1), data(1).t(end)]);
end

lgd3 = legend(nexttile(tl3, 1), h_leg3, case_labels, ...
    'Orientation', 'horizontal', 'FontSize', fs, 'Interpreter', 'none');
lgd3.Layout.Tile = 'north';
title(tl3, strrep(sgtitle_str, 'Robustez', 'Robustez — Torques'), ...
      'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    if ~exist(output_dir, 'dir'); mkdir(output_dir); end

    fnames = {sprintf('plot_rob_%s_cart_tracking', rob_tag), ...
              sprintf('plot_rob_%s_cart_error',    rob_tag), ...
              sprintf('plot_rob_%s_torques',       rob_tag)};
    for f = 1:3
        exportgraphics(figure(f), fullfile(output_dir, [fnames{f} '.png']), 'Resolution', 300);
        exportgraphics(figure(f), fullfile(output_dir, [fnames{f} '.eps']), ...
                       'ContentType', 'vector', 'Resolution', 600);
    end
    fprintf('Graficas guardadas en:\n  %s\n', output_dir);
end
