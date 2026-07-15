%% plots_SMC_comp_art.m
% Graficas COMPARATIVAS — Control SMC Articular (sign vs sat)
% OpenMANIPULATOR-X, Laboratorio 6 — Actividad 1
%
%   Figura 1 — Seguimiento de posiciones articulares  q1..q4
%   Figura 2 — Error de seguimiento articular         e_q1..e_q4
%   Figura 3 — Superficies deslizantes                s1..s4
%   Figura 4 — Torques de control                     tau1..tau4
%
%   Metricas comparativas reportadas en consola por articulacion:
%     e_max [rad]  |  e_RMS [rad]  |  max|tau| [N·m]  |  tau_RMS [N·m]  |  Sat [%]  |  TV(tau)
%
% Configurar la seccion "Configuracion" y ejecutar.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
mode = 'sim';   % 'sim' = simulacion Gazebo | 'real' = hardware

% test_num independiente por funcion de conmutacion
test_num_sign = 1;
test_num_sat  = 1;

EXPORT_FIGS = false;    % true = guardar PNG (300 dpi) y EPS vectorial (600 dpi)

TAU_MAX = 1.2;          % [N·m] limite de torque (debe coincidir con el nodo)

pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

%% ── Rutas dinamicas ──────────────────────────────────────────────────────────
switch mode
    case 'sim'
        data_dir   = fullfile(pkg_dir, 'data',  'lab6', 'sim',  'act1');
        output_dir = fullfile(pkg_dir, 'plots', 'lab6', 'sim',  'act1', 'comp_sf');
        mode_label = 'Simulacion';
        prefix     = 'gz_smc_joint';
    case 'real'
        data_dir   = fullfile(pkg_dir, 'data',  'lab6', 'real', 'act1');
        output_dir = fullfile(pkg_dir, 'plots', 'lab6', 'real', 'act1', 'comp_sf');
        mode_label = 'Implementacion';
        prefix     = 'hw_smc_joint';
    otherwise
        error('mode debe ser ''sim'' o ''real''.');
end

%% ── Carga de datos ───────────────────────────────────────────────────────────
rho_names = {'sign', 'sat'};
test_nums = [test_num_sign, test_num_sat];

data = struct();
for k = 1:2
    rho  = rho_names{k};
    tnum = test_nums(k);
    fpath = fullfile(data_dir, sprintf('%s_%s_%d.csv', prefix, rho, tnum));
    if ~isfile(fpath)
        error('Archivo no encontrado:\n  %s\nVerificar mode y test_num_%s.', fpath, rho);
    end
    Tk = readtable(fpath);
    fprintf('[%s | rho=%s | test=%d]  Cargado: %d muestras\n', ...
            mode_label, rho, tnum, height(Tk));

    data(k).t     = Tk.t - Tk.t(1);
    data(k).q     = [Tk.q1,    Tk.q2,    Tk.q3,    Tk.q4   ];
    data(k).q_des = [Tk.q1_des,Tk.q2_des,Tk.q3_des,Tk.q4_des];
    data(k).e_q   = data(k).q - data(k).q_des;
    data(k).s_q   = [Tk.s1,    Tk.s2,    Tk.s3,    Tk.s4   ];
    data(k).tau   = [Tk.tau1,  Tk.tau2,  Tk.tau3,  Tk.tau4 ];
    data(k).sat   = abs(data(k).tau) >= 0.99 * TAU_MAX;   % criterio de la guia
end

%% ── Metricas comparativas ────────────────────────────────────────────────────
fprintf('\n%s\n', repmat('═', 1, 82));
fprintf(' Metricas comparativas SMC Articular  [%s]\n', mode_label);
fprintf('%s\n', repmat('═', 1, 82));
for k = 1:2
    fprintf('\n  rho = %s  (test=%d)\n', rho_names{k}, test_nums(k));
    fprintf('  %-6s  %-12s  %-12s  %-14s  %-14s  %-8s  %-10s\n', ...
            'Joint','e_max[rad]','e_RMS[rad]','max|tau|[N·m]','tau_RMS[N·m]','Sat[%]','TV(tau)');
    fprintf('  %s\n', repmat('-', 1, 84));
    for i = 1:4
        e_max_i   = max(abs(data(k).e_q(:,i)));
        e_rms_i   = sqrt(mean(data(k).e_q(:,i).^2));
        tau_max_i = max(abs(data(k).tau(:,i)));
        tau_rms_i = sqrt(mean(data(k).tau(:,i).^2));
        sat_pct_i = 100 * mean(data(k).sat(:,i));
        tv_i      = sum(abs(diff(data(k).tau(:,i))));
        fprintf('  q%-5d  %12.5f  %12.5f  %14.4f  %14.4f  %8.2f  %10.4f\n', ...
                i, e_max_i, e_rms_i, tau_max_i, tau_rms_i, sat_pct_i, tv_i);
    end
end
fprintf('%s\n\n', repmat('═', 1, 82));

%% ── Estilo comun ─────────────────────────────────────────────────────────────
lw       = 1.5;
fs       = 11;
fs_title = 14;

colors = [0.0000 0.4470 0.7410;   % azul   — sign
          0.8500 0.3250 0.0980];  % naranja — sat

lstyles  = {'-', '-'};
rho_disp = {'sign(s)', 'sat(s/\phi)'};

color_ref  = [0.4 0.4 0.4];   % gris — referencia
jointNames = {'Articulacion 1','Articulacion 2','Articulacion 3','Articulacion 4'};
sgtitle_str = sprintf('SMC Articular — Comparacion [%s]  sign | sat', mode_label);

%% ── Figura 1 — Seguimiento de posiciones articulares ─────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [50 620 1100 580]);
tl1 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

h_leg = gobjects(1, 3);   % sign, sat + ref
for i = 1:4
    ax = nexttile(tl1);
    % Referencia (igual para todos; usar la del primer dataset)
    hr = plot(data(1).t, data(1).q_des(:,i), '-', ...
              'Color', color_ref, 'LineWidth', 1.2); hold on;
    for k = 1:2
        hk = plot(data(k).t, data(k).q(:,i), lstyles{k}, ...
                  'Color', colors(k,:), 'LineWidth', lw);
        if i == 1; h_leg(k+1) = hk; end
    end
    if i == 1; h_leg(1) = hr; end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs);
    grid on; box on; set(gca, 'FontSize', fs);
    xlim([data(1).t(1), data(1).t(end)]);
end

lgd = legend(nexttile(tl1, 1), h_leg, ...
    ['Referencia', cellfun(@(s) ['\rho = ' s], rho_disp, 'UniformOutput', false)], ...
    'Orientation', 'horizontal', 'FontSize', fs, 'Interpreter', 'tex');
lgd.Layout.Tile = 'north';
title(tl1, sgtitle_str, 'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Figura 2 — Error de seguimiento articular ────────────────────────────────
figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [70 390 1100 580]);
tl2 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

h_leg2 = gobjects(1, 2);
for i = 1:4
    nexttile(tl2);
    yline(0, ':', 'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]); hold on;
    for k = 1:2
        hk = plot(data(k).t, data(k).e_q(:,i), lstyles{k}, ...
                  'Color', colors(k,:), 'LineWidth', lw);
        if i == 1; h_leg2(k) = hk; end
    end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$e_{q_%d}$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs);
    grid on; box on; set(gca, 'FontSize', fs);
    xlim([data(1).t(1), data(1).t(end)]);
end

lgd2 = legend(nexttile(tl2, 1), h_leg2, ...
    cellfun(@(s) ['\rho = ' s], rho_disp, 'UniformOutput', false), ...
    'Orientation', 'horizontal', 'FontSize', fs, 'Interpreter', 'tex');
lgd2.Layout.Tile = 'north';
title(tl2, [strrep(sgtitle_str, 'Comparacion', 'Comparacion — Errores')], ...
      'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Figura 3 — Superficies deslizantes ───────────────────────────────────────
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [90 160 1100 580]);
tl3 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

h_leg3 = gobjects(1, 2);
for i = 1:4
    nexttile(tl3);
    yline(0, ':', 'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]); hold on;
    for k = 1:2
        hk = plot(data(k).t, data(k).s_q(:,i), lstyles{k}, ...
                  'Color', colors(k,:), 'LineWidth', lw);
        if i == 1; h_leg3(k) = hk; end
    end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$s_{q_%d}$ [rad/s]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs);
    grid on; box on; set(gca, 'FontSize', fs);
    xlim([data(1).t(1), data(1).t(end)]);
end

lgd3 = legend(nexttile(tl3, 1), h_leg3, ...
    cellfun(@(s) ['\rho = ' s], rho_disp, 'UniformOutput', false), ...
    'Orientation', 'horizontal', 'FontSize', fs, 'Interpreter', 'tex');
lgd3.Layout.Tile = 'north';
title(tl3, [strrep(sgtitle_str, 'Comparacion', 'Comparacion — Superficies Deslizantes')], ...
      'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Figura 4 — Torques de control ────────────────────────────────────────────
figure(4); clf;
set(gcf, 'Color', 'w', 'Position', [110 -80 1100 580]);
tl4 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

h_leg4 = gobjects(1, 2);
for i = 1:4
    nexttile(tl4);
    yline( 0,       ':',   'LineWidth', 0.9, 'Color', [0.5 0.5 0.5]); hold on;
    yline( TAU_MAX, '--k', 'LineWidth', 1.0, 'Label', sprintf('+%.1f N·m', TAU_MAX));
    yline(-TAU_MAX, '--k', 'LineWidth', 1.0, 'Label', sprintf('\x2212%.1f N·m', TAU_MAX));
    for k = 1:2
        hk = plot(data(k).t, data(k).tau(:,i), lstyles{k}, ...
                  'Color', colors(k,:), 'LineWidth', lw);
        if i == 1; h_leg4(k) = hk; end
    end
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N{\\cdot}m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs);
    ylim([-TAU_MAX * 1.25, TAU_MAX * 1.25]);
    grid on; box on; set(gca, 'FontSize', fs);
    xlim([data(1).t(1), data(1).t(end)]);
end

lgd4 = legend(nexttile(tl4, 1), h_leg4, ...
    cellfun(@(s) ['\rho = ' s], rho_disp, 'UniformOutput', false), ...
    'Orientation', 'horizontal', 'FontSize', fs, 'Interpreter', 'tex');
lgd4.Layout.Tile = 'north';
title(tl4, [strrep(sgtitle_str, 'Comparacion', 'Comparacion — Torques')], ...
      'FontSize', fs_title, 'FontWeight', 'bold');

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    if ~exist(output_dir, 'dir'); mkdir(output_dir); end

    fnames = {'plot_comp_q_tracking', 'plot_comp_q_error', ...
              'plot_comp_surfaces',   'plot_comp_torques'};
    for f = 1:4
        exportgraphics(figure(f), fullfile(output_dir, [fnames{f} '.png']), 'Resolution', 300);
        exportgraphics(figure(f), fullfile(output_dir, [fnames{f} '.eps']), ...
                       'ContentType', 'vector', 'Resolution', 600);
    end
    fprintf('Graficas guardadas en:\n  %s\n', output_dir);
end
