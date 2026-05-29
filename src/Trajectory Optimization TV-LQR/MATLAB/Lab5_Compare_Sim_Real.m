%% Lab5_Compare_Sim_Real.m
% Compara metricas y graficas entre la simulacion en Gazebo y la
% implementacion en hardware del Lab 5 (TV-LQR, Actividad 1).
%
% Requiere haber ejecutado el nodo en ambas modalidades y tener los CSV
% correspondientes en data/lab5/sim/ y data/lab5/real/.

clear; clc; close all;

%% ── Configuracion ────────────────────────────────────────────────────────────
test_num_sim  = 1;   % numero de log de simulacion
test_num_real = 1;   % numero de log de hardware

EXPORT_FIGS = false;

pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

%% ── Carga de datos ───────────────────────────────────────────────────────────
csv_sim  = fullfile(pkg_dir, 'data', 'lab5', 'sim', ...
                    sprintf('data_log_sim_lab5_%d.csv',  test_num_sim));
csv_real = fullfile(pkg_dir, 'data', 'lab5', 'real', ...
                    sprintf('data_log_real_lab5_%d.csv', test_num_real));

assert(isfile(csv_sim),  'Archivo no encontrado:\n  %s', csv_sim);
assert(isfile(csv_real), 'Archivo no encontrado:\n  %s', csv_real);

Ts = readtable(csv_sim);
Tr = readtable(csv_real);

ts   = Ts.t - Ts.t(1);
tr   = Tr.t - Tr.t(1);

qs   = [Ts.q1,  Ts.q2,  Ts.q3,  Ts.q4];
qr   = [Tr.q1,  Tr.q2,  Tr.q3,  Tr.q4];

q_refs = [Ts.q1_ref, Ts.q2_ref, Ts.q3_ref, Ts.q4_ref];
q_refr = [Tr.q1_ref, Tr.q2_ref, Tr.q3_ref, Tr.q4_ref];

taus = [Ts.tau1, Ts.tau2, Ts.tau3, Ts.tau4];
taur = [Tr.tau1, Tr.tau2, Tr.tau3, Tr.tau4];

%% ── Estilo ───────────────────────────────────────────────────────────────────
lw        = 1.6;
fs        = 11;
fs_title  = 12;
c_sim     = [0.0000 0.4470 0.7410];   % azul  — simulacion
c_real    = [0.8500 0.3250 0.0980];   % naranja — hardware
c_ref     = [0.5 0.5 0.5];           % gris  — referencia
jointNames = {'Articulacion 1', 'Articulacion 2', ...
              'Articulacion 3', 'Articulacion 4'};

%% ── Figura 1 — Comparacion de posiciones articulares ────────────────────────
figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 1100 700]);

for i = 1:4
    subplot(2,2,i);
    plot(ts, qs(:,i),     '-',  'Color', c_sim,  'LineWidth', lw); hold on;
    plot(tr, qr(:,i),     '-',  'Color', c_real, 'LineWidth', lw);
    plot(ts, q_refs(:,i), '--', 'Color', c_ref,  'LineWidth', lw);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$q_%d$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    if i == 1
        legend({'Simulacion', 'Hardware', 'Referencia'}, ...
               'Location', 'best', 'FontSize', fs-1);
    end
    grid on; box on; set(gca, 'FontSize', fs);
    xlim([0, max(ts(end), tr(end))]);
end
sgtitle('[Comparacion Sim vs Real] Posiciones articulares', ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 2 — Comparacion de errores articulares ───────────────────────────
e_sim  = qs - q_refs;
e_real = qr - q_refr;

figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [120 120 1100 560]);
tl2 = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:4
    nexttile(tl2);
    plot(ts, e_sim(:,i),  '-', 'Color', c_sim,  'LineWidth', lw); hold on;
    plot(tr, e_real(:,i), '-', 'Color', c_real, 'LineWidth', lw);
    yline(0, ':', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$e_{q_%d}$ [rad]', i), 'Interpreter', 'latex', 'FontSize', fs);
    title(jointNames{i}, 'FontSize', fs_title);
    if i == 1
        legend({'Simulacion', 'Hardware'}, 'Location', 'best', 'FontSize', fs-1);
    end
    grid on; box on; set(gca, 'FontSize', fs);
    xlim([0, max(ts(end), tr(end))]);
end
title(tl2, '[Comparacion Sim vs Real] Error de seguimiento articular', ...
      'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 3 — Comparacion de torques ───────────────────────────────────────
figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [140 140 1100 700]);

for i = 1:4
    subplot(2,2,i);
    plot(ts, taus(:,i), '-', 'Color', c_sim,  'LineWidth', lw); hold on;
    plot(tr, taur(:,i), '-', 'Color', c_real, 'LineWidth', lw);
    yline( 0.82, 'k--', 'LineWidth', 0.8);
    yline(-0.82, 'k--', 'LineWidth', 0.8);
    xlabel('Tiempo [s]', 'FontSize', fs);
    ylabel(sprintf('$\\tau_%d\\;[\\mathrm{N\\cdot m}]$', i), ...
           'Interpreter', 'latex', 'FontSize', fs);
    title(['Torque - ' jointNames{i}], 'FontSize', fs_title);
    if i == 1
        legend({'Simulacion', 'Hardware', '$\pm$TAU\_MAX'}, ...
               'Interpreter', 'latex', 'Location', 'best', 'FontSize', fs-1);
    end
    grid on; box on; set(gca, 'FontSize', fs);
    xlim([0, max(ts(end), tr(end))]);
end
sgtitle('[Comparacion Sim vs Real] Torques de control', ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Tabla de metricas ────────────────────────────────────────────────────────
fprintf('\n======================================================\n');
fprintf('Comparacion de metricas — Lab5 Actividad 1\n');
fprintf('======================================================\n');
fprintf('%-32s  %10s  %10s\n', 'Metrica', 'Sim', 'Real');
fprintf('%s\n', repmat('-', 1, 56));

for i = 1:4
    fprintf('Max |e_q%d| [rad]              %10.5f  %10.5f\n', i, ...
            max(abs(e_sim(:,i))), max(abs(e_real(:,i))));
end
fprintf('%s\n', repmat('-', 1, 56));
for i = 1:4
    fprintf('RMS  e_q%d  [rad]              %10.5f  %10.5f\n', i, ...
            sqrt(mean(e_sim(:,i).^2)), sqrt(mean(e_real(:,i).^2)));
end
fprintf('%s\n', repmat('-', 1, 56));
for i = 1:4
    sat_s = 100*mean(abs(taus(:,i)) >= 0.82 - 1e-3);
    sat_r = 100*mean(abs(taur(:,i)) >= 0.82 - 1e-3);
    fprintf('Saturacion tau%d [%%]            %10.2f  %10.2f\n', i, sat_s, sat_r);
end
fprintf('%s\n', repmat('-', 1, 56));
for i = 1:4
    fprintf('Max |tau%d| [N.m]              %10.4f  %10.4f\n', i, ...
            max(abs(taus(:,i))), max(abs(taur(:,i))));
end

%% ── Exportacion ──────────────────────────────────────────────────────────────
if EXPORT_FIGS
    out_dir = fullfile(pkg_dir, 'plots', 'lab5', 'compare', ...
                       sprintf('sim%d_real%d', test_num_sim, test_num_real));
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    fig_names = {'compare_q', 'compare_error', 'compare_tau'};
    for fi = 1:3
        base = fullfile(out_dir, fig_names{fi});
        exportgraphics(figure(fi), [base '.png'], 'Resolution', 300);
        exportgraphics(figure(fi), [base '.eps'], 'ContentType', 'vector', 'Resolution', 600);
    end
    fprintf('\nGraficas guardadas en: %s\n', out_dir);
end
