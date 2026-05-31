%% validate_omdyn_vs_urdf.m
% Compara OMDyn.m (modelo simbolico code-generated) con la dinamica
% obtenida desde openmani.urdf via importrobot (Robotics System Toolbox).
%
% Para cada configuracion de prueba (q, dq) calcula y compara:
%   M(q)        — matriz de inercia 4x4
%   g(q)        — torques de gravedad (dq = 0)
%   phib(q,dq)  — Coriolis/centrifuga + gravedad
%
% Requisito: Robotics System Toolbox (importrobot, massMatrix,
%            velocityProduct, gravityTorque).
%
% Resultado: PASS si el error relativo max en M y phib es < TOL_REL,
%            WARN si esta entre TOL_REL y 10*TOL_REL,
%            FAIL si supera 10*TOL_REL (indica parametros diferentes).

clear; close all; clc;

%% ========================================================================
%  Configuracion
%  ========================================================================

pkg_dir  = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';
urdf_path = fullfile(pkg_dir, 'urdf', 'openmani.urdf');

TOL_REL  = 1e-2;   % 1% — umbral PASS/WARN
SHOW_FIGS = true;

%% ========================================================================
%  1. Cargar robot desde URDF
%  ========================================================================

fprintf('Cargando URDF: %s\n', urdf_path);
robot = importrobot(urdf_path, 'MeshPath', 'none');
robot.Gravity    = [0, 0, -9.81];   % eje Z apunta hacia arriba en el frame base
robot.DataFormat = 'column';        % requerido por massMatrix/velocityProduct/gravityTorque

% Inspeccion: recopilar joints no-fijos desde robot.Bodies (orden del arbol)
all_names = {};
for ii = 1:robot.NumBodies
    jt = robot.Bodies{ii}.Joint;
    if ~strcmp(jt.Type, 'fixed')
        all_names{end+1} = jt.Name; %#ok<AGROW>
    end
end
nq_full = numel(all_names);

fprintf('\nJoints no-fijos detectados (%d total):\n', nq_full);
for ii = 1:nq_full
    fprintf('  [%d]  %s\n', ii, all_names{ii});
end

% Indices de joint1..joint4 dentro del vector de configuracion completo
arm_names = {'joint1', 'joint2', 'joint3', 'joint4'};
arm_idx   = zeros(1, 4);
for ii = 1:4
    pos = find(strcmp(all_names, arm_names{ii}));
    if isempty(pos)
        error('Joint "%s" no encontrado en el URDF. Verificar nombre.', arm_names{ii});
    end
    arm_idx(ii) = pos;
end
fprintf('\nIndices de joint1..4 en el vector completo: [%s]\n\n', num2str(arm_idx));

%% ========================================================================
%  2. Puntos de prueba
%  ========================================================================

% Limites articulares del OpenManipulator-X [rad]
q_lower = [-pi/2; -pi/2; -pi/2; -1.7907];
q_upper = [ pi/2;  pi/2;  pi/2;  2.0420];

rng(42);
n_rand = 6;
Q_rand = q_lower + (q_upper - q_lower) .* rand(4, n_rand);
DQ_rand = -5 + 10*rand(4, n_rand);  % [-5, 5] rad/s

% Incluir configuraciones especiales al inicio
q_home  = [0; 0; 0; 0];
q_lab5  = [pi/2; 0; pi/6; pi/3];  % x0 Lab 5 Act. 1
dq_zero = zeros(4,1);

Q_test  = [q_home, q_lab5, Q_rand];
DQ_test = [dq_zero, dq_zero, DQ_rand];
n_test  = size(Q_test, 2);

test_labels = ['Home', 'Lab5-x0', arrayfun(@(k) sprintf('Rand%d',k), 1:n_rand, 'UniformOutput', false)];
test_labels = [{'Home','Lab5-x0'}, arrayfun(@(k) sprintf('Rand%d',k), 1:n_rand, 'UniformOutput', false)];

%% ========================================================================
%  3. Bucle de validacion
%  ========================================================================

err_M_abs    = zeros(1, n_test);
err_M_rel    = zeros(1, n_test);
err_phib_abs = zeros(1, n_test);
err_phib_rel = zeros(1, n_test);
err_g_abs    = zeros(1, n_test);
err_g_rel    = zeros(1, n_test);

M_omdyn_all    = zeros(4, 4, n_test);
M_urdf_all     = zeros(4, 4, n_test);
phib_omdyn_all = zeros(4, n_test);
phib_urdf_all  = zeros(4, n_test);

fprintf('%-10s  %-12s  %-12s  %-12s  %-12s  %-12s  %-12s  %s\n', ...
    'Test', 'err_M_abs', 'err_M_rel', 'err_g_abs', 'err_g_rel', ...
    'err_ph_abs', 'err_ph_rel', 'Estado');
fprintf('%s\n', repmat('-', 1, 100));

for k = 1:n_test
    q  = Q_test(:,k);
    dq = DQ_test(:,k);

    %% — OMDyn —
    [M_om, phib_om]   = OMDyn(q, dq);
    [~,    g_om]      = OMDyn(q, zeros(4,1));   % phib con dq=0 = g(q)

    %% — URDF via importrobot —
    q_full  = zeros(nq_full, 1);
    q_full(arm_idx) = q;
    dq_full = zeros(nq_full, 1);
    dq_full(arm_idx) = dq;

    M_full    = massMatrix(robot, q_full);
    vp_full   = velocityProduct(robot, q_full, dq_full);
    gt_full   = gravityTorque(robot, q_full);

    % Extraer submatriz/subvector del brazo (indices arm_idx)
    M_urdf    = M_full(arm_idx, arm_idx);
    phib_urdf = vp_full(arm_idx) + gt_full(arm_idx);
    g_urdf    = gt_full(arm_idx);

    %% — Errores —
    diff_M    = M_urdf - M_om;
    diff_phib = phib_urdf - phib_om;
    diff_g    = g_urdf - g_om;

    denom_M    = max(norm(M_om, 'fro'),    1e-12);
    denom_phib = max(norm(phib_om),        1e-12);
    denom_g    = max(norm(g_om),           1e-12);

    err_M_abs(k)    = norm(diff_M, 'fro');
    err_M_rel(k)    = err_M_abs(k) / denom_M;
    err_phib_abs(k) = norm(diff_phib);
    err_phib_rel(k) = err_phib_abs(k) / denom_phib;
    err_g_abs(k)    = norm(diff_g);
    err_g_rel(k)    = err_g_abs(k) / denom_g;

    %% — Estado —
    worst_rel = max([err_M_rel(k), err_phib_rel(k)]);
    if worst_rel < TOL_REL
        estado = 'PASS';
    elseif worst_rel < 10*TOL_REL
        estado = 'WARN';
    else
        estado = 'FAIL';
    end

    fprintf('%-10s  %-12.2e  %-12.2e  %-12.2e  %-12.2e  %-12.2e  %-12.2e  %s\n', ...
        test_labels{k}, err_M_abs(k), err_M_rel(k), err_g_abs(k), err_g_rel(k), ...
        err_phib_abs(k), err_phib_rel(k), estado);

    M_omdyn_all(:,:,k)  = M_om;
    M_urdf_all(:,:,k)   = M_urdf;
    phib_omdyn_all(:,k) = phib_om;
    phib_urdf_all(:,k)  = phib_urdf;
end

%% ========================================================================
%  4. Resumen
%  ========================================================================

fprintf('%s\n', repmat('-', 1, 100));
fprintf('  Max error relativo  M     : %.4e\n', max(err_M_rel));
fprintf('  Max error relativo  phib  : %.4e\n', max(err_phib_rel));
fprintf('  Max error relativo  g(q)  : %.4e\n', max(err_g_rel));

worst_overall = max([max(err_M_rel), max(err_phib_rel)]);
fprintf('\n');
if worst_overall < TOL_REL
    fprintf('  VEREDICTO FINAL: PASS  — OMDyn.m es consistente con el URDF (err < %.0f%%)\n', TOL_REL*100);
elseif worst_overall < 10*TOL_REL
    fprintf('  VEREDICTO FINAL: WARN  — Discrepancia entre %.1f%% y %.0f%% (posible diferencia de parametros)\n', TOL_REL*100, 10*TOL_REL*100);
else
    fprintf('  VEREDICTO FINAL: FAIL  — OMDyn.m NO coincide con el URDF (err > %.0f%%)\n', 10*TOL_REL*100);
    fprintf('  Causa probable: OMDyn fue generado con parametros inerciales distintos a openmani.urdf.\n');
end

%% ========================================================================
%  5. Graficas
%  ========================================================================

if ~SHOW_FIGS, return; end

t_idx = 1:n_test;
lw = 1.3;
ms = 6;
fs = 10;

%% Figura 1: errores relativos por test

figure(1); clf;
set(gcf, 'Name', 'Errores relativos OMDyn vs URDF', 'Color', 'w', ...
    'Position', [80 80 900 400]);

subplot(1,2,1);
semilogy(t_idx, err_M_rel, 'o-', 'LineWidth', lw, 'MarkerSize', ms); hold on;
yline(TOL_REL,    '--r', 'PASS 1%',  'LabelHorizontalAlignment','left','FontSize',fs);
yline(10*TOL_REL, '--m', 'WARN 10%', 'LabelHorizontalAlignment','left','FontSize',fs);
set(gca, 'XTick', t_idx, 'XTickLabel', test_labels, 'XTickLabelRotation', 30, 'FontSize', fs);
ylabel('Error relativo ||M_{URDF}-M_{OM}||_F / ||M_{OM}||_F', 'FontSize', fs);
title('Matriz de inercia M(q)', 'FontSize', fs+1, 'FontWeight', 'bold');
grid on; box on;

subplot(1,2,2);
semilogy(t_idx, err_phib_rel, 's-', 'Color', [0.85 0.33 0.10], ...
    'LineWidth', lw, 'MarkerSize', ms); hold on;
yline(TOL_REL,    '--r', 'PASS 1%',  'LabelHorizontalAlignment','left','FontSize',fs);
yline(10*TOL_REL, '--m', 'WARN 10%', 'LabelHorizontalAlignment','left','FontSize',fs);
set(gca, 'XTick', t_idx, 'XTickLabel', test_labels, 'XTickLabelRotation', 30, 'FontSize', fs);
ylabel('Error relativo ||\phi_{URDF}-\phi_{OM}|| / ||\phi_{OM}||', 'FontSize', fs);
title('Vector de bias \phi(q,dq) = C·dq + g', 'FontSize', fs+1, 'FontWeight', 'bold');
grid on; box on;

sgtitle('Validacion OMDyn.m vs openmani.urdf', 'FontSize', fs+3, 'FontWeight', 'bold');

%% Figura 2: comparacion elemento a elemento de M para test Lab5-x0

k_ref   = 2;   % indice del test Lab5-x0
M_om_r  = M_omdyn_all(:,:,k_ref);
M_ur_r  = M_urdf_all(:,:,k_ref);

figure(2); clf;
set(gcf, 'Name', 'Comparacion M(q) — Lab5-x0', 'Color', 'w', ...
    'Position', [100 120 1000 700]);

tl = tiledlayout(4, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:4
    for col = 1:4
        nexttile(tl);
        vals = [M_om_r(row,col), M_ur_r(row,col)];
        bar(vals, 0.5); hold on;
        set(gca, 'XTickLabel', {'OMDyn','URDF'}, 'FontSize', 8);
        title(sprintf('M(%d,%d)', row, col), 'FontSize', 8);
        grid on; box on;
        err_ij = abs(M_om_r(row,col) - M_ur_r(row,col));
        if err_ij > 1e-8
            xlabel(sprintf('\\Deltaabs=%.2e', err_ij), 'FontSize', 7);
        end
    end
end
title(tl, sprintf('Elementos de M(q)  —  config: %s', test_labels{k_ref}), ...
    'FontSize', 12, 'FontWeight', 'bold');

%% Figura 3: comparacion phib por articulacion — todos los tests

figure(3); clf;
set(gcf, 'Name', 'Comparacion phi(q,dq) por articulacion', 'Color', 'w', ...
    'Position', [120 140 1000 600]);

tl3 = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
c_om = [0.00 0.45 0.74];
c_ur = [0.85 0.33 0.10];

for jj = 1:4
    nexttile(tl3);
    plot(t_idx, phib_omdyn_all(jj,:), 'o-', 'Color', c_om, 'LineWidth', lw, 'MarkerSize', ms); hold on;
    plot(t_idx, phib_urdf_all(jj,:),  's--','Color', c_ur, 'LineWidth', lw, 'MarkerSize', ms);
    set(gca, 'XTick', t_idx, 'FontSize', fs);
    if jj == 4
        set(gca, 'XTickLabel', test_labels, 'XTickLabelRotation', 25);
    else
        set(gca, 'XTickLabel', {});
    end
    ylabel(sprintf('\\phi_%d [N·m]', jj), 'FontSize', fs);
    grid on; box on;
    if jj == 1
        legend('OMDyn', 'URDF', 'Orientation', 'horizontal', ...
               'FontSize', fs, 'Location', 'northoutside');
    end
end
title(tl3, 'Vector \phi(q,dq) = C(q,dq)·\dot{q} + g(q)  —  OMDyn vs URDF', ...
    'FontSize', 12, 'FontWeight', 'bold');
