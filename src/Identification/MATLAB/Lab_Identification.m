%% Lab_Identification.m
% Identificacion offline de parametros torque→corriente
% OpenMANIPULATOR-X, 4 joints (XM430-W350)
%
% Modelo por joint i:
%   I_meas_i(t) ≈ α_i · τ_model_i(t)
%                + Fv_i · dq_i(t)
%                + Fc_i · tanh(dq_i(t) / epsilon)
%                + I_offset_i
%
%   θ_i = [α_i, Fv_i, Fc_i, I_offset_i]   (lineal en θ → OLS)
%
% Estrategia:
%   J2, J3 → OLS completo (4 params): σ(τ) grande, cond(Φ) < 15  ✓
%   J1, J4 → OLS parcial  (3 params): σ(τ) pequeño → α fijo
%             α_fijo = media(α_J2, α_J3), justificado porque todos
%             los motores son XM430-W350 (mismo kt)
%
% Configurar la seccion "Configuracion" y ejecutar el script completo.

clear; clc; close all;

%% ── Configuracion ─────────────────────────────────────────────────────────────
log_id   = 17;          % ID del archivo CSV (hw_fl_data_<log_id>.csv)
epsilon  = 0.05;        % [rad/s] suavizado del tanh para friccion de Coulomb
EXPORT   = true;       % true → guardar PNG + EPS en output_dir

pkg_dir    = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';
csv_file   = fullfile(pkg_dir, 'data', 'lab4', 'real', 'act1', ...
                      sprintf('hw_fl_data_%d.csv', log_id));
output_dir = fullfile(pkg_dir, 'plots', 'identification', sprintf('log%d', log_id));

%% ── Constantes del motor ──────────────────────────────────────────────────────
kt        = 1.7826;          % [N·m/A]  constante de torque XM430-W350 (datasheet)
Iu        = 0.00269;         % [A/tick] unidad de corriente Dynamixel
kappa     = kt * Iu;         % [N·m/tick] = TORQUE_PER_CURRENT_TICK
alpha_nom = 1.0 / kappa;     % [ticks/N·m] valor nominal ≈ 208.5

%% ── Cargar datos ──────────────────────────────────────────────────────────────
T    = readtable(csv_file);
t    = T.t;
N    = height(T);
fprintf('Archivo : %s\n', csv_file);
fprintf('Muestras: %d   t_final=%.1f s\n\n', N, t(end));

% Extraer vectores por joint
tau  = [T.tau1,        T.tau2,        T.tau3,        T.tau4];        % [N·m]
dq   = [T.dq1,         T.dq2,         T.dq3,         T.dq4];         % [rad/s]
im   = [T.curr_meas1,  T.curr_meas2,  T.curr_meas3,  T.curr_meas4];  % [ticks]

%% ── OLS completo — J2, J3 (4 parámetros) ─────────────────────────────────────
%  Φ = [τ, dq, tanh(dq/ε), 1]   y = I_meas   θ = [α, Fv, Fc, I_off]

joints_full    = [2, 3];
theta_full     = zeros(4, 4);   % fila j = θ del joint j
ci_full        = zeros(4, 4, 2); % (:,j,1)=low  (:,j,2)=high
R2_full        = zeros(1, 4);
rmse_full      = zeros(1, 4);
cond_full      = zeros(1, 4);
I_pred_full    = zeros(N, 4);

fprintf('══════════════════════════════════════════════════════════════════\n');
fprintf('OLS COMPLETO  θ = [α, Fv, Fc, I_offset]   (J2, J3)\n');
fprintf('══════════════════════════════════════════════════════════════════\n');

for j = joints_full
    tanh_j = tanh(dq(:,j) / epsilon);
    Phi    = [tau(:,j), dq(:,j), tanh_j, ones(N,1)];
    y_j    = im(:,j);

    % OLS
    theta_j = (Phi' * Phi) \ (Phi' * y_j);
    y_hat   = Phi * theta_j;
    res     = y_j - y_hat;

    % Estadísticos
    sigma2   = sum(res.^2) / (N - 4);
    cov_th   = sigma2 * inv(Phi' * Phi);
    se       = sqrt(diag(cov_th));
    t_crit   = tinv(0.975, N - 4);
    ci_lo    = theta_j - t_crit * se;
    ci_hi    = theta_j + t_crit * se;

    ss_tot = sum((y_j - mean(y_j)).^2);
    R2     = 1 - sum(res.^2) / ss_tot;
    rmse   = sqrt(mean(res.^2));
    cond_j = cond(Phi);

    theta_full(:,j)   = theta_j;
    ci_full(:,j,1)    = ci_lo;
    ci_full(:,j,2)    = ci_hi;
    R2_full(j)        = R2;
    rmse_full(j)      = rmse;
    cond_full(j)      = cond_j;
    I_pred_full(:,j)  = y_hat;

    fprintf('\n  J%d   R²=%.4f   RMSE=%.2f ticks   cond(Φ)=%.1f\n', j, R2, rmse, cond_j);
    fprintf('    α     = %+8.2f  ticks/N·m     (nominal %.1f)   CI95[%+.1f, %+.1f]\n', ...
            theta_j(1), alpha_nom, ci_lo(1), ci_hi(1));
    fprintf('    Fv    = %+8.3f  ticks/(rad/s) CI95[%+.3f, %+.3f]\n', ...
            theta_j(2), ci_lo(2), ci_hi(2));
    fprintf('    Fc    = %+8.3f  ticks         CI95[%+.3f, %+.3f]\n', ...
            theta_j(3), ci_lo(3), ci_hi(3));
    fprintf('    I_off = %+8.3f  ticks         CI95[%+.3f, %+.3f]\n', ...
            theta_j(4), ci_lo(4), ci_hi(4));
end

%% ── α fijo para J1, J4 ───────────────────────────────────────────────────────
alpha_fixed = mean(theta_full(1, joints_full));   % promedio α_J2 y α_J3
fprintf('\n──────────────────────────────────────────────────────────────────\n');
fprintf('α FIJO = %.2f ticks/N·m  (media α_J2=%.1f, α_J3=%.1f)\n', ...
        alpha_fixed, theta_full(1,2), theta_full(1,3));
fprintf('  Justificacion: todos los joints usan XM430-W350 → mismo kt\n');
fprintf('──────────────────────────────────────────────────────────────────\n');

%% ── OLS parcial — J1, J4 (3 parámetros: Fv, Fc, I_offset) ───────────────────
joints_partial = [1, 4];
theta_partial  = zeros(3, 4);
ci_partial     = zeros(3, 4, 2);
R2_partial     = zeros(1, 4);
rmse_partial   = zeros(1, 4);

fprintf('\nOLS PARCIAL   θ = [Fv, Fc, I_offset]  con α fijo  (J1, J4)\n');
fprintf('══════════════════════════════════════════════════════════════════\n');

for j = joints_partial
    tanh_j  = tanh(dq(:,j) / epsilon);
    y_j     = im(:,j) - alpha_fixed * tau(:,j);   % restar contribucion de α
    Phi_p   = [dq(:,j), tanh_j, ones(N,1)];

    theta_p = (Phi_p' * Phi_p) \ (Phi_p' * y_j);
    y_hat_p = Phi_p * theta_p;
    res_p   = y_j - y_hat_p;

    sigma2   = sum(res_p.^2) / (N - 3);
    cov_p    = sigma2 * inv(Phi_p' * Phi_p);
    se_p     = sqrt(diag(cov_p));
    t_crit   = tinv(0.975, N - 3);
    ci_lo_p  = theta_p - t_crit * se_p;
    ci_hi_p  = theta_p + t_crit * se_p;

    y_full   = im(:,j);
    y_hat_full = alpha_fixed * tau(:,j) + y_hat_p;
    ss_tot   = sum((y_full - mean(y_full)).^2);
    R2       = 1 - sum((y_full - y_hat_full).^2) / ss_tot;
    rmse     = sqrt(mean((y_full - y_hat_full).^2));

    theta_partial(:,j)  = theta_p;
    ci_partial(:,j,1)   = ci_lo_p;
    ci_partial(:,j,2)   = ci_hi_p;
    R2_partial(j)       = R2;
    rmse_partial(j)     = rmse;
    I_pred_full(:,j)    = y_hat_full;

    fprintf('\n  J%d   R²=%.4f   RMSE=%.2f ticks   (α fijo=%.2f)\n', j, R2, rmse, alpha_fixed);
    fprintf('    Fv    = %+8.3f  ticks/(rad/s) CI95[%+.3f, %+.3f]\n', ...
            theta_p(1), ci_lo_p(1), ci_hi_p(1));
    fprintf('    Fc    = %+8.3f  ticks         CI95[%+.3f, %+.3f]\n', ...
            theta_p(2), ci_lo_p(2), ci_hi_p(2));
    fprintf('    I_off = %+8.3f  ticks         CI95[%+.3f, %+.3f]\n', ...
            theta_p(3), ci_lo_p(3), ci_hi_p(3));
end

%% ── Tabla resumen del modelo identificado ─────────────────────────────────────
fprintf('\n══════════════════════════════════════════════════════════════════\n');
fprintf('MODELO FINAL IDENTIFICADO\n');
fprintf('  I_meas_i ≈ α_i·τ_i + Fv_i·dq_i + Fc_i·tanh(dq_i/%.2f) + I_off_i\n\n', epsilon);
fprintf('  %5s | %9s | %12s | %10s | %13s | %5s\n', ...
        'Joint', 'α [t/Nm]', 'Fv [t/r/s]', 'Fc [t]', 'I_offset [t]', 'R²');
fprintf('  %s\n', repmat('-', 1, 65));

alpha_all = [alpha_fixed, theta_full(1,2), theta_full(1,3), alpha_fixed];
Fv_all    = [theta_partial(1,1), theta_full(2,2), theta_full(2,3), theta_partial(1,4)];
Fc_all    = [theta_partial(2,1), theta_full(3,2), theta_full(3,3), theta_partial(2,4)];
Ioff_all  = [theta_partial(3,1), theta_full(4,2), theta_full(4,3), theta_partial(3,4)];
R2_all    = [R2_partial(1), R2_full(2), R2_full(3), R2_partial(4)];

for j = 1:4
    src = 'parcial'; if any(joints_full == j), src = 'completo'; end
    fprintf('  J%d (%s) | %9.2f | %12.3f | %10.3f | %13.3f | %.4f\n', ...
            j, src, alpha_all(j), Fv_all(j), Fc_all(j), Ioff_all(j), R2_all(j));
end
fprintf('\n  α nominal = %.1f ticks/N·m   α fijo = %.2f ticks/N·m\n', ...
        alpha_nom, alpha_fixed);
fprintf('  kt efectivo = 1/(α_fijo × Iu) = %.4f N·m/A  (datasheet: %.4f)\n', ...
        1/(alpha_fixed * Iu), kt);

%% ── Figura 1: Serie temporal I_meas vs I_pred ─────────────────────────────────
fig1 = figure('Name', 'Identificacion: Serie temporal', ...
              'Color', 'w', 'Position', [50 50 1200 800]);

joint_labels = {'J1 (yaw)', 'J2 (hombro)', 'J3 (codo)', 'J4 (muneca)'};
colors = lines(2);

for j = 1:4
    subplot(4, 1, j);
    hold on; grid on; box on;
    plot(t, im(:,j),          'Color', [0.6 0.6 0.6], 'LineWidth', 0.8, ...
         'DisplayName', 'I_{meas}');
    plot(t, I_pred_full(:,j), 'Color', colors(2,:), 'LineWidth', 1.5, ...
         'DisplayName', 'I_{pred}');
    ylabel('[ticks]');
    title(sprintf('%s   R²=%.4f   RMSE=%.1f ticks', joint_labels{j}, R2_all(j)));
    if j == 1, legend('Location', 'northeast'); end
    if j == 4, xlabel('Tiempo [s]'); end
    xlim([t(1), t(end)]);
end
sgtitle(sprintf('Identificacion torque→corriente — Log %d', log_id), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 2: Grafico de paridad (I_meas vs I_pred) ──────────────────────────
fig2 = figure('Name', 'Identificacion: Paridad', ...
              'Color', 'w', 'Position', [100 100 900 800]);

for j = 1:4
    subplot(2, 2, j);
    hold on; grid on; box on; axis equal;
    scatter(I_pred_full(:,j), im(:,j), 4, t, 'filled', 'MarkerFaceAlpha', 0.5);
    lims = [min([I_pred_full(:,j); im(:,j)]), max([I_pred_full(:,j); im(:,j)])];
    plot(lims, lims, 'r--', 'LineWidth', 1.5, 'DisplayName', 'ideal');
    xlabel('I_{pred} [ticks]');
    ylabel('I_{meas} [ticks]');
    title(sprintf('%s   R²=%.4f', joint_labels{j}, R2_all(j)));
    colorbar; clim([t(1), t(end)]);
end
sgtitle(sprintf('Paridad I_{pred} vs I_{meas} — Log %d', log_id), ...
        'FontSize', 14, 'FontWeight', 'bold');
colormap('parula');

%% ── Figura 3: Residuos ────────────────────────────────────────────────────────
fig3 = figure('Name', 'Identificacion: Residuos', ...
              'Color', 'w', 'Position', [150 150 1200 700]);

for j = 1:4
    res_j = im(:,j) - I_pred_full(:,j);
    subplot(4, 2, 2*j-1);
    hold on; grid on; box on;
    plot(t, res_j, 'Color', colors(1,:), 'LineWidth', 0.8);
    yline(0, 'k--', 'LineWidth', 1);
    xlabel('Tiempo [s]');
    ylabel('[ticks]');
    title(sprintf('Residuos %s', joint_labels{j}));
    xlim([t(1), t(end)]);
    rmse_j = sqrt(mean(res_j.^2));
    text(0.02, 0.88, sprintf('RMSE=%.2f ticks', rmse_j), ...
         'Units', 'normalized', 'FontSize', 9);

    subplot(4, 2, 2*j);
    histogram(res_j, 40, 'FaceColor', colors(1,:), 'FaceAlpha', 0.7, ...
              'EdgeColor', 'none', 'Normalization', 'pdf');
    hold on; grid on; box on;
    xr = linspace(min(res_j), max(res_j), 200);
    plot(xr, normpdf(xr, mean(res_j), std(res_j)), 'k-', 'LineWidth', 1.5);
    xlabel('Residuo [ticks]');
    ylabel('PDF');
    title(sprintf('Histograma %s  σ=%.2f', joint_labels{j}, std(res_j)));
end
sgtitle(sprintf('Residuos del modelo identificado — Log %d', log_id), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Figura 4: Parametros identificados con intervalos de confianza ────────────
fig4 = figure('Name', 'Identificacion: Parametros', ...
              'Color', 'w', 'Position', [200 200 1100 700]);

param_names = {'\alpha  [ticks/N\cdotm]', ...
               'F_v  [ticks/(rad/s)]', ...
               'F_c  [ticks]', ...
               'I_{offset}  [ticks]'};
param_vals  = [alpha_all; Fv_all; Fc_all; Ioff_all];

% CI para cada parametro (unificado: parcial para J1/J4, completo para J2/J3)
ci_lo_all = zeros(4,4);
ci_hi_all = zeros(4,4);
% α: solo J2/J3 tienen CI analitico; J1/J4 α es fijo → CI = NaN
for j = joints_full
    ci_lo_all(1,j) = ci_full(1,j,1);
    ci_hi_all(1,j) = ci_full(1,j,2);
    ci_lo_all(2,j) = ci_full(2,j,1);
    ci_hi_all(2,j) = ci_full(2,j,2);
    ci_lo_all(3,j) = ci_full(3,j,1);
    ci_hi_all(3,j) = ci_full(3,j,2);
    ci_lo_all(4,j) = ci_full(4,j,1);
    ci_hi_all(4,j) = ci_full(4,j,2);
end
for j = joints_partial
    ci_lo_all(1,j) = alpha_fixed;  % fijo
    ci_hi_all(1,j) = alpha_fixed;
    ci_lo_all(2,j) = ci_partial(1,j,1);
    ci_hi_all(2,j) = ci_partial(1,j,2);
    ci_lo_all(3,j) = ci_partial(2,j,1);
    ci_hi_all(3,j) = ci_partial(2,j,2);
    ci_lo_all(4,j) = ci_partial(3,j,1);
    ci_hi_all(4,j) = ci_partial(3,j,2);
end

bar_colors = lines(4);
jx = 1:4;
for p = 1:4
    subplot(2, 2, p);
    hold on; grid on; box on;
    b = bar(jx, param_vals(p,:), 0.6, 'FaceAlpha', 0.8);
    b.CData = bar_colors;
    b.FaceColor = 'flat';
    err_lo = param_vals(p,:) - ci_lo_all(p,:);
    err_hi = ci_hi_all(p,:)  - param_vals(p,:);
    % Para α fijo (J1/J4) err = 0 → no grafica barra de error
    errorbar(jx, param_vals(p,:), err_lo, err_hi, 'k.', 'LineWidth', 1.5, ...
             'CapSize', 8);
    if p == 1
        yline(alpha_nom, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Nominal');
        legend('Location', 'northeast');
    end
    xticks(jx);
    xticklabels({'J1','J2','J3','J4'});
    xlabel('Joint');
    ylabel(param_names{p});
    title(param_names{p}, 'Interpreter', 'tex');
end
sgtitle(sprintf('Parametros identificados ± IC95%%  — Log %d', log_id), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ── Exportar figuras ──────────────────────────────────────────────────────────
if EXPORT
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end
    fig_list  = {fig1, fig2, fig3, fig4};
    fig_names = {'01_serie_temporal', '02_paridad', '03_residuos', '04_parametros'};
    for k = 1:numel(fig_list)
        base = fullfile(output_dir, fig_names{k});
        exportgraphics(fig_list{k}, [base '.png'], 'Resolution', 300);
        exportgraphics(fig_list{k}, [base '.eps'], 'ContentType', 'vector', ...
                       'Resolution', 600);
        fprintf('Guardado: %s\n', base);
    end
end
