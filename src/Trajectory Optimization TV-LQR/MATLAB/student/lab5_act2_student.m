%% lab5_act2_student.m
% Trajectory Optimization y TV-LQR — Laboratorio 5, Actividad 2
% Control de Sistemas No Lineales — OpenMANIPULATOR-X de 4 GDL
%
% Objetivo: Mismo x0 y yf que Act. 1, evitando el obstaculo MDF.
%
% Archivos requeridos en el path de MATLAB:
%   - OMDyn.m
%   - open_manx_fkin.m
%
% Figuras generadas:
%   Figura 1 — Trayectorias articulares optimizadas q_ref
%   Figura 2 — Entradas optimizadas u_ref
%   Figura 3 — Seguimiento articular TV-LQR (simulacion vs referencia)
%   Figura 4 — Trayectoria cartesiana 3D + arco MDF
%   Figura 5 — Señal de control TV-LQR vs referencia

clear; close all; clc;
rng(1);

EXPORT_FIGS = false;

% ── Identificadores de sesion (editar antes de cada ejecucion) ────────────
act_num            = 2;
trial_num          = 1;
use_saved_solution = false;

pkg_dir    = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';
matlab_dir = fullfile(pkg_dir, 'src', 'Trajectory Optimization TV-LQR', 'MATLAB');
zmin_dir   = fullfile(matlab_dir, 'zmin');
if ~exist(zmin_dir, 'dir'), mkdir(zmin_dir); end
zmin_file  = fullfile(zmin_dir, sprintf('zmin_act%d_%d.mat', act_num, trial_num));

% Metodo Riccati: 'zoh' (ZOH exacto) | 'sqrt_rk4' (forma sqrt, RK4 hacia atras)
riccati_method = 'zoh';

%% ========================================================================
%  1. Parametros generales
%  ========================================================================

% ── COMPLETAR ─────────────────────────────────────────────────────────────
N  = ;        % Numero de nodos/intervalos de control
Ts = ;        % Tiempo de muestreo [s]
% ──────────────────────────────────────────────────────────────────────────
nx = 8;
nu = 4;

% ── COMPLETAR ─────────────────────────────────────────────────────────────
x0 = ;        % Estado inicial: [q1; q2; q3; q4; dq1; dq2; dq3; dq4]
yf = ;        % Objetivo cartesiano: [x; y; z; phi]
% ──────────────────────────────────────────────────────────────────────────

if use_saved_solution
    if ~exist(zmin_file, 'file')
        error('use_saved_solution=true pero no existe: %s', zmin_file);
    end
    sv   = load(zmin_file, 'zmin', 'exitflag', 'output', 'N', 'Ts', 'x0', 'yf');
    zmin = sv.zmin;  exitflag = sv.exitflag;  output = sv.output;
    N = sv.N;  Ts = sv.Ts;  x0 = sv.x0;  yf = sv.yf;
    fprintf('Cargado: %s  (N=%d  Ts=%.3f s)\n\n', zmin_file, N, Ts);
end

ukmax =  1.0;
ukmin = -1.0;

q_lower = [-3/4*pi; -11/18*pi; -11/18*pi;  -5/9*pi];
q_upper = [ 3/4*pi;   5/9*pi;     pi/2; 23/36*pi];
dq_max  = 10;

%% ========================================================================
%  2. Parametros del obstaculo MDF
%  ========================================================================

x_obs  = 0.22725;
y_obs  = 0.0;
z_ceil = 0.158;
rx_obs = 0.075;
ry_obs = 0.160;
x_lo   = x_obs - rx_obs;
x_hi   = x_obs + rx_obs;
y_lo   = y_obs - ry_obs;
y_hi   = y_obs + ry_obs;

% Constraint XZ: x_k - x_lo - alpha_z*(z_k - z_ceil_safe)^2 <= 0
alpha_z     = 50.0;
z_ceil_safe = z_ceil + 0.010;

fprintf('--- Obstaculo MDF ---\n');
fprintf('  Footprint link1: X=[%.5f, %.5f]  Y=[%.3f, %.3f] m\n', x_lo, x_hi, y_lo, y_hi);
fprintf('  z_ceil=%.3f m  z_ceil_safe=%.3f m (+10 mm)  alpha_z=%.1f\n\n', ...
        z_ceil, z_ceil_safe, alpha_z);

%% ========================================================================
%  3. Inicializacion de la optimizacion
%  ========================================================================

[~, u0g] = OMDyn(x0(1:4), zeros(4,1));
u0g = u0g(:);
u0g = max(min(u0g, ukmax), ukmin);

x_lower = [q_lower;             -dq_max * ones(4,1)];
x_upper = [q_upper;              dq_max * ones(4,1)];
lb = [repmat(x_lower, N, 1);  ukmin * ones(nu*N, 1)];
ub = [repmat(x_upper, N, 1);  ukmax * ones(nu*N, 1)];

z0 = [kron(ones(N,1), x0); kron(ones(N,1), u0g)];
fprintf('Warm start: punto constante en x0 con compensacion gravitatoria\n\n');

options = optimoptions('fmincon', ...
    'Display',               'iter', ...
    'Algorithm',             'sqp', ...
    'MaxFunctionEvaluations', 200000, ...
    'MaxIterations',          1000, ...
    'OptimalityTolerance',    1e-4, ...
    'ConstraintTolerance',    1e-4, ...
    'OutputFcn',              @logTimingFcn);

%% ========================================================================
%  4. Trajectory optimization
%  ========================================================================

if ~use_saved_solution
    exitflag = NaN;
    output   = struct();
    [zmin, ~, exitflag, output] = fmincon( ...
        @(z) Jcosto(z, Ts, N, nx, nu, x0, yf), ...
        z0, [], [], [], [], lb, ub, ...
        @(z) restr(z, Ts, N, nx, nu, x0, x_lo, z_ceil_safe, alpha_z), ...
        options);
    save(zmin_file, 'zmin', 'exitflag', 'output', 'N', 'Ts', 'x0', 'yf');
    fprintf('Guardado: %s\n', zmin_file);
end

fprintf('\nExitflag fmincon: %g\n', exitflag);
if isfield(output, 'message'), fprintf('Mensaje: %s\n\n', output.message); end

%% ========================================================================
%  5. Recuperar trayectorias
%  ========================================================================

Xref = reshape(zmin(1:nx*N),        [nx N]);
Uref = reshape(zmin(nx*N+(1:nu*N)), [nu N]);

%% ========================================================================
%  6. Verificar evasion del obstaculo
%  ========================================================================

max_cxz      = -inf;
min_phys_clr = inf;
for k = 1:N
    yk   = open_manx_fkin(Xref(1:4,k));
    c_xz = yk(1) - x_lo - alpha_z*(yk(3) - z_ceil_safe)^2;
    if c_xz > max_cxz, max_cxz = c_xz; end
    if yk(1) >= x_lo
        clr = yk(3) - z_ceil;
        if clr < min_phys_clr, min_phys_clr = clr; end
    end
end

fprintf('--- Verificacion obstaculo (constraint XZ) ---\n');
if max_cxz <= 0
    fprintf('  OK — constraint satisfecho (max c = %.5f)\n\n', max_cxz);
else
    fprintf('  ADVERTENCIA: constraint violado (max c = %.5f > 0)\n', max_cxz);
    fprintf('  EE entra al arco cerca del techo. Re-optimizar.\n\n');
end
if ~isinf(min_phys_clr)
    fprintf('  Clearance minimo sobre techo fisico (z_ceil=%.3f m): %.4f m\n\n', z_ceil, min_phys_clr);
else
    fprintf('  INFO: trayectoria fuera del footprint en todos los pasos.\n\n');
end

%% ========================================================================
%  7. Metricas de la trayectoria optimizada
%  ========================================================================

yref = zeros(4, N+1);
yref(:,1) = open_manx_fkin(x0(1:4));
for i = 1:N
    yref(:,i+1) = open_manx_fkin(Xref(1:4,i));
end

metrics_trajopt.exitflag     = exitflag;
metrics_trajopt.max_abs_uref = max(abs(Uref), [], 2);

fprintf('\n============================================================\n');
fprintf('Lab 5 Actividad 2 - Metricas de la trayectoria optimizada\n');
fprintf('============================================================\n');
fprintf('Exitflag fmincon                 : %g\n',   metrics_trajopt.exitflag);
fprintf('Max |u_ref| por articulacion     : [%s] N.m\n', ...
        num2str(metrics_trajopt.max_abs_uref', ' %.4f'));

%% ========================================================================
%  8. Linealizacion numerica variante en el tiempo
%  ========================================================================

Ak = zeros(nx, nx, N);
Bk = zeros(nx, nu, N);
eps_jac = 1e-6;

for k = 1:N
    xk = Xref(:,k); uk = Uref(:,k);
    for i = 1:nx
        dx = zeros(nx,1); dx(i) = eps_jac;
        % ── COMPLETAR ──────────────────────────────────────────────────────
        Ak(:,i,k) = ;
        % ───────────────────────────────────────────────────────────────────
    end
    for i = 1:nu
        du = zeros(nu,1); du(i) = eps_jac;
        % ── COMPLETAR ──────────────────────────────────────────────────────
        Bk(:,i,k) = ;
        % ───────────────────────────────────────────────────────────────────
    end
end

%% ========================================================================
%  9. TV-LQR — calculo de ganancias variantes en el tiempo
%  ========================================================================

% ── COMPLETAR: definir matrices de ponderacion ─────────────────────────────
Qk = diag([ ; ; ; ; ; ; ; ]);
Rk = *eye(nu);
% ──────────────────────────────────────────────────────────────────────────
Qf = Qk;
K_TV = zeros(nu, nx, N);

switch riccati_method
    case 'zoh'
        S_next = Qf;
        for k = N:-1:1
            A = Ak(:,:,k);  B = Bk(:,:,k);
            % ── COMPLETAR ──────────────────────────────────────────────────
            % Discretizar (A,B) por ZOH usando expm.
            % Calcular K_TV(:,:,k) y actualizar S_next (simetrizar).
            % ───────────────────────────────────────────────────────────────
        end

    case 'sqrt_rk4'
        Pk = zeros(nx, nx, N);
        Sk = zeros(nx, nx, N);
        Pk(:,:,N)   = sqrtm(0.05 * Qf);
        Sk(:,:,N)   = Pk(:,:,N) * Pk(:,:,N)';
        K_TV(:,:,N) = Rk \ (Bk(:,:,N)' * Sk(:,:,N));
        for k = N:-1:2
            P   = Pk(:,:,k);
            A   = Ak(:,:,k);
            B   = Bk(:,:,k);
            jj  = 100;
            TsR = Ts / jj;
            % ── COMPLETAR ──────────────────────────────────────────────────
            % Integrar Ricc_sqrt con RK4 hacia atras (jj sub-pasos).
            % Actualizar Pk(:,:,k-1), Sk(:,:,k-1) y K_TV(:,:,k-1).
            % ───────────────────────────────────────────────────────────────
        end

    otherwise
        error('riccati_method debe ser ''zoh'' o ''sqrt_rk4''.');
end

assert(all(isfinite(K_TV(:))), 'K_TV contiene NaN/Inf.');
fprintf('K_TV calculado.\n');

%% ========================================================================
%  10. Simulacion TV-LQR (sistema no lineal, perturbacion inicial)
%  ========================================================================

x0sim = x0 + 0.05*randn(nx,1);
[T, Xsim] = ode45( ...
    @(t,x) OM4dof(t, x, controlTVLQR(t, x, Uref, Xref, Ts, N, K_TV)), ...
    0:Ts:(N*Ts), x0sim);
Xsim = Xsim';

ysim = zeros(4, numel(T));
for i = 1:numel(T)
    ysim(:,i) = open_manx_fkin(Xsim(1:4,i));
end

U_tvlqr_raw = zeros(nu, numel(T));
U_tvlqr_sat = zeros(nu, numel(T));
for i = 1:numel(T)
    U_tvlqr_raw(:,i) = controlTVLQR(T(i), Xsim(:,i), Uref, Xref, Ts, N, K_TV);
    U_tvlqr_sat(:,i) = max(min(U_tvlqr_raw(:,i), ukmax), ukmin);
end

joint_error = Xsim(1:4,:) - [x0(1:4), Xref(1:4,:)];

metrics_tvlqr.final_error_cart    = norm(ysim(1:3,end) - yf(1:3));
metrics_tvlqr.max_abs_utvlqr      = max(abs(U_tvlqr_sat), [], 2);
metrics_tvlqr.sat_percent         = 100*mean(abs(U_tvlqr_raw) >= (ukmax - 1e-8), 2);
metrics_tvlqr.max_abs_joint_error = max(abs(joint_error), [], 2);
metrics_tvlqr.rms_joint_error     = sqrt(mean(joint_error.^2, 2));

fprintf('\n============================================================\n');
fprintf('Lab 5 Actividad 2 - Metricas de seguimiento TV-LQR\n');
fprintf('============================================================\n');
fprintf('Error cartesiano final ||y(tf)-yf||: %.6f m\n',  metrics_tvlqr.final_error_cart);
fprintf('Max |u_TVLQR| por articulacion    : [%s] N.m\n', num2str(metrics_tvlqr.max_abs_utvlqr', ' %.4f'));
fprintf('Saturacion TV-LQR por articulacion: [%s] %%\n',  num2str(metrics_tvlqr.sat_percent', ' %.2f'));
fprintf('Error max artic                   : [%s] rad\n', num2str(metrics_tvlqr.max_abs_joint_error', ' %.5f'));
fprintf('Error RMS artic                   : [%s] rad\n', num2str(metrics_tvlqr.rms_joint_error', ' %.5f'));

%% ========================================================================
%  11. Estilo comun
%  ========================================================================

lw          = 1.3;
fs          = 11;
fs_ttl      = 13;
c_ref       = [0.8500 0.3250 0.0980];
c_sim       = [0.0000 0.4470 0.7410];
c_obs_color = [0.80 0.50 0.20];
xlims       = [T(1), T(end)];
qLabels     = {'$q_1$ [rad]','$q_2$ [rad]','$q_3$ [rad]','$q_4$ [rad]'};
tX = Ts:Ts:N*Ts;
tU = 0:Ts:(N-1)*Ts;

%% ========================================================================
%  Fig 1 — Trayectorias articulares optimizadas
%  ========================================================================

figure(1); clf;
set(gcf, 'Name', 'q_ref optimizado Act.2', 'Color', 'w', 'Position', [100 80 1100 650]);
tl1 = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for iq = 1:4
    nexttile(tl1);
    plot(tX, Xref(iq,:), '-', 'Color', c_ref, 'LineWidth', lw);
    ylabel(qLabels{iq}, 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on; set(gca, 'FontSize', fs); xlim([tX(1), tX(end)]);
    if iq < 4, set(gca, 'XTickLabel', []); else, xlabel('Tiempo [s]', 'FontSize', fs); end
end
title(tl1, 'Trayectorias articulares optimizadas $q_{ref}$ --- Act. 2 (con obstaculo)', ...
      'Interpreter', 'latex', 'FontSize', fs_ttl, 'FontWeight', 'bold');

%% ========================================================================
%  Fig 2 — Entradas optimizadas
%  ========================================================================

figure(2); clf;
set(gcf, 'Name', 'u_ref optimizado Act.2', 'Color', 'w', 'Position', [120 100 1100 560]);
tl2 = tiledlayout(nu, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for iu = 1:nu
    nexttile(tl2);
    plot(tU, Uref(iu,:), '-', 'Color', c_ref, 'LineWidth', lw);
    ylabel(sprintf('$u_%d\\;[\\mathrm{N\\cdot m}]$', iu), 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on; set(gca, 'FontSize', fs); xlim([tU(1), tU(end)]);
    if iu < nu, set(gca, 'XTickLabel', []); else, xlabel('Tiempo [s]', 'FontSize', fs); end
end
title(tl2, 'Entradas optimizadas $u_{ref}$ --- Act. 2', 'Interpreter', 'latex', ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');

%% ========================================================================
%  Fig 3 — Seguimiento articular TV-LQR
%  ========================================================================

qRefPlot = [x0(1:4), Xref(1:4,:)];

figure(3); clf;
set(gcf, 'Name', 'Seguimiento TV-LQR Act.2', 'Color', 'w', 'Position', [140 80 1100 650]);
tl3 = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
h_s = []; h_r = [];
for iq = 1:4
    ax = nexttile(tl3);
    h1 = plot(T, Xsim(iq,:),    '-',  'Color', c_sim, 'LineWidth', lw); hold on;
    h2 = plot(T, qRefPlot(iq,:), '--', 'Color', c_ref, 'LineWidth', lw);
    if iq == 1, h_s = h1; h_r = h2; end
    ylabel(qLabels{iq}, 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
    if iq < 4, set(gca, 'XTickLabel', []); else, xlabel('Tiempo [s]', 'FontSize', fs); end
end
legend(nexttile(tl3,1), [h_s, h_r], {'Simulacion', 'Referencia'}, ...
       'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
title(tl3, 'Seguimiento articular TV-LQR — Act. 2 (con obstaculo)', ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');

%% ========================================================================
%  Fig 4 — Trayectoria 3D + arco MDF
%  ========================================================================

figure(4); clf;
set(gcf, 'Name', 'Trayectoria 3D + Obstaculo', 'Color', 'w', 'Position', [160 60 1100 780]);
hold on; grid on; box on;

h_obs_surf = patch([x_lo x_hi x_hi x_lo], [y_lo y_lo y_hi y_hi], ...
                   [z_ceil z_ceil z_ceil z_ceil], c_obs_color, ...
                   'FaceAlpha', 0.55, 'EdgeColor', [0.4 0.25 0.1], 'LineWidth', 0.8);
patch([x_lo x_hi x_hi x_lo], repmat(y_lo, 1, 4), [0 0 z_ceil z_ceil], c_obs_color, ...
      'FaceAlpha', 0.35, 'EdgeColor', [0.4 0.25 0.1], 'LineWidth', 0.8);
patch([x_lo x_hi x_hi x_lo], repmat(y_hi, 1, 4), [0 0 z_ceil z_ceil], c_obs_color, ...
      'FaceAlpha', 0.35, 'EdgeColor', [0.4 0.25 0.1], 'LineWidth', 0.8);

for rr = 1:20
    x0mc = x0 + 0.10*randn(nx,1);
    [~, Xmc] = ode45( ...
        @(t,x) OM4dof(t, x, controlTVLQR(t, x, Uref, Xref, Ts, N, K_TV)), ...
        0:Ts:N*Ts, x0mc);
    Xmc = Xmc';
    ymc = zeros(4, size(Xmc,2));
    for i = 1:size(Xmc,2), ymc(:,i) = open_manx_fkin(Xmc(1:4,i)); end
    if rr == 1
        h_mc = plot3(ymc(1,:), ymc(2,:), ymc(3,:), ...
                     '-', 'Color', [0.55 0.70 1.0], 'LineWidth', 0.8);
    else
        plot3(ymc(1,:), ymc(2,:), ymc(3,:), '-', 'Color', [0.55 0.70 1.0], 'LineWidth', 0.8);
    end
end

h_sim  = plot3(ysim(1,:), ysim(2,:), ysim(3,:), '-',  'Color', [0.0 0.2 0.8], 'LineWidth', 2.2);
h_ref  = plot3(yref(1,:), yref(2,:), yref(3,:), '--o', 'Color', c_ref, 'LineWidth', 1.4, 'MarkerSize', 4);
h_goal = plot3(yf(1), yf(2), yf(3), 'kx', 'MarkerSize', 13, 'LineWidth', 2.4);
h_x0   = plot3(yref(1,1), yref(2,1), yref(3,1), 'gs', 'MarkerSize', 9, ...
               'LineWidth', 2.0, 'MarkerFaceColor', 'g');

view(50, 22);
xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', fs);
ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', fs);
zlabel('$z$ [m]', 'Interpreter', 'latex', 'FontSize', fs);
legend([h_obs_surf, h_mc, h_sim, h_ref, h_goal, h_x0], ...
       {'Arco MDF', 'MC perturbado', 'Simulacion TV-LQR', ...
        'Referencia optimizada', '$y_f$ (objetivo)', '$y_0$ (inicio)'}, ...
       'Interpreter', 'latex', 'Location', 'northeastoutside', 'FontSize', fs-1);
title('Trayectoria cartesiana 3D con obstaculo MDF — TV-LQR Act. 2', ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');
set(gca, 'FontSize', fs);

%% ========================================================================
%  Fig 5 — Señal de control TV-LQR
%  ========================================================================

Uref_plot = zeros(nu, numel(T));
for i = 1:numel(T)
    k_idx = min(max(floor(T(i)/Ts)+1, 1), N);
    Uref_plot(:,i) = Uref(:,k_idx);
end

figure(5); clf;
set(gcf, 'Name', 'Control TV-LQR Act.2', 'Color', 'w', 'Position', [180 110 1100 560]);
tl5 = tiledlayout(nu, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
hu = []; hur = [];
for iu = 1:nu
    nexttile(tl5);
    h1 = plot(T, U_tvlqr_sat(iu,:), '-',  'Color', c_sim, 'LineWidth', lw); hold on;
    h2 = plot(T, Uref_plot(iu,:),   '--', 'Color', c_ref, 'LineWidth', lw);
    if iu == 1, hu = h1; hur = h2; end
    ylabel(sprintf('$u_%d\\;[\\mathrm{N\\cdot m}]$', iu), 'Interpreter', 'latex', 'FontSize', fs);
    ydata  = [U_tvlqr_sat(iu,:), Uref_plot(iu,:)];
    yrange = max(max(ydata) - min(ydata), 0.05);
    ylim([min(ydata) - 0.15*yrange, max(ydata) + 0.15*yrange]);
    grid on; box on; set(gca, 'FontSize', fs); xlim(xlims);
    if iu < nu, set(gca, 'XTickLabel', []); else, xlabel('Tiempo [s]', 'FontSize', fs); end
end
legend(nexttile(tl5,1), [hu, hur], {'TV-LQR', 'Referencia'}, ...
       'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
title(tl5, 'Señal de control TV-LQR vs referencia — Act. 2', ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');

%% ========================================================================
%  12. Exportacion de figuras
%  ========================================================================

if EXPORT_FIGS
    out_dir = fullfile(pkg_dir, 'plots', 'lab5', 'matlab', 'act2');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end
    fig_names = {'act2_qref', 'act2_uref', 'act2_tvlqr_seguimiento', ...
                 'act2_trayectoria_3d', 'act2_tvlqr_control'};
    for fi = 1:5
        base = fullfile(out_dir, fig_names{fi});
        exportgraphics(figure(fi), [base '.png'], 'Resolution', 300);
        exportgraphics(figure(fi), [base '.pdf'], 'ContentType', 'vector');
    end
    fprintf('\nFiguras guardadas en: %s\n', out_dir);
end

%% ========================================================================
%  Funciones locales
%  ========================================================================

function J = Jcosto(z, Ts, N, nx, nu, x0, yf)
% Funcion de costo para trajectory optimization.
% Penaliza: error cartesiano terminal, error cartesiano durante el horizonte,
% velocidades articulares, magnitud y variacion del torque.
% El obstaculo se maneja como hard constraint en restr().

    J = 0;

    % ── COMPLETAR ──────────────────────────────────────────────────────────
    % Definir matrices de peso Qy, Qf_cost, Qv, R.
    % Calcular y acumular los terminos del costo en J.
    % ───────────────────────────────────────────────────────────────────────
end

function [c_des, c_eq] = restr(z, Ts, N, nx, nu, x0, x_lo, z_ceil, alpha_z)
% Restricciones del problema de optimizacion.
% c_eq:  dinamica discreta (RK4)
% c_des: constraint de evasion del obstaculo (plano XZ)
%        x_k - x_lo - alpha_z*(z_k - z_ceil)^2 <= 0

    c_eq  = zeros(nx*N, 1);
    c_des = zeros(N, 1);

    for k = 0:N-1
        if k == 0, xk = x0; else, xk = z(nx*(k-1) + (1:nx)); end
        uk   = z(nx*N + nu*k + (1:nu));
        xkp1 = z(nx*k + (1:nx));

        % ── COMPLETAR ──────────────────────────────────────────────────────
        % Integrar la dinamica con RK4 usando OM4dof.
        % Calcular c_eq(nx*k+(1:nx)) = xkp1 - x_next_RK4.
        % ───────────────────────────────────────────────────────────────────

        % ── COMPLETAR ──────────────────────────────────────────────────────
        % Calcular la posicion cartesiana yk de xkp1 usando open_manx_fkin.
        % Calcular c_des(k+1) con el constraint de evasion del techo.
        % ───────────────────────────────────────────────────────────────────
    end
end

function dx = OM4dof(t, x, u) %#ok<INUSD>
% Modelo de espacio de estados del OpenManipulator-X de 4 GDL.
%   x = [q; dq],  dx = [dq; ddq]
%   M(q)*ddq + Phi(q,dq) = u - tau_fric,   tau_fric = bf*dq

    % ── COMPLETAR ──────────────────────────────────────────────────────────
end

function u = controlTVLQR(t, x, Uref, Xref, Ts, N, K_TV)
% Ley de control TV-LQR: u(t) = u_ref,k - K_k*(x - x_ref,k)

    % ── COMPLETAR ──────────────────────────────────────────────────────────
end

function dP = Ricc_sqrt(P, A, B, Q, R)
% Riccati forma sqrt, integrado hacia atras: S = P*P'.
% dP/dtau = A'P - (1/2)*P*P'*B*R^{-1}*B'*P + (1/2)*Q*P'^{-1}

    % ── COMPLETAR ──────────────────────────────────────────────────────────
end

function stop = logTimingFcn(~, optimValues, state)
    persistent t_prev t_start
    stop = false;
    switch state
        case 'init'
            t_start = tic; t_prev = tic;
            fprintf('[TIME] Optimizacion iniciada: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
        case 'iter'
            dt = toc(t_prev); elapsed = toc(t_start);
            fprintf('[TIME] iter=%4d  fval=%.6e  dt=%7.3fs  elapsed=%8.1fs\n', ...
                    optimValues.iteration, optimValues.fval, dt, elapsed);
            t_prev = tic;
        case 'done'
            elapsed = toc(t_start);
            fprintf('[TIME] Optimizacion finalizada: %s  total=%.1fs\n', ...
                    datestr(now, 'yyyy-mm-dd HH:MM:SS'), elapsed);
    end
end
