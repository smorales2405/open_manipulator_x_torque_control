%% lab5_act1_student.m
% Trajectory Optimization y TV-LQR — Laboratorio 5, Actividad 1
% Control de Sistemas No Lineales — OpenMANIPULATOR-X de 4 GDL
%
% Archivos requeridos en el path de MATLAB:
%   - OMDyn.m
%   - open_manx_fkin.m
%
% Figuras generadas:
%   Figura 1 — Trayectorias articulares optimizadas q_ref
%   Figura 2 — Entradas optimizadas u_ref
%   Figura 3 — Seguimiento articular con TV-LQR (simulacion vs referencia)
%   Figura 4 — Trayectoria cartesiana del efector final (Monte Carlo)
%   Figura 5 — Señal de control TV-LQR vs referencia

clear; close all; clc;
rng(1);

%% Configuracion

EXPORT_FIGS = false;

act_num            = 1;
trial_num          = 1;
use_saved_solution = false;

pkg_dir    = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';
matlab_dir = fullfile(pkg_dir, 'src', 'Trajectory Optimization TV-LQR', 'MATLAB');
zmin_dir   = fullfile(matlab_dir, 'zmin');
if ~exist(zmin_dir, 'dir'), mkdir(zmin_dir); end
zmin_file  = fullfile(zmin_dir, sprintf('zmin_act%d_%d.mat', act_num, trial_num));

% Metodo Riccati: 'zoh' (ZOH exacto) | 'sqrt' (forma sqrt, RK4 hacia atras)
riccati_method = 'zoh';
riccati_jj     = 100;   % sub-pasos RK4 por intervalo Ts (solo 'sqrt')

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

ukmax =  1;
ukmin = -1;

q_lower = [-3/4*pi; -11/18*pi; -11/18*pi;  -5/9*pi];
q_upper = [ 3/4*pi;   5/9*pi;     pi/2; 23/36*pi];
dq_max  = 10;

%% ========================================================================
%  2. Inicializacion de la optimizacion
%  ========================================================================

[~, u0g] = OMDyn(x0(1:4), x0(5:8));
u0g = u0g(:);
u0g = max(min(u0g, ukmax), ukmin);

x_lower = [q_lower;             -dq_max * ones(4,1)];
x_upper = [q_upper;              dq_max * ones(4,1)];
lb = [repmat(x_lower, N, 1);  ukmin * ones(nu*N, 1)];
ub = [repmat(x_upper, N, 1);  ukmax * ones(nu*N, 1)];

z0 = [kron(ones(N,1), x0); kron(ones(N,1), u0g)];

options = optimoptions('fmincon', ...
    'Display',               'iter', ...
    'Algorithm',             'sqp', ...
    'MaxFunctionEvaluations', 100000, ...
    'MaxIterations',          500, ...
    'OptimalityTolerance',    1e-4, ...
    'ConstraintTolerance',    1e-4, ...
    'OutputFcn',              @logTimingFcn);

%% ========================================================================
%  3. Trajectory optimization
%  ========================================================================

if ~use_saved_solution
    exitflag = NaN;
    output   = struct();
    fprintf('Warm start: punto constante en x0 con compensacion gravitatoria\n\n');
    [zmin, ~, exitflag, output] = fmincon( ...
        @(z) Jcosto(z, Ts, N, nx, nu, x0, yf), ...
        z0, [], [], [], [], lb, ub, ...
        @(z) restr(z, Ts, N, nx, nu, x0), ...
        options);
    save(zmin_file, 'zmin', 'exitflag', 'output', 'N', 'Ts', 'x0', 'yf');
    fprintf('Guardado: %s\n', zmin_file);
end

fprintf('\nExitflag fmincon: %g\n', exitflag);
if isfield(output, 'message')
    fprintf('Mensaje: %s\n\n', output.message);
end

%%  4. Recuperar trayectorias optimizadas

X = zmin(1:nx*N);
U = zmin(nx*N + (1:nu*N));

Xref = reshape(X, [nx N]);
Uref = reshape(U, [nu N]);

%% Graficas: qref y uref

tX = Ts:Ts:N*Ts;
tU = 0:Ts:(N-1)*Ts;

fs     = 11;
fs_ttl = 13;
lw     = 1.3;
c_ref  = [0.8500 0.3250 0.0980];
qLabels = {'$q_1$ [rad]', '$q_2$ [rad]', '$q_3$ [rad]', '$q_4$ [rad]'};

figure(1); clf;
set(gcf, 'Name', 'Trayectorias articulares optimizadas', ...
         'Color', 'w', 'Position', [120 80 1100 650]);
tlB1 = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for iq = 1:4
    nexttile(tlB1);
    plot(tX, Xref(iq,:), '-', 'Color', c_ref, 'LineWidth', lw);
    ylabel(qLabels{iq}, 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on; set(gca, 'FontSize', fs); xlim([tX(1), tX(end)]);
    if iq < 4, set(gca, 'XTickLabel', []); else, xlabel('Tiempo [s]', 'FontSize', fs); end
end
title(tlB1, 'Trayectorias articulares optimizadas $q_{ref}$', ...
      'Interpreter', 'latex', 'FontSize', fs_ttl, 'FontWeight', 'bold');

figure(2); clf;
set(gcf, 'Name', 'Entradas optimizadas', ...
         'Color', 'w', 'Position', [160 100 1100 560]);
tlB2 = tiledlayout(nu, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for iu = 1:nu
    nexttile(tlB2);
    plot(tU, Uref(iu,:), '-', 'Color', c_ref, 'LineWidth', lw);
    ylabel(sprintf('$u_%d$ [N.m]', iu), 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on; set(gca, 'FontSize', fs); xlim([tU(1), tU(end)]);
    if iu < nu, set(gca, 'XTickLabel', []); else, xlabel('Tiempo [s]', 'FontSize', fs); end
end
title(tlB2, 'Entradas optimizadas $u_{ref}$', ...
      'Interpreter', 'latex', 'FontSize', fs_ttl, 'FontWeight', 'bold');

%% 4.1 Metricas de la trayectoria optimizada

yref = zeros(4, N+1);
yref(:,1) = open_manx_fkin(x0(1:4));
for i = 1:N
    yref(:,i+1) = open_manx_fkin(Xref(1:4,i));
end

metrics_trajopt = struct();
metrics_trajopt.exitflag     = exitflag;
metrics_trajopt.max_abs_uref = max(abs(Uref), [], 2);

printMetricsTrajOpt(metrics_trajopt);

%% ========================================================================
%  5. Linealizacion numerica variante en el tiempo
%  ========================================================================

Ak = zeros(nx, nx, N);
Bk = zeros(nx, nu, N);
eps_jac = 1e-6;

for k = 1:N
    xrefk = Xref(:,k);
    urefk = Uref(:,k);
    for i = 1:nx
        incx = zeros(nx,1); incx(i) = eps_jac;
        % ── COMPLETAR ──────────────────────────────────────────────────────
        Ak(:,i,k) = ;
        % ───────────────────────────────────────────────────────────────────
    end
    for i = 1:nu
        incu = zeros(nu,1); incu(i) = eps_jac;
        % ── COMPLETAR ──────────────────────────────────────────────────────
        Bk(:,i,k) = ;
        % ───────────────────────────────────────────────────────────────────
    end
end

%% ========================================================================
%  6. TV-LQR: calculo de ganancias variantes en el tiempo
%  ========================================================================

% ── COMPLETAR: definir matrices de ponderacion ─────────────────────────────
Qk = diag([ ; ; ; ; ; ; ; ]);
Rk = *eye(nu);
% ──────────────────────────────────────────────────────────────────────────
Qf = Qk;

K_TV = zeros(nu, nx, N);
fprintf('Calculando K_TV (metodo: %s)...\n', riccati_method);

switch riccati_method

    case 'zoh'
        S_next = Qf;
        for k = N:-1:1
            A = Ak(:,:,k);   B = Bk(:,:,k);
            % ── COMPLETAR ──────────────────────────────────────────────────
            % Discretizar (A,B) por ZOH usando expm.
            % Calcular K_TV(:,:,k) y actualizar S_next (simetrizar).
            % ───────────────────────────────────────────────────────────────
        end

    case 'sqrt'
        TsR = Ts / riccati_jj;
        Pk  = zeros(nx, nx, N);
        Pk(:,:,N)   = sqrtm(0.05*Qf);
        Sk_N        = Pk(:,:,N)*Pk(:,:,N)';
        K_TV(:,:,N) = Rk \ (Bk(:,:,N)'*Sk_N);
        for k = N:-1:2
            P = Pk(:,:,k);   A = Ak(:,:,k);   B = Bk(:,:,k);
            % ── COMPLETAR ──────────────────────────────────────────────────
            % Integrar Ricc_sqrt con RK4 hacia atras (riccati_jj sub-pasos).
            % Actualizar Pk(:,:,k-1) y K_TV(:,:,k-1).
            % ───────────────────────────────────────────────────────────────
        end

    otherwise
        error('riccati_method invalido: ''%s''. Usar ''zoh'' o ''sqrt''.', riccati_method);
end

if ~all(isfinite(K_TV(:)))
    error('K_TV contiene NaN/Inf.');
end
fprintf('K_TV calculado. max||K_TV(:,:,k)|| = %.4f\n', ...
    max(arrayfun(@(k) norm(K_TV(:,:,k)), 1:N)));

%% ========================================================================
%  7. Simulacion del sistema no lineal con TV-LQR
%  ========================================================================

x0sim = x0 + 0.05*randn(nx,1);

[T, Xsim] = ode45( ...
    @(t,x) OM4dof(t, x, controlTVLQR(t, x, Uref, Xref, Ts, N, K_TV)), ...
    0:Ts:(N*Ts), x0sim);
Xsim = Xsim';

%% 8. Calculo de salidas y metricas de seguimiento TV-LQR

qRefPlot = [x0(1:4), Xref(1:4,:)];

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

joint_error = Xsim(1:4,:) - qRefPlot;

metrics_tvlqr = struct();
metrics_tvlqr.final_error_cart    = norm(ysim(:,end) - yf);
metrics_tvlqr.max_abs_utvlqr      = max(abs(U_tvlqr_sat), [], 2);
metrics_tvlqr.sat_percent         = 100*mean(abs(U_tvlqr_raw) >= (ukmax - 1e-8), 2);
metrics_tvlqr.max_abs_joint_error = max(abs(joint_error), [], 2);
metrics_tvlqr.rms_joint_error     = sqrt(mean(joint_error.^2, 2));

printMetricsTVLQR(metrics_tvlqr);

%% 9. Graficas

c_sim = [0.0000 0.4470 0.7410];

figure(3); clf;
set(gcf, 'Name', 'Seguimiento articular con TV-LQR', ...
         'Color', 'w', 'Position', [120 80 1100 650]);
tl1  = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
axs1 = gobjects(1, 4);
h_sim_q = []; h_ref_q = [];
for iq = 1:4
    axs1(iq) = nexttile(tl1);
    h1 = plot(T, Xsim(iq,:),    '-',  'Color', c_sim, 'LineWidth', lw); hold on;
    h2 = plot(T, qRefPlot(iq,:), '--', 'Color', c_ref, 'LineWidth', lw);
    if iq == 1, h_sim_q = h1; h_ref_q = h2; end
    ylabel(qLabels{iq}, 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on; set(gca, 'FontSize', fs); xlim([T(1), T(end)]);
    if iq < 4, set(gca, 'XTickLabel', []); else, xlabel('Tiempo [s]', 'FontSize', fs); end
end
lgd1 = legend(axs1(1), [h_sim_q, h_ref_q], {'Simulacion', 'Referencia'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd1.Box = 'on';
try, lgd1.Layout.Tile = 'north'; catch, end
title(tl1, 'Seguimiento articular con TV-LQR: simulacion vs referencia', ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');

figure(4); clf;
set(gcf, 'Name', 'Trayectoria cartesiana con TV-LQR', ...
         'Color', 'w', 'Position', [150 80 1050 720]);
hold on; grid on; box on;
h_mc = gobjects(1,1);
for rrsim = 1:20
    x0sim_mc = x0 + 0.1*randn(nx,1);
    [~, Xsim_mc] = ode45( ...
        @(t,x) OM4dof(t, x, controlTVLQR(t, x, Uref, Xref, Ts, N, K_TV)), ...
        0:Ts:(N*Ts), x0sim_mc);
    Xsim_mc = Xsim_mc';
    ysim_mc = zeros(4, size(Xsim_mc,2));
    for i = 1:size(Xsim_mc,2)
        ysim_mc(:,i) = open_manx_fkin(Xsim_mc(1:4,i));
    end
    if rrsim == 1
        h_mc = plot3(ysim_mc(1,:), ysim_mc(2,:), ysim_mc(3,:), ...
                     '-', 'Color', [0.55 0.70 1.00], 'LineWidth', 0.8);
    else
        plot3(ysim_mc(1,:), ysim_mc(2,:), ysim_mc(3,:), ...
              '-', 'Color', [0.55 0.70 1.00], 'LineWidth', 0.8);
    end
end
h_sim  = plot3(ysim(1,:), ysim(2,:), ysim(3,:), ...
               '-', 'Color', [0.0000 0.2000 0.8000], 'LineWidth', 2.0);
h_ref  = plot3(yref(1,:), yref(2,:), yref(3,:), ...
               '--o', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.4, 'MarkerSize', 4);
h_goal = plot3(yf(1), yf(2), yf(3), 'kx', 'MarkerSize', 12, 'LineWidth', 2.2);
view(45, 70);
xlabel('x [m]', 'FontSize', fs);
ylabel('y [m]', 'FontSize', fs);
zlabel('z [m]', 'FontSize', fs);
title('Trayectoria cartesiana del efector final', 'FontSize', fs_ttl, 'FontWeight', 'bold');
legend([h_mc, h_sim, h_ref, h_goal], ...
       {'Trayectorias simuladas', 'Simulacion evaluada', ...
        'Referencia optimizada', 'Punto final $y_f$'}, ...
       'Interpreter', 'latex', 'Location', 'northeastoutside');
set(gca, 'FontSize', fs);

figure(5); clf;
set(gcf, 'Name', 'Señal de control TV-LQR', ...
         'Color', 'w', 'Position', [170 110 1100 560]);
tl4  = tiledlayout(nu, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
axs4 = gobjects(1, nu);
h_uact = []; h_uref2 = [];
Uref_plot = zeros(nu, numel(T));
for i = 1:numel(T)
    k = min(max(floor(T(i)/Ts)+1, 1), N);
    Uref_plot(:,i) = Uref(:,k);
end
for iu = 1:nu
    axs4(iu) = nexttile(tl4);
    h1 = plot(T, U_tvlqr_sat(iu,:), '-',  'Color', c_sim, 'LineWidth', lw); hold on;
    h2 = plot(T, Uref_plot(iu,:),   '--', 'Color', c_ref, 'LineWidth', lw);
    if iu == 1, h_uact = h1; h_uref2 = h2; end
    ylabel(sprintf('$u_%d$ [N.m]', iu), 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on; set(gca, 'FontSize', fs); xlim([T(1), T(end)]);
    ydata  = [U_tvlqr_sat(iu,:), Uref_plot(iu,:)];
    yrange = max(max(ydata) - min(ydata), 0.05);
    ylim([min(ydata) - 0.15*yrange, max(ydata) + 0.15*yrange]);
    if iu < nu, set(gca, 'XTickLabel', []); else, xlabel('Tiempo [s]', 'FontSize', fs); end
end
lgd4 = legend(axs4(1), [h_uact, h_uref2], {'TV-LQR', 'Referencia'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd4.Box = 'on';
try, lgd4.Layout.Tile = 'north'; catch, end
title(tl4, 'Señal de control aplicada: TV-LQR vs referencia', ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');

%% ========================================================================
%  10. Exportacion de figuras
%  ========================================================================

if EXPORT_FIGS
    output_dir = fullfile(pkg_dir, 'plots', 'lab5', 'matlab', 'act1');
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end
    fig_names = {'act1_qref_optimo', 'act1_uref_optimo', ...
                 'act1_tvlqr_seguimiento_q', 'act1_tvlqr_trayectoria_cart', 'act1_tvlqr_control'};
    for fi = 1:5
        exportgraphics(figure(fi), fullfile(output_dir, [fig_names{fi} '.png']), 'Resolution', 300);
        exportgraphics(figure(fi), fullfile(output_dir, [fig_names{fi} '.eps']), ...
                       'ContentType', 'vector', 'Resolution', 600);
    end
    fprintf('\nGraficas guardadas en: %s\n', output_dir);
end

%% ========================================================================
%  Funciones locales
%  ========================================================================

function J = Jcosto(z, Ts, N, nx, nu, x0, yf) %#ok<INUSD>
% Funcion de costo para trajectory optimization.
% Penaliza: error cartesiano terminal, error cartesiano durante el horizonte,
% velocidades articulares, magnitud y variacion del torque.

    J = 0;

    % ── COMPLETAR ──────────────────────────────────────────────────────────
    % Definir matrices de peso Qy, Qf_cost, Qv, R.
    % Calcular y acumular los terminos del costo en J.
    % ───────────────────────────────────────────────────────────────────────
end

function [c_des, c_eq] = restr(z, Ts, N, nx, nu, x0)
% Restricciones de igualdad: dinamica discreta (RK4).
% Limites articulares y saturacion de torque manejados via lb/ub en fmincon.

    c_eq  = zeros(nx*N, 1);
    c_des = [];

    for k = 0:N-1
        if k == 0, xk = x0; else, xk = z(nx*(k-1) + (1:nx)); end
        uk   = z(nx*N + nu*k + (1:nu));
        xkp1 = z(nx*k + (1:nx));

        % ── COMPLETAR ──────────────────────────────────────────────────────
        % Integrar la dinamica con RK4 usando OM4dof.
        % Calcular c_eq(nx*k+(1:nx)) = xkp1 - x_next_RK4.
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

function printMetricsTrajOpt(metrics_trajopt)
    fprintf('\n============================================================\n');
    fprintf('Lab 5 Actividad 1 - Metricas de la trayectoria optimizada\n');
    fprintf('============================================================\n');
    fprintf('Exitflag fmincon                 : %g\n',   metrics_trajopt.exitflag);
    fprintf('Max |u_ref| por articulacion      : [%s] N.m\n', ...
            num2str(metrics_trajopt.max_abs_uref', ' %.4f'));
end

function printMetricsTVLQR(metrics_tvlqr)
    fprintf('\n============================================================\n');
    fprintf('Lab 5 Actividad 1 - Metricas de seguimiento TV-LQR\n');
    fprintf('============================================================\n');
    fprintf('Error cartesiano final ||y(tf)-yf||: %.6f\n',    metrics_tvlqr.final_error_cart);
    fprintf('Max |u_TVLQR| por articulacion    : [%s] N.m\n', ...
            num2str(metrics_tvlqr.max_abs_utvlqr', ' %.4f'));
    fprintf('Saturacion TV-LQR por articulacion: [%s] %%\n',  ...
            num2str(metrics_tvlqr.sat_percent', ' %.2f'));
    fprintf('Error maximo articular            : [%s] rad\n', ...
            num2str(metrics_tvlqr.max_abs_joint_error', ' %.5f'));
    fprintf('Error RMS articular               : [%s] rad\n', ...
            num2str(metrics_tvlqr.rms_joint_error', ' %.5f'));
end
