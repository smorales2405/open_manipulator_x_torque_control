%% lab5_act1_sol.m
% Trajectory Optimization y TV-LQR — Laboratorio 5, Actividad 1
% Control de Sistemas No Lineales — OpenMANIPULATOR-X de 4 GDL
%
% Trayectoria: x0 = [pi/2; 0; pi/6; pi/3; 0; 0; 0; 0]
%              yf = [0.2; -0.13; 0.2; 0]  (sin obstaculo)
% N = 40, Ts = 0.1 s  (tf = 4.0 s)
%
% Figuras generadas:
%   Figura 1 — Trayectorias articulares optimizadas q_ref
%   Figura 2 — Entradas optimizadas u_ref
%   Figura 3 — Seguimiento articular con TV-LQR (simulacion vs referencia)
%   Figura 4 — Trayectoria cartesiana del efector final (Monte Carlo)
%   Figura 5 — Señal de control TV-LQR vs referencia

clear; close all; clc;
rng(1);

%% ========================================================================
%  Configuracion
%  ========================================================================

EXPORT_FIGS = false;   % true  = guardar PNG (300 dpi) y EPS vectorial (600 dpi)
                       % false = solo visualizar

% ── Identificadores de sesion (editar antes de cada ejecucion) ────────────
act_num            = 1;     % numero de actividad
trial_num          = 4;     % numero de prueba — nombra el log y el zmin
use_saved_solution = true; % true → carga N, Ts, x0, yf y zmin desde el .mat

pkg_dir    = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';
matlab_dir = fullfile(pkg_dir, 'src', 'Trajectory Optimization TV-LQR', 'MATLAB');
log_dir    = fullfile(matlab_dir, 'logs');
zmin_dir   = fullfile(matlab_dir, 'zmin');
if ~exist(log_dir,  'dir'), mkdir(log_dir);  end
if ~exist(zmin_dir, 'dir'), mkdir(zmin_dir); end

zmin_file = fullfile(zmin_dir, sprintf('zmin_act%d_%d.mat', act_num, trial_num));
log_file  = fullfile(log_dir,  sprintf('lab5_act%d_log%d.txt', act_num, trial_num));
if exist(log_file, 'file'), delete(log_file); end
diary(log_file); diary on;
fprintf('=== lab5_act1_sol  %s ===\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

% ── Metodo de Riccati para ganancias TV-LQR ──────────────────────────────
% 'zoh'  — Discreto ZOH exacto via expm (recomendado para Ts >= 0.02 s).
%           Garantiza S > 0 algebraicamente sin integrar un ODE continuo.
% 'sqrt' — Continuo forma sqrt (S = P*P'), integrado con RK4 hacia atras.
%           Puede singularizarse (P -> 0) con Ts grande; aumentar riccati_jj.
riccati_method = 'zoh';
riccati_jj     = 100;   % sub-pasos RK4 por intervalo Ts (solo 'sqrt')

% ── Aceleracion de la optimizacion ────────────────────────────────────────
% USE_PARALLEL: fmincon evalua las diferencias finitas del gradiente en
%   paralelo (requiere Parallel Computing Toolbox; el primer parpool de la
%   sesion tarda ~10-30 s en abrir).
% MEX: ejecutar build_omdyn_mex.m una vez — genera OMDyn.<mexext> y
%   open_manx_fkin.<mexext>, que tienen precedencia sobre los .m y aceleran
%   cada iteracion ~5-10x. Ambas mejoras se combinan.
USE_PARALLEL = true;

%% ========================================================================
%  1. Parametros generales
%  ========================================================================

N  = 80;
Ts = 0.05;
nx = 8;
nu = 4;

x0 = [pi/2; 0; pi/6; pi/3; 0; 0; 0; 0];      % Estado inicial (q,dq)
yf = [0.27; -0.11; 0.2; 0];               % Salida deseada (posición y orientación)

% ── Carga desde .mat si use_saved_solution = true ────────────────────────
% Sobreescribe N, Ts, x0, yf (y carga zmin/exitflag/output) desde el archivo.
% Los valores definidos arriba sirven solo de documentacion en ese caso.
if use_saved_solution
    if ~exist(zmin_file, 'file')
        error('use_saved_solution=true pero no existe: %s', zmin_file);
    end
    sv       = load(zmin_file, 'zmin', 'exitflag', 'output', 'N', 'Ts', 'x0', 'yf');
    zmin     = sv.zmin;    exitflag = sv.exitflag;    output = sv.output;
    N  = sv.N;    Ts = sv.Ts;
    x0 = sv.x0;  yf = sv.yf;
    fprintf('Cargado: %s  (N=%d  Ts=%.3f s)\n\n', zmin_file, N, Ts);
end

ukmax =  1;
ukmin = -1;

% Limites articulares del OpenManipulator-X [rad]
q_lower = [-3/4*pi; -11/18*pi; -11/18*pi;  -5/9*pi];
q_upper = [ 3/4*pi;   5/9*pi;     pi/2; 23/36*pi];
dq_max  = 10;  % [rad/s]

%% ========================================================================
%  2. Inicializacion de la optimizacion
%  ========================================================================

[~, u0g] = OMDyn(x0(1:4), x0(5:8));
u0g = u0g(:);
u0g = max(min(u0g, ukmax), ukmin);

% Bounds: posicion articular [q_lower, q_upper] + velocidad ±dq_max + torque ±ukmax
x_lower = [q_lower;             -dq_max * ones(4,1)];
x_upper = [q_upper;              dq_max * ones(4,1)];
lb = [repmat(x_lower, N, 1);  ukmin * ones(nu*N, 1)];
ub = [repmat(x_upper, N, 1);  ukmax * ones(nu*N, 1)];

z0 = [kron(ones(N,1), x0); kron(ones(N,1), u0g)];

% ── Aceleracion: estado del MEX + pool paralelo ──────────────────────────
if endsWith(which('OMDyn'), mexext)
    fprintf('OMDyn: MEX compilado — %s\n', which('OMDyn'));
else
    fprintf(['OMDyn: version .m interpretada. Ejecutar build_omdyn_mex.m ' ...
             'para acelerar ~5-10x cada iteracion.\n']);
end
if USE_PARALLEL && isempty(ver('parallel'))
    fprintf('Parallel Computing Toolbox no instalado — continuando en serie.\n');
    USE_PARALLEL = false;
end
if USE_PARALLEL
    try
        if isempty(gcp('nocreate')), parpool; end
    catch pool_err
        warning('Sin pool paralelo (%s). Continuando en serie.', pool_err.message);
        USE_PARALLEL = false;
    end
end

% Algoritmo SQP: resuelve el KKT directamente sin barrier functions.
% Para N=40 (480 variables, 320 igualdades) converge en 50-150 iter vs 200+ en interior-point.
% UseParallel reparte las ~N*(nx+nu)+1 evaluaciones de diferencias finitas
% de cada iteracion entre los workers del pool.
options = optimoptions('fmincon', ...
    'Display',               'iter', ...
    'Algorithm',             'sqp', ...
    'UseParallel',           USE_PARALLEL, ...
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

% Figura 1 — Trayectorias articulares optimizadas
figure(1); clf;
set(gcf, 'Name', 'Trayectorias articulares optimizadas', ...
         'Color', 'w', 'Position', [120 80 1100 650]);

tlB1 = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for iq = 1:4
    nexttile(tlB1);
    plot(tX, Xref(iq,:), '-', 'Color', c_ref, 'LineWidth', lw);
    ylabel(qLabels{iq}, 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on; set(gca, 'FontSize', fs); xlim([tX(1), tX(end)]);
    if iq < 4
        set(gca, 'XTickLabel', []);
    else
        xlabel('Tiempo [s]', 'FontSize', fs);
    end
end
title(tlB1, 'Trayectorias articulares optimizadas $q_{ref}$', ...
      'Interpreter', 'latex', 'FontSize', fs_ttl, 'FontWeight', 'bold');

% Figura 2 — Entradas optimizadas
figure(2); clf;
set(gcf, 'Name', 'Entradas optimizadas', ...
         'Color', 'w', 'Position', [160 100 1100 560]);

tlB2 = tiledlayout(nu, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for iu = 1:nu
    nexttile(tlB2);
    plot(tU, Uref(iu,:), '-', 'Color', c_ref, 'LineWidth', lw);
    ylabel(sprintf('$u_%d$ [N.m]', iu), 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on; set(gca, 'FontSize', fs); xlim([tU(1), tU(end)]);
    if iu < nu
        set(gca, 'XTickLabel', []);
    else
        xlabel('Tiempo [s]', 'FontSize', fs);
    end
end
title(tlB2, 'Entradas optimizadas $u_{ref}$', ...
      'Interpreter', 'latex', 'FontSize', fs_ttl, 'FontWeight', 'bold');

%% 4.1 Métricas de la trayectoria optimizada

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
        Ak(:,i,k) = (OM4dof(0, xrefk + incx, urefk) - ...
                     OM4dof(0, xrefk - incx, urefk)) / (2*eps_jac);
    end

    for i = 1:nu
        incu = zeros(nu,1); incu(i) = eps_jac;
        Bk(:,i,k) = (OM4dof(0, xrefk, urefk + incu) - ...
                     OM4dof(0, xrefk, urefk - incu)) / (2*eps_jac);
    end
end

%% ========================================================================
%  6. TV-LQR: calculo de ganancias variantes en el tiempo
%  ========================================================================

Qk = diag([400; 400; 1200; 10000; 1; 1; 2; 10]);
Rk = diag([1; 1; 1; 0.2]);
Qf = Qk;

K_TV = zeros(nu, nx, N);
fprintf('Calculando K_TV (metodo: %s)...\n', riccati_method);

switch riccati_method

    % ── ZOH exacto (recomendado) ─────────────────────────────────────────
    case 'zoh'
        S_next = Qf;      % condicion terminal S_{N+1} = Qf
        for k = N:-1:1
            A  = Ak(:,:,k);   B  = Bk(:,:,k);
            Z  = expm([[A, B]; [zeros(nu, nx+nu)]] * Ts);
            Ad = Z(1:nx, 1:nx);   Bd = Z(1:nx, nx+1:nx+nu);
            Mterm       = Rk + Bd'*S_next*Bd;
            K_TV(:,:,k) = Mterm \ (Bd'*S_next*Ad);
            S_cur       = Qk + Ad'*S_next*(Ad - Bd*K_TV(:,:,k));
            S_next      = 0.5*(S_cur + S_cur');
        end

    % ── Sqrt form (ODE continuo, S = P*P', RK4 hacia atras) ──────────────
    case 'sqrt'
        TsR = Ts / riccati_jj;
        Pk  = zeros(nx, nx, N);
        Pk(:,:,N)   = sqrtm(0.05*Qf);     % condicion terminal (idem Lab5_Sol.m)
        Sk_N        = Pk(:,:,N)*Pk(:,:,N)';
        K_TV(:,:,N) = Rk \ (Bk(:,:,N)'*Sk_N);
        for k = N:-1:2
            P = Pk(:,:,k);   A = Ak(:,:,k);   B = Bk(:,:,k);
            for JJ = 1:riccati_jj
                k1 = Ricc_sqrt(P,            A, B, Qk, Rk);
                k2 = Ricc_sqrt(P + k1*TsR/2, A, B, Qk, Rk);
                k3 = Ricc_sqrt(P + k2*TsR/2, A, B, Qk, Rk);
                k4 = Ricc_sqrt(P + k3*TsR,   A, B, Qk, Rk);
                P  = P + TsR*(k1 + 2*k2 + 2*k3 + k4)/6;
            end
            Pk(:,:,k-1)   = P;
            K_TV(:,:,k-1) = Rk \ (B'*(P*P'));
        end

    otherwise
        error('riccati_method invalido: ''%s''. Usar ''zoh'' o ''sqrt''.', ...
              riccati_method);
end

if ~all(isfinite(K_TV(:)))
    error('K_TV contiene NaN/Inf — revisar linealizacion y metodo Riccati (''%s'').', ...
          riccati_method);
end

% ── Piso de rigidez para J4 (hardware) ────────────────────────────────────
% La Riccati asigna a J4 una ganancia de posicion de apenas ~0.4 N·m/rad
% (su inercia en el modelo es minima, B44 enorme), pero en el robot real la
% stiction + offset de corriente (~0.08 N·m) dejan un error de equilibrio de
% ~0.2 rad. Se impone una rigidez minima estilo FL (el Lab 4 valido la
% muneca a ~3.3 N·m/rad; el presupuesto de torque sobra: TAU_MAX=1.2).
for k = 1:N
    K_TV(4,4,k) = max(K_TV(4,4,k), 1.5);    % posicion  [N·m/rad]
    K_TV(4,8,k) = max(K_TV(4,8,k), 0.08);   % velocidad [N·m/(rad/s)]
end

fprintf('K_TV calculado. max||K_TV(:,:,k)|| = %.4f\n', ...
    max(arrayfun(@(k) norm(K_TV(:,:,k)), 1:N)));

%% ========================================================================
%  7. Simulacion del sistema no lineal con TV-LQR
%  ========================================================================

x0sim = x0 + 0.05*randn(nx,1);

% Control en lazo cerrado con saturacion explicita (OM4dof ya no satura
% internamente — ver nota en OM4dof).
u_cl = @(t,x) max(min(controlTVLQR(t, x, Uref, Xref, Ts, N, K_TV), ukmax), ukmin);

[T, Xsim] = ode45(@(t,x) OM4dof(t, x, u_cl(t,x)), 0:Ts:(N*Ts), x0sim);
Xsim = Xsim';

%% 8. Calculo de salidas y métricas de seguimiento TV-LQR

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

c_sim  = [0.0000 0.4470 0.7410];

% Figura 3 — Seguimiento articular con TV-LQR
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
    if iq < 4
        set(gca, 'XTickLabel', []);
    else
        xlabel('Tiempo [s]', 'FontSize', fs);
    end
end

lgd1 = legend(axs1(1), [h_sim_q, h_ref_q], ...
              {'Simulacion', 'Referencia'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd1.Box = 'on';
try, lgd1.Layout.Tile = 'north'; catch, end

title(tl1, 'Seguimiento articular con TV-LQR: simulacion vs referencia', ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');

% Figura 4 — Trayectoria cartesiana (sin obstaculo)
figure(4); clf;
set(gcf, 'Name', 'Trayectoria cartesiana con TV-LQR', ...
         'Color', 'w', 'Position', [150 80 1050 720]);
hold on; grid on; box on;

h_mc = gobjects(1,1);
for rrsim = 1:20
    x0sim_mc = x0 + 0.1*randn(nx,1);
    [~, Xsim_mc] = ode45(@(t,x) OM4dof(t, x, u_cl(t,x)), 0:Ts:(N*Ts), x0sim_mc);
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

h_sim  = plot3(ysim(1,:),  ysim(2,:),  ysim(3,:), ...
               '-',  'Color', [0.0000 0.2000 0.8000], 'LineWidth', 2.0);
h_ref  = plot3(yref(1,:),  yref(2,:),  yref(3,:), ...
               '--o','Color', [0.8500 0.3250 0.0980], ...
               'LineWidth', 1.4, 'MarkerSize', 4);
h_goal = plot3(yf(1), yf(2), yf(3), ...
               'kx', 'MarkerSize', 12, 'LineWidth', 2.2);

view(45, 70);
xlabel('x [m]', 'FontSize', fs);
ylabel('y [m]', 'FontSize', fs);
zlabel('z [m]', 'FontSize', fs);
title('Trayectoria cartesiana del efector final', ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');
legend([h_mc, h_sim, h_ref, h_goal], ...
       {'Trayectorias simuladas', 'Simulacion evaluada', ...
        'Referencia optimizada',  'Punto final $y_f$'}, ...
       'Interpreter', 'latex', 'Location', 'northeastoutside');
set(gca, 'FontSize', fs);

% Figura 5 — Señal de control TV-LQR vs referencia
figure(5); clf;
set(gcf, 'Name', 'Señal de control TV-LQR', ...
         'Color', 'w', 'Position', [170 110 1100 560]);

tl4  = tiledlayout(nu, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
axs4 = gobjects(1, nu);
h_uact = []; h_uref2 = [];

Uref_plot = zeros(nu, numel(T));
for i = 1:numel(T)
    k = floor(T(i)/Ts) + 1;
    k = min(max(k,1), N);
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
    if iu < nu
        set(gca, 'XTickLabel', []);
    else
        xlabel('Tiempo [s]', 'FontSize', fs);
    end
end

lgd4 = legend(axs4(1), [h_uact, h_uref2], ...
              {'TV-LQR', 'Referencia'}, ...
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

    % PNG (300 dpi)
    for fi = 1:5
        exportgraphics(figure(fi), fullfile(output_dir, [fig_names{fi} '.png']), ...
                       'Resolution', 300);
    end

    % EPS vectorial (600 dpi)
    for fi = 1:5
        exportgraphics(figure(fi), fullfile(output_dir, [fig_names{fi} '.eps']), ...
                       'ContentType', 'vector', 'Resolution', 600);
    end

    fprintf('\nGraficas guardadas en: %s\n', output_dir);
end

diary off;

%% ========================================================================
%  Funciones locales
%  ========================================================================

function J = Jcosto(z, Ts, N, nx, nu, x0, yf) %#ok<INUSD>

    Qv = 0.1 * eye(4);                      % Penalizacion sobre velocidades
    Qy = diag([1000; 1000; 1000; 10]);       % Penalizacion sobre trayectoria cartesiana
    Qf_cost = diag([1000; 1000; 1000; 10]);  % Penalizacion sobre error terminal
    R  = 0.01*eye(nu);                       % Penalizacion sobre entradas

    J  = 0;
    xN = z(nx*(N-1) + (1:nx));
    y0 = open_manx_fkin(x0(1:4));
    yN = open_manx_fkin(xN(1:4));

    % Error terminal en espacio cartesiano + llegada en reposo.
    % Sin el termino de velocidad terminal el optimo llega a yf "en
    % movimiento" (dq_ref(end) hasta ~0.6 rad/s): en hw eso deja la
    % compensacion de Coulomb (tanh(dq_ref)) empujando durante el hold
    % final y produce sobrepaso en la llegada.
    J = J + (yN - yf)'*Qf_cost*(yN - yf) + 100 * (xN(5:8)'*xN(5:8));

    % Error cartesiano intermedio + penalizacion de velocidades
    for k = 1:N
        xk    = z(nx*(k-1) + (1:nx));
        yk    = open_manx_fkin(xk(1:4));
        yrefk = y0 + (yf - y0)*(k/N);
        dqk   = xk(5:8);
        J = J + dqk'*Qv*dqk + (yk - yrefk)'*Qy*(yk - yrefk);
    end

    % Magnitud y suavidad de las entradas
    for k = 0:N-2
        uk   = z(nx*N + nu*k     + (1:nu));
        ukp1 = z(nx*N + nu*(k+1) + (1:nu));
        J = J + uk'*R*uk + (uk - ukp1)'*R*(uk - ukp1);
    end

    uk = z(nx*N + nu*(N-1) + (1:nu));
    J  = J + uk'*R*uk;
end

function [c_des, c_eq] = restr(z, Ts, N, nx, nu, x0)
% Restricciones de igualdad: dinamica discreta (RK4).
% Limites articulares y saturacion de torque manejados via lb/ub en fmincon.

    c_eq  = zeros(nx*N, 1);
    c_des = [];

    for k = 0:N-1
        if k == 0
            xk = x0;
        else
            xk = z(nx*(k-1) + (1:nx));
        end
        uk   = z(nx*N + nu*k + (1:nu));
        xkp1 = z(nx*k + (1:nx));

        k1 = OM4dof(k*Ts,        xk,            uk);
        k2 = OM4dof(k*Ts + Ts/2, xk + Ts*k1/2, uk);
        k3 = OM4dof(k*Ts + Ts/2, xk + Ts*k2/2, uk);
        k4 = OM4dof(k*Ts + Ts,   xk + Ts*k3,   uk);

        x_next_RK4 = xk + Ts*(k1 + 2*k2 + 2*k3 + k4)/6;
        c_eq(nx*k + (1:nx)) = xkp1 - x_next_RK4;
    end
end

function dx = OM4dof(t, x, u) %#ok<INUSD>
% Modelo no lineal SIN saturacion interna de u:
%   - En la optimizacion los bounds lb/ub ya imponen |u| <= 1. Un clamp aqui
%     anularia el gradiente por diferencias finitas justo en u = ±1
%     (columnas cero en B_k) y estanca el paso SQP con entradas saturadas.
%   - En la simulacion en lazo cerrado la saturacion se aplica explicitamente
%     sobre la salida de controlTVLQR (handle u_cl, seccion 7).

    q  = x(1:4);
    dq = x(5:8);
    u  = u(:);

    [M, phib] = OMDyn(q, dq);
    phib = phib(:);

    Bdyn     = eye(4);
    bf       = 0.001;
    tau_fric = bf*dq;

    ddq = M \ (Bdyn*u - phib - tau_fric);
    dx  = [dq; ddq];
end

function u = controlTVLQR(t, x, Uref, Xref, Ts, N, K_TV)

    k = floor(t/Ts) + 1;
    k = min(max(k, 1), N);
    u = Uref(:,k) - K_TV(:,:,k)*(x - Xref(:,k));
end

function dP = Ricc_sqrt(P, A, B, Q, R)
% Riccati forma sqrt integrado hacia atras; S = P*P'.
% dP/dtau = A'P - (1/2)*P*P'*B*R^{-1}*B'*P + (1/2)*Q*P'^{-1}
    dP = A'*P - 0.5*P*P'*B*(R \ (B'*P)) + 0.5*(Q/P');
end

function stop = logTimingFcn(~, optimValues, state)
% Registra tiempo por iteracion en el diary. Llamada por fmincon OutputFcn.
    persistent t_prev t_start
    stop = false;
    switch state
        case 'init'
            t_start = tic;
            t_prev  = tic;
            fprintf('[TIME] Optimizacion iniciada: %s\n', ...
                    datestr(now, 'yyyy-mm-dd HH:MM:SS'));
        case 'iter'
            dt      = toc(t_prev);
            elapsed = toc(t_start);
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
    fprintf('Lab 5 Actividad 1 - Métricas de la trayectoria optimizada\n');
    fprintf('============================================================\n');
    fprintf('Exitflag fmincon                 : %g\n',   metrics_trajopt.exitflag);
    fprintf('Max |u_ref| por articulacion      : [%s] N.m\n', ...
            num2str(metrics_trajopt.max_abs_uref', ' %.4f'));
end

function printMetricsTVLQR(metrics_tvlqr)
    fprintf('\n============================================================\n');
    fprintf('Lab 5 Actividad 1 - Métricas de seguimiento TV-LQR\n');
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
