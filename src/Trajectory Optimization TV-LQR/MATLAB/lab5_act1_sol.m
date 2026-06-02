%% lab5_act1_sol.m
% Trajectory Optimization y TV-LQR — Laboratorio 5, Actividad 1
% Control de Sistemas No Lineales — OpenMANIPULATOR-X de 4 GDL
%
% Trayectoria: x0 = [pi/2; 0; pi/6; pi/3; 0; 0; 0; 0]
%              yf = [0.2; -0.13; 0.2; 0]  (sin obstaculo)
% N = 30, Ts = 0.05 s  (tf = 1.5 s)
%
% Figuras generadas:
%   Figura 1 — Trayectorias articulares optimizadas q_ref
%   Figura 2 — Entradas optimizadas u_ref
%   Figura 3 — Seguimiento articular con TV-LQR (simulacion vs referencia)
%   Figura 4 — Trayectoria cartesiana del efector final (Monte Carlo)
%   Figura 5 — Senal de control TV-LQR vs referencia

clear; close all; clc;
rng(1);

%% ========================================================================
%  Configuracion
%  ========================================================================

EXPORT_FIGS = false;   % true  = guardar PNG (300 dpi) y EPS vectorial (600 dpi)
                       % false = solo visualizar

% Directorio raiz del paquete ROS 2
pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

% ── Metodo de Riccati para ganancias TV-LQR ──────────────────────────────
% 'zoh'  — Discreto ZOH exacto via expm (recomendado para Ts >= 0.02 s).
%           Garantiza S > 0 algebraicamente sin integrar un ODE continuo.
% 'std'  — Continuo estandar sobre S, integrado con RK4 hacia atras.
%           Puede perder definitividad positiva si ||A||*Ts es grande.
% 'sqrt' — Continuo forma sqrt (S = P*P'), integrado con RK4 hacia atras.
%           Puede singularizarse (P -> 0) con Ts grande; aumentar riccati_jj.
riccati_method = 'zoh';
riccati_jj     = 100;   % sub-pasos RK4 por intervalo Ts (solo 'std' y 'sqrt')

%% ========================================================================
%  1. Parametros generales
%  ========================================================================

N  = 30;
Ts = 0.05;
nx = 8;
nu = 4;

x0 = [pi/2; 0.9158; 0.6565; -1.6751; 0; 0; 0; 0];
yf = [0.2860; 0.0; 0.2045; 0];

ukmax =  1;
ukmin = -1;

% Limites articulares del OpenManipulator-X [rad]
q_lower = [-pi/2; -pi/2; -pi/2; -1.7907];
q_upper = [ pi/2;  pi/2;  pi/2;  2.0420];
dq_max  = 10;  % [rad/s]

%% ========================================================================
%  2. Inicializacion de la optimizacion
%  ========================================================================

[~, u0g] = OMDyn(x0(1:4), x0(5:8));
u0g = u0g(:);
u0g = max(min(u0g, ukmax), ukmin);

% Bounds: SOLO torques como box bounds (lb/ub eficiente para constraints lineales).
% Limites articulares y velocidad ELIMINADOS de lb/ub: su interaccion con las
% restricciones de igualdad (dinamica RK4) crea KKT mal condicionado en
% interior-point, forzando pasos pequenos y cientos de iteraciones extra.
% El optimizador respeta limites articulares indirectamente via la funcion de costo.
lb = [repmat(-inf(nx,1), N, 1);  ukmin*ones(nu*N, 1)];
ub = [repmat( inf(nx,1), N, 1);  ukmax*ones(nu*N, 1)];

z0 = [kron(ones(N,1), x0); kron(ones(N,1), u0g)];

% Algoritmo SQP: resuelve el KKT directamente sin barrier functions.
% Para N=30 (360 variables, 240 igualdades) converge en 50-150 iter vs 200+ en interior-point.
options = optimoptions('fmincon', ...
    'Display',               'iter', ...
    'Algorithm',             'sqp', ...
    'MaxFunctionEvaluations', 100000, ...
    'MaxIterations',          500, ...
    'OptimalityTolerance',    1e-4, ...
    'ConstraintTolerance',    1e-4);

%% ========================================================================
%  3. Trajectory optimization
%  ========================================================================

use_saved_solution = false;  % false = regenerar con limites articulares
zmin_file = 'zmin6.mat';
exitflag = NaN;
output = struct();

if use_saved_solution && exist(zmin_file, 'file') == 2
    data = load(zmin_file);
    zmin = data.zmin;
    if isfield(data, 'exitflag'), exitflag = data.exitflag; end
    if isfield(data, 'output'),   output   = data.output;   end
    fprintf('Se cargo %s correctamente.\n', zmin_file);
else
    [zmin, ~, exitflag, output] = fmincon( ...
        @(z) Jcosto(z, Ts, N, nx, nu, x0, yf), ...
        z0, [], [], [], [], lb, ub, ...
        @(z) restr(z, Ts, N, nx, nu, x0), ...
        options);

    save(zmin_file, 'zmin', 'exitflag', 'output');
    fprintf('Se guardo la trayectoria optimizada en %s.\n', zmin_file);
end

fprintf('\nExitflag fmincon: %g\n', exitflag);
if isfield(output, 'message')
    fprintf('Mensaje fmincon: %s\n', output.message);
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

%% 4.1 Metricas - Parte B

yref = zeros(4, N+1);
yref(:,1) = open_manx_fkin(x0(1:4));
for i = 1:N
    yref(:,i+1) = open_manx_fkin(Xref(1:4,i));
end

metricsB = struct();
metricsB.exitflag     = exitflag;
metricsB.max_abs_uref = max(abs(Uref), [], 2);

printMetricsParteB(metricsB, 'Lab 5 Actividad 1 - Parte B');

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

Qk = diag([100; 100; 100; 100; 1; 1; 1; 1]);
Rk = 100*eye(nu);
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

    % ── Estandar sobre S (ODE continuo, RK4 hacia atras) ─────────────────
    case 'std'
        TsR    = Ts / riccati_jj;
        S_next = Qf;      % condicion terminal S_{N+1} = Qf
        for k = N:-1:1
            A = Ak(:,:,k);   B = Bk(:,:,k);   S = S_next;
            for JJ = 1:riccati_jj
                f1 = Ricc_std(S,            A, B, Qk, Rk);
                f2 = Ricc_std(S + f1*TsR/2, A, B, Qk, Rk);
                f3 = Ricc_std(S + f2*TsR/2, A, B, Qk, Rk);
                f4 = Ricc_std(S + f3*TsR,   A, B, Qk, Rk);
                S  = S + TsR*(f1 + 2*f2 + 2*f3 + f4)/6;
                S  = 0.5*(S + S');
            end
            S_next      = S;
            K_TV(:,:,k) = Rk \ (B'*S);
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
        error('riccati_method invalido: ''%s''. Usar ''zoh'', ''std'' o ''sqrt''.', ...
              riccati_method);
end

if ~all(isfinite(K_TV(:)))
    error('K_TV contiene NaN/Inf — revisar linealizacion y metodo Riccati (''%s'').', ...
          riccati_method);
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

%% 8. Calculo de salidas y metricas - Parte D

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

metricsD = struct();
metricsD.final_error_cart    = norm(ysim(:,end) - yf);
metricsD.max_abs_utvlqr      = max(abs(U_tvlqr_sat), [], 2);
metricsD.sat_percent         = 100*mean(abs(U_tvlqr_raw) >= (ukmax - 1e-8), 2);
metricsD.max_abs_joint_error = max(abs(joint_error), [], 2);
metricsD.rms_joint_error     = sqrt(mean(joint_error.^2, 2));

printMetricsParteD(metricsD, 'Lab 5 Actividad 1 - Parte D');

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

% Figura 5 — Senal de control TV-LQR vs referencia
figure(5); clf;
set(gcf, 'Name', 'Senal de control TV-LQR', ...
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
              {'TV-LQR saturado', 'Referencia'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, 'Location', 'northoutside');
lgd4.Box = 'on';
try, lgd4.Layout.Tile = 'north'; catch, end

title(tl4, 'Senal de control aplicada: TV-LQR vs referencia', ...
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

    % Error terminal en espacio cartesiano
    J = J + (yN - yf)'*Qf_cost*(yN - yf);

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

    q  = x(1:4);
    dq = x(5:8);
    u  = max(min(u(:), 1), -1);

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

function dS = Ricc_std(S, A, B, Q, R)
% Riccati estandar continuo integrado hacia atras (tau = tf - t).
% dS/dtau = A'S + SA - S*B*R^{-1}*B'*S + Q
    dS = A'*S + S*A - S*B*(R \ (B'*S)) + Q;
end

function dP = Ricc_sqrt(P, A, B, Q, R)
% Riccati forma sqrt integrado hacia atras; S = P*P'.
% dP/dtau = A'P - (1/2)*P*P'*B*R^{-1}*B'*P + (1/2)*Q*P'^{-1}
    dP = A'*P - 0.5*P*P'*B*(R \ (B'*P)) + 0.5*(Q/P');
end

function printMetricsParteB(metricsB, label)

    fprintf('\n============================================================\n');
    fprintf('Metricas - %s\n', label);
    fprintf('============================================================\n');
    fprintf('Exitflag fmincon                 : %g\n',   metricsB.exitflag);
    fprintf('Max |u_ref| por articulacion      : [%s] N.m\n', ...
            num2str(metricsB.max_abs_uref', ' %.4f'));
end

function printMetricsParteD(metricsD, label)

    fprintf('\n============================================================\n');
    fprintf('Metricas - %s\n', label);
    fprintf('============================================================\n');
    fprintf('Error cartesiano final ||y(tf)-yf||: %.6f\n',    metricsD.final_error_cart);
    fprintf('Max |u_TVLQR| por articulacion    : [%s] N.m\n', ...
            num2str(metricsD.max_abs_utvlqr', ' %.4f'));
    fprintf('Saturacion TV-LQR por articulacion: [%s] %%\n',  ...
            num2str(metricsD.sat_percent', ' %.2f'));
    fprintf('Error maximo articular            : [%s] rad\n', ...
            num2str(metricsD.max_abs_joint_error', ' %.5f'));
    fprintf('Error RMS articular               : [%s] rad\n', ...
            num2str(metricsD.rms_joint_error', ' %.5f'));
end
