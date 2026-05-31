%% PreLab 5 - Solucion comentada y alineada a la guia 2026-1
% Control de Sistemas No Lineales
% Tema: Trajectory Optimization y TV-LQR para OpenManipulator-X de 4 GDL
%
% Cambios incorporados:
%   - Uso de open_manx_fkin(q) para cinematica directa.
%   - Restricciones dinamicas por transcripcion directa con RK4.
%   - Uso de operadores \ y / en lugar de inv(.).
%   - Vectores y arreglos preasignados con zeros.
%   - Figuras con fondo blanco y leyenda general superior en subplots.
%   - Metricas separadas segun la guia: Parte B (optimizacion) y
%     Parte D (seguimiento TV-LQR).

clear; close all; clc;
rng(1);

%% ========================================================================
%  1. Parametros generales definidos en la guia
%  ========================================================================

N  = 200;      % Numero de nodos/intervalos de control
Ts = 0.05;    % Tiempo de muestreo [s]
nx = 8;       % Estados: x = [q1 q2 q3 q4 dq1 dq2 dq3 dq4]^T
nu = 4;       % Entradas: u = [tau1 tau2 tau3 tau4]^T

x0 = [pi/2; 0; pi/6; pi/3; 0; 0; 0; 0];
yf = [0.16; 0.08; 0.10; pi/2];

ukmax =  1;
ukmin = -1;

%% ========================================================================
%  2. Inicializacion de la optimizacion
%  ========================================================================

% Compensacion gravitatoria aproximada para inicializar los torques.
% Para dq = 0, phib contiene principalmente el aporte gravitacional.
[~, u0g] = OMDyn(x0(1:4), x0(5:8));
u0g = u0g(:);
u0g = max(min(u0g, ukmax), ukmin);

% Vector de decision inicial: z = [xvec; uvec].
z0 = [kron(ones(N,1), x0); ...
      kron(ones(N,1), u0g)];

options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'MaxFunctionEvaluations', 40000, ...
    'MaxIterations', 1000, ...
    'OptimalityTolerance', 1e-4, ...
    'ConstraintTolerance', 1e-4);

%% ========================================================================
%  3. Trajectory optimization
%  ========================================================================

use_saved_solution = true;      % Cambiar a true para reutilizar zmin.mat
zmin_file = 'zmin.mat';
exitflag = NaN;
output = struct();

if use_saved_solution && exist(zmin_file, 'file') == 2
    data = load(zmin_file);
    zmin = data.zmin;
    if isfield(data, 'exitflag')
        exitflag = data.exitflag;
    end
    if isfield(data, 'output')
        output = data.output;
    end
    fprintf('Se cargo %s correctamente.\n', zmin_file);
else
    [zmin, ~, exitflag, output] = fmincon( ...
        @(z) Jcosto(z, Ts, N, nx, nu, x0, yf), ...
        z0, [], [], [], [], [], [], ...
        @(z) restr(z, Ts, N, nx, nu, x0, yf, ukmax, ukmin), ...
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

% Figura 1. Trayectorias articulares optimizadas
figure(1); clf;
set(gcf, 'Name', 'Trayectorias articulares optimizadas', ...
         'Color', 'w', 'Position', [120 80 1100 650]);

qLabels = {'$q_1$ [rad]', '$q_2$ [rad]', '$q_3$ [rad]', '$q_4$ [rad]'};
fs = 11;
fs_ttl = 13;
lw = 1.3;
c_ref = [0.8500 0.3250 0.0980];

tlB1 = tiledlayout(4,1, 'TileSpacing','compact', 'Padding','compact');
axsB1 = gobjects(1,4);

for iq = 1:4
    axsB1(iq) = nexttile(tlB1);
    plot(tX, Xref(iq,:), '-', 'Color', c_ref, 'LineWidth', lw);
    ylabel(qLabels{iq}, 'Interpreter','latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim([tX(1), tX(end)]);
    xticks(0:0.25:tX(end));

    if iq < 4
        set(gca, 'XTickLabel', []);
    else
        xlabel('Tiempo [s]', 'FontSize', fs);
    end
end

title(tlB1, 'Trayectorias articulares optimizadas $q_{ref}$', ...
      'Interpreter','latex', 'FontSize', fs_ttl, 'FontWeight','bold');


% Figura 2. Entradas optimizadas
figure(2); clf;
set(gcf, 'Name', 'Entradas optimizadas', ...
         'Color', 'w', 'Position', [160 100 1100 560]);

tlB2 = tiledlayout(nu,1, 'TileSpacing','compact', 'Padding','compact');
axsB2 = gobjects(1,nu);

for iu = 1:nu
    axsB2(iu) = nexttile(tlB2);
    plot(tU, Uref(iu,:), '-', 'Color', c_ref, 'LineWidth', lw);
    ylabel(sprintf('$u_%d$ [N.m]', iu), 'Interpreter','latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim([tU(1), tU(end)]);
    xticks(0:0.25:tU(end));

    if iu < nu
        set(gca, 'XTickLabel', []);
    else
        xlabel('Tiempo [s]', 'FontSize', fs);
    end
end

title(tlB2, 'Entradas optimizadas $u_{ref}$', ...
      'Interpreter','latex', 'FontSize', fs_ttl, 'FontWeight','bold');

%% 4.1 Metricas solicitadas - Parte B: trayectoria optimizada

obs_center = [0.10; 0.11];
obs_radius = 0.04;

% Trayectoria cartesiana de referencia optimizada.
yref = zeros(4, N+1);
yref(:,1) = open_manx_fkin(x0(1:4));
for i = 1:N
    yref(:,i+1) = open_manx_fkin(Xref(1:4,i));
end

dist_xy_ref = sqrt((yref(1,:) - obs_center(1)).^2 + ...
                   (yref(2,:) - obs_center(2)).^2);

metricsB = struct();
metricsB.exitflag        = exitflag;
metricsB.max_abs_uref    = max(abs(Uref), [], 2);
metricsB.min_dist_xy_ref = min(dist_xy_ref);

printMetricsParteB(metricsB, 'Pre-Laboratorio 5 - Parte B');


%% ========================================================================
%  5. Linealizacion numerica variante en el tiempo
%  ========================================================================
%  Se usa diferencia central, segun la formulacion de la guia.

Ak = zeros(nx, nx, N);
Bk = zeros(nx, nu, N);

eps_jac = 1e-6;

for k = 1:N
    xrefk = Xref(:,k);
    urefk = Uref(:,k);

    for i = 1:nx
        incx = zeros(nx,1);
        incx(i) = eps_jac;
        Ak(:,i,k) = (OM4dof(0, xrefk + incx, urefk) - ...
                     OM4dof(0, xrefk - incx, urefk))/(2*eps_jac);
    end

    for i = 1:nu
        incu = zeros(nu,1);
        incu(i) = eps_jac;
        Bk(:,i,k) = (OM4dof(0, xrefk, urefk + incu) - ...
                     OM4dof(0, xrefk, urefk - incu))/(2*eps_jac);
    end
end

fprintf('\nDimension de A1: %d x %d\n', size(Ak(:,:,1),1), size(Ak(:,:,1),2));
fprintf('Dimension de B1: %d x %d\n', size(Bk(:,:,1),1), size(Bk(:,:,1),2));
disp('A1 = '); disp(Ak(:,:,1));
disp('B1 = '); disp(Bk(:,:,1));

%% ========================================================================
%  6. TV-LQR: calculo de ganancias variantes en el tiempo
%  ========================================================================

Qk = diag([100; 100; 100; 100; 1; 1; 1; 1]);
Rk = 40*eye(nu);
Qf = Qk;

Pk   = zeros(nx, nx, N);
Sk   = zeros(nx, nx, N);
K_TV = zeros(nu, nx, N);

Pk(:,:,N) = sqrtm(0.05*Qf);
Sk(:,:,N) = Pk(:,:,N)*Pk(:,:,N)';
K_TV(:,:,N) = Rk \ (Bk(:,:,N)'*Sk(:,:,N));

for k = N:-1:2
    P = Pk(:,:,k);
    A = Ak(:,:,k);
    B = Bk(:,:,k);
    Q = Qk;
    R = Rk;

    jj = 100;
    TsR = Ts/jj;

    for JJ = 1:jj
        k1 = Ricc_sqrt(0, P,            A, B, Q, R);
        k2 = Ricc_sqrt(0, P + k1*TsR/2, A, B, Q, R);
        k3 = Ricc_sqrt(0, P + k2*TsR/2, A, B, Q, R);
        k4 = Ricc_sqrt(0, P + k3*TsR,   A, B, Q, R);
        P = P + TsR*(k1 + 2*k2 + 2*k3 + k4)/6;
    end

    Pk(:,:,k-1) = P;
    Sk(:,:,k-1) = P*P';
    K_TV(:,:,k-1) = R \ (B'*Sk(:,:,k-1));
end

%% ========================================================================
%  7. Simulacion del sistema no lineal con TV-LQR
%  ========================================================================

x0sim = x0 + 0.05*randn(nx,1);

[T, Xsim] = ode45( ...
    @(t,x) OM4dof(t, x, controlTVLQR(t, x, Uref, Xref, Ts, N, K_TV)), ...
    0:Ts:(N*Ts), x0sim);
Xsim = Xsim';

%% 8. Calculo de salidas y metricas solicitadas - Parte D

qRefPlot = [x0(1:4), Xref(1:4,:)];

% Trayectoria cartesiana de la simulacion principal.
ysim = zeros(4, numel(T));
for i = 1:numel(T)
    ysim(:,i) = open_manx_fkin(Xsim(1:4,i));
end

% Senal de control aplicada por TV-LQR evaluada en los instantes de ode45.
U_tvlqr_raw = zeros(nu, numel(T));
U_tvlqr_sat = zeros(nu, numel(T));
for i = 1:numel(T)
    U_tvlqr_raw(:,i) = controlTVLQR(T(i), Xsim(:,i), Uref, Xref, Ts, N, K_TV);
    U_tvlqr_sat(:,i) = max(min(U_tvlqr_raw(:,i), ukmax), ukmin);
end

% Metricas solicitadas para el seguimiento con TV-LQR.
dist_xy_sim = sqrt((ysim(1,:) - obs_center(1)).^2 + ...
                   (ysim(2,:) - obs_center(2)).^2);

joint_error = Xsim(1:4,:) - qRefPlot;

metricsD = struct();
metricsD.final_error_cart     = norm(ysim(:,end) - yf);
metricsD.max_abs_utvlqr       = max(abs(U_tvlqr_sat), [], 2);
metricsD.sat_percent          = 100*mean(abs(U_tvlqr_raw) >= (ukmax - 1e-8), 2);
metricsD.max_abs_joint_error  = max(abs(joint_error), [], 2);
metricsD.rms_joint_error      = sqrt(mean(joint_error.^2, 2));
metricsD.min_dist_xy_sim      = min(dist_xy_sim);

printMetricsParteD(metricsD, 'Pre-Laboratorio 5 - Parte D');

%% 9. Graficas

% Figura 3 - Seguimiento articular con TV-LQR

figure(3); clf;
set(gcf, 'Name', 'Seguimiento articular con TV-LQR', ...
         'Color', 'w', 'Position', [120 80 1100 650]);

qLabels = {'$q_1$ [rad]', '$q_2$ [rad]', '$q_3$ [rad]', '$q_4$ [rad]'};

fs     = 11;
fs_ttl = 13;
lw     = 1.3;
c_sim  = [0.0000 0.4470 0.7410];
c_ref  = [0.8500 0.3250 0.0980];

tl1  = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
axs1 = gobjects(1, 4);
h_sim_q = [];
h_ref_q = [];

for iq = 1:4
    axs1(iq) = nexttile(tl1);

    h1 = plot(T, Xsim(iq,:), '-',  'Color', c_sim, 'LineWidth', lw); hold on;
    h2 = plot(T, qRefPlot(iq,:), '--', 'Color', c_ref, 'LineWidth', lw);

    if iq == 1
        h_sim_q = h1;
        h_ref_q = h2;
    end

    ylabel(qLabels{iq}, 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim([T(1), T(end)]);
    xticks(T(1):0.25:T(end));

    if iq < 4
        set(gca, 'XTickLabel', []);
    else
        xlabel('Tiempo [s]', 'FontSize', fs);
    end
end

lgd1 = legend(axs1(1), [h_sim_q, h_ref_q], ...
              {'Simulacion', 'Referencia'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, ...
              'Location', 'northoutside');
lgd1.Box = 'on';
try
    lgd1.Layout.Tile = 'north';
catch
   
end

title(tl1, 'Seguimiento articular con TV-LQR: simulacion vs referencia', ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');


% Figura 4 - Trayectoria cartesiana, punto final y obstaculo

figure(4); clf;
set(gcf, 'Name', 'Trayectoria cartesiana con TV-LQR', ...
         'Color', 'w', 'Position', [150 80 1050 720]);
hold on; grid on; box on;

% Trayectorias simuladas adicionales 
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

h_sim = plot3(ysim(1,:), ysim(2,:), ysim(3,:), ...
              '-', 'Color', [0.0000 0.2000 0.8000], 'LineWidth', 2.0);
h_ref = plot3(yref(1,:), yref(2,:), yref(3,:), ...
              '--o', 'Color', [0.8500 0.3250 0.0980], ...
              'LineWidth', 1.4, 'MarkerSize', 4);
h_goal = plot3(yf(1), yf(2), yf(3), ...
               'kx', 'MarkerSize', 12, 'LineWidth', 2.2);

% Obstaculo cilindrico vertical.
[Xcil, Ycil, Zcil] = cylinder(obs_radius, 40);
Xcil = Xcil + obs_center(1);
Ycil = Ycil + obs_center(2);
Zcil = Zcil*0.20;
h_obs = surf(Xcil, Ycil, Zcil, ...
             'EdgeColor', [0.20 0.20 0.20], ...
             'FaceAlpha', 0.35);

x_span = 0.30;
y_span = 0.30;
z_lims = [-0.05 0.30];
xlim(obs_center(1) + 0.5*x_span*[-1 1]);
ylim(obs_center(2) + 0.5*y_span*[-1 1]);
zlim(z_lims);
view(45, 70);
xlabel('x [m]', 'FontSize', fs);
ylabel('y [m]', 'FontSize', fs);
zlabel('z [m]', 'FontSize', fs);
title('Trayectoria cartesiana del efector final', ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');
lgd2 = legend([h_mc, h_sim, h_ref, h_goal, h_obs], ...
       {'Trayectorias simuladas', 'Simulacion evaluada', ...
        'Referencia optimizada', 'Punto final $y_f$', 'Obstaculo'}, ...
       'Interpreter', 'latex', 'Location', 'northeastoutside');
lgd2.Box = 'on';
set(gca, 'FontSize', fs);

% Figura 5 - Senal de control TV-LQR vs referencia

figure(5); clf;
set(gcf, 'Name', 'Senal de control TV-LQR', ...
         'Color', 'w', 'Position', [170 110 1100 560]);

tl4  = tiledlayout(nu, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
axs4 = gobjects(1, nu);
h_uact = [];
h_uref2 = [];

Uref_plot = zeros(nu, numel(T));
for i = 1:numel(T)
    k = floor(T(i)/Ts) + 1;
    k = min(max(k,1), N);
    Uref_plot(:,i) = Uref(:,k);
end

for iu = 1:nu
    axs4(iu) = nexttile(tl4);
    h1 = plot(T, U_tvlqr_sat(iu,:), '-', 'Color', c_sim, 'LineWidth', lw); hold on;
    h2 = plot(T, Uref_plot(iu,:), '--', 'Color', c_ref, 'LineWidth', lw);

    if iu == 1
        h_uact = h1;
        h_uref2 = h2;
    end

    ylabel(sprintf('$u_%d$ [N.m]', iu), 'Interpreter', 'latex', 'FontSize', fs);
    grid on; box on;
    set(gca, 'FontSize', fs);
    xlim([T(1), T(end)]);
    xticks(T(1):0.25:T(end));

    ydata = [U_tvlqr_sat(iu,:), Uref_plot(iu,:)];
    ymin_data = min(ydata);
    ymax_data = max(ydata);
    yrange = max(ymax_data - ymin_data, 0.05);
    ylim([ymin_data - 0.15*yrange, ymax_data + 0.15*yrange]);

    if iu < nu
        set(gca, 'XTickLabel', []);
    else
        xlabel('Tiempo [s]', 'FontSize', fs);
    end
end

lgd4 = legend(axs4(1), [h_uact, h_uref2], ...
              {'TV-LQR saturado', 'Referencia'}, ...
              'Orientation', 'horizontal', 'FontSize', fs, ...
              'Location', 'northoutside');
lgd4.Box = 'on';
try
    lgd4.Layout.Tile = 'north';
catch
end

title(tl4, 'Senal de control aplicada: TV-LQR vs referencia', ...
      'FontSize', fs_ttl, 'FontWeight', 'bold');


%% ========================================================================
%  Funciones locales
%  ========================================================================

function J = Jcosto(z, Ts, N, nx, nu, x0, yf)
%JCOSTO Funcion de costo para trajectory optimization.
%
% Penaliza:
%   1) Error cartesiano terminal.
%   2) Error cartesiano durante el horizonte.
%   3) Velocidades articulares.
%   4) Magnitud del torque.
%   5) Variacion entre torques consecutivos.

    J = 0;

    Qy = diag([100; 100; 100; 100]);   % Peso sobre y = [x; y; z; phi]
    R  = 0.1*eye(nu);                  % Peso sobre torque

    xN = z(nx*(N-1) + (1:nx));

    y0 = open_manx_fkin(x0(1:4));
    yN = open_manx_fkin(xN(1:4));

    % Error terminal en espacio cartesiano.
    J = J + 10*N*(yN - yf)'*Qy*(yN - yf);

    % Error cartesiano a lo largo del horizonte y penalizacion de velocidad.
    for k = 1:N
        xk = z(nx*(k-1) + (1:nx));
        yk = open_manx_fkin(xk(1:4));

        % Referencia cartesiana lineal entre la salida inicial y yf.
        yrefk = y0 + (yf - y0)*(k/N);

        J = J + (yk - yrefk)'*Qy*(yk - yrefk) ...
              + 0.1*xk(5:8)'*xk(5:8);
    end

    % Magnitud y suavidad de las entradas.
    for k = 0:N-2
        uk   = z(nx*N + nu*k     + (1:nu));
        ukp1 = z(nx*N + nu*(k+1) + (1:nu));

        J = J + uk'*R*uk + 10*(uk - ukp1)'*R*(uk - ukp1);
    end

    % Penalizacion del ultimo torque.
    uk = z(nx*N + nu*(N-1) + (1:nu));
    J = J + uk'*R*uk;
end

function [c_des, c_eq] = restr(z, Ts, N, nx, nu, x0, yf, ukmax, ukmin) %#ok<INUSD>
%RESTR Restricciones no lineales para fmincon.
%
% MATLAB/fmincon usa:
%   c_des(z) <= 0
%   c_eq(z)  = 0
%
% Restricciones implementadas:
%   1) Dinamica discreta con RK4.
%   2) Saturacion de torque ukmin <= u_k <= ukmax.
%   3) Evasion de cilindro vertical en el plano XY.

    obs_center = [0.10; 0.11];
    obs_radius = 0.04;

    % Preasignacion para evitar crecimiento dinamico de arreglos.
    c_eq  = zeros(nx*N, 1);
    c_des = zeros(2*nu*N + N, 1);

    % Igualdades: dinamica discreta por transcripcion directa con RK4.
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

        idx_eq = nx*k + (1:nx);
        c_eq(idx_eq) = xkp1 - x_next_RK4;
    end

    % Desigualdades: torque y obstaculo.
    idx = 1;
    for k = 0:N-1
        uk = z(nx*N + nu*k + (1:nu));

        % uk <= ukmax y uk >= ukmin.
        c_des(idx:idx+nu-1) = uk - ukmax;
        idx = idx + nu;
        c_des(idx:idx+nu-1) = ukmin - uk;
        idx = idx + nu;

        % Obstaculo cilindrico vertical:
        %   distancia_xy >= obs_radius
        % En formato c(z) <= 0:
        %   obs_radius^2 - distancia_xy^2 <= 0
        xk = z(nx*k + (1:nx));
        yk = open_manx_fkin(xk(1:4));

        dist2_xy = (yk(1) - obs_center(1))^2 + ...
                   (yk(2) - obs_center(2))^2;

        c_des(idx) = obs_radius^2 - dist2_xy;
        idx = idx + 1;
    end
end

function dx = OM4dof(t, x, u) %#ok<INUSD>
%OM4DOF Modelo no lineal de espacio de estados del OpenManipulator-X.
%
% Estado:
%   x = [q; dq]
%
% Dinamica:
%   M(q)*ddq + Phi(q,dq) = Bdyn*u - tau_fric(dq)
%   tau_fric = bf*dq

    q  = x(1:4);
    dq = x(5:8);

    % Saturacion fisica de torque usada durante la simulacion.
    u = max(min(u(:), 1), -1);

    [M, phib] = OMDyn(q, dq);
    phib = phib(:);

    Bdyn = eye(4);
    bf = 0.001;
    tau_fric = bf*dq;

    ddq = M \ (Bdyn*u - phib - tau_fric);
    dx = [dq; ddq];
end

function u = controlTVLQR(t, x, Uref, Xref, Ts, N, K_TV)
%CONTROLTVLQR Ley de control TV-LQR.
%
%   u(t) = u_ref,k - K_k*(x(t) - x_ref,k)

    k = floor(t/Ts) + 1;
    k = min(max(k, 1), N);

    u = Uref(:,k) - K_TV(:,:,k)*(x - Xref(:,k));
end

function dP = Ricc_sqrt(t, P, A, B, Q, R) %#ok<INUSD>
%RICC_SQRT Ecuacion diferencial de Riccati en forma sqrt.
%
%   dP = A'P - 1/2*P*P'*B*R^{-1}*B'*P + 1/2*Q*P^{-T}
%
% Se evita inv(.) usando operadores de division matricial.

    dP = A'*P - 0.5*P*P'*B*(R \ (B'*P)) + 0.5*(Q/P');
end

function printMetricsParteB(metricsB, label)

    fprintf('\n============================================================\n');
    fprintf('Metricas - %s\n', label);
    fprintf('============================================================\n');
    fprintf('Exitflag fmincon                 : %g\n', metricsB.exitflag);
    fprintf('Max |u_ref| por articulacion      : [%s] N.m\n', num2str(metricsB.max_abs_uref', ' %.4f'));
    fprintf('Distancia minima al eje cilindro  : %.6f m\n', metricsB.min_dist_xy_ref);
end

function printMetricsParteD(metricsD, label)

    fprintf('\n============================================================\n');
    fprintf('Metricas - %s\n', label);
    fprintf('============================================================\n');
    fprintf('Error cartesiano final ||y(tf)-yf||: %.6f\n', metricsD.final_error_cart);
    fprintf('Max |u_TVLQR| por articulacion    : [%s] N.m\n', num2str(metricsD.max_abs_utvlqr', ' %.4f'));
    fprintf('Saturacion TV-LQR por articulacion: [%s] %%\n', num2str(metricsD.sat_percent', ' %.2f'));
    fprintf('Error maximo articular            : [%s] rad\n', num2str(metricsD.max_abs_joint_error', ' %.5f'));
    fprintf('Error RMS articular               : [%s] rad\n', num2str(metricsD.rms_joint_error', ' %.5f'));
    fprintf('Distancia minima al eje cilindro  : %.6f m\n', metricsD.min_dist_xy_sim);
end
