%% lab5_act2_sol.m
% Trajectory Optimization y TV-LQR — Laboratorio 5, Actividad 2
% Control de Sistemas No Lineales — OpenMANIPULATOR-X (4 GDL)
%
% Objetivo : Mismo x0 y yf que Act. 1, evitando el obstáculo MDF.
%
% Obstáculo MDF en Gazebo (sim_init_config.yaml):
%   pose: x=0.08 m, y=0, z=0, yaw=pi/2
%   Marco world (post-rotación yaw=pi/2):
%     X ∈ [0.005, 0.155] m   (ancho 150 mm, centrado en x_c=0.08)
%     Y ∈ [-0.160, 0.160] m  (largo 320 mm, centrado en y=0)
%     Z ∈ [0,     0.158] m   (altura techo 158 mm)
%
% Modelo de obstáculo — paraboloide con pendiente elevada:
%   z_obs(x,y) = max(0,  z_top - α_x·(x-x_c)² - α_y·(y-y_c)²)
%   donde:
%     z_top = 0.168 m  (techo + 10 mm margen de seguridad)
%     α_x   = z_top/rx²  ≈ 29.9 m⁻¹  (pendiente elevada en X)
%     α_y   = z_top/ry²  ≈  6.6 m⁻¹  (pendiente en Y)
%   El paraboloide vale cero exactamente en el borde del footprint y alcanza
%   z_top en el centro (x_c, y_c), modelando la cubierta del arco.
%
% Restricción de evasión (soft, penalización cuadrática en J):
%   violation_k = max(0,  z_obs(x_k, y_k) - z_k)
%   J_obs       = c_obs · Σ violation_k²
%
% Figuras:
%   Fig 1 — Trayectorias articulares optimizadas q_ref
%   Fig 2 — Entradas optimizadas u_ref
%   Fig 3 — Seguimiento articular TV-LQR (simulación vs referencia)
%   Fig 4 — Trayectoria cartesiana 3D + paraboloide obstáculo (meshgrid)
%   Fig 5 — Señal de control TV-LQR vs referencia
%
% Para usar en Gazebo: tras correr este script, actualizar Lab5_Export_Refs.m
%   con  zmin_file='zmin_act2.mat'  y  N=30.

clear; close all; clc;
rng(1);

EXPORT_FIGS = true;

pkg_dir = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';

riccati_method = 'sqrt';

%% ========================================================================
%  1. Parámetros generales  (iguales a Act. 1)
%  ========================================================================

N  = 30;    % 50 → 30: reduce variables de 600 a 360 (2.8× más rápido por iteración)
Ts = 0.05;
nx = 8;
nu = 4;

x0 = [pi/2; 0; pi/6; pi/3; 0; 0; 0; 0];
yf = [0.2; -0.13; 0.2; 0];

ukmax =  1.0;
ukmin = -1.0;

q_lower = [-pi/2; -pi/2; -pi/2; -1.7907];
q_upper = [ pi/2;  pi/2;  pi/2;  2.0420];
dq_max  = 10;

%% ========================================================================
%  2. Parámetros del obstáculo MDF
%  ========================================================================

x_obs  = 0.08;    % [m] centro X del obstáculo en marco world
y_obs  = 0.0;     % [m] centro Y
z_top  = 0.168;   % [m] altura pico paraboloide = z_techo(0.158) + 10 mm
rx_obs = 0.075;   % [m] semi-ancho en X (footprint = 150 mm)
ry_obs = 0.160;   % [m] semi-ancho en Y (footprint = 320 mm)

alpha_x = z_top / rx_obs^2;   % ≈ 29.87  pendiente elevada
alpha_y = z_top / ry_obs^2;   % ≈  6.56

c_obs   = 5e5;    % peso de penalización de obstáculo en J

% Handle para evaluación vectorial del paraboloide
z_obs_fn = @(x, y) max(0, z_top - alpha_x*(x - x_obs).^2 ...
                                  - alpha_y*(y - y_obs).^2);

fprintf('--- Obstáculo MDF ---\n');
fprintf('  Footprint world : X=[%.3f, %.3f]  Y=[%.3f, %.3f] m\n', ...
        x_obs-rx_obs, x_obs+rx_obs, -ry_obs, ry_obs);
fprintf('  z_top paraboloide = %.3f m  (techo 0.158 + 10 mm)\n', z_top);
fprintf('  alpha_x = %.2f m^-1   alpha_y = %.2f m^-1\n\n', alpha_x, alpha_y);

%% ========================================================================
%  3. Inicialización de la optimización
%  ========================================================================

[~, u0g] = OMDyn(x0(1:4), zeros(4,1));
u0g = u0g(:);
u0g = max(min(u0g, ukmax), ukmin);

% Solo torques como box bounds (ver comentario en lab5_act1_sol.m)
lb = [repmat(-inf(nx,1), N, 1);  ukmin*ones(nu*N, 1)];
ub = [repmat( inf(nx,1), N, 1);  ukmax*ones(nu*N, 1)];
x0_guess = x0;

% Warm start: usar solución Act.1 (N=50) resampleada a N=30 como punto inicial.
% Esto coloca el EE cerca de yf desde el principio, dando gradientes útiles.
ws_file = 'zmin6.mat';
if exist(ws_file, 'file')
    d_ws    = load(ws_file);
    X_ws50  = reshape(d_ws.zmin(1:8*50),       [8  50]);
    U_ws50  = reshape(d_ws.zmin(8*50+(1:4*50)), [4  50]);
    idx_rs  = round(linspace(1, 50, N));         % resampleo N=50 → N=30
    X_ws    = X_ws50(:, idx_rs);
    U_ws    = U_ws50(:, idx_rs);
    % Proyectar dentro de lb/ub
    X_ws    = X_ws;   % sin proyeccion: bounds de estado eliminados
    U_ws    = max(ukmin, min(ukmax, U_ws));
    z0      = [X_ws(:); U_ws(:)];
    fprintf('Warm start: %s resampleado N=50→%d\n\n', ws_file, N);
else
    z0 = [kron(ones(N,1), x0_guess); kron(ones(N,1), u0g)];
    fprintf('Warm start: punto constante en x0 (%s no encontrado)\n\n', ws_file);
end

options = optimoptions('fmincon', ...
    'Display',               'iter', ...
    'Algorithm',             'sqp', ...
    'MaxFunctionEvaluations', 100000, ...
    'MaxIterations',          500, ...
    'OptimalityTolerance',    1e-4, ...
    'ConstraintTolerance',    1e-4);

%% ========================================================================
%  4. Trajectory optimization
%  ========================================================================

use_saved_solution = false;
zmin_file = 'zmin_act2.mat';
exitflag  = NaN;
output    = struct();

if use_saved_solution && exist(zmin_file, 'file') == 2
    data = load(zmin_file);
    zmin = data.zmin;
    if isfield(data,'exitflag'), exitflag = data.exitflag; end
    if isfield(data,'output'),   output   = data.output;   end
    fprintf('Cargado: %s\n', zmin_file);
else
    [zmin, ~, exitflag, output] = fmincon( ...
        @(z) Jcosto(z, Ts, N, nx, nu, x0, yf, ...
                    x_obs, y_obs, z_top, alpha_x, alpha_y, c_obs), ...
        z0, [], [], [], [], lb, ub, ...
        @(z) restr(z, Ts, N, nx, nu, x0), ...
        options);
    save(zmin_file, 'zmin', 'exitflag', 'output');
    fprintf('Guardado: %s\n', zmin_file);
end

fprintf('\nExitflag fmincon: %g\n', exitflag);
if isfield(output,'message'), fprintf('Mensaje: %s\n\n', output.message); end

%% ========================================================================
%  5. Recuperar trayectorias
%  ========================================================================

Xref = reshape(zmin(1:nx*N),           [nx N]);
Uref = reshape(zmin(nx*N+(1:nu*N)),    [nu N]);

%% ========================================================================
%  6. Verificar evasión del obstáculo
%  ========================================================================

min_clearance = inf;
for k = 1:N
    yk   = open_manx_fkin(Xref(1:4,k));
    zobs = z_obs_fn(yk(1), yk(2));
    clr  = yk(3) - zobs;
    if clr < min_clearance, min_clearance = clr; end
end

fprintf('--- Verificación obstáculo ---\n');
if min_clearance >= 0
    fprintf('  OK — clearance mínimo = %.4f m\n\n', min_clearance);
else
    fprintf('  ADVERTENCIA: penetración máxima = %.4f m\n', -min_clearance);
    fprintf('  Considerar aumentar c_obs o MaxIterations.\n\n');
end

%% ========================================================================
%  7. Métricas optimización
%  ========================================================================

yref = zeros(4, N+1);
yref(:,1) = open_manx_fkin(x0(1:4));
for i = 1:N
    yref(:,i+1) = open_manx_fkin(Xref(1:4,i));
end

metricsB.exitflag     = exitflag;
metricsB.max_abs_uref = max(abs(Uref), [], 2);
metricsB.min_clearance = min_clearance;

fprintf('Max |u_ref| por articulación: [%s] N·m\n', ...
        num2str(metricsB.max_abs_uref', ' %.4f'));

%% ========================================================================
%  8. Linealización numérica variante en el tiempo
%  ========================================================================

Ak = zeros(nx, nx, N);
Bk = zeros(nx, nu, N);
eps_jac = 1e-6;
for k = 1:N
    xk = Xref(:,k); uk = Uref(:,k);
    for i = 1:nx
        dx = zeros(nx,1); dx(i) = eps_jac;
        Ak(:,i,k) = (OM4dof(0, xk+dx, uk) - OM4dof(0, xk-dx, uk)) / (2*eps_jac);
    end
    for i = 1:nu
        du = zeros(nu,1); du(i) = eps_jac;
        Bk(:,i,k) = (OM4dof(0, xk, uk+du) - OM4dof(0, xk, uk-du)) / (2*eps_jac);
    end
end

%% ========================================================================
%  9. TV-LQR — cálculo de ganancias variantes en el tiempo
%  ========================================================================

Qk = diag([100; 100; 100; 500;   1;  1;  1;  5]);
Rk = 100*eye(nu);
Qf = Qk;
K_TV = zeros(nu, nx, N);

switch riccati_method
    case 'zoh'
        S_next = Qf;
        for k = N:-1:1
            A  = Ak(:,:,k);  B  = Bk(:,:,k);
            Z  = expm([[A,B]; [zeros(nu,nx+nu)]] * Ts);
            Ad = Z(1:nx,1:nx);  Bd = Z(1:nx,nx+1:nx+nu);
            Mt          = Rk + Bd'*S_next*Bd;
            K_TV(:,:,k) = Mt \ (Bd'*S_next*Ad);
            Sc          = Qk + Ad'*S_next*(Ad - Bd*K_TV(:,:,k));
            S_next      = 0.5*(Sc + Sc');
        end
    otherwise
        error('Solo riccati_method=''zoh'' implementado en este script.');
end

assert(all(isfinite(K_TV(:))), 'K_TV contiene NaN/Inf.');
fprintf('K_TV calculado.\n');

%% ========================================================================
%  10. Simulación TV-LQR (sistema no lineal, perturbación inicial)
%  ========================================================================

x0sim = x0 + 0.05*randn(nx,1);
[T, Xsim] = ode45( ...
    @(t,x) OM4dof(t, x, controlTVLQR(t,x,Uref,Xref,Ts,N,K_TV)), ...
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

metricsD.final_error_cart    = norm(ysim(1:3,end) - yf(1:3));
metricsD.max_abs_utvlqr      = max(abs(U_tvlqr_sat), [], 2);
metricsD.sat_percent         = 100*mean(abs(U_tvlqr_raw) >= (ukmax-1e-8), 2);
metricsD.max_abs_joint_error = max(abs(joint_error), [], 2);
metricsD.rms_joint_error     = sqrt(mean(joint_error.^2, 2));

fprintf('\nError cartesiano final ||y(tf)-yf|| = %.6f m\n', metricsD.final_error_cart);
fprintf('Max |u_TVLQR|  : [%s] N·m\n', num2str(metricsD.max_abs_utvlqr', ' %.4f'));
fprintf('Error max artic: [%s] rad\n', num2str(metricsD.max_abs_joint_error', ' %.5f'));
fprintf('Error RMS artic: [%s] rad\n', num2str(metricsD.rms_joint_error', ' %.5f'));

%% ========================================================================
%  11. Estilo común
%  ========================================================================

lw       = 1.3;
fs       = 11;
fs_ttl   = 13;
c_ref    = [0.8500 0.3250 0.0980];
c_sim    = [0.0000 0.4470 0.7410];
c_obs_color = [0.80 0.50 0.20];
xlims    = [T(1), T(end)];
qLabels  = {'$q_1$ [rad]','$q_2$ [rad]','$q_3$ [rad]','$q_4$ [rad]'};
tX = Ts:Ts:N*Ts;
tU = 0:Ts:(N-1)*Ts;

%% ========================================================================
%  Fig 1 — Trayectorias articulares optimizadas
%  ========================================================================

figure(1); clf;
set(gcf,'Name','q_ref optimizado Act.2','Color','w','Position',[100 80 1100 650]);
tl1 = tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
for iq = 1:4
    nexttile(tl1);
    plot(tX, Xref(iq,:), '-', 'Color', c_ref, 'LineWidth', lw);
    ylabel(qLabels{iq},'Interpreter','latex','FontSize',fs);
    grid on; box on; set(gca,'FontSize',fs); xlim([tX(1),tX(end)]);
    if iq < 4, set(gca,'XTickLabel',[]); else, xlabel('Tiempo [s]','FontSize',fs); end
end
title(tl1,'Trayectorias articulares optimizadas $q_{ref}$ — Act. 2 (con obstáculo)', ...
      'Interpreter','latex','FontSize',fs_ttl,'FontWeight','bold');

%% ========================================================================
%  Fig 2 — Entradas optimizadas
%  ========================================================================

figure(2); clf;
set(gcf,'Name','u_ref optimizado Act.2','Color','w','Position',[120 100 1100 560]);
tl2 = tiledlayout(nu,1,'TileSpacing','compact','Padding','compact');
for iu = 1:nu
    nexttile(tl2);
    plot(tU, Uref(iu,:), '-', 'Color', c_ref, 'LineWidth', lw);
    ylabel(sprintf('$u_%d$ [N·m]',iu),'Interpreter','latex','FontSize',fs);
    grid on; box on; set(gca,'FontSize',fs); xlim([tU(1),tU(end)]);
    if iu < nu, set(gca,'XTickLabel',[]); else, xlabel('Tiempo [s]','FontSize',fs); end
end
title(tl2,'Entradas optimizadas $u_{ref}$ — Act. 2','Interpreter','latex', ...
      'FontSize',fs_ttl,'FontWeight','bold');

%% ========================================================================
%  Fig 3 — Seguimiento articular TV-LQR
%  ========================================================================

qRefPlot = [x0(1:4), Xref(1:4,:)];

figure(3); clf;
set(gcf,'Name','Seguimiento TV-LQR Act.2','Color','w','Position',[140 80 1100 650]);
tl3 = tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
h_s = []; h_r = [];
for iq = 1:4
    ax = nexttile(tl3);
    h1 = plot(T, Xsim(iq,:),    '-',  'Color',c_sim,'LineWidth',lw); hold on;
    h2 = plot(T, qRefPlot(iq,:),'--', 'Color',c_ref,'LineWidth',lw);
    if iq==1, h_s=h1; h_r=h2; end
    ylabel(qLabels{iq},'Interpreter','latex','FontSize',fs);
    grid on; box on; set(gca,'FontSize',fs); xlim(xlims);
    if iq < 4, set(gca,'XTickLabel',[]); else, xlabel('Tiempo [s]','FontSize',fs); end
end
legend(nexttile(tl3,1),[h_s,h_r],{'Simulación','Referencia'}, ...
       'Orientation','horizontal','FontSize',fs,'Location','northoutside');
title(tl3,'Seguimiento articular TV-LQR — Act. 2 (con obstáculo)', ...
      'FontSize',fs_ttl,'FontWeight','bold');

%% ========================================================================
%  Fig 4 — Trayectoria 3D + paraboloide obstáculo (meshgrid)
%  ========================================================================

figure(4); clf;
set(gcf,'Name','Trayectoria 3D + Obstáculo','Color','w','Position',[160 60 1100 780]);
hold on; grid on; box on;

% ── Paraboloide: meshgrid ─────────────────────────────────────────────────
xg = linspace(x_obs - rx_obs - 0.02, x_obs + rx_obs + 0.02, 100);
yg = linspace(-ry_obs - 0.03, ry_obs + 0.03, 100);
[Xg, Yg] = meshgrid(xg, yg);
Zg = z_obs_fn(Xg, Yg);
Zg(Zg <= 0) = NaN;   % solo dibujar dentro del footprint

h_obs_surf = surf(Xg, Yg, Zg, ...
    'FaceAlpha', 0.50, ...
    'EdgeColor', 'none', ...
    'FaceColor', c_obs_color);

% ── Contorno base del footprint en Z=0 (elipse paraboloide) ─────────────
th = linspace(0, 2*pi, 200);
xfp = x_obs + rx_obs*cos(th);
yfp = y_obs + ry_obs*sin(th);
plot3(xfp, yfp, zeros(size(th)), '--', 'Color',[0.5 0.3 0.1], 'LineWidth', 1.0);

% ── Caja del obstáculo físico (trazo de referencia) ─────────────────────
x_box = [x_obs-rx_obs, x_obs+rx_obs, x_obs+rx_obs, x_obs-rx_obs, x_obs-rx_obs];
y_box1 = [-ry_obs, -ry_obs, ry_obs, ry_obs, -ry_obs];
patch([x_box, fliplr(x_box)], [y_box1, fliplr(y_box1)], ...
      [repmat(z_top-0.01,1,5), repmat(z_top-0.01,1,5)], ...
      [0.6 0.45 0.25], 'FaceAlpha', 0.15, 'EdgeColor', [0.4 0.25 0.1], ...
      'LineStyle', ':');

% ── Monte Carlo (20 realizaciones) ───────────────────────────────────────
for rr = 1:20
    x0mc = x0 + 0.10*randn(nx,1);
    [~,Xmc] = ode45( ...
        @(t,x) OM4dof(t,x,controlTVLQR(t,x,Uref,Xref,Ts,N,K_TV)), ...
        0:Ts:N*Ts, x0mc);
    Xmc = Xmc';
    ymc = zeros(4, size(Xmc,2));
    for i = 1:size(Xmc,2), ymc(:,i) = open_manx_fkin(Xmc(1:4,i)); end
    if rr == 1
        h_mc = plot3(ymc(1,:),ymc(2,:),ymc(3,:), ...
                     '-','Color',[0.55 0.70 1.0],'LineWidth',0.8);
    else
        plot3(ymc(1,:),ymc(2,:),ymc(3,:),'-','Color',[0.55 0.70 1.0],'LineWidth',0.8);
    end
end

% ── Trayectorias principales ─────────────────────────────────────────────
h_sim  = plot3(ysim(1,:),  ysim(2,:),  ysim(3,:), ...
               '-',  'Color',[0.0 0.2 0.8],'LineWidth',2.2);
h_ref  = plot3(yref(1,:),  yref(2,:),  yref(3,:), ...
               '--o','Color',c_ref,'LineWidth',1.4,'MarkerSize',4);
h_goal = plot3(yf(1), yf(2), yf(3), 'kx','MarkerSize',13,'LineWidth',2.4);
h_x0   = plot3(yref(1,1), yref(2,1), yref(3,1), ...
               'gs','MarkerSize',9,'LineWidth',2.0,'MarkerFaceColor','g');

view(50, 22);
xlabel('$x$ [m]','Interpreter','latex','FontSize',fs);
ylabel('$y$ [m]','Interpreter','latex','FontSize',fs);
zlabel('$z$ [m]','Interpreter','latex','FontSize',fs);
legend([h_obs_surf, h_mc, h_sim, h_ref, h_goal, h_x0], ...
       {'Paraboloide obstáculo','MC perturbado','Simulación TV-LQR', ...
        'Referencia optimizada','$y_f$ (objetivo)','$y_0$ (inicio)'}, ...
       'Interpreter','latex','Location','northeastoutside','FontSize',fs-1);
title('Trayectoria cartesiana 3D con obstáculo MDF — TV-LQR Act. 2', ...
      'FontSize',fs_ttl,'FontWeight','bold');
set(gca,'FontSize',fs);

%% ========================================================================
%  Fig 5 — Señal de control TV-LQR
%  ========================================================================

Uref_plot = zeros(nu, numel(T));
for i = 1:numel(T)
    k_idx = min(max(floor(T(i)/Ts)+1,1),N);
    Uref_plot(:,i) = Uref(:,k_idx);
end

figure(5); clf;
set(gcf,'Name','Control TV-LQR Act.2','Color','w','Position',[180 110 1100 560]);
tl5 = tiledlayout(nu,1,'TileSpacing','compact','Padding','compact');
hu = []; hur = [];
for iu = 1:nu
    nexttile(tl5);
    h1 = plot(T, U_tvlqr_sat(iu,:), '-',  'Color',c_sim,'LineWidth',lw); hold on;
    h2 = plot(T, Uref_plot(iu,:),   '--', 'Color',c_ref,'LineWidth',lw);
    if iu==1, hu=h1; hur=h2; end
    ylabel(sprintf('$u_%d$ [N·m]',iu),'Interpreter','latex','FontSize',fs);
    ydata  = [U_tvlqr_sat(iu,:), Uref_plot(iu,:)];
    yrange = max(max(ydata)-min(ydata), 0.05);
    ylim([min(ydata)-0.15*yrange, max(ydata)+0.15*yrange]);
    grid on; box on; set(gca,'FontSize',fs); xlim(xlims);
    if iu < nu, set(gca,'XTickLabel',[]); else, xlabel('Tiempo [s]','FontSize',fs); end
end
legend(nexttile(tl5,1),[hu,hur],{'TV-LQR saturado','Referencia'}, ...
       'Orientation','horizontal','FontSize',fs,'Location','northoutside');
title(tl5,'Señal de control TV-LQR vs referencia — Act. 2', ...
      'FontSize',fs_ttl,'FontWeight','bold');

%% ========================================================================
%  12. Exportación de figuras
%  ========================================================================

if EXPORT_FIGS
    out_dir = fullfile(pkg_dir, 'plots', 'lab5', 'matlab', 'act2');
    if ~exist(out_dir,'dir'), mkdir(out_dir); end

    fig_names = {'qref_act2','uref_act2','tvlqr_seguimiento_act2', ...
                 'trayectoria_3d_act2','tvlqr_control_act2'};

    for fi = 1:5
        base = fullfile(out_dir, fig_names{fi});
        exportgraphics(figure(fi), [base '.png'], 'Resolution', 300);
        exportgraphics(figure(fi), [base '.eps'], ...
                       'ContentType','vector','Resolution',600);
    end
    fprintf('\nFiguras guardadas en: %s\n', out_dir);
end

%% ========================================================================
%  Funciones locales
%  ========================================================================

function J = Jcosto(z, Ts, N, nx, nu, x0, yf, ...
                    x_obs, y_obs, z_top, alpha_x, alpha_y, c_obs) %#ok<INUSD>

    Qv      = 0.1 * eye(4);
    Qy      = diag([1000; 1000; 1000; 10]);
    Qf_cost = diag([1000; 1000; 1000; 10]);
    R       = 0.01 * eye(nu);

    J  = 0;
    y0 = open_manx_fkin(x0(1:4));
    xN = z(nx*(N-1) + (1:nx));
    yN = open_manx_fkin(xN(1:4));

    J = J + (yN - yf)' * Qf_cost * (yN - yf);

    for k = 1:N
        xk    = z(nx*(k-1) + (1:nx));
        yk    = open_manx_fkin(xk(1:4));
        yrefk = y0 + (yf - y0)*(k/N);
        dqk   = xk(5:8);
        J = J + dqk'*Qv*dqk + (yk - yrefk)'*Qy*(yk - yrefk);

        % Penalización obstáculo — smooth max(0,x) para gradientes continuos.
        % Aproximación: (x + sqrt(x²+ε²) - ε) / 2  →  max(0,x) cuando ε→0
        % Diferenciable en toda ℝ, sin esquinas que frenen el gradiente.
        eps_sm = 1e-3;   % suavizado [m] (~1 mm zona de transición)
        zobs   = max(0, z_top - alpha_x*(yk(1)-x_obs)^2 - alpha_y*(yk(2)-y_obs)^2);
        x_arg  = zobs - yk(3);
        viol   = (x_arg + sqrt(x_arg^2 + eps_sm^2) - eps_sm) / 2;
        J = J + c_obs * viol^2;
    end

    for k = 0:N-2
        uk   = z(nx*N + nu*k     + (1:nu));
        ukp1 = z(nx*N + nu*(k+1) + (1:nu));
        J = J + uk'*R*uk + (uk - ukp1)'*R*(uk - ukp1);
    end
    uk = z(nx*N + nu*(N-1) + (1:nu));
    J  = J + uk'*R*uk;
end

% ─────────────────────────────────────────────────────────────────────────

function [c_des, c_eq] = restr(z, Ts, N, nx, nu, x0)

    c_eq  = zeros(nx*N, 1);
    c_des = [];

    for k = 0:N-1
        if k == 0, xk = x0; else, xk = z(nx*(k-1) + (1:nx)); end
        uk   = z(nx*N + nu*k + (1:nu));
        xkp1 = z(nx*k + (1:nx));

        k1 = OM4dof(k*Ts,        xk,            uk);
        k2 = OM4dof(k*Ts + Ts/2, xk + Ts*k1/2, uk);
        k3 = OM4dof(k*Ts + Ts/2, xk + Ts*k2/2, uk);
        k4 = OM4dof(k*Ts + Ts,   xk + Ts*k3,   uk);

        c_eq(nx*k + (1:nx)) = xkp1 - (xk + Ts*(k1 + 2*k2 + 2*k3 + k4)/6);
    end
end

% ─────────────────────────────────────────────────────────────────────────

function dx = OM4dof(t, x, u) %#ok<INUSD>

    q  = x(1:4);
    dq = x(5:8);
    u  = max(min(u(:), 1), -1);

    [M, phib] = OMDyn(q, dq);
    phib = phib(:);
    bf   = 0.001;

    ddq = M \ (u - phib - bf*dq);
    dx  = [dq; ddq];
end

% ─────────────────────────────────────────────────────────────────────────

function u = controlTVLQR(t, x, Uref, Xref, Ts, N, K_TV)

    k = min(max(floor(t/Ts) + 1, 1), N);
    u = Uref(:,k) - K_TV(:,:,k) * (x - Xref(:,k));
end
