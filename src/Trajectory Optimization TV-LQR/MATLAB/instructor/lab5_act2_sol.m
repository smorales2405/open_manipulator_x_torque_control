%% lab5_act2_sol.m
% Trajectory Optimization y TV-LQR — Laboratorio 5, Actividad 2
% Control de Sistemas No Lineales — OpenMANIPULATOR-X (4 GDL)
%
% Objetivo : Mismo x0 y yf que Act. 1, evitando el obstáculo MDF.
%
% Obstáculo MDF en Gazebo (sim_init_config.yaml):
%   pose: x=0.075 m, y=0, z=0, yaw=pi/2
%   Marco link1 —usado por open_manx_fkin— (offset base ≈ 114.75 mm respecto a Gazebo):
%     X ∈ [0.11475, 0.26475] m   (ancho 150 mm, centrado en x_c=0.18975)
%     Y ∈ [-0.160, 0.160] m  (largo 320 mm, centrado en y=0)
%     Z ∈ [0,     0.158] m   (altura techo 158 mm)
%
% Modelo de obstáculo — paraboloide con pendiente elevada:
%   z_obs(x,y) = max(0,  z_top - α_x·(x-x_c)² - α_y·(y-y_c)²)
%   donde:
%     z_top = 0.183 m  (techo 158 mm + 10 mm base robot + 15 mm espesor eslabón)
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
% Para usar en Gazebo: tras correr este script (deja N, Ts, nx, nu, x0, yf,
%   Xref, Uref, K_TV en el workspace), ejecutar Lab5_Export_Refs.m para
%   regenerar la carpeta references/.

clear; close all; clc;
rng(1);

EXPORT_FIGS = true;

% ── Identificadores de sesión (editar antes de cada ejecución) ───────────────
act_num            = 2;     % número de actividad
trial_num          = 16;    % número de prueba — nombra el log y el zmin
use_saved_solution = false; % true → carga N, Ts, x0, yf y zmin desde el .mat

pkg_dir    = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';
matlab_dir = fullfile(pkg_dir, 'src', 'Trajectory Optimization TV-LQR', 'MATLAB');
log_dir    = fullfile(matlab_dir, 'logs');
zmin_dir   = fullfile(matlab_dir, 'zmin');
if ~exist(log_dir,  'dir'), mkdir(log_dir);  end
if ~exist(zmin_dir, 'dir'), mkdir(zmin_dir); end

zmin_file = fullfile(zmin_dir, sprintf('zmin_act%d_%d.mat', act_num, trial_num));
log_file  = fullfile(log_dir,  sprintf('lab5_act%d_log%d.txt', act_num, trial_num));
if exist(log_file,'file'), delete(log_file); end
diary(log_file); diary on;
fprintf('=== lab5_act2_sol  %s ===\n\n', datestr(now,'yyyy-mm-dd HH:MM:SS'));

% Método Riccati para TV-LQR:
%   'zoh'      — ZOH discreto (Riccati algebraico por paso, más preciso)
%   'sqrt_rk4' — Sqrt RK4 continuo (según guía del lab, integración hacia atrás)
riccati_method = 'zoh';

% ── Aceleracion de la optimizacion ────────────────────────────────────────
% USE_PARALLEL: fmincon evalua las diferencias finitas del gradiente en
%   paralelo (requiere Parallel Computing Toolbox; el primer parpool de la
%   sesion tarda ~10-30 s en abrir).
% MEX: ejecutar build_omdyn_mex.m una vez — genera OMDyn.<mexext> y
%   open_manx_fkin.<mexext>, que tienen precedencia sobre los .m y aceleran
%   cada iteracion ~5-10x. Ambas mejoras se combinan.
USE_PARALLEL = true;

%% ========================================================================
%  1. Parámetros generales  (iguales a Act. 1)
%  ========================================================================

N  = 40;    % 480 variables de decision: N*(nx+nu)
Ts = 0.1;
nx = 8;
nu = 4;  

x0 = [pi/2; 0; pi/6; pi/3; 0; 0; 0; 0];      % Estado inicial (q,dq)
%x0 = [0; deg2rad(50); deg2rad(35); deg2rad(-105); 0; 0; 0; 0];      % Estado inicial (q,dq)
yf = [0.2; -0.13; 0.2; 0];               % Salida deseada (posición y orientación)

% ── Carga desde .mat si use_saved_solution = true ────────────────────────────
% Sobreescribe N, Ts, x0, yf (y carga zmin/exitflag/output) desde el archivo.
% Los valores definidos arriba sirven solo de documentación en ese caso.
if use_saved_solution
    if ~exist(zmin_file, 'file')
        error('use_saved_solution=true pero no existe: %s', zmin_file);
    end
    sv       = load(zmin_file, 'zmin', 'exitflag', 'output', 'N', 'Ts', 'x0', 'yf');
    zmin     = sv.zmin;    exitflag = sv.exitflag;    output = sv.output;
    N  = sv.N;    Ts = sv.Ts;
    x0 = sv.x0;   yf = sv.yf;
    fprintf('Cargado: %s  (N=%d  Ts=%.3f s)\n\n', zmin_file, N, Ts);
end

ukmax =  1.0;
ukmin = -1.0;

q_lower = [-3/4*pi; -11/18*pi; -11/18*pi;  -5/9*pi];
%q_lower = [-3/4*pi; -11/18*pi; -11/18*pi;  -deg2rad(106)];
q_upper = [ 3/4*pi;   5/9*pi;     pi/2; 23/36*pi];
dq_max  = 10;

%% ========================================================================
%  2. Parámetros del obstáculo MDF
%  ========================================================================

x_obs  = 0.22725; % [m] centro X en marco link1 (= 0.075 m Gazebo + 114.75 mm offset base)
y_obs  = 0.0;     % [m] centro Y
z_ceil = 0.158;   % [m] techo físico del arco MDF
rx_obs = 0.075;   % [m] semi-ancho en X (footprint = 150 mm)
ry_obs = 0.160;   % [m] semi-ancho en Y (footprint = 320 mm)
x_lo   = x_obs - rx_obs;   % 0.11475 m
x_hi   = x_obs + rx_obs;   % 0.26475 m
y_lo   = y_obs - ry_obs;   % -0.160 m
y_hi   = y_obs + ry_obs;   % +0.160 m

% Constraint XZ — análogo a la placa del Lab 2025:
%   x_k - x_lo - alpha_z*(z_k - z_ceil)^2 ≤ 0
% Para avanzar en x más allá de x_lo, el EE debe estar en z alejado de z_ceil.
% Condición mínima: alpha_z ≥ (x_yf-x_lo)/(z_yf-z_ceil)^2 = 0.085/0.00176 ≈ 48.2
alpha_z = 50.0;    % [m⁻¹]
% z_ceil_safe: techo efectivo del constraint = z_ceil + 10 mm de margen.
% Garantiza que el EE entre al footprint al menos 10 mm sobre el techo físico.
% Factibilidad yf: alpha_z*(z_yf-z_ceil_safe)^2 = 50*(0.032)^2 = 0.051 ≥ x_yf-x_lo = 0.048 ✓
z_ceil_safe = z_ceil + 0.010;  % [m] = 0.168 m

fprintf('--- Obstáculo MDF ---\n');
fprintf('  Footprint link1 : X=[%.5f, %.5f]  Y=[%.3f, %.3f] m\n', x_lo, x_hi, y_lo, y_hi);
fprintf('  z_ceil=%.3f m (fisico)  z_ceil_safe=%.3f m (+10 mm)  alpha_z=%.1f\n\n', ...
        z_ceil, z_ceil_safe, alpha_z);

%% ========================================================================
%  3. Inicialización de la optimización
%  ========================================================================

[~, u0g] = OMDyn(x0(1:4), zeros(4,1));
u0g = u0g(:);
u0g = max(min(u0g, ukmax), ukmin);

% Bounds: posición articular [q_lower, q_upper] + velocidad ±dq_max + torque ±ukmax
x_lower = [q_lower;              -dq_max * ones(4,1)];
x_upper = [q_upper;               dq_max * ones(4,1)];
lb = [repmat(x_lower, N, 1);  ukmin * ones(nu*N, 1)];
ub = [repmat(x_upper, N, 1);  ukmax * ones(nu*N, 1)];
x0_guess = x0;

% Warm start: por defecto punto constante (todos los estados en x0, torques
% de compensacion gravitatoria — factible para el hard constraint en y0).
% Si existe la solucion de la Actividad 1 con el mismo N y Ts, se usa como
% semilla: mismo x0/yf sin obstaculo, tipicamente reduce las iteraciones de
% fmincon a la mitad.
warm_start_act1_trial = 1;   % 0 = desactivar; n>0 = usar zmin_act1_n.mat

z0     = [kron(ones(N,1), x0); kron(ones(N,1), u0g)];
ws_msg = 'punto constante en x0 con compensacion gravitatoria';
if warm_start_act1_trial > 0
    ws_file = fullfile(zmin_dir, sprintf('zmin_act1_%d.mat', warm_start_act1_trial));
    if exist(ws_file, 'file')
        ws = load(ws_file, 'zmin', 'N', 'Ts');
        if ws.N == N && abs(ws.Ts - Ts) < 1e-12
            z0     = ws.zmin;
            ws_msg = sprintf('solucion de la Act. 1 (%s)', ws_file);
        else
            fprintf('Aviso: %s tiene N=%d Ts=%.3f (aqui N=%d Ts=%.3f) — se ignora.\n', ...
                    ws_file, ws.N, ws.Ts, N, Ts);
        end
    end
end
fprintf('Warm start: %s\n\n', ws_msg);

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

% UseParallel reparte las ~N*(nx+nu)+1 evaluaciones de diferencias finitas
% de cada iteracion entre los workers del pool.
options = optimoptions('fmincon', ...
    'Display',               'iter', ...
    'Algorithm',             'sqp', ...
    'UseParallel',           USE_PARALLEL, ...
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
if isfield(output,'message'), fprintf('Mensaje: %s\n\n', output.message); end

%% ========================================================================
%  5. Recuperar trayectorias
%  ========================================================================

Xref = reshape(zmin(1:nx*N),           [nx N]);
Uref = reshape(zmin(nx*N+(1:nu*N)),    [nu N]);

%% ========================================================================
%  6. Verificar evasión del obstáculo
%  ========================================================================

% Verificación del constraint XZ: x_k - x_lo - alpha_z*(z_k - z_ceil_safe)^2 ≤ 0
% + clearance físico sobre techo real (z_ceil).
max_cxz    = -inf;
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

fprintf('--- Verificación obstáculo (constraint XZ) ---\n');
if max_cxz <= 0
    fprintf('  OK — constraint (z_ceil_safe=%.3f m) satisfecho (max c = %.5f)\n\n', z_ceil_safe, max_cxz);
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
%  7. Métricas optimización
%  ========================================================================

yref = zeros(4, N+1);
yref(:,1) = open_manx_fkin(x0(1:4));
for i = 1:N
    yref(:,i+1) = open_manx_fkin(Xref(1:4,i));
end

metrics_trajopt.exitflag     = exitflag;
metrics_trajopt.max_abs_uref = max(abs(Uref), [], 2);

fprintf('\n============================================================\n');
fprintf('Lab 5 Actividad 2 - Métricas de la trayectoria optimizada\n');
fprintf('============================================================\n');
fprintf('Exitflag fmincon                 : %g\n',   metrics_trajopt.exitflag);
fprintf('Max |u_ref| por articulacion     : [%s] N·m\n', ...
        num2str(metrics_trajopt.max_abs_uref', ' %.4f'));

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

Qk = diag([100; 100; 100; 500;   10;  10;  10;  50]);
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
    case 'sqrt_rk4'
        % Integración backward de Riccati en forma sqrt con RK4 (según guía lab).
        % -dP/dt = A'P - 0.5·P·P'·B·R⁻¹·B'·P + 0.5·Q·P'^{-1}
        % donde S = P·P'  →  K = R⁻¹·B'·S
        Pk = zeros(nx, nx, N);
        Sk = zeros(nx, nx, N);
        Pk(:,:,N) = sqrtm(0.05 * Qf);
        Sk(:,:,N) = Pk(:,:,N) * Pk(:,:,N)';
        K_TV(:,:,N) = Rk \ (Bk(:,:,N)' * Sk(:,:,N));

        for k = N:-1:2
            P   = Pk(:,:,k);
            A   = Ak(:,:,k);
            B   = Bk(:,:,k);
            jj  = 100;          % sub-pasos RK4 dentro de cada intervalo Ts
            TsR = Ts / jj;
            for JJ = 1:jj
                r1 = Ricc_sqrt(P,            A, B, Qk, Rk);
                r2 = Ricc_sqrt(P + r1*TsR/2, A, B, Qk, Rk);
                r3 = Ricc_sqrt(P + r2*TsR/2, A, B, Qk, Rk);
                r4 = Ricc_sqrt(P + r3*TsR,   A, B, Qk, Rk);
                P  = P + TsR*(r1 + 2*r2 + 2*r3 + r4)/6;
            end
            Pk(:,:,k-1) = P;
            Sk(:,:,k-1) = P * P';
            K_TV(:,:,k-1) = Rk \ (B' * Sk(:,:,k-1));
        end

    otherwise
        error('riccati_method debe ser ''zoh'' o ''sqrt_rk4''.');
end

assert(all(isfinite(K_TV(:))), 'K_TV contiene NaN/Inf.');
fprintf('K_TV calculado.\n');

%% ========================================================================
%  10. Simulación TV-LQR (sistema no lineal, perturbación inicial)
%  ========================================================================

x0sim = x0 + 0.05*randn(nx,1);

% Control en lazo cerrado con saturacion explicita (OM4dof ya no satura
% internamente — ver nota en OM4dof).
u_cl = @(t,x) max(min(controlTVLQR(t,x,Uref,Xref,Ts,N,K_TV), ukmax), ukmin);

[T, Xsim] = ode45(@(t,x) OM4dof(t, x, u_cl(t,x)), 0:Ts:(N*Ts), x0sim);
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
metrics_tvlqr.sat_percent         = 100*mean(abs(U_tvlqr_raw) >= (ukmax-1e-8), 2);
metrics_tvlqr.max_abs_joint_error = max(abs(joint_error), [], 2);
metrics_tvlqr.rms_joint_error     = sqrt(mean(joint_error.^2, 2));

fprintf('\n============================================================\n');
fprintf('Lab 5 Actividad 2 - Métricas de seguimiento TV-LQR\n');
fprintf('============================================================\n');
fprintf('Error cartesiano final ||y(tf)-yf||: %.6f m\n',  metrics_tvlqr.final_error_cart);
fprintf('Max |u_TVLQR| por articulacion    : [%s] N·m\n', num2str(metrics_tvlqr.max_abs_utvlqr', ' %.4f'));
fprintf('Saturacion TV-LQR por articulacion: [%s] %%\n',  num2str(metrics_tvlqr.sat_percent', ' %.2f'));
fprintf('Error max artic                   : [%s] rad\n', num2str(metrics_tvlqr.max_abs_joint_error', ' %.5f'));
fprintf('Error RMS artic                   : [%s] rad\n', num2str(metrics_tvlqr.rms_joint_error', ' %.5f'));

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
title(tl1,'Trayectorias articulares optimizadas $q_{ref}$ --- Act. 2 (con obstaculo)', ...
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
    ylabel(sprintf('$u_%d\\;[\\mathrm{N\\cdot m}]$',iu),'Interpreter','latex','FontSize',fs);
    grid on; box on; set(gca,'FontSize',fs); xlim([tU(1),tU(end)]);
    if iu < nu, set(gca,'XTickLabel',[]); else, xlabel('Tiempo [s]','FontSize',fs); end
end
title(tl2,'Entradas optimizadas $u_{ref}$ --- Act. 2','Interpreter','latex', ...
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

% ── Arco MDF: techo plano + dos paredes laterales ─────────────────────────
% (z_ceil, x_lo/hi, y_lo/hi definidos en Sección 2)

% Techo plano
h_obs_surf = patch([x_lo x_hi x_hi x_lo], [y_lo y_lo y_hi y_hi], ...
                   [z_ceil z_ceil z_ceil z_ceil], c_obs_color, ...
                   'FaceAlpha', 0.55, 'EdgeColor', [0.4 0.25 0.1], 'LineWidth', 0.8);

% Pared lateral y = y_lo
patch([x_lo x_hi x_hi x_lo], repmat(y_lo,1,4), ...
      [0 0 z_ceil z_ceil], c_obs_color, ...
      'FaceAlpha', 0.35, 'EdgeColor', [0.4 0.25 0.1], 'LineWidth', 0.8);

% Pared lateral y = y_hi
patch([x_lo x_hi x_hi x_lo], repmat(y_hi,1,4), ...
      [0 0 z_ceil z_ceil], c_obs_color, ...
      'FaceAlpha', 0.35, 'EdgeColor', [0.4 0.25 0.1], 'LineWidth', 0.8);

% ── Monte Carlo (20 realizaciones) ───────────────────────────────────────
for rr = 1:20
    x0mc = x0 + 0.10*randn(nx,1);
    [~,Xmc] = ode45(@(t,x) OM4dof(t, x, u_cl(t,x)), 0:Ts:N*Ts, x0mc);
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
       {'Arco MDF','MC perturbado','Simulación TV-LQR', ...
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
    ylabel(sprintf('$u_%d\\;[\\mathrm{N\\cdot m}]$',iu),'Interpreter','latex','FontSize',fs);
    ydata  = [U_tvlqr_sat(iu,:), Uref_plot(iu,:)];
    yrange = max(max(ydata)-min(ydata), 0.05);
    ylim([min(ydata)-0.15*yrange, max(ydata)+0.15*yrange]);
    grid on; box on; set(gca,'FontSize',fs); xlim(xlims);
    if iu < nu, set(gca,'XTickLabel',[]); else, xlabel('Tiempo [s]','FontSize',fs); end
end
legend(nexttile(tl5,1),[hu,hur],{'TV-LQR','Referencia'}, ...
       'Orientation','horizontal','FontSize',fs,'Location','northoutside');
title(tl5,'Señal de control TV-LQR vs referencia — Act. 2', ...
      'FontSize',fs_ttl,'FontWeight','bold');

%% ========================================================================
%  12. Exportación de figuras
%  ========================================================================

if EXPORT_FIGS
    out_dir = fullfile(pkg_dir, 'plots', 'lab5', 'matlab', 'act2');
    if ~exist(out_dir,'dir'), mkdir(out_dir); end

    fig_names = {'act2_qref','act2_uref','act2_tvlqr_seguimiento', ...
                 'act2_trayectoria_3d','act2_tvlqr_control'};

    for fi = 1:5
        base = fullfile(out_dir, fig_names{fi});
        exportgraphics(figure(fi), [base '.png'], 'Resolution', 300);
        exportgraphics(figure(fi), [base '.pdf'], 'ContentType','vector');
    end
    fprintf('\nFiguras guardadas en: %s\n', out_dir);
end

diary off;

%% ========================================================================
%  Funciones locales
%  ========================================================================

function J = Jcosto(z, Ts, N, nx, nu, x0, yf)
% Solo términos de tracking y control. El obstáculo se maneja como
% hard constraint en restr() — igual que la solución de referencia del lab.

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

function [c_des, c_eq] = restr(z, Ts, N, nx, nu, x0, x_lo, z_ceil, alpha_z)
% Restricciones del problema de optimización.
% Constraint de obstáculo (plano XZ, análogo al Lab 2025):
%   x_k - x_lo - alpha_z*(z_k - z_ceil)^2 ≤ 0
% Para avanzar en x más allá de x_lo, el EE debe estar en z alejado de z_ceil.

    c_eq  = zeros(nx*N, 1);
    c_des = zeros(N, 1);

    for k = 0:N-1
        if k == 0, xk = x0; else, xk = z(nx*(k-1) + (1:nx)); end
        uk   = z(nx*N + nu*k + (1:nu));
        xkp1 = z(nx*k + (1:nx));

        % Restricciones de dinámica (RK4)
        k1 = OM4dof(k*Ts,        xk,            uk);
        k2 = OM4dof(k*Ts + Ts/2, xk + Ts*k1/2, uk);
        k3 = OM4dof(k*Ts + Ts/2, xk + Ts*k2/2, uk);
        k4 = OM4dof(k*Ts + Ts,   xk + Ts*k3,   uk);
        c_eq(nx*k + (1:nx)) = xkp1 - (xk + Ts*(k1 + 2*k2 + 2*k3 + k4)/6);

        % Restricción evasión techo arco MDF — plano XZ (Lab 2025 style).
        yk = open_manx_fkin(xkp1(1:4));
        c_des(k+1) = yk(1) - x_lo - alpha_z*(yk(3) - z_ceil)^2;
    end
end

% ─────────────────────────────────────────────────────────────────────────

function dx = OM4dof(t, x, u) %#ok<INUSD>
% Modelo no lineal SIN saturacion interna de u:
%   - En la optimizacion los bounds lb/ub ya imponen |u| <= 1. Un clamp aqui
%     anularia el gradiente por diferencias finitas justo en u = ±1
%     (columnas cero en B_k) y estanca el paso SQP con entradas saturadas.
%   - En la simulacion en lazo cerrado la saturacion se aplica explicitamente
%     sobre la salida de controlTVLQR (handle u_cl, seccion 10).

    q  = x(1:4);
    dq = x(5:8);
    u  = u(:);

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

% ─────────────────────────────────────────────────────────────────────────

function dP = Ricc_sqrt(P, A, B, Q, R)
% Ecuación de Riccati en forma sqrt para integración backward continua.
% Entrada:  P  — factor sqrt de S (S = P*P')
% Salida:   dP — derivada de P
    dP = A'*P - 0.5*P*(P'*B*(R\B')*P) + 0.5*Q*(P'\eye(size(P,1)));
end

% ─────────────────────────────────────────────────────────────────────────

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
                    datestr(now,'yyyy-mm-dd HH:MM:SS'), elapsed);
    end
end
