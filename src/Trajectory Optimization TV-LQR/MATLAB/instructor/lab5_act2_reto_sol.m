%% lab5_act2_reto_sol.m
% Trajectory Optimization y TV-LQR — Laboratorio 5, RETO (paso bajo → encima)
% Control de Sistemas No Lineales — OpenMANIPULATOR-X (4 GDL)
%
% Objetivo : Partir con el gripper BAJO el arco MDF y terminar ENCIMA del
% techo, en la posicion cartesiana de q_fin = [0;0;0;0]:
%   x0 = [0; 0.92; 0.54; -1.71]  → EE (0.264, 0, 0.043),  c_box = -0.050
%   yf = fkin([0;0;0;0])         → EE (0.286, 0, 0.2045), c_box = -0.0085
% El EE debe salir por debajo del techo (z ≤ 0.093 dentro del footprint),
% rodear la cara frontal de la caja de exclusion y aterrizar sobre el techo.
%
% CLAVE DEL RETO — clase de homotopia (verificado numericamente 2026-07-14):
% con la semilla constante en x0 (la de la Act. 2), fmincon termina con
% exitflag 1 ("Local minimum found that satisfies the constraints") pero
% ATASCADO bajo la caja, a ~110 mm de yf: para subir tendria que retroceder
% primero (empeorando el costo transitoriamente) y SQP es un metodo local.
% La solucion NO es agregar un waypoint a la formulacion (restriccion o
% costo): es codificarlo en la SEMILLA z0, que es quien elige la familia de
% trayectorias (Seccion 3). Costo, restricciones y opciones son IDENTICOS a
% lab5_act2_sol.m.
%
% Obstaculo, marco link1 y caja de exclusion: identicos a lab5_act2_sol.m
% (ver la derivacion completa en su cabecera y su Seccion 2).
%
% Figuras:
%   Fig 1 — Trayectorias articulares optimizadas q_ref
%   Fig 2 — Entradas optimizadas u_ref
%   Fig 3 — Seguimiento articular TV-LQR (simulación vs referencia)
%   Fig 4 — Trayectoria cartesiana 3D + arco MDF (techo y paredes)
%   Fig 5 — Señal de control TV-LQR vs referencia
%
% Para usar en Gazebo: tras correr este script (deja N, Ts, nx, nu, x0, yf,
%   Xref, Uref, K_TV en el workspace), ejecutar Lab5_Export_Refs.m para
%   regenerar la carpeta references/.

clear; close all; clc;
rng(1);

EXPORT_FIGS = false;

% ── Identificadores de sesión (editar antes de cada ejecución) ───────────────
trial_num          = 1;     % número de prueba — nombra el log y el zmin
use_saved_solution = false; % true → carga N, Ts, x0, yf y zmin desde el .mat

pkg_dir    = '/home/utec/open_manx_ws/src/open_manipulator_x_torque_control';
matlab_dir = fullfile(pkg_dir, 'src', 'Trajectory Optimization TV-LQR', 'MATLAB');
log_dir    = fullfile(matlab_dir, 'logs');
zmin_dir   = fullfile(matlab_dir, 'zmin');
if ~exist(log_dir,  'dir'), mkdir(log_dir);  end
if ~exist(zmin_dir, 'dir'), mkdir(zmin_dir); end

zmin_file = fullfile(zmin_dir, sprintf('zmin_reto_%d.mat', trial_num));
log_file  = fullfile(log_dir,  sprintf('lab5_reto_log%d.txt', trial_num));
if exist(log_file,'file'), delete(log_file); end
diary(log_file); diary on;
fprintf('=== lab5_act2_reto_sol  %s ===\n\n', datestr(now,'yyyy-mm-dd HH:MM:SS'));

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

N  = 100;   % 1200 variables de decision: N*(nx+nu). Horizonte 5 s (el rodeo
            % es ~3x mas largo que la trayectoria de la Act. 2). u_ref toca
            % |u|=1 solo 1 muestra en J2 y J3 (t~3 s, al remontar la esquina
            % frontal con el brazo extendido) — puntual y aceptable; tambien
            % converge con N=80/4 s con la misma saturacion puntual.
Ts = 0.05;
nx = 8;
nu = 4;

q_fin = [0; 0; 0; 0];                     % configuracion final (sobre el techo)
x0 = [0; 0.92; 0.54; -1.71; 0; 0; 0; 0];  % bajo el arco: EE (0.264, 0, 0.043)
yf = open_manx_fkin(q_fin);               % (0.286, 0, 0.2045, 0) en marco link1
yf(2) = 0;                                % limpia residuo numerico ~1e-17 en y

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

% Centro del arco en marco link1 = pose Gazebo (mundo) + offset world→link1
% (xacro world_fixed: link1 en mundo (-0.127, 0, +0.010)).
OBS_GZ_X = 0.1125;   % [m] obstacle_pose x en sim_init_config.yaml (marco mundo)
X_W2L1   = 0.127;    % [m] offset mundo→link1 en x (world_fixed del xacro)

x_obs  = OBS_GZ_X + X_W2L1;   % 0.2395 m — centro X en marco link1
y_obs  = 0.0;                 % [m] centro Y
z_ceil = 0.158 - 0.010;       % 0.148 m — techo fisico en marco link1 (techo a
                              % 158 mm del piso; link1 a 10 mm por la placa)
rx_obs = 0.075;   % [m] semi-fondo en X (footprint = 150 mm)
ry_obs = 0.158;   % [m] semi-ancho en Y (paredes en y=±0.1565, borde exterior)
x_lo   = x_obs - rx_obs;   % 0.1645 m (cara frontal, lado robot)
x_hi   = x_obs + rx_obs;   % 0.3145 m (cara trasera)
y_lo   = y_obs - ry_obs;
y_hi   = y_obs + ry_obs;

% ── Zona de exclusion (plano XZ): caja del techo inflada por el gripper ──
% La parabola x-x_lo-α(z-z_safe)²≤0 de la version anterior tiene una rama
% inferior simetrica: cerca del borde frontal (vertice) admite pasar con el
% EE apenas sobre el techo fisico → el gripper colisiono en Gazebo. Se
% reemplaza por una caja de exclusion con margenes anisotropicos tomados de
% la geometria del gripper (mallas URDF / spec ROBOTIS):
h_fing    = 0.029;   % [m] dedos bajo el eje del EE (gripper horizontal)
g_fwd     = 0.021;   % [m] punta de dedos mas alla del punto EE (fkin)
% Con el gripper inclinado (pitch φ>0, punta abajo) la esquina inferior-
% delantera de los dedos cae h_fing·cosφ + g_fwd·sinφ bajo el eje. La
% referencia cruza el arco con φ hasta ~0.3 rad → 34 mm (+2 mm por
% apertura de pinza / incertidumbre de mallas). Sin esta correccion el
% gripper rozo el borde del techo en Gazebo (test 3, t≈3.1-3.2 s).
h_grip    = 0.036;   % [m] envolvente vertical bajo el eje (valida si φ ≤ 0.3)
h_grip_up = 0.040;   % [m] cuerpo sobre el eje del EE → margen bajo el techo
                     %     (relevante para el paso por debajo del reto)
m_clr     = 0.010;   % [m] margen de seguridad adicional
EPS_CLR   = 0.002;   % [m] holgura minima exigida fuera de la caja

box_x = [x_lo - g_fwd - m_clr,   x_hi + g_fwd + m_clr];    % [0.1335, 0.3455]
box_z = [z_ceil - 0.003 - h_grip_up - m_clr, ...           %  0.095 (bajo techo)
         z_ceil + h_grip + m_clr];                         %  0.194 (sobre techo)

% Constraint (ver restr): fuera de la caja en x O en z, con holgura EPS_CLR:
%   c = EPS_CLR - max( max(box_x(1)-x, x-box_x(2)), max(box_z(1)-z, z-box_z(2)) ) ≤ 0
% Cruce por ARRIBA: z ≥ box_z(2)+EPS = 0.196 → esquina de dedos (con pitch)
%   ≥ +14 mm sobre el techo fisico.
% Paso por DEBAJO: z ≤ box_z(1)-EPS = 0.093 → tope del gripper (eje + 40 mm)
%   ≥ +12 mm bajo la cara inferior del techo (0.145).
% Factibilidad reto: c_box(x0) = -0.050 (bajo la caja) y c_box(yf) = -0.0085
%   (z_yf - box_z(2) = 10.5 mm ≥ EPS_CLR) — ambos extremos fuera de la caja ✓

fprintf('--- Obstáculo MDF (marco link1) ---\n');
fprintf('  Footprint: X=[%.4f, %.4f]  Y=[%.3f, %.3f] m   z_ceil=%.3f m\n', ...
        x_lo, x_hi, y_lo, y_hi, z_ceil);
fprintf('  Caja de exclusion (EE): X=[%.4f, %.4f]  Z=[%.3f, %.3f]  EPS=%.0f mm\n', ...
        box_x(1), box_x(2), box_z(1), box_z(2), 1e3*EPS_CLR);
fprintf('  Cruce sobre techo: z >= %.3f (gripper >= %+.0f mm)\n', ...
        box_z(2)+EPS_CLR, 1e3*(box_z(2)+EPS_CLR - h_grip - z_ceil));
fprintf('  Paso por debajo  : z <= %.3f (tope gripper >= %+.0f mm bajo techo)\n\n', ...
        box_z(1)-EPS_CLR, 1e3*(z_ceil - 0.003 - (box_z(1)-EPS_CLR) - h_grip_up));

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
% ── Semilla de 3 tramos alrededor de la caja (CLAVE del reto) ─────────────
% SQP es un metodo local: la semilla elige la clase de homotopia. Con la
% semilla constante en x0 (la de la Act. 2) el optimizador queda atrapado
% bajo la caja — exitflag 1 engañoso, EE a ~110 mm de yf. Aqui z0 se
% construye interpolando en espacio articular q_init → q_exit → q_up →
% q_fin, con duracion de cada tramo proporcional a su longitud articular;
% dq por diferencia finita y u de compensacion gravitatoria.
% q_exit y q_up se hallaron por barrido de malla IK (41^3 configs, q1=0)
% filtrando: c_box <= -0.01, esquina de dedos >= 15 mm sobre la placa y
% margen >= 0.15 rad a los limites articulares.
q_exit = [0; -1.0968; 1.1017; 0.9940];  % EE (0.101, 0.050): atras, aun bajo
q_up   = [0; -1.0442; 0.0985; 1.1224];  % EE (0.110, 0.240): sobre la esquina

QW      = [x0(1:4), q_exit, q_up, q_fin];
seg_len = vecnorm(diff(QW, 1, 2));
seg_t   = [0, cumsum(seg_len) / sum(seg_len)] * (N*Ts);
q_seed  = zeros(4, N+1);
for k = 0:N
    tk  = k*Ts;
    seg = min(find(tk <= seg_t(2:4) + 1e-9, 1), 3);
    tau = (tk - seg_t(seg)) / max(seg_t(seg+1) - seg_t(seg), eps);
    q_seed(:,k+1) = QW(:,seg) + tau * (QW(:,seg+1) - QW(:,seg));
end

z0 = zeros((nx+nu)*N, 1);
for k = 1:N
    dqk = (q_seed(:,k+1) - q_seed(:,k)) / Ts;
    z0((k-1)*nx + (1:nx)) = [q_seed(:,k+1); dqk];
    [~, ugk] = OMDyn(q_seed(:,k), zeros(4,1));
    z0(nx*N + (k-1)*nu + (1:nu)) = max(min(ugk(:), ukmax), ukmin);
end

% La ruta seed debe quedar completamente FUERA de la caja (c <= 0 en todos
% los nodos); si esto falla, revisar q_exit/q_up contra la geometria.
c_box_seed = @(y) EPS_CLR - max(max(box_x(1)-y(1), y(1)-box_x(2)), ...
                                max(box_z(1)-y(3), y(3)-box_z(2)));
c_seed_max = -inf;
for k = 1:N+1
    c_seed_max = max(c_seed_max, c_box_seed(open_manx_fkin(q_seed(:,k))));
end
fprintf('Warm start: semilla 3 tramos alrededor de la caja (max c_seed = %+.4f)\n\n', ...
        c_seed_max);

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
        @(z) restr(z, Ts, N, nx, nu, x0, box_x, box_z, EPS_CLR), ...
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

% Verificación del constraint de caja (nodos + puntos medios) y clearances
% físicos POR FASE — en el reto la trayectoria pasa legitimamente por debajo
% del techo, asi que el "clearance sobre el techo" solo aplica a la fase
% alta. Fase baja (EE bajo la cara inferior del techo, dentro del
% footprint): tope del gripper (eje + h_grip_up) contra la cara inferior.
% Fase alta (EE sobre z_ceil): EE y esquina de dedos (con el pitch φ del
% nodo: h_fing·cosφ + g_fwd·sinφ bajo el eje) sobre el techo. Ademas, dedos
% contra la placa (z=0 en marco link1) en TODA la trayectoria.
c_box_chk = @(y) EPS_CLR - max(max(box_x(1) - y(1), y(1) - box_x(2)), ...
                               max(box_z(1) - y(3), y(3) - box_z(2)));
z_roof_inf = z_ceil - 0.003;   % cara inferior del techo (plancha de 3 mm)

Qchk = [x0(1:4), Xref(1:4,:)];                              % nodos 0..N
Qchk = [Qchk(:,1), reshape([0.5*(Qchk(:,1:N) + Qchk(:,2:N+1)); ...
                            Qchk(:,2:N+1)], 4, [])];        % + puntos medios
max_cxz       = -inf;
min_over_ee   = inf;    % fase alta: EE sobre el techo fisico
min_over_grip = inf;    % fase alta: esquina de dedos sobre el techo
min_under_top = inf;    % fase baja: tope del gripper bajo la cara inferior
min_floor     = inf;    % dedos sobre la placa (toda la trayectoria)
for k = 1:size(Qchk, 2)
    yk = open_manx_fkin(Qchk(:,k));
    max_cxz = max(max_cxz, c_box_chk(yk));
    h_eff = h_fing*cos(yk(4)) + g_fwd*max(sin(yk(4)), 0);
    min_floor = min(min_floor, yk(3) - h_eff);
    if yk(1) >= x_lo && yk(1) <= x_hi && abs(yk(2)) <= ry_obs
        if yk(3) >= z_ceil
            min_over_ee   = min(min_over_ee,   yk(3) - z_ceil);
            min_over_grip = min(min_over_grip, yk(3) - h_eff - z_ceil);
        elseif yk(3) < z_roof_inf
            min_under_top = min(min_under_top, z_roof_inf - (yk(3) + h_grip_up));
        end
    end
end

fprintf('--- Verificación obstáculo (caja de exclusion, nodos + medios) ---\n');
if max_cxz <= 1e-4    % ConstraintTolerance de fmincon
    fprintf('  OK — constraint satisfecho (max c = %.2e)\n\n', max_cxz);
else
    fprintf('  ADVERTENCIA: constraint violado (max c = %.2e > 1e-4)\n', max_cxz);
    fprintf('  EE entra a la caja de exclusion del techo. Re-optimizar.\n\n');
end
fprintf('  Fase baja : tope del gripper vs cara inferior del techo  min %+.1f mm\n', ...
        1e3*min_under_top);
fprintf('  Fase alta : EE %+.1f mm | esquina de dedos %+.1f mm sobre el techo\n', ...
        1e3*min_over_ee, 1e3*min_over_grip);
fprintf('  Dedos vs placa (toda la trayectoria)                     min %+.1f mm\n\n', ...
        1e3*min_floor);

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
fprintf('Lab 5 Reto - Métricas de la trayectoria optimizada\n');
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

% Pesos validados en hardware (Act. 1, test 2): posicion agresiva y R bajo.
% Con los antiguos (Q=[100..500], R=100) las ganancias eran sub-stiction y
% J4 se desbocaba (ver lab5_act1_sol.m, seccion 6).
Qk = diag([400; 400; 400; 10000;   1; 1; 1; 10]);
Rk = diag([1; 1; 1; 0.2]);
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

% ── Piso de rigidez para J4 (hardware) ────────────────────────────────────
% La Riccati asigna a J4 ~0.4 N·m/rad (inercia minima en el modelo), pero la
% stiction + offset de corriente (~0.08 N·m) dejan ~0.2 rad de error de
% equilibrio en el robot real. Rigidez minima estilo FL (Lab 4: ~3.3 N·m/rad).
for k = 1:N
    K_TV(4,4,k) = max(K_TV(4,4,k), 1.5);    % posicion  [N·m/rad]
    K_TV(4,8,k) = max(K_TV(4,8,k), 0.08);   % velocidad [N·m/(rad/s)]
end

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
fprintf('Lab 5 Reto - Métricas de seguimiento TV-LQR\n');
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
set(gcf,'Name','q_ref optimizado Reto','Color','w','Position',[100 80 1100 650]);
tl1 = tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
for iq = 1:4
    nexttile(tl1);
    plot(tX, Xref(iq,:), '-', 'Color', c_ref, 'LineWidth', lw);
    ylabel(qLabels{iq},'Interpreter','latex','FontSize',fs);
    grid on; box on; set(gca,'FontSize',fs); xlim([tX(1),tX(end)]);
    if iq < 4, set(gca,'XTickLabel',[]); else, xlabel('Tiempo [s]','FontSize',fs); end
end
title(tl1,'Trayectorias articulares optimizadas $q_{ref}$ --- Reto (paso bajo $\rightarrow$ encima)', ...
      'Interpreter','latex','FontSize',fs_ttl,'FontWeight','bold');

%% ========================================================================
%  Fig 2 — Entradas optimizadas
%  ========================================================================

figure(2); clf;
set(gcf,'Name','u_ref optimizado Reto','Color','w','Position',[120 100 1100 560]);
tl2 = tiledlayout(nu,1,'TileSpacing','compact','Padding','compact');
for iu = 1:nu
    nexttile(tl2);
    plot(tU, Uref(iu,:), '-', 'Color', c_ref, 'LineWidth', lw);
    ylabel(sprintf('$u_%d\\;[\\mathrm{N\\cdot m}]$',iu),'Interpreter','latex','FontSize',fs);
    grid on; box on; set(gca,'FontSize',fs); xlim([tU(1),tU(end)]);
    if iu < nu, set(gca,'XTickLabel',[]); else, xlabel('Tiempo [s]','FontSize',fs); end
end
title(tl2,'Entradas optimizadas $u_{ref}$ --- Reto','Interpreter','latex', ...
      'FontSize',fs_ttl,'FontWeight','bold');

%% ========================================================================
%  Fig 3 — Seguimiento articular TV-LQR
%  ========================================================================

qRefPlot = [x0(1:4), Xref(1:4,:)];

figure(3); clf;
set(gcf,'Name','Seguimiento TV-LQR Reto','Color','w','Position',[140 80 1100 650]);
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
title(tl3,'Seguimiento articular TV-LQR — Reto (paso bajo)', ...
      'FontSize',fs_ttl,'FontWeight','bold');

%% ========================================================================
%  Fig 4 — Trayectoria 3D + arco MDF (techo y paredes)
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
title('Trayectoria cartesiana 3D con obstáculo MDF — TV-LQR Reto', ...
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
set(gcf,'Name','Control TV-LQR Reto','Color','w','Position',[180 110 1100 560]);
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
title(tl5,'Señal de control TV-LQR vs referencia — Reto', ...
      'FontSize',fs_ttl,'FontWeight','bold');

%% ========================================================================
%  12. Exportación de figuras
%  ========================================================================

if EXPORT_FIGS
    out_dir = fullfile(pkg_dir, 'plots', 'lab5', 'matlab', 'reto');
    if ~exist(out_dir,'dir'), mkdir(out_dir); end

    fig_names = {'reto_qref','reto_uref','reto_tvlqr_seguimiento', ...
                 'reto_trayectoria_3d','reto_tvlqr_control'};

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
% NOTA reto: la referencia de camino yrefk es la recta y0→yf, que ATRAVIESA
% la caja de exclusion; ese termino solo regulariza — el desvio correcto lo
% imponen el hard constraint y la semilla z0 (identico a lab5_act2_sol.m).

    Qv      = 0.1 * eye(4);
    Qy      = diag([1000; 1000; 1000; 10]);
    Qf_cost = diag([1000; 1000; 1000; 10]);
    R       = 0.01 * eye(nu);

    J  = 0;
    y0 = open_manx_fkin(x0(1:4));
    xN = z(nx*(N-1) + (1:nx));
    yN = open_manx_fkin(xN(1:4));

    % Error terminal cartesiano + llegada en reposo (sin el termino de
    % velocidad terminal el optimo llega a yf "en movimiento" y en hw la
    % compensacion de Coulomb sigue empujando durante el hold final).
    J = J + (yN - yf)' * Qf_cost * (yN - yf) + 100 * (xN(5:8)' * xN(5:8));

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

function [c_des, c_eq] = restr(z, Ts, N, nx, nu, x0, box_x, box_z, eps_clr)
% Restricciones del problema de optimización.
% Constraint de obstáculo: el EE debe quedar FUERA de la caja de exclusion
% del techo (inflada con la geometria del gripper) en x O en z:
%   c = eps_clr - max(dx_out, dz_out) ≤ 0
%   dx_out = max(box_x(1)-x, x-box_x(2))   (>0 si esta fuera de la caja en x)
%   dz_out = max(box_z(1)-z, z-box_z(2))   (>0 si esta fuera de la caja en z)
% Dentro de la caja ambos son negativos → c > 0 (violado, magnitud =
% penetracion + eps_clr). Se evalúa en cada nodo Y en el punto medio
% articular de cada intervalo: con Ts=0.05 el EE recorre hasta ~15 mm entre
% nodos y podría "cortar" la esquina del techo entre dos nodos factibles.

    c_eq  = zeros(nx*N, 1);
    c_des = zeros(2*N, 1);

    c_box = @(y) eps_clr - max(max(box_x(1) - y(1), y(1) - box_x(2)), ...
                               max(box_z(1) - y(3), y(3) - box_z(2)));

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

        % Evasión del techo del arco MDF: nodo k+1 y punto medio del intervalo
        c_des(2*k+1) = c_box(open_manx_fkin(xkp1(1:4)));
        c_des(2*k+2) = c_box(open_manx_fkin(0.5*(xk(1:4) + xkp1(1:4))));
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
