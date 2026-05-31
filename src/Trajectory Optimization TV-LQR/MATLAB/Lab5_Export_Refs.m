%% Lab5_Export_Refs.m
% Carga la solucion del Pre-Laboratorio 5, recalcula las ganancias TV-LQR
% y exporta los archivos de referencia para los nodos ROS 2.
%
% REQUISITOS:
%   - OMDyn.m y open_manx_fkin.m deben estar en el MATLAB path.
%     (Copiarlos aqui o ejecutar: addpath('<ruta_a_los_archivos>'))
%   - zmin.mat debe estar en esta misma carpeta (generado por PreLab5_Sol_Final.m).
%
% ARCHIVOS GENERADOS en references/:
%   time_ref.txt   N x 1    instantes de muestreo [s]
%   q_ref.txt      N x 4    posiciones articulares de referencia [rad]
%   dq_ref.txt     N x 4    velocidades articulares de referencia [rad/s]
%   u_ref.txt      N x 4    entradas optimizadas (torques) [N.m]
%   K_TV.txt       N x 32   ganancias TV-LQR (reshape col-major de K_k 4x8)
%   y_ref.txt     (N+1)x4   trayectoria cartesiana de referencia

clear; clc; close all;

%% ========================================================================
%  Parametros (deben coincidir con PreLab5_Sol_Final.m)
%  ========================================================================
N  = 30;
Ts = 0.05;
nx = 8;
nu = 4;

x0 = [pi/2; 0; pi/6; pi/3; 0; 0; 0; 0];   % debe coincidir con lab5_act1_sol.m
yf = [0.2; -0.13; 0.2; 0];

zmin_file = 'zmin4.mat';   % debe coincidir con el archivo generado por lab5_act1_sol.m
ref_dir   = '../references';

%% ========================================================================
%  1. Verificar dependencias
%  ========================================================================
if ~exist('OMDyn', 'file')
    error(['OMDyn.m no encontrado en el MATLAB path.\n' ...
           'Copie OMDyn.m a esta carpeta o ejecute:\n' ...
           '  addpath(''<ruta_a_OMDyn>'')']);
end
if ~exist('open_manx_fkin', 'file')
    error(['open_manx_fkin.m no encontrado en el MATLAB path.\n' ...
           'Copie open_manx_fkin.m a esta carpeta o ejecute:\n' ...
           '  addpath(''<ruta_a_open_manx_fkin>'')']);
end

%% ========================================================================
%  2. Cargar solucion optimizada
%  ========================================================================
assert(exist(zmin_file, 'file') == 2, ...
    'No se encontro %s. Ejecutar PreLab5_Sol_Final.m primero.', zmin_file);

data = load(zmin_file);
zmin = data.zmin;
assert(numel(zmin) == nx*N + nu*N, ...
    'Dimension de zmin incorrecta: esperado %d, obtenido %d.', ...
    nx*N + nu*N, numel(zmin));

exitflag = NaN;
if isfield(data, 'exitflag'), exitflag = data.exitflag; end
if isnan(exitflag) || exitflag < 0
    error('exitflag = %g: solucion completamente fallida (infactible). Revisar PreLab5_Sol_Final.m.', exitflag);
elseif exitflag == 0
    warning(['exitflag = 0: el solver agoto las iteraciones. La solucion puede ser suboptima.\n' ...
             'Se procedera con la ultima iteracion guardada en zmin.mat.']);
end
fprintf('zmin.mat cargado. exitflag = %g\n', exitflag);

%% ========================================================================
%  3. Reconstruir Xref y Uref
%  ========================================================================
Xref = reshape(zmin(1:nx*N),          [nx N]);   % 8 x N
Uref = reshape(zmin(nx*N+1:nx*N+nu*N),[nu N]);   % 4 x N

%% ========================================================================
%  4. Validaciones previas a la exportacion
%  ========================================================================

% 4a. Restriccion de torque
assert(max(abs(Uref(:))) <= 1 + 1e-4, ...
    'max|Uref| = %.4f supera el limite de torque (1 N.m).', max(abs(Uref(:))));

% 4b. Limites articulares del OpenManipulator-X
q_lower = [-1.5708; -1.5708; -1.5708; -1.7907];
q_upper = [ 1.5708;  1.5708;  1.5708;  2.0420];
for k = 1:N
    for i = 1:4
        assert(Xref(i,k) >= q_lower(i)-1e-3 && Xref(i,k) <= q_upper(i)+1e-3, ...
            'q%d en k=%d fuera de limites: %.4f rad.', i, k, Xref(i,k));
    end
end
fprintf('Limites articulares: OK\n');

% 4c. Trayectoria cartesiana de referencia
yref = zeros(4, N+1);
yref(:,1) = open_manx_fkin(x0(1:4));
for i = 1:N
    yref(:,i+1) = open_manx_fkin(Xref(1:4,i));
end
assert(all(isfinite(yref(:))), 'yref contiene NaN/Inf.');

%% ========================================================================
%  5. Linealizacion numerica variante en el tiempo
%  ========================================================================
fprintf('Calculando Ak, Bk...\n');
Ak = zeros(nx, nx, N);
Bk = zeros(nx, nu, N);
eps_jac = 1e-6;

for k = 1:N
    xk = Xref(:,k);
    uk = Uref(:,k);
    for i = 1:nx
        dx = zeros(nx,1); dx(i) = eps_jac;
        Ak(:,i,k) = (OM4dof_local(xk+dx, uk) - OM4dof_local(xk-dx, uk)) / (2*eps_jac);
    end
    for i = 1:nu
        du = zeros(nu,1); du(i) = eps_jac;
        Bk(:,i,k) = (OM4dof_local(xk, uk+du) - OM4dof_local(xk, uk-du)) / (2*eps_jac);
    end
end

%% ========================================================================
%  6. Calculo de ganancias TV-LQR
%  ========================================================================

Qk = diag([100; 100; 100; 100; 1; 1; 1; 1]);
Rk = 100*eye(nu);
Qf = Qk;

% ── Metodo de Riccati ────────────────────────────────────────────────────
% 'zoh'  — Discreto ZOH exacto via expm (recomendado para Ts >= 0.02 s)
% 'std'  — Continuo estandar sobre S (RK4; puede ser inestable con Ts grande)
% 'sqrt' — Continuo forma sqrt (RK4; puede singularizarse; aumentar riccati_jj)
riccati_method = 'zoh';
riccati_jj     = 100;   % sub-pasos RK4 por intervalo (solo 'std' y 'sqrt')

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
                f1 = Ricc_std_local(S,            A, B, Qk, Rk);
                f2 = Ricc_std_local(S + f1*TsR/2, A, B, Qk, Rk);
                f3 = Ricc_std_local(S + f2*TsR/2, A, B, Qk, Rk);
                f4 = Ricc_std_local(S + f3*TsR,   A, B, Qk, Rk);
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
        Pk(:,:,N)   = sqrtm(0.05*Qf);     % condicion terminal
        Sk_N        = Pk(:,:,N)*Pk(:,:,N)';
        K_TV(:,:,N) = Rk \ (Bk(:,:,N)'*Sk_N);
        for k = N:-1:2
            P = Pk(:,:,k);   A = Ak(:,:,k);   B = Bk(:,:,k);
            for JJ = 1:riccati_jj
                k1 = Ricc_sqrt_local(P,            A, B, Qk, Rk);
                k2 = Ricc_sqrt_local(P + k1*TsR/2, A, B, Qk, Rk);
                k3 = Ricc_sqrt_local(P + k2*TsR/2, A, B, Qk, Rk);
                k4 = Ricc_sqrt_local(P + k3*TsR,   A, B, Qk, Rk);
                P  = P + TsR*(k1 + 2*k2 + 2*k3 + k4)/6;
            end
            Pk(:,:,k-1)   = P;
            K_TV(:,:,k-1) = Rk \ (B'*(P*P'));
        end

    otherwise
        error('riccati_method invalido: ''%s''. Usar ''zoh'', ''std'' o ''sqrt''.', ...
              riccati_method);
end

assert(all(isfinite(K_TV(:))), ...
    'K_TV contiene NaN/Inf (metodo: %s). Revisar linealizacion o aumentar riccati_jj.', ...
    riccati_method);
fprintf('K_TV: OK\n');

%% ========================================================================
%  7. Exportacion de archivos de referencia
%  ========================================================================
if ~exist(ref_dir, 'dir'), mkdir(ref_dir); end

t_ref  = (Ts:Ts:N*Ts)';    % N x 1
q_ref  = Xref(1:4,:)';     % N x 4
dq_ref = Xref(5:8,:)';     % N x 4
u_ref  = Uref';             % N x 4
y_ref  = yref';             % (N+1) x 4

% Compensacion gravitatoria en x0: OMDyn(q0, dq0=0) da el torque para
% sostener el robot quieto en x0. El nodo ROS 2 la usa en la fase de warmup.
[~, tau_grav_vec] = OMDyn(x0(1:4), zeros(4,1));
tau_grav = tau_grav_vec(:)';   % 1 x 4

% K_TV: N x 32, orden col-major (reshape MATLAB es col-major)
K_TV_vec = zeros(N, nu*nx);
for k = 1:N
    K_TV_vec(k,:) = reshape(K_TV(:,:,k), 1, []);
end

writematrix(t_ref,    fullfile(ref_dir, 'time_ref.txt'),    'Delimiter', ' ');
writematrix(q_ref,    fullfile(ref_dir, 'q_ref.txt'),       'Delimiter', ' ');
writematrix(dq_ref,   fullfile(ref_dir, 'dq_ref.txt'),      'Delimiter', ' ');
writematrix(u_ref,    fullfile(ref_dir, 'u_ref.txt'),       'Delimiter', ' ');
writematrix(K_TV_vec, fullfile(ref_dir, 'K_TV.txt'),        'Delimiter', ' ');
writematrix(y_ref,    fullfile(ref_dir, 'y_ref.txt'),       'Delimiter', ' ');
writematrix(tau_grav, fullfile(ref_dir, 'tau_gravity.txt'), 'Delimiter', ' ');

fprintf('tau_gravity en x0: [%.4f %.4f %.4f %.4f] N.m\n', tau_grav);

%% ========================================================================
%  8. Verificacion de archivos generados
%  ========================================================================
files_out = {'time_ref.txt','q_ref.txt','dq_ref.txt','u_ref.txt','K_TV.txt','y_ref.txt','tau_gravity.txt'};
for f = files_out
    fpath = fullfile(ref_dir, f{1});
    assert(exist(fpath, 'file') == 2, 'No se creo: %s', fpath);
end

%% ========================================================================
%  9. Resumen
%  ========================================================================
fprintf('\n======================================================\n');
fprintf('Lab5_Export_Refs — Resumen de exportacion\n');
fprintf('======================================================\n');
fprintf('  N                     : %d\n',   N);
fprintf('  Ts                    : %.3f s\n', Ts);
fprintf('  Tiempo total          : %.2f s\n', N*Ts);
fprintf('  exitflag              : %g\n',   exitflag);
fprintf('  max|Uref| [N.m]       : [%.4f %.4f %.4f %.4f]\n', max(abs(Uref),[],2)');
fprintf('  Carpeta de salida     : %s\n', fullfile(pwd, ref_dir));
fprintf('\n  Archivos exportados:\n');
for f = files_out
    info = dir(fullfile(ref_dir, f{1}));
    fprintf('    %-20s  %.1f KB\n', f{1}, info.bytes/1024);
end
fprintf('\nListo. Los archivos pueden ser leidos por los nodos ROS 2.\n');

%% ========================================================================
%  Funciones locales
%  ========================================================================
function dx = OM4dof_local(x, u)
    q  = x(1:4);
    dq = x(5:8);
    u  = max(min(u(:), 1), -1);
    [M, phib] = OMDyn(q, dq);
    phib = phib(:);
    bf   = 0.001;
    ddq  = M \ (u - phib - bf*dq);
    dx   = [dq; ddq];
end

function dS = Ricc_std_local(S, A, B, Q, R)
% Riccati estandar continuo integrado hacia atras (tau = tf - t).
% dS/dtau = A'S + SA - S*B*R^{-1}*B'*S + Q
    dS = A'*S + S*A - S*B*(R \ (B'*S)) + Q;
end

function dP = Ricc_sqrt_local(P, A, B, Q, R)
% Riccati forma sqrt integrado hacia atras; S = P*P'.
% dP/dtau = A'P - (1/2)*P*P'*B*R^{-1}*B'*P + (1/2)*Q*P'^{-1}
    dP = A'*P - 0.5*P*P'*B*(R \ (B'*P)) + 0.5*(Q/P');
end

