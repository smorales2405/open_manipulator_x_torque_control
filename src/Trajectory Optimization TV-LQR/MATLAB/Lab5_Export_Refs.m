%% Lab5_Export_Refs.m
% Exporta las referencias TV-LQR a archivos .txt para los nodos ROS 2.
%
% PREREQUISITO: ejecutar lab5_act2_sol.m (o lab5_act1_sol.m) primero.
% Las siguientes variables deben existir en el workspace:
%   N, Ts, nx, nu        — parametros del problema
%   x0, yf               — condiciones de frontera
%   Xref   (nx x N)      — trayectoria de estados optima
%   Uref   (nu x N)      — entradas optimas
%   K_TV   (nu x nx x N) — ganancias TV-LQR
%
% ARCHIVOS GENERADOS en references/:
%   time_ref.txt     N x 1    instantes de muestreo [s]
%   q_ref.txt        N x 4    posiciones articulares de referencia [rad]
%   dq_ref.txt       N x 4    velocidades articulares de referencia [rad/s]
%   u_ref.txt        N x 4    entradas optimizadas (torques) [N.m]
%   K_TV.txt         N x 32   ganancias TV-LQR (reshape col-major de K_k 4x8)
%   tau_gravity.txt  1 x 4    torque gravitatorio en x0 [N.m]

clc;

ref_dir = '../references';

%% ========================================================================
%  1. Validar variables del workspace
%  ========================================================================
required = {'N','Ts','nx','nu','x0','yf','Xref','Uref','K_TV'};
missing  = {};
for i = 1:numel(required)
    if ~exist(required{i}, 'var')
        missing{end+1} = required{i}; 
    end
end
if ~isempty(missing)
    error(['Lab5_Export_Refs: variables no encontradas en el workspace: %s\n' ...
           'Ejecutar lab5_act2_sol.m (o lab5_act1_sol.m) primero.'], ...
          strjoin(missing, ', '));
end
fprintf('Workspace OK: N=%d  Ts=%.4f s  (tiempo total: %.2f s)\n', N, Ts, N*Ts);

%% ========================================================================
%  2. Verificar dependencias
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
%  3. Validaciones previas a la exportacion
%  ========================================================================

% 3a. Restriccion de torque
assert(max(abs(Uref(:))) <= 1 + 1e-4, ...
    'max|Uref| = %.4f supera el limite de torque (1 N.m).', max(abs(Uref(:))));

% 3b. Limites articulares
q_lower = [-3/4*pi; -11/18*pi; -11/18*pi; -5/9*pi];
q_upper = [ 3/4*pi;   5/9*pi;     pi/2; 23/36*pi];
for k = 1:N
    for i = 1:4
        assert(Xref(i,k) >= q_lower(i)-1e-3 && Xref(i,k) <= q_upper(i)+1e-3, ...
            'q%d en k=%d fuera de limites: %.4f rad.', i, k, Xref(i,k));
    end
end
fprintf('Limites articulares: OK\n');

% 3c. K_TV valido
assert(all(isfinite(K_TV(:))), 'K_TV contiene NaN/Inf.');
fprintf('K_TV: OK\n');

% 3d. Trayectoria cartesiana finita
yref = zeros(4, N+1);
yref(:,1) = open_manx_fkin(x0(1:4));
for i = 1:N
    yref(:,i+1) = open_manx_fkin(Xref(1:4,i));
end
assert(all(isfinite(yref(:))), 'yref contiene NaN/Inf.');

%% ========================================================================
%  4. Exportacion de archivos de referencia
%  ========================================================================
if ~exist(ref_dir, 'dir'), mkdir(ref_dir); end

t_ref  = (Ts:Ts:N*Ts)';    % N x 1
q_ref  = Xref(1:4,:)';     % N x 4
dq_ref = Xref(5:8,:)';     % N x 4
u_ref  = Uref';             % N x 4

[~, tau_grav_vec] = OMDyn(x0(1:4), zeros(4,1));
tau_grav = tau_grav_vec(:)';   % 1 x 4

K_TV_vec = zeros(N, nu*nx);
for k = 1:N
    K_TV_vec(k,:) = reshape(K_TV(:,:,k), 1, []);
end

writematrix(t_ref,    fullfile(ref_dir, 'time_ref.txt'),    'Delimiter', ' ');
writematrix(q_ref,    fullfile(ref_dir, 'q_ref.txt'),       'Delimiter', ' ');
writematrix(dq_ref,   fullfile(ref_dir, 'dq_ref.txt'),      'Delimiter', ' ');
writematrix(u_ref,    fullfile(ref_dir, 'u_ref.txt'),       'Delimiter', ' ');
writematrix(K_TV_vec, fullfile(ref_dir, 'K_TV.txt'),        'Delimiter', ' ');
writematrix(tau_grav, fullfile(ref_dir, 'tau_gravity.txt'), 'Delimiter', ' ');

fprintf('tau_gravity en x0: [%.4f %.4f %.4f %.4f] N.m\n', tau_grav);

%% ========================================================================
%  5. Verificacion de archivos generados
%  ========================================================================
files_out = {'time_ref.txt','q_ref.txt','dq_ref.txt','u_ref.txt','K_TV.txt','tau_gravity.txt'};
for f = files_out
    fpath = fullfile(ref_dir, f{1});
    assert(exist(fpath, 'file') == 2, 'No se creo: %s', fpath);
end

%% ========================================================================
%  6. Resumen
%  ========================================================================
fprintf('\n======================================================\n');
fprintf('Lab5_Export_Refs — Resumen de exportacion\n');
fprintf('======================================================\n');
fprintf('  N                     : %d\n',   N);
fprintf('  Ts                    : %.4f s\n', Ts);
fprintf('  Tiempo total          : %.2f s\n', N*Ts);
fprintf('  max|Uref| [N.m]       : [%.4f %.4f %.4f %.4f]\n', max(abs(Uref),[],2)');
fprintf('  Carpeta de salida     : %s\n', fullfile(pwd, ref_dir));
fprintf('\n  Archivos exportados:\n');
for f = files_out
    info = dir(fullfile(ref_dir, f{1}));
    fprintf('    %-20s  %.1f KB\n', f{1}, info.bytes/1024);
end
fprintf('\nListo. Los archivos pueden ser leidos por los nodos ROS 2.\n');
