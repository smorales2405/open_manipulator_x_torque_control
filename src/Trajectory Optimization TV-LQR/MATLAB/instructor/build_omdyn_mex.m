%% build_omdyn_mex.m
% Compila OMDyn.m y open_manx_fkin.m de ESTA carpeta a MEX (MATLAB Coder).
%
% Motivacion (Lab 5): fmincon calcula los gradientes por diferencias finitas
% densas — con N=40 nodos son ~481 evaluaciones de restr/Jcosto por iteracion,
% es decir ~77,000 llamadas a OMDyn por iteracion. OMDyn.m interpretado toma
% ~80 us/llamada; compilado a MEX baja a ~1-2 us, reduciendo el tiempo por
% iteracion de ~6-12 s a <1 s (aun mas combinado con USE_PARALLEL).
%
% El binario OMDyn.<mexext> tiene PRECEDENCIA sobre OMDyn.m en la misma
% carpeta: lab5_act1_sol.m / lab5_act2_sol.m lo usan automaticamente sin
% ningun cambio de codigo. Lo mismo aplica a open_manx_fkin.
%
% Uso:
%   >> cd <esta carpeta>
%   >> build_omdyn_mex
%
% Requisitos: MATLAB Coder + compilador C configurado (mex -setup).
%
% IMPORTANTE: si se regenera OMDyn.m (test/gen_OMDyn.m), volver a ejecutar
% este script — de lo contrario el MEX viejo seguiria teniendo precedencia
% sobre el nuevo .m.

this_dir = fileparts(mfilename('fullpath'));
old_dir  = cd(this_dir);
restore  = onCleanup(@() cd(old_dir));

assert(~isempty(which('codegen')), ...
    'build_omdyn_mex: requiere MATLAB Coder (funcion codegen no encontrada).');

%% ── 1. Eliminar MEX previos (la referencia se muestrea con la version .m) ──
for name = {'OMDyn', 'open_manx_fkin'}
    mex_path = fullfile(this_dir, [name{1} '.' mexext]);
    if isfile(mex_path), delete(mex_path); end
end
clear OMDyn open_manx_fkin
rehash;   % re-escanear la carpeta: sin esto MATLAB resuelve al MEX borrado

%% ── 2. Muestras de referencia con la version .m ─────────────────────────────
rng(0);
n_test = 200;
Q  = (rand(4, n_test) - 0.5) * 2 * pi;
DQ = (rand(4, n_test) - 0.5) * 10;

M_ref  = zeros(4, 4, n_test);
ph_ref = zeros(4, n_test);
y_ref  = zeros(4, n_test);
for i = 1:n_test
    [M_ref(:,:,i), ph] = OMDyn(Q(:,i), DQ(:,i));
    ph_ref(:,i) = ph(:);
    y_ref(:,i)  = open_manx_fkin(Q(:,i));
end

t_m_omdyn = timeit(@() OMDyn(Q(:,1), DQ(:,1)));
t_m_fkin  = timeit(@() open_manx_fkin(Q(:,1)));

%% ── 3. Compilar ──────────────────────────────────────────────────────────────
fprintf('Compilando OMDyn.m -> OMDyn.%s ...\n', mexext);
codegen('OMDyn', '-args', {zeros(4,1), zeros(4,1)}, ...
        '-o', fullfile(this_dir, 'OMDyn'));

fprintf('Compilando open_manx_fkin.m -> open_manx_fkin.%s ...\n', mexext);
codegen('open_manx_fkin', '-args', {zeros(4,1)}, ...
        '-o', fullfile(this_dir, 'open_manx_fkin'));

clear OMDyn open_manx_fkin   % recargar: ahora resuelven al MEX
rehash;

assert(endsWith(which('OMDyn'),          mexext), 'OMDyn MEX no quedo activo.');
assert(endsWith(which('open_manx_fkin'), mexext), 'open_manx_fkin MEX no quedo activo.');

%% ── 4. Verificacion MEX vs .m ────────────────────────────────────────────────
err_M = 0;  err_ph = 0;  err_y = 0;
for i = 1:n_test
    [Mi, phi_i] = OMDyn(Q(:,i), DQ(:,i));
    yi          = open_manx_fkin(Q(:,i));
    err_M  = max(err_M,  max(abs(Mi(:)    - reshape(M_ref(:,:,i), [], 1))));
    err_ph = max(err_ph, max(abs(phi_i(:) - ph_ref(:,i))));
    err_y  = max(err_y,  max(abs(yi(:)    - y_ref(:,i))));
end
assert(max([err_M, err_ph, err_y]) < 1e-10, ...
    'Verificacion MEX vs .m fallo: err_M=%.2e err_ph=%.2e err_y=%.2e', ...
    err_M, err_ph, err_y);

t_mex_omdyn = timeit(@() OMDyn(Q(:,1), DQ(:,1)));
t_mex_fkin  = timeit(@() open_manx_fkin(Q(:,1)));

%% ── 5. Resumen ───────────────────────────────────────────────────────────────
fprintf('\n=================================================================\n');
fprintf('  build_omdyn_mex — LISTO (verificado en %d puntos aleatorios)\n', n_test);
fprintf('=================================================================\n');
fprintf('  Errores max MEX vs .m : M=%.1e  phib=%.1e  fkin=%.1e\n', err_M, err_ph, err_y);
fprintf('  OMDyn                 : %7.1f us (.m)  ->  %6.2f us (MEX)   x%.0f\n', ...
        1e6*t_m_omdyn, 1e6*t_mex_omdyn, t_m_omdyn/t_mex_omdyn);
fprintf('  open_manx_fkin        : %7.1f us (.m)  ->  %6.2f us (MEX)   x%.0f\n', ...
        1e6*t_m_fkin, 1e6*t_mex_fkin, t_m_fkin/t_mex_fkin);
fprintf('  Binarios en: %s\n', this_dir);
fprintf('  (recuerda re-ejecutar este script si regeneras OMDyn.m)\n');
fprintf('=================================================================\n');
