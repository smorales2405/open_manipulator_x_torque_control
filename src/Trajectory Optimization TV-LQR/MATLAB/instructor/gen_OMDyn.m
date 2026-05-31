%% gen_OMDyn.m
% Regenera OMDyn.m a partir de los parametros cinematicos/inerciales de
% openmani.urdf usando derivacion simbolica (Lagrangiana).
%
% Requiere: MATLAB Symbolic Math Toolbox
% Tiempo estimado: 15-35 minutos (sin llamadas a simplify)
%
% Salida:
%   OMDyn_original.m  — copia de seguridad del OMDyn.m existente
%   OMDyn.m           — nueva version consistente con openmani.urdf
%
% Parametros tomados de openmani.urdf:
%   - Masas link1..5, centros de masa, tensores de inercia en frame del link
%   - Origenes y ejes de joint1..4

clear; clc;
fprintf('=================================================================\n');
fprintf('  gen_OMDyn.m  —  Generacion simbolica de dinamica del robot\n');
fprintf('=================================================================\n\n');
t0 = tic;

%% ========================================================================
%  1. Parametros del URDF  (openmani.urdf)
%  ========================================================================

% Masas [kg]  — link1 es la base fija, no contribuye a la dinamica
m = [9.8406837e-02;   % link2
     1.3850917e-01;   % link3
     1.3274562e-01;   % link4
     1.4327573e-01];  % link5

% Centros de masa en el frame del link correspondiente [m]
rc = [ -3.0184870e-04,  5.4043684e-04, 4.7433464e-02;   % link2
        1.0308393e-02,  3.7743363e-04, 1.0170197e-01;   % link3
        9.0909590e-02,  3.8929816e-04, 2.2413279e-04;   % link4
        4.4206755e-02,  3.6839985e-07, 8.9142216e-03];  % link5

% Tensores de inercia en el CoM, en el frame del link [kg·m^2]
%   orden: Ixx, Iyy, Izz, Ixy, Ixz, Iyz
Ic = {[ 3.4543422e-05, -1.6031095e-08, -3.8375155e-07;
       -1.6031095e-08,  3.2689329e-05,  2.8511935e-08;
       -3.8375155e-07,  2.8511935e-08,  1.8850320e-05], ...  % link2
      [ 3.3055381e-04, -9.7940978e-08, -3.8505711e-05;
       -9.7940978e-08,  3.4290447e-04, -1.5717516e-06;
       -3.8505711e-05, -1.5717516e-06,  6.0346498e-05], ...  % link3
      [ 3.0654178e-05, -1.2764155e-06, -2.6874417e-07;
       -1.2764155e-06,  2.4230292e-04,  1.1559550e-08;
       -2.6874417e-07,  1.1559550e-08,  2.5155057e-04], ...  % link4
      [ 8.0870749e-05,  0.0,           -1.0157896e-06;
        0.0,            7.5980465e-05,  0.0;
       -1.0157896e-06,  0.0,            9.3127351e-05]};      % link5

% Origenes de los joints en el frame del link padre [m]
pj = [0.012, 0.0,   0.017;    % joint1 en link1
      0.0,   0.0,   0.0595;   % joint2 en link2
      0.024, 0.0,   0.128;    % joint3 en link3
      0.124, 0.0,   0.0];     % joint4 en link4

% Ejes de los joints en el frame del link hijo
%   joint1: Z = [0,0,1]
%   joint2,3,4: Y = [0,1,0]
axes = {[0;0;1], [0;1;0], [0;1;0], [0;1;0]};

% Gravedad en el frame world [m/s^2]  (Z hacia arriba)
grav = [0; 0; -9.81];

n = 4;   % numero de joints / GDL

%% ========================================================================
%  2. Variables simbolicas
%  ========================================================================

syms q1 q2 q3 q4 real
syms dq1 dq2 dq3 dq4 real

q_sym  = [q1; q2; q3; q4];
dq_sym = [dq1; dq2; dq3; dq4];

% Matrices de rotacion elementales
Rz = @(a) [cos(a), -sin(a), 0;  sin(a), cos(a), 0;  0, 0, 1];
Ry = @(a) [cos(a), 0, sin(a);   0, 1, 0;  -sin(a), 0, cos(a)];

% Transformacion homogenea: rotacion R + traslacion p
Thom = @(R, p) [R, p(:); 0 0 0 1];

%% ========================================================================
%  3. Transformaciones homogeneas world → frame de cada link
%  ========================================================================

fprintf('[%.0fs] Calculando transformaciones homogeneas...\n', toc(t0));

% T{k}: transformacion del world al frame de link_{k+1}
%   (link indices en URDF: link2=index1, link3=index2, link4=3, link5=4)
%
% Convenio URDF:  T_world_child = T_world_parent * Trans(pj) * Rot(axis, qk)
%
%   T_world_link1 = I  (link1 = base = world frame)
%   T_world_link2 = Trans(pj1) * Rz(q1)
%   T_world_link3 = T_world_link2 * Trans(pj2) * Ry(q2)
%   etc.

% Matrices de rotacion de cada joint en world
Rjoint = {Rz(q1), Ry(q2), Ry(q3), Ry(q4)};

T = cell(n, 1);
T_prev = eye(4);
for k = 1:n
    Trans_k = Thom(eye(3), pj(k,:)');   % traslacion al origin del joint k
    Rot_k   = Thom(Rjoint{k}, [0;0;0]); % rotacion por q_k
    T{k}    = T_prev * Trans_k * Rot_k;
    T_prev  = T{k};
end

% Matrices de rotacion de cada link frame respecto al world
R_link = cellfun(@(Tk) Tk(1:3,1:3), T, 'UniformOutput', false);

% Posicion del origen de cada link en world frame
p_origin = cellfun(@(Tk) Tk(1:3,4), T, 'UniformOutput', false);

fprintf('[%.0fs] Transformaciones calculadas.\n', toc(t0));

%% ========================================================================
%  4. Posiciones CoM y origenes/ejes de joints en world frame
%  ========================================================================

% Posicion del CoM del link i en world frame
pc = cell(n, 1);
for k = 1:n
    pc{k} = R_link{k} * rc(k,:)' + p_origin{k};
end

% Posicion del origen del joint k en world frame
po = cell(n, 1);
for k = 1:n
    if k == 1
        po{k} = pj(1,:)';  % joint1 origin = Trans(pj1) en link1 frame = world frame
    else
        po{k} = R_link{k-1} * pj(k,:)' + p_origin{k-1};
    end
end

% Eje del joint k expresado en world frame
z = cell(n, 1);
for k = 1:n
    if k == 1
        z{k} = axes{k};   % joint1 en world frame (link1 = world)
    else
        z{k} = R_link{k-1} * axes{k};
    end
end

fprintf('[%.0fs] CoM y ejes de joints calculados.\n', toc(t0));

%% ========================================================================
%  5. Jacobianos traslacional y rotacional por link
%  ========================================================================

% Jv{k} (3×n): Jacobiano traslacional al CoM del link k+1
%   columna j = z{j} × (pc{k} - po{j})  si j <= k  (joint j afecta link k+1)
%             = 0                          si j > k
%
% Jw{k} (3×n): Jacobiano rotacional del link k+1
%   columna j = z{j}  si j <= k
%             = 0     si j > k

fprintf('[%.0fs] Calculando Jacobianos simbolicos...\n', toc(t0));

Jv = cell(n, 1);
Jw = cell(n, 1);

for lnk = 1:n
    Jv_k = sym(zeros(3, n));
    Jw_k = sym(zeros(3, n));
    for jnt = 1:lnk
        d = pc{lnk} - po{jnt};
        Jv_k(:, jnt) = cross(z{jnt}, d);
        Jw_k(:, jnt) = z{jnt};
    end
    Jv{lnk} = Jv_k;
    Jw{lnk} = Jw_k;
end

fprintf('[%.0fs] Jacobianos calculados.\n', toc(t0));

%% ========================================================================
%  6. Matriz de inercia M(q)
%      M = sum_k [ m_k * Jv_k^T * Jv_k  +  Jw_k^T * Ic_k_world * Jw_k ]
%  ========================================================================

fprintf('[%.0fs] Calculando matriz de inercia M(q)...\n', toc(t0));

M_sym = sym(zeros(n, n));

for lnk = 1:n
    mi   = m(lnk);
    Ri   = R_link{lnk};
    Ici_world = Ri * Ic{lnk} * Ri';   % tensor en frame world

    M_sym = M_sym ...
          + mi * (Jv{lnk}' * Jv{lnk}) ...
          + Jw{lnk}' * Ici_world * Jw{lnk};
end

% Simetrizar para eliminar errores numericos simbolicos
M_sym = (M_sym + M_sym') / 2;

fprintf('[%.0fs] M(q) calculada.\n', toc(t0));

%% ========================================================================
%  7. Energia potencial V(q)
%      V = sum_k m_k * 9.81 * z_ck   (z hacia arriba)
%        = -sum_k m_k * grav' * pc_k
%  ========================================================================

V_sym = sym(0);
for lnk = 1:n
    V_sym = V_sym - m(lnk) * (grav' * pc{lnk});
end

fprintf('[%.0fs] Energia potencial calculada.\n', toc(t0));

%% ========================================================================
%  8. Derivadas parciales de M respecto a cada q_k  (pre-computadas)
%     Se usan para calcular phib via formula de Lagrange sin simplify.
%  ========================================================================

fprintf('[%.0fs] Pre-calculando derivadas de M (40 operaciones)...\n', toc(t0));

% dM{i,j,l} = diff(M_sym(i,j), q_sym(l))
% Aprovecha simetria: solo calcula upper triangular
dM = cell(n, n, n);
for ii = 1:n
    for jj = ii:n
        for ll = 1:n
            dM{ii,jj,ll} = diff(M_sym(ii,jj), q_sym(ll));
            dM{jj,ii,ll} = dM{ii,jj,ll};   % simetria de M
        end
    end
    if mod(ii*n, n*n/2) == 0
        fprintf('  [%.0fs] %d/%d filas de derivadas completadas\n', toc(t0), ii, n);
    end
end

fprintf('[%.0fs] Derivadas de M calculadas.\n', toc(t0));

%% ========================================================================
%  9. Vector de bias  phib(q, dq) = C(q,dq)*dq + g(q)
%
%     Formula de Lagrange (sin Christoffel explicitos):
%
%       phib_k = sum_{j,l} [dM_kj/dq_l * dq_j * dq_l]
%              - (1/2) * sum_{i,j} [dM_ij/dq_k * dq_i * dq_j]
%              + dV/dq_k
%
%     Equivalente a los simbolos de Christoffel pero mas directo.
%  ========================================================================

fprintf('[%.0fs] Calculando phib(q,dq) via formula de Lagrange...\n', toc(t0));

phib_sym = sym(zeros(n, 1));

for k = 1:n
    % Termino Coriolis/Centrifugo: sum_{j,l} dM_{kj}/dq_l * dq_j * dq_l
    corio = sym(0);
    for jj = 1:n
        for ll = 1:n
            corio = corio + dM{k,jj,ll} * dq_sym(jj) * dq_sym(ll);
        end
    end

    % Termino centrifugo complementario: -(1/2) sum_{i,j} dM_{ij}/dq_k * dq_i * dq_j
    centri = sym(0);
    for ii = 1:n
        for jj = 1:n
            centri = centri + dM{ii,jj,k} * dq_sym(ii) * dq_sym(jj);
        end
    end

    % Termino gravitacional
    grav_k = diff(V_sym, q_sym(k));

    phib_sym(k) = corio - sym(1)/sym(2) * centri + grav_k;

    fprintf('  [%.0fs] phib(%d) calculado\n', toc(t0), k);
end

fprintf('[%.0fs] phib calculado.\n', toc(t0));

%% ========================================================================
%  10. Generar OMDyn.m con matlabFunction  (Optimize=true aplica CSE)
%  ========================================================================

script_dir  = fileparts(mfilename('fullpath'));
orig_file   = fullfile(script_dir, 'OMDyn.m');
backup_file = fullfile(script_dir, 'OMDyn_original.m');
output_file = fullfile(script_dir, 'OMDyn.m');

% Copia de seguridad del OMDyn.m original
if isfile(orig_file)
    copyfile(orig_file, backup_file);
    fprintf('\nCopia de seguridad guardada en: OMDyn_original.m\n');
end

fprintf('[%.0fs] Generando OMDyn.m con matlabFunction (Optimize=true)...\n', toc(t0));

matlabFunction(M_sym, phib_sym, ...
    'File',     output_file, ...
    'Vars',     {q_sym, dq_sym}, ...
    'Outputs',  {'M', 'phib'}, ...
    'Optimize', true, ...
    'Comments', 'Generado automaticamente por gen_OMDyn.m desde openmani.urdf');

fprintf('\n=================================================================\n');
fprintf('  LISTO: OMDyn.m generado en %.1f minutos\n', toc(t0)/60);
fprintf('  Copia de seguridad: OMDyn_original.m\n');
fprintf('  Verifica con: validate_omdyn_vs_urdf.m\n');
fprintf('=================================================================\n');
