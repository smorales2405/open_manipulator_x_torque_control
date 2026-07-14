%% gen_OMDyn.m
% Regenera OMDyn.m a partir de los parametros cinematicos/inerciales de
% open_manipulator_x.urdf usando derivacion simbolica (Lagrangiana).
%
% Requiere: MATLAB Symbolic Math Toolbox
% Tiempo estimado: 15-35 minutos (sin llamadas a simplify)
%
% Salida:
%   OMDyn_prev.m  — copia de seguridad del OMDyn.m existente en esta carpeta
%   OMDyn.m       — nueva version consistente con open_manipulator_x.urdf
%
% Parametros tomados VERBATIM de los bloques <inertial> de
% urdf/open_manipulator_x.urdf (masas pesadas en balanza, inercias del CAD
% escaladas — la misma referencia que usan Gazebo y los nodos hw/gz via
% Pinocchio):
%   - Masas link2..5, centros de masa y tensores de inercia (frame del link)
%   - Origenes y ejes de joint1..4
% Los links del gripper (gripper_link, gripper_link_sub, end_effector_link)
% son placeholders de 1 g en el URDF; se lumpean rigidamente en link5 para
% coincidir con el modelo reducido de Pinocchio/Gazebo (gripper bloqueado).
%
% Tras regenerar: copiar OMDyn.m a ../instructor/ y ../student/, y
% recompilar el MEX con ../instructor/build_omdyn_mex.m.

clear; clc;
fprintf('=================================================================\n');
fprintf('  gen_OMDyn.m  —  Generacion simbolica de dinamica del robot\n');
fprintf('=================================================================\n\n');
t0 = tic;

%% ========================================================================
%  1. Parametros del URDF  (open_manipulator_x.urdf)
%  ========================================================================

% Masas [kg]  — link1 es la base fija, no contribuye a la dinamica
% Fuente: bloques <inertial> de urdf/open_manipulator_x.urdf (verbatim)
m = [0.0970;    % link2
     0.1235;    % link3
     0.1151;    % link4
     0.2206];   % link5

% Centros de masa en el frame del link correspondiente [m]
% Fuente: <inertial><origin xyz> del URDF (verbatim)
rc = [ 0.000000, 0.000657, 0.044319;    % link2
       0.007606, 0.000389, 0.099924;    % link3
       0.089168, 0.000437, 0.000001;    % link4
       0.062451, 0.000000, 0.005878];   % link5

% Tensores de inercia en el CoM, en el frame del link [kg·m^2]
%   I = [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz]
% Fuente: <inertial><inertia> del URDF (verbatim)
Ic = {[ 3.078317e-05,  0.0,            0.0;
        0.0,            2.670366e-05,  -1.289659e-07;
        0.0,           -1.289659e-07,   1.627175e-05], ...  % link2
      [ 1.979680e-04, -2.511443e-07, -2.497601e-05;
       -2.511443e-07,  2.046627e-04, -1.351040e-06;
       -2.497601e-05, -1.351040e-06,  3.374865e-05], ...    % link3
      [ 1.930305e-05, -1.189078e-06,  3.461654e-09;
       -1.189078e-06,  1.483284e-04,  0.0;
        3.461654e-09,  0.0,           1.547550e-04], ...    % link4
      [ 1.344328e-04, -1.788649e-08,  2.259163e-05;
       -1.788649e-08,  1.892619e-04,  9.936937e-07;
        2.259163e-05,  9.936937e-07,  2.174510e-04]};       % link5

% ── Lumping de los links del gripper en link5 ────────────────────────────
% El URDF une gripper_link, gripper_link_sub y end_effector_link a link5
% (placeholders de 1 g, inercia 1e-6). Gazebo y los nodos hw/gz (Pinocchio
% con gripper bloqueado) los simulan como parte del cuerpo distal, asi que
% se combinan aqui en link5 con el teorema de ejes paralelos.
m_g = [1.0e-3; 1.0e-3; 1.0e-3];        % masas [kg]
p_g = [0.0817,  0.021, 0;              % gripper_link      (frame link5, q_grip=0)
       0.0817, -0.021, 0;              % gripper_link_sub
       0.126,   0.0,   0];             % end_effector_link
I_g = 1.0e-6 * eye(3);                 % inercia propia de cada placeholder

paxis = @(d) dot(d,d)*eye(3) - d(:)*d(:)';   % matriz de ejes paralelos

m5_new  = m(4) + sum(m_g);
rc5_new = (m(4)*rc(4,:) + m_g'*p_g) / m5_new;
Ic5_new = Ic{4} + m(4)*paxis(rc(4,:) - rc5_new);
for gg = 1:numel(m_g)
    Ic5_new = Ic5_new + I_g + m_g(gg)*paxis(p_g(gg,:) - rc5_new);
end
m(4)    = m5_new;
rc(4,:) = rc5_new;
Ic{4}   = Ic5_new;
fprintf('link5 con gripper lumpeado: m=%.4f kg  rc=[%.6f %.6f %.6f] m\n\n', ...
        m(4), rc(4,:));

% Origenes de los joints en el frame del link padre [m]
% Nota: identicos a openmani.urdf (joint1 Z=0.017 conservado en omx.urdf)
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
backup_file = fullfile(script_dir, 'OMDyn_prev.m');   % backup de la version previa
output_file = fullfile(script_dir, 'OMDyn.m');

% Copia de seguridad del OMDyn.m existente en esta carpeta
if isfile(orig_file)
    copyfile(orig_file, backup_file);
    fprintf('\nCopia de seguridad guardada en: OMDyn_prev.m\n');
end

fprintf('[%.0fs] Generando OMDyn.m con matlabFunction (Optimize=true)...\n', toc(t0));

matlabFunction(M_sym, phib_sym, ...
    'File',     output_file, ...
    'Vars',     {q_sym, dq_sym}, ...
    'Outputs',  {'M', 'phib'}, ...
    'Optimize', true, ...
    'Comments', 'Generado automaticamente por gen_OMDyn.m desde open_manipulator_x.urdf');

fprintf('\n=================================================================\n');
fprintf('  LISTO: OMDyn.m generado en %.1f minutos\n', toc(t0)/60);
fprintf('  Copia de seguridad: OMDyn_prev.m\n');
fprintf('  Verifica con: validate_omdyn_vs_urdf.m\n');
fprintf('  Luego: copiar OMDyn.m a ../instructor/ y ../student/, y\n');
fprintf('  recompilar el MEX con ../instructor/build_omdyn_mex.m\n');
fprintf('=================================================================\n');

% Los binarios MEX (build_omdyn_mex.m) tienen precedencia sobre OMDyn.m:
% si existe uno de una version anterior, avisar que quedaria desactualizado.
for d = {script_dir, fullfile(script_dir, '..', 'instructor'), ...
         fullfile(script_dir, '..', 'student')}
    stale_mex = fullfile(d{1}, ['OMDyn.' mexext]);
    if isfile(stale_mex)
        fprintf(['\nADVERTENCIA: existe %s\n' ...
                 '  El MEX tiene precedencia sobre OMDyn.m y quedaria DESACTUALIZADO.\n' ...
                 '  Tras copiar el nuevo OMDyn.m, recompilar con build_omdyn_mex.m ' ...
                 '(o borrar el MEX).\n'], stale_mex);
    end
end
