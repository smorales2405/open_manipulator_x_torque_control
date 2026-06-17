% ============================================================================
%  fk_cart_ref.m
%
%  Calcula y visualiza la trayectoria cartesiana de referencia y(t) = [x,y,z,phi]
%  y sus derivadas analiticas (ydot, yddot) a partir de trayectorias articulares
%  de la forma:
%
%       qi(t) = A(i) + B(i) * sin(w*t),   i = 1..4
%
%  Metodo: diferenciacion simbolica de la FK (open_manx_fkin.m).
%
%  Salidas:
%    - Consola : expresiones analiticas de posicion y velocidad, pose inicial
%    - Figura 1: trayectorias articulares q1..q4
%    - Figura 2: posicion | velocidad | aceleracion cartesianas (4x3 subplots)
%    - Figura 3: trayectoria 3D del efector final en espacio de trabajo
%
%  Requiere: open_manx_fkin.m en el mismo directorio o en el path de MATLAB.
%
%  OpenManipulator-X — Lab 6 SMC
% ============================================================================

clear; clc; close all;

% ── PARAMETROS DE ENTRADA ─────────────────────────────────────────────────────
%   qi(t) = A(i) + B(i)*sin(w*t)
%   Modificar A, B y w para explorar distintas trayectorias articulares.

%             q1       q2      q3      q4
A = [    0,  -0.45,   0.35,  pi/4];   % offsets   [rad]
B = [ pi/4,   0.50,  -0.50,  0.25];   % amplitudes [rad]
w = 1.0;                               % frecuencia angular [rad/s]

t_end = 2 * (2*pi / w);               % duracion: 2 periodos completos [s]
N_pts = 2000;                          % puntos de evaluacion numerica
% ─────────────────────────────────────────────────────────────────────────────

% ── CALCULO SIMBOLICO ────────────────────────────────────────────────────────
fprintf('Calculando expresiones simbolicas...\n');
syms t real

% Trayectoria articular simbolica (4x1)
q_sym = sym(A(:)) + sym(B(:)) .* sin(w * t);

% FK simbolica — open_manx_fkin acepta sym directamente (cos/sin/+/* son sym-compatible)
y_sym     = open_manx_fkin(q_sym);     % [x; y; z; phi]  (4x1 sym)
ydot_sym  = diff(y_sym,     t);         % velocidad cartesiana
yddot_sym = diff(ydot_sym,  t);         % aceleracion cartesiana

% Simplificar para visualizacion en consola
y_simp     = simplify(y_sym,     'Steps', 30);
ydot_simp  = simplify(ydot_sym,  'Steps', 30);
yddot_simp = simplify(yddot_sym, 'Steps', 30);

% ── CONSOLA: expresiones y pose inicial ──────────────────────────────────────
ylabels = {'x', 'y', 'z', 'phi'};
yunits  = {'m', 'm', 'm', 'rad'};

fprintf('\n=== Trayectoria articular de referencia ===\n');
fprintf('  qi(t) = A(i) + B(i)*sin(%.2f*t)\n\n', w);
fprintf('  i |   A [rad]    B [rad]  | A [deg]  B [deg]\n');
fprintf('  --|-------------------------------------------\n');
for i = 1:4
    fprintf('  %d |  %7.4f   %7.4f  | %7.3f  %7.3f\n', ...
        i, A(i), B(i), rad2deg(A(i)), rad2deg(B(i)));
end

fprintf('\n=== Expresiones analiticas (simplificadas) ===\n');
for i = 1:4
    fprintf('\n  [%s]  [%s]\n', ylabels{i}, yunits{i});
    fprintf('    pos  : %s\n',  char(y_simp(i)));
    fprintf('    vel  : %s\n',  char(ydot_simp(i)));
    fprintf('    acel : %s\n',  char(yddot_simp(i)));
end

y0 = double(subs(y_sym, t, 0));
fprintf('\n=== Pose inicial FK(q(0)) ===\n');
for i = 1:4
    fprintf('  %s(0) = %+.4f %s\n', ylabels{i}, y0(i), yunits{i});
end

% ── EVALUACION NUMERICA (matlabFunction para eficiencia) ─────────────────────
y_fun     = matlabFunction(y_sym,     'Vars', {t});
ydot_fun  = matlabFunction(ydot_sym,  'Vars', {t});
yddot_fun = matlabFunction(yddot_sym, 'Vars', {t});

t_vec    = linspace(0, t_end, N_pts);
y_num    = y_fun(t_vec);       % 4 x N
ydot_num = ydot_fun(t_vec);
yddot_num = yddot_fun(t_vec);

% Trayectoria articular numerica
q_num    = A(:) + B(:) .* sin(w * t_vec);   % 4 x N

% ── FIGURA 1: Trayectorias articulares ───────────────────────────────────────
figure('Name', 'Trayectorias articulares', 'NumberTitle', 'off', ...
       'Position', [50 500 900 400]);

qlabels = {'q_1', 'q_2', 'q_3', 'q_4'};
colors  = {'b', 'r', [0 0.6 0], [0.8 0.4 0]};

for i = 1:4
    subplot(2, 2, i);
    plot(t_vec, rad2deg(q_num(i,:)), 'Color', colors{i}, 'LineWidth', 1.5);
    xlabel('t  [s]');
    ylabel([qlabels{i} '  [°]']);
    A_deg = rad2deg(A(i));
    B_deg = rad2deg(B(i));
    if B_deg >= 0
        title(sprintf('%s(t) = %.1f° + %.1f°·sin(%.1ft)', qlabels{i}, A_deg, B_deg, w));
    else
        title(sprintf('%s(t) = %.1f° - %.1f°·sin(%.1ft)', qlabels{i}, A_deg, abs(B_deg), w));
    end
    grid on; box on;
    xlim([0 t_end]);
end
sgtitle('Trayectorias articulares de referencia', 'FontSize', 12, 'FontWeight', 'bold');

% ── FIGURA 2: Trayectorias cartesianas (posicion | velocidad | aceleracion) ──
y_ylabels    = {'x_d  [m]',   'y_d  [m]',   'z_d  [m]',   '\phi_d  [rad]'};
yd_ylabels   = {'\dot{x}_d  [m/s]',  '\dot{y}_d  [m/s]',  '\dot{z}_d  [m/s]',  '\dot{\phi}_d  [rad/s]'};
ydd_ylabels  = {'\ddot{x}_d  [m/s²]', '\ddot{y}_d  [m/s²]', '\ddot{z}_d  [m/s²]', '\ddot{\phi}_d  [rad/s²]'};
col_colors   = {[0 0.45 0.74], [0.85 0.33 0.10], [0.49 0.18 0.56]};  % azul, naranja, morado

figure('Name', 'Trayectorias cartesianas de referencia', 'NumberTitle', 'off', ...
       'Position', [80 50 1100 700]);

for i = 1:4
    % Posicion
    subplot(4, 3, 3*(i-1)+1);
    plot(t_vec, y_num(i,:), 'Color', col_colors{1}, 'LineWidth', 1.5);
    xlabel('t  [s]'); ylabel(y_ylabels{i});
    title(y_ylabels{i});
    grid on; box on; xlim([0 t_end]);

    % Velocidad
    subplot(4, 3, 3*(i-1)+2);
    plot(t_vec, ydot_num(i,:), 'Color', col_colors{2}, 'LineWidth', 1.5);
    xlabel('t  [s]'); ylabel(yd_ylabels{i});
    title(yd_ylabels{i});
    grid on; box on; xlim([0 t_end]);

    % Aceleracion
    subplot(4, 3, 3*(i-1)+3);
    plot(t_vec, yddot_num(i,:), 'Color', col_colors{3}, 'LineWidth', 1.5);
    xlabel('t  [s]'); ylabel(ydd_ylabels{i});
    title(ydd_ylabels{i});
    grid on; box on; xlim([0 t_end]);
end
sgtitle('Trayectoria cartesiana de referencia — posicion | velocidad | aceleracion', ...
        'FontSize', 12, 'FontWeight', 'bold');

% ── FIGURA 3: Trayectoria 3D en espacio de trabajo ───────────────────────────
figure('Name', 'Trayectoria 3D en espacio de trabajo', 'NumberTitle', 'off', ...
       'Position', [1000 300 550 480]);

% Colorear por tiempo
surface([y_num(1,:); y_num(1,:)], ...
        [y_num(2,:); y_num(2,:)], ...
        [y_num(3,:); y_num(3,:)], ...
        [t_vec; t_vec], ...
        'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', 2);
colormap(parula); cb = colorbar; cb.Label.String = 't  [s]';
hold on;

% Inicio y fin
plot3(y_num(1,1),   y_num(2,1),   y_num(3,1),   'go', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Inicio');
plot3(y_num(1,end), y_num(2,end), y_num(3,end), 'rs', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Fin');

xlabel('x  [m]'); ylabel('y  [m]'); zlabel('z  [m]');
title('Trayectoria 3D del efector final');
legend('Location', 'best'); grid on; box on;
view(40, 25);

fprintf('\nListo. Se generaron 3 figuras.\n');
