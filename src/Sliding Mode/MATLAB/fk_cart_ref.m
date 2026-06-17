% ============================================================================
%  fk_cart_ref.m
%
%  Calcula y visualiza la trayectoria cartesiana y(t) = [x,y,z,phi] y sus
%  derivadas analiticas (ydot, yddot) a partir de trayectorias articulares:
%
%       qi(t) = A(i) + B(i) * sin(w*t),   i = 1..4
%
%  Metodo para las expresiones en consola:
%    1. FK simbolica exacta → integracion adaptativa → coeficientes de Fourier
%    2. Redondeo a 3 decimales; terminos < 0.001 eliminados
%    3. ydot y yddot = derivadas ANALITICAS de la serie de posicion retenida
%       (garantiza consistencia exacta entre pos, vel y acel mostradas)
%
%  Las graficas usan la FK simbolica exacta (sin truncamiento de Fourier).
%
%  Requiere: open_manx_fkin.m en el mismo directorio o en el MATLAB path.
%  OpenManipulator-X — Lab 6 SMC
% ============================================================================

clear; clc; close all;

% ── PARAMETROS DE ENTRADA ─────────────────────────────────────────────────────
%             q1       q2      q3      q4
A = [    0,  -0.45,   0.35,  pi/4];   % offsets    [rad]
B = [ pi/4,   0.50,  -0.50,  0.25];   % amplitudes [rad]
w = 1.0;                               % frecuencia angular [rad/s]

t_end  = 2 * (2*pi / w);              % duracion: 2 periodos completos [s]
N_pts  = 2000;                         % puntos para graficas
N_harm = 8;                            % numero de armonicos de Fourier
% ─────────────────────────────────────────────────────────────────────────────

% ── FK SIMBOLICA ─────────────────────────────────────────────────────────────
fprintf('Calculando FK simbolica...\n');
syms t real
q_sym     = sym(A(:)) + sym(B(:)) .* sin(w * t);
y_sym     = open_manx_fkin(q_sym);         % [x; y; z; phi]  (4x1 sym)
ydot_sym  = diff(y_sym,    t);
yddot_sym = diff(ydot_sym, t);

% Handle vectorial exacto (para graficas)
y_fun     = matlabFunction(y_sym,     'Vars', {t});
ydot_fun  = matlabFunction(ydot_sym,  'Vars', {t});
yddot_fun = matlabFunction(yddot_sym, 'Vars', {t});

% Handles escalares por salida (para integracion adaptativa de Fourier)
yi_fun = cell(4, 1);
for i = 1:4
    yi_fun{i} = matlabFunction(y_sym(i), 'Vars', {t});
end

% ── EVALUACION NUMERICA EXACTA (para graficas) ───────────────────────────────
t_vec     = linspace(0, t_end, N_pts);
y_num     = y_fun(t_vec);       % 4 x N
ydot_num  = ydot_fun(t_vec);
yddot_num = yddot_fun(t_vec);
q_num     = A(:) + B(:) .* sin(w * t_vec);   % 4 x N

% ── COEFICIENTES DE FOURIER (integracion adaptativa, periodo exacto) ──────────
fprintf('Calculando coeficientes de Fourier...\n');
T_period  = 2*pi / w;

a0_raw = zeros(4, 1);
An_raw = zeros(4, N_harm);   % amplitudes de cos(n*w*t)
Bn_raw = zeros(4, N_harm);   % amplitudes de sin(n*w*t)

for i = 1:4
    fi = yi_fun{i};
    a0_raw(i) = (1/T_period) * integral(fi, 0, T_period, 'AbsTol', 1e-10);
    for n = 1:N_harm
        An_raw(i,n) = (2/T_period) * integral( ...
            @(tv) fi(tv) .* cos(n*w*tv), 0, T_period, 'AbsTol', 1e-10);
        Bn_raw(i,n) = (2/T_period) * integral( ...
            @(tv) fi(tv) .* sin(n*w*tv), 0, T_period, 'AbsTol', 1e-10);
    end
end

% Redondear a 3 decimales; eliminar terminos < 0.001
a0_r = round(a0_raw, 3);
An_r = round(An_raw, 3);
Bn_r = round(Bn_raw, 3);
a0_r(abs(a0_r) < 5e-4) = 0;
An_r(abs(An_r) < 5e-4) = 0;
Bn_r(abs(Bn_r) < 5e-4) = 0;

% ── CONSOLA ───────────────────────────────────────────────────────────────────
ylabels = {'x', 'y', 'z', 'phi'};
yunits  = {'m', 'm', 'm', 'rad'};

fprintf('\n=== Trayectoria articular de referencia ===\n');
fprintf('  qi(t) = A(i) + B(i)*sin(%.2f*t)\n', w);
fprintf('\n  i |   A [rad]    B [rad]  | A [grados]  B [grados]\n');
fprintf('  --|----------------------------------------------\n');
for i = 1:4
    fprintf('  %d |  %7.4f   %7.4f  |  %8.3f   %8.3f\n', ...
        i, A(i), B(i), rad2deg(A(i)), rad2deg(B(i)));
end

y0 = y_fun(0);
fprintf('\n=== Pose inicial FK(q(0)) ===\n');
for i = 1:4
    fprintf('  %s(0) = %+.4f %s\n', ylabels{i}, y0(i), yunits{i});
end

fprintf(['\n=== Expresiones simplificadas (serie de Fourier) ===\n', ...
         '    Coeficientes redondeados a 3 decimales; terminos < 0.001 eliminados.\n', ...
         '    vel y acel son las derivadas analiticas exactas de la pos simplificada.\n\n']);

for i = 1:4
    s_pos  = fourier_str(a0_r(i), An_r(i,:), Bn_r(i,:), w, 0);
    s_vel  = fourier_str(a0_r(i), An_r(i,:), Bn_r(i,:), w, 1);
    s_acel = fourier_str(a0_r(i), An_r(i,:), Bn_r(i,:), w, 2);
    fprintf('  [%s_d]  [%s]\n', ylabels{i}, yunits{i});
    fprintf('    pos  : %s\n',  s_pos);
    fprintf('    vel  : %s\n',  s_vel);
    fprintf('    acel : %s\n\n', s_acel);
end

% ── FIGURA 1: Trayectorias articulares ───────────────────────────────────────
figure('Name', 'Trayectorias articulares', 'NumberTitle', 'off', ...
       'Position', [50 520 900 380]);
qlabels = {'q_1', 'q_2', 'q_3', 'q_4'};
clr     = {'b', 'r', [0 0.6 0], [0.8 0.4 0]};
for i = 1:4
    subplot(2, 2, i);
    plot(t_vec, rad2deg(q_num(i,:)), 'Color', clr{i}, 'LineWidth', 1.5);
    xlabel('t  [s]'); ylabel([qlabels{i} '  [°]']);
    Ad = rad2deg(A(i)); Bd = rad2deg(B(i));
    if Bd >= 0
        title(sprintf('%s = %.1f° + %.1f°·sin(%.1ft)', qlabels{i}, Ad, Bd, w));
    else
        title(sprintf('%s = %.1f° − %.1f°·sin(%.1ft)', qlabels{i}, Ad, abs(Bd), w));
    end
    grid on; box on; xlim([0 t_end]);
end
sgtitle('Trayectorias articulares de referencia', 'FontSize', 12, 'FontWeight', 'bold');

% ── FIGURA 2: Trayectorias cartesianas exactas (4 filas x 3 columnas) ────────
y_lbl   = {'x_d  [m]',         'y_d  [m]',         'z_d  [m]',         'phi_d  [rad]'      };
yd_lbl  = {'xdot_d  [m/s]',    'ydot_d  [m/s]',    'zdot_d  [m/s]',    'phidot_d  [rad/s]' };
ydd_lbl = {'xddot_d  [m/s2]',  'yddot_d  [m/s2]',  'zddot_d  [m/s2]',  'phiddot_d [rad/s2]'};
col3 = {[0 0.45 0.74], [0.85 0.33 0.10], [0.49 0.18 0.56]};

figure('Name', 'Trayectorias cartesianas de referencia', 'NumberTitle', 'off', ...
       'Position', [80 50 1100 700]);
for i = 1:4
    subplot(4, 3, 3*(i-1)+1);
    plot(t_vec, y_num(i,:),     'Color', col3{1}, 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel(y_lbl{i},   'Interpreter', 'none'); grid on; box on; xlim([0 t_end]);

    subplot(4, 3, 3*(i-1)+2);
    plot(t_vec, ydot_num(i,:),  'Color', col3{2}, 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel(yd_lbl{i},  'Interpreter', 'none'); grid on; box on; xlim([0 t_end]);

    subplot(4, 3, 3*(i-1)+3);
    plot(t_vec, yddot_num(i,:), 'Color', col3{3}, 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel(ydd_lbl{i}, 'Interpreter', 'none'); grid on; box on; xlim([0 t_end]);
end
sgtitle('Trayectoria cartesiana de referencia  --  pos | vel | acel  (FK exacta)', ...
        'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'none');

% ── FIGURA 3: Trayectoria 3D en espacio de trabajo ───────────────────────────
figure('Name', 'Trayectoria 3D', 'NumberTitle', 'off', 'Position', [1000 300 550 480]);
surface([y_num(1,:); y_num(1,:)], [y_num(2,:); y_num(2,:)], ...
        [y_num(3,:); y_num(3,:)], [t_vec; t_vec], ...
        'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', 2);
colormap(parula); cb = colorbar; cb.Label.String = 't  [s]';
hold on;
plot3(y_num(1,1),   y_num(2,1),   y_num(3,1),   'go', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Inicio');
plot3(y_num(1,end), y_num(2,end), y_num(3,end), 'rs', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Fin');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Trayectoria 3D del efector final'); legend('Location', 'best');
grid on; box on; view(40, 25);

fprintf('Listo. Figuras generadas.\n');

% ── FUNCION LOCAL ─────────────────────────────────────────────────────────────
function s = fourier_str(a0, An, Bn, w, deriv)
% Genera la cadena de texto de una serie de Fourier o su k-esima derivada.
%
%   Posicion  (deriv=0): a0 + sum_n [ An*cos(nwt) + Bn*sin(nwt) ]
%   Velocidad (deriv=1): d/dt de lo anterior
%   Aceleracion(deriv=2): d^2/dt^2 de lo anterior
%
%   Los coeficientes de vel y acel se derivan de los de posicion,
%   garantizando consistencia analitica. Se redondean a 3 decimales
%   y se eliminan los terminos cuyo coeficiente sea < 0.001.

N = numel(An);
terms = {};

% Termino constante (desaparece al derivar)
if deriv == 0 && abs(a0) >= 5e-4
    terms{end+1} = sprintf('%.3f', a0);
end

for n = 1:N
    nw   = n * w;
    An_n = An(n);
    Bn_n = Bn(n);

    % d^k/dt^k [ An*cos(nwt) + Bn*sin(nwt) ]
    %  k=0: An*cos + Bn*sin
    %  k=1: -An*nw*sin + Bn*nw*cos
    %  k=2: -An*(nw)^2*cos - Bn*(nw)^2*sin
    switch deriv
        case 0
            c_cos = An_n;
            c_sin = Bn_n;
        case 1
            c_cos =  Bn_n * nw;
            c_sin = -An_n * nw;
        case 2
            c_cos = -An_n * nw^2;
            c_sin = -Bn_n * nw^2;
        otherwise
            c_cos = 0;
            c_sin = 0;
    end

    % Redondear los coeficientes derivados
    c_cos = round(c_cos, 3);
    c_sin = round(c_sin, 3);

    % Argumento de frecuencia: 't', '2*t', '3*t', etc.
    freq_i = round(nw);
    if abs(nw - freq_i) < 1e-9
        if freq_i == 1
            arg = 't';
        else
            arg = sprintf('%d*t', freq_i);
        end
    else
        arg = sprintf('%.3f*t', nw);
    end

    if abs(c_cos) >= 5e-4
        terms{end+1} = sprintf('%+.3f*cos(%s)', c_cos, arg);  %#ok<*AGROW>
    end
    if abs(c_sin) >= 5e-4
        terms{end+1} = sprintf('%+.3f*sin(%s)', c_sin, arg);
    end
end

if isempty(terms)
    s = '0';
    return;
end

s = strjoin(terms, ' ');
if s(1) == '+'            % quitar '+' inicial si el primer termino es positivo
    s = s(2:end);
end
s = strtrim(s);
end
