clc; clear; close all
%%

% Pre-Laboratorio 5: Trajectory Optimization y TV-LQR

% Parámetros de horizonte temporal
N = 30;       % Número de pasos de discretización
Ts = 0.05;    % Tiempo de muestreo
nx = 8;        % Número de estados: [q1,q2,q3,q4,dq1,dq2,dq3,dq4]
nu = 4;        % Número de entradas: torques

x0 = [0; 0; pi/6; pi/3; 0; 0; 0; 0];      % Estado inicial (q,dq)
yf = [0.2; -0.13; 0.2; 0];               % Salida deseada (posición y orientación)

% Saturaciones de torque
ukmax = 0.6;
ukmin = -0.6;

% Cargar trayectoria previa (opcional)
%load('zmin');

% Inicialización del vector de optimización z0
z0 = zeros(N*(nx+nu),1) ; % Inicializar como vector de ceros del tamaño (N*(nx+nu),1)
[~, u0g] = OMDyn(x0(1:4), x0(5:8)); % Obtener torques de compensación gravitatoria con OMDyn(q,dq)
z0 = [kron(ones(N,1),x0); kron(ones(N,1),u0g)]; % vector z = [xvec; uvec]

% Optimización de trayectoria (Trajectory Optimization)
options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',100000,'OptimalityTolerance',1e-4);
zmin = fmincon(@(z)Jcosto(z,Ts,N,nx,nu,x0,yf),z0,[],[],[],[],[],[], ...
               @(z)restr(z,Ts,N,nx,nu,x0,yf,ukmax,ukmin),options);
save('zmin','zmin'); % Guardar la trayectoria óptima
%%
% Cargar trayectoria previa (opcional)
load('zmin');
%% TVLQR: A y B Matrices variantes

%% TV-LQR: Cálculo de matrices A y B variantes en el tiempo
X = zmin(1:nx*N);
U = zmin(nx*N+(1:nu*N));
X = reshape(X, [nx N]);
U = reshape(U, [nu N]);

for k = 1:N
    t = 0;
    xrefk = X(:,k);
    urefk = U(:,k);
    delta = 1e-5;

    for i = 1:nx
        dx = zeros(nx,1); dx(i) = delta;
        f1 = OM4dof(0, xrefk + dx, urefk);
        f2 = OM4dof(0, xrefk, urefk); 
        Ak(:,i,k) = (f1 - f2) / (delta); % Derivada ∂f/∂x (analítica o numérica)
    end    

    for i = 1:nu
        du = zeros(nu,1); du(i) = delta;
        f1 = OM4dof(0, xrefk, urefk + du);
        f2 = OM4dof(0, xrefk, urefk);
        Bk(:,i,k) = (f1 - f2) / (delta) ;% Derivada ∂f/∂u (analítica o numérica)
    end
end
%% Trajectory Optimization

figure(1); clf
% Trayectoria de referencia
yref = [];
for i = 1:size(X,2)
    yref(:,i) = fkin(0, X(1:4,i), 0);
end
h_ref = plot3(yref(1,:), yref(2,:), yref(3,:), '-or', 'LineWidth', 1); hold on;

% Punto final deseado
h_goal = plot3(yf(1), yf(2), yf(3), '*k', 'LineWidth', 1);

grid on;
axis([-0.05 0.23 -.12 .15 -0.05 0.3]);
view(45,70);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Trayectoria de referencia del efector final')
legend([h_ref, h_goal], ...
       {'Trayectoria de referencia', 'Punto final deseado'}, ...
       'Location', 'northeastoutside');
%% TVLQR: Ganancias variantes

%% TV-LQR: Cálculo de ganancias variantes en el tiempo
Qk = diag([100; 100; 100; 100; 1; 1; 1; 1]);
Rk = 100 * diag([1;1;1;1]); % Mayor peso para evitar explosión de K_TV
Qf = Qk;

% Inicialización de Riccati hacia atrás
Pk(:,:,N) = sqrtm(0.05 * Qf);
Sk(:,:,N) = Pk(:,:,N) * Pk(:,:,N)';
K_TV(:,:,N) = inv(Rk) * Bk(:,:,N)' * Sk(:,:,N);

for k = N:-1:2
    P = Pk(:,:,k); A = Ak(:,:,k); B = Bk(:,:,k);
    Q = Qk; R = Rk;

    % Integración con Runge-Kutta 4 del Riccati invertido
    jj = 100;
    TsR = Ts / jj;
    for JJ = 1:jj
        k1 = Ricc_sqrt(0,P,A,B,Q,R);
        k2 = Ricc_sqrt(0,P + k1*TsR/2,A,B,Q,R);
        k3 = Ricc_sqrt(0,P + k2*TsR/2,A,B,Q,R);
        k4 = Ricc_sqrt(0,P + k3*TsR,A,B,Q,R);
        P = P + TsR*(k1 + 2*k2 + 2*k3 + k4)/6;
    end

    Pk(:,:,k-1) = P;
    Sk(:,:,k-1) = Pk(:,:,k-1) * Pk(:,:,k-1)';
    K_TV(:,:,k-1) = inv(R) * B' * Sk(:,:,k-1);
end
K_TV
%% Simulación con TV-LQR

%% Simulación con TV-LQR desde condición perturbada
x0sim = x0 + 0.05 * randn(8,1);
Uref = U;
Xref = X;
[T,Xsim] = ode45(@(t,x)OM4dof(t,x,controlTVLQR(t,x,Uref,Xref,Ts,N,K_TV)), 0:Ts:(N*Ts), x0sim);
Xsim = Xsim';
%% Gráfica de trayectorias y entradas resultantes

figure(1), clf;
for i = 1:4
    subplot(4,1,i)
    plot(0:Ts:(N-1)*Ts, Xref(i,:), 'LineWidth', 1.2)
    ylabel(['x', num2str(i), '_{ref} (rad)'])
    grid on
end
xlabel('Tiempo (s)')
sgtitle('Trayectoria de referencia x_{ref} - Posiciones articulares')
% 
figure(2), clf;
for i = 5:8
    subplot(4,1,i-4)
    plot(0:Ts:(N-1)*Ts, Xref(i,:), 'LineWidth', 1.2)
    ylabel(['x', num2str(i), '_{ref} (rad/s)'])
    grid on
end
xlabel('Tiempo (s)')
sgtitle('Trayectoria de referencia x_{ref} - Velocidades articulares')


figure(3), clf;
for i = 1:nu
    subplot(nu,1,i)
    plot(0:Ts:(N-1)*Ts, Uref(i,:), 'LineWidth', 1.2)
    ylabel(['u_', num2str(i), ' (N·m)'])
    grid on
end
xlabel('Tiempo (s)')
sgtitle('Entradas óptimas resultantes u_{ref}')
%% Grafica de Joints

% Graficar q vs q_ref 
figure(1), clf;
for i = 1:4
    subplot(4,1,i)
    plot(T,Xsim(i,:), 'b', 'LineWidth', 1); hold on; grid on;
    plot(T,[x0sim(i), Xref(i,:)], 'r--', 'LineWidth', 2)
    legend(['q',num2str(i)], ['q',num2str(i),'ref'])
end

han = axes(gcf, 'visible', 'off');
han.XLabel.Visible = 'on'; han.YLabel.Visible = 'on';
xlabel(han, 'Tiempo (s)');
ylabel(han, 'Posición (rad)');
sgtitle('Trayectorias de las articulaciones (q vs q_{ref})')
%% Plots visual (Solo si la gráfica anterior es estable)

%% Plots en espacio cartesiano (sólo si el sistema es estable)
figure(2); clf
for rrsim = 1:20
    x0sim = x0 + 0.05 * randn(8,1);
    [T,Xsim] = ode45(@(t,x)OM4dof(t,x,controlTVLQR(t,x,Uref,Xref,Ts,N,K_TV)), 0:Ts:(N*Ts), x0sim);
    Xsim = Xsim';
    ysim = [];
    for i = 1:size(Xsim,2)
        ysim(:,i) = fkin(0,Xsim(1:4,i),0);
    end
    if rrsim == 1
        h_sim = plot3(ysim(1,:), ysim(2,:), ysim(3,:), '-b', 'LineWidth', 1); hold on;
    else
        plot3(ysim(1,:), ysim(2,:), ysim(3,:), '-b', 'LineWidth', 1); hold on;
    end
end

% Trayectoria de referencia
yref = [];
for i = 1:size(Xref,2)
    yref(:,i) = fkin(0,Xref(1:4,i),0);
end
h_ref = plot3(yref(1,:), yref(2,:), yref(3,:), '-or', 'LineWidth', 1);

% Punto final deseado
h_goal = plot3(yf(1), yf(2), yf(3), '*k', 'LineWidth', 1);

grid on;
axis([-0.05 0.23 -.12 .15 -0.05 0.3]);
view(45,70);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Trayectorias del efector final con TV-LQR')
legend([h_sim, h_ref, h_goal], ...
       {'Trayectorias seguidas por el efector final', ...
        'Trayectoria de referencia', ...
        'Punto final deseado'}, ...
       'Location', 'northeastoutside');

%% Funciones

% Costo del Trajectory Optimization
%%
function J = Jcosto(z,Ts,N,nx,nu,x0,yf)
    Qv = 0.1 * eye(4);    % Penalización sobre estado (velocidades)
    Qy = diag([1000 1000 1000 10]);   % Penalización sobre trayectoria en el espacio cartesiano
    R = 0.01*eye(nu);    % Penalización sobre entradas

    J = 0;
    xN = z(nx*(N-1)+(1:nx));
    y0 = fkin(0, x0(1:4), 0); % Evaluar fkin(x0)
    yN = fkin(0, xN(1:4), 0); % Evaluar fkin(xN)

    % Penalización sobre el error final
    Qf = diag([1000 1000 1000 10]);
    J = J + (yN - yf)' * Qf * (yN - yf);

    for k = 1:N
        xk = z(nx*(k-1)+(1:nx));
        yk = fkin(0, xk(1:4), 0); % Evaluar fkin(xk)
        yrefk = (yf - y0) / N * k + y0; % Interpolación lineal
        dqk = xk(5:8);
        J = J + (dqk' * Qv * dqk + (yk - yrefk)'*Qy*(yk - yrefk)); % Penalización sobre error intermedio y velocidades
    end

    for k = 0:N-2
        uk = z(nx*N + nu*k + (1:nu));
        ukp1 = z(nx*N + nu*(k+1) + (1:nu));
        J = J + uk'*R*uk + (uk - ukp1)'*R*(uk - ukp1); % Penalización sobre energía y suavidad de control
    end

    % Último término de control
    uk = z(nx*N + nu*(N-1) + (1:nu));
    J = J + uk' * R * uk;
end

% Restricciones del problema
function [c_des, c_eq] = restr(z,Ts,N,nx,nu,x0,yf,ukmax,ukmin)
    c_eq = [];
    for k = 0:N-1
        if k == 0
            xk = x0;
        else
            xk = z(nx*(k-1)+(1:nx));
        end
        uk = z(nx*N + nu*k + (1:nu));
        xkp1 = z(nx*k + (1:nx));

        % Restricciones dinámicas usando Runge-Kutta 4
        k1 = OM4dof(0,xk,uk);
        k2 = OM4dof(0,xk + Ts*k1/2, uk);
        k3 = OM4dof(0,xk + Ts*k2/2, uk);
        k4 = OM4dof(0,xk + Ts*k3, uk);
        xk_next = xk + Ts*(k1 + 2*k2 + 2*k3 + k4)/6;
        c_eq = [c_eq; xkp1 - xk_next];
    end

    c_des = [];
    for k = 0:N-1
        uk = z(nx*N + nu*k + (1:nu));
        xk = z(nx*k + (1:nx));
        yk = fkin(0, xk(1:4), 0); % Evaluar fkin(xk)

        % Restricciones de torque
        c_des = [c_des; uk - ukmax; -uk + ukmin]; 

    end
end

% Modelo dinámico del brazo de 4DOF
function dx = OM4dof(t,x,u)
    q = x(1:4);
    dq = x(5:8);
    [M, phib] = OMDyn(q, dq);
    bf = 0.001;
    ddq = M \ (u - phib - bf*dq);
    dx = [dq; ddq]; % Calcular dx/dt con M(q), phib(q,dq) desde OMDyn
end

% Cinemática directa del efector final
function yout = fkin(t,q,u)
    l1=0.077; l2=0.13; l3=0.124; l4=0.126;
    q1=q(1); q2=q(2); q3=q(3); q4=q(4);
    q2des=0.185;

    z = l1 + l2*cos(q2des+q2) + l3*cos(pi/2+q3+q2) + l4*cos(pi/2+q4+q3+q2);
    xy = l2*sin(q2des+q2) + l3*sin(pi/2+q3+q2) + l4*sin(pi/2+q4+q3+q2);
    x = xy*cos(q1); y = xy*sin(q1); th = q2+q3+q4;
    yout = [x; y; z; th];
end

% Controlador TV-LQR en tiempo discreto
function u = controlTVLQR(t,x,Uref,Xref,Ts,N,K_TV)
    k = min(floor(t/Ts)+1,N);
    xref = Xref(:,k);
    uref = Uref(:,k);
    K = K_TV(:,:,k);
    u = uref - K*(x - xref); % u(t) = uref(t) - K(t)*(x - xref(t))
end

% Ecuación de Riccati en forma sqrt
function dP = Ricc_sqrt(t,P,A,B,Q,R)
    dP = A'*P - 0.5*P*P'*B*(R\B')*P + 0.5*Q*inv(P'); 
end