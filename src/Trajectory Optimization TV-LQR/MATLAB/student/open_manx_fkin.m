function yout = open_manx_fkin(q)
% Cinematica directa del OpenManipulator-X
%
%   yout = open_manx_fkin(q)
%
%   Entradas:
%       q : vector de posiciones articulares [q1 q2 q3 q4]^T [rad].
%
%   Salida:
%       yout = [x; y; z; phi]
%       x,y,z : posicion del end_effector_link respecto a link1 [m]
%       phi   : orientacion/elevacion del gripper respecto a la horizontal [rad]
%       (phi = q2 + q3 + q4)

q = q(:);
if numel(q) ~= 4
    error('open_manx_fkin: q debe contener 4 articulaciones.');
end

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);

% Offsets del URDF [m]
x_base = 0.012;
z_base = 0.017 + 0.0595;

x23 = 0.024;   % componente x del vector joint2 -> joint3
z23 = 0.128;   % componente z del vector joint2 -> joint3
l34 = 0.124;   % vector joint3 -> joint4 sobre x local
l4e = 0.126;   % vector joint4 -> end_effector sobre x local

% Cinematica planar en el plano radial-z despues del giro de base q1.
r = x23*cos(q2) + z23*sin(q2) + ...
    l34*cos(q2 + q3) + ...
    l4e*cos(q2 + q3 + q4);

z = z_base + ...
    (-x23*sin(q2) + z23*cos(q2)) - ...
    l34*sin(q2 + q3) - ...
    l4e*sin(q2 + q3 + q4);

x = x_base + r*cos(q1);
y =          r*sin(q1);

% Orientacion del gripper respecto a la horizontal
phi = q2 + q3 + q4;

yout = [x; y; z; phi];
end
