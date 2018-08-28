close all; clear; clc
import casadi.*

sim_tri = 0;
m       = 0.18;         % kg
g       = 9.81;         % m/s/s
kf      = 6.11*10^-8;
km      = 1.5*10^-9;
a1      = 3.7408;       % a1*dc = F
if sim_tri
    I       = [0.00025 0 0; 0 0.000232 0; 0 0 0.0003738];
    l_tri1  = 0.15;
    l_tri2  = 0.15;
    l_tri3  = 0.15;
    u       = SX.sym('u', 3); % MicroSeconds of duty-cycle
else
    I       = [0.00025 0 2.55e-6; 0 0.000232 0; 2.55e-6 0 0.0003738];
    l       = 0.086;    % m
    u       = SX.sym('u', 4);   % MicroSeconds of duty-cycle
end
I_inv   = I^-1;
m_inv   = [1/m 0 0; 0 1/m 0; 0 0 1/m];

x       = SX.sym('X', 12); % x, y, z || phi, theta, psi || u, v, w || p, q, r
F       = a1*u;
M       = km/kf*a1*u;
phi     = x(4);
theta   = x(5);
psi     = x(6);
R_b2e   = [cos(theta)*cos(psi) sin(theta)*sin(phi)*cos(psi)+cos(phi)*sin(psi) sin(theta)*cos(phi)*cos(psi)-sin(phi)*sin(psi)
           cos(theta)*sin(psi) sin(theta)*sin(phi)*sin(psi)-cos(phi)*cos(psi) sin(theta)*cos(phi)*sin(psi)-sin(phi)*cos(psi)
                   -sin(theta)                            cos(theta)*sin(psi)                            cos(theta)*cos(phi)];
R_rot2euler = [1        0          -sin(theta)
               0  cos(phi) sin(phi)*cos(theta)
               0 -sin(phi) cos(phi)*cos(theta)];
               
xyz_dot     = [x(7); x(8); x(9)];
euler_dot   = R_rot2euler*[x(10); x(11); x(12)];
if sim_tri
    v_dot   = m_inv*([0; 0; -m*g] + R_b2e*[0; 0; F(1) + F(2) + F(3)]);
    w_dot   = I_inv*([l_tri2*sqrt(3)/2*F(2) - l_tri3*sqrt(3)/2*F(3); -l_tri1*F(1) + l_tri2/2*F(2) + l_tri3/2*F(3); -M(1) + M(2) + M(3)] - ...
              cross([x(10); x(11); x(12)], I*[x(10); x(11); x(12)]));
else
    v_dot  = m_inv*([0; 0; -m*g] + R_b2e*[0; 0; F(1) + F(2) + F(3) + F(4)]); 

    w_dot  = I_inv*([l*(F(2)- F(4)); l*(F(3) - F(1)); -M(1) + M(2) - M(3) + M(4)] - ...
              cross([x(10); x(11); x(12)], I*[x(10); x(11); x(12)]));
end   
x_dot       = Function('x_dot', {x, u}, {[xyz_dot; euler_dot; v_dot; w_dot]});

% Linearize
A = jacobian([xyz_dot; euler_dot; v_dot; w_dot], x);
A = Function('A', {x, u}, {A});
A = full(A(zeros(12,1), m*g/a1/4*ones(4,1)));
B = jacobian([xyz_dot; euler_dot; v_dot; w_dot], u);
B = Function('B', {x, u}, {B});
B = full(B(zeros(12,1), m*g/a1/4*ones(4,1)));

% Integrator function (Runge-Kutta-4)
x0  = SX.sym('x0', 12);
x = x0;
h = 0.001;
for ii = 1:1
    k1 = x_dot(x, u);
    k2 = x_dot(x + h/2*k1, u);
    k3 = x_dot(x + h/2*k2, u);
    k4 = x_dot(x + h  *k3, u);
    x  = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
RK4 = Function('RK4', {x0, u}, {x});





%-------------------------------------------------------------------------%
% Simulation
x = zeros(12,1);
t = 0;
if sim_tri
    u    = m*g/a1/3;
    u(2) = u(1);
    u(3) = u(2);
else
    u = m*g/a1/4*ones(4,1);
end

for ii = 1:1000
    t(ii+1) = t(ii) + h;
    x(:,ii+1) = full(RK4(x(:,ii), u));
end




%-------------------------------------------------------------------------%
% Visualize 
figure
subplot(4,3,1)
    plot(t, x(1,:));
    xlim([t(1) t(end)]);
    ylabel('x in m')
subplot(4,3,2)
    plot(t, x(2,:));
    xlim([t(1) t(end)]);
    ylabel('y in m')
subplot(4,3,3)
    plot(t, x(3,:));
    xlim([t(1) t(end)]);
    ylabel('z in m')
subplot(4,3,4)
    plot(t, x(4,:));
    xlim([t(1) t(end)]);
    ylabel('phi in rad')
subplot(4,3,5)
    plot(t, x(5,:));
    xlim([t(1) t(end)]);
    ylabel('theta in rad')
subplot(4,3,6)
    plot(t, x(6,:));
    xlim([t(1) t(end)]);
    ylabel('psi in rad')
subplot(4,3,7)
    plot(t, x(7,:));
    xlim([t(1) t(end)]);
    ylabel('u in m/s')
subplot(4,3,8)
    plot(t, x(8,:));
    xlim([t(1) t(end)]);
    ylabel('v in m/s')
subplot(4,3,9)
    plot(t, x(9,:));
    xlim([t(1) t(end)]);
    ylabel('w in m/s')
subplot(4,3,10)
    plot(t, x(10,:));
    xlim([t(1) t(end)]);
    ylabel('p in rad/s')
subplot(4,3,11)
    plot(t, x(11,:));
    xlim([t(1) t(end)]);
    ylabel('q in rad/s')
subplot(4,3,12)
    plot(t, x(12,:));
    xlim([t(1) t(end)]);
    ylabel('r in rad/s')


    