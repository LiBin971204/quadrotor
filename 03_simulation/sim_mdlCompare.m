clear; close all; clc;

% Get model parameters
Par_mdlQuad = mdl_quadrotor_par();


% Set linear system dynamics
x0                  = [0;0;1;zeros(9,1)];
u0                  = Par_mdlQuad.m*Par_mdlQuad.g/4*ones(4,1);
[~, Ode_lin]        = mdl_quadrotor_dynamicsLin();
Ode_lin.systemDynamics = mdl_quadrotor_dynamicsLin(x0, u0, Par_mdlQuad);


% Set nonlinear system dynamics
Ode = aux_setDynamics('mdl_quadrotor');


% Simulation with input from simulink simulation
load('data_mdlCompare_simulink.mat');
t_sim = thrust.Time;
T     = 1e-04;

% Nonlinear simulation
x     = zeros(Ode.Size.n_states, length(t_sim));
y     = zeros(Ode.Size.n_outputs, length(t_sim));
if exist('data_mdlCompare_matlab.mat','file')
    load('data_mdlCompare_matlab.mat')
else
    for ii = 1:length(t_sim)
        h1  = full(Ode.systemDynamics(x(:, ii)       , thrust.Data(ii, :).'));
        h2  = full(Ode.systemDynamics(x(:, ii)+h1*T/2, thrust.Data(ii, :).'));
        h3  = full(Ode.systemDynamics(x(:, ii)+h2*T/2, thrust.Data(ii, :).'));
        h4  = full(Ode.systemDynamics(x(:, ii)+h3*T  , thrust.Data(ii, :).'));
        x(:, ii+1) = x(:,ii) + T/6*(h1 + 2*h2 + 2*h3 + h4);
        y(:, ii) = full(Ode.systemOutput(x(:,ii) , thrust.Data(ii, :).'));
        
    end
    save('sim_scriptBased', 't_sim', 'x');
end

% Linear simulation I 
x_lin = zeros(Ode_lin.Size.n_states, length(t_sim));
% y_lin = zeros(Ode_lin.Size.n_outputs, length(t_sim));

Flags.Define.linearSim_withCasadi = 0;
switch Flags.Define.linearSim_withCasadi
    case true
        import casadi.*
        x_sym = MX.sym('x_sym', Ode.Size.n_states);
        u_sym = MX.sym('u_sym', Ode.Size.n_inputs);
        A = jacobian(Ode.systemDynamics(x_sym, u_sym), x_sym);
        A_fun = Function('A_fun', {x_sym, u_sym}, {A});
        A = full(A_fun(x0, u0));
        B = jacobian(Ode.systemDynamics(x_sym, u_sym), u_sym);
        B_fun = Function('B_fun', {x_sym, u_sym}, {B});
        B = full(B_fun(x0, u0));
    case false
        A = Ode_lin.systemDynamics.mA;
        B = Ode_lin.systemDynamics.mB;
end
        
for ii = 1:length(t_sim)
        h1  = A* x_lin(:, ii)        + B*(thrust.Data(ii, :).'-u0);
        h2  = A*(x_lin(:, ii)+h1*T/2)+ B*(thrust.Data(ii, :).'-u0);
        h3  = A*(x_lin(:, ii)+h2*T/2)+ B*(thrust.Data(ii, :).'-u0);
        h4  = A*(x_lin(:, ii)+h3*T)  + B*(thrust.Data(ii, :).'-u0);
        x_lin(:, ii+1)  = x_lin(:,ii) + T/6*(h1 + 2*h2 + 2*h3 + h4);
%         y_lin(:, ii)    = C*x(:,ii);        
end
clear x_sym u_sym




% Compare results
figure
title('Translation')
subplot(3,2,1)
plot(t_sim, x(1, 1:end-1))
hold on
plot(States_ext.Pos_x_mtr.Time, States_ext.Pos_x_mtr.Data)
plot(t_sim, x_lin(1, 1:end-1))
grid on
ylabel('x in m')
legend('Matlab', 'Simulink', 'Linear')

subplot(3,2,3)
plot(t_sim, x(2, 1:end-1))
hold on
plot(States_ext.Pos_y_mtr.Time, States_ext.Pos_y_mtr.Data)
plot(t_sim, x_lin(2, 1:end-1))
grid on
ylabel('y in m')

subplot(3,2,5)
plot(t_sim, x(3, 1:end-1))
hold on
plot(States_ext.Pos_z_mtr.Time, States_ext.Pos_z_mtr.Data)
plot(t_sim, x_lin(3, 1:end-1))
grid on
ylabel('z in m')

subplot(3,2,2)
plot(t_sim, x(4, 1:end-1))
hold on
plot(States_ext.V_u_mtrPerScnd.Time, States_ext.V_u_mtrPerScnd.Data)
plot(t_sim, x_lin(4, 1:end-1))
grid on
ylabel('u in m/s')

subplot(3,2,4)
plot(t_sim, x(5, 1:end-1))
hold on
plot(States_ext.V_v_mtrPerScnd.Time, States_ext.V_v_mtrPerScnd.Data)
plot(t_sim, x_lin(5, 1:end-1))
grid on
ylabel('v in m/s')

subplot(3,2,6)
plot(t_sim, x(6, 1:end-1))
hold on
plot(States_ext.V_w_mtrPerScnd.Time, States_ext.V_w_mtrPerScnd.Data)
plot(t_sim, x_lin(6, 1:end-1))
grid on
ylabel('w in m/s')

figure
title('Rotation')
subplot(3,2,1)
plot(t_sim, x(7, 1:end-1)*180/pi)
hold on
plot(States_ext.Ag_phi_rad.Time, States_ext.Ag_phi_rad.Data*180/pi)
plot(t_sim, x_lin(7, 1:end-1)*180/pi)
grid on
ylabel('phi in deg')

subplot(3,2,3)
plot(t_sim, x(8, 1:end-1)*180/pi)
hold on
plot(States_ext.Ag_theta_rad.Time, States_ext.Ag_theta_rad.Data*180/pi)
plot(t_sim, x_lin(8, 1:end-1)*180/pi)
grid on
ylabel('theta in deg')

subplot(3,2,5)
plot(t_sim, x(9, 1:end-1)*180/pi)
hold on
plot(States_ext.Ag_psi_rad.Time, States_ext.Ag_psi_rad.Data*180/pi)
plot(t_sim, x_lin(9, 1:end-1)*180/pi)
grid on
ylabel('psi in deg')

subplot(3,2,2)
plot(t_sim, x(10, 1:end-1))
hold on
plot(States_ext.AgVel_p_radPerScnd.Time, States_ext.AgVel_p_radPerScnd.Data)
plot(t_sim, x_lin(10, 1:end-1))
grid on
ylabel('p in rad/s')

subplot(3,2,4)
plot(t_sim, x(11, 1:end-1))
hold on
plot(States_ext.AgVel_q_radPerScnd.Time, States_ext.AgVel_q_radPerScnd.Data)
plot(t_sim, x_lin(11, 1:end-1))
grid on
ylabel('q in rad/s')

subplot(3,2,6)
plot(t_sim, x(12, 1:end-1))
hold on
plot(States_ext.AgVel_r_radPerScnd.Time, States_ext.AgVel_r_radPerScnd.Data)
plot(t_sim, x_lin(12, 1:end-1))
grid on
legend('Matlab', 'Simulink', 'Linear')
ylabel('r in rad/s')