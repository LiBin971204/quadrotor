function Par_controlLqr = mdl_control_lqr_par( )
% function Par_controlLqr = mdl_control_lqr_par( )
%
%   Date        : Winter 2018
%
%   Description : Sets the feedback gain matrix obtained via LQR synthesis
% 
%   Parameters  : None
% 
%   Return      : Par_controLqr - a struct which contains the feedbackGain
%
%-------------------------------------------------------------------------%

% Get model parameters
Par_mdlQuad = mdl_quadrotor_par();

% Set linear system dynamics
x0                  = [0;0;1;zeros(9,1)];
u0                  = Par_mdlQuad.m*Par_mdlQuad.g/4*ones(4,1);
[~, Ode_lin]        = mdl_quadrotor_dynamicsLin();
Ode_lin.systemDynamics = mdl_quadrotor_dynamicsLin(x0, u0, Par_mdlQuad);

% Get subsystem that is to be controlled (first exclude x,y dynamics, then
% psi)
A   = [Ode_lin.systemDynamics.mA(3,3) Ode_lin.systemDynamics.mA(3,6:end);
       Ode_lin.systemDynamics.mA(6:end,3) Ode_lin.systemDynamics.mA(6:end, 6:end)];
B   = [Ode_lin.systemDynamics.mB(3,:); Ode_lin.systemDynamics.mB(6:end,:)];
A(end,:) = [];
A(5,:) = [];
A(:,end) = [];
A(:,5) = [];
B(end,:) = [];
B(5,:) = [];

% Design LQR
Q = eye(6);
R = 0.1*eye(4);

[k, S, e] = lqr(A, B, Q, R);
% disp(e);
Par_control_lqr.k = k;

% k = [-1 -2];          % PD-Control parameters 
% k = [-1 -2.5];          % PD-Control parameters 
% par_ctrlLqr.k = [-4 -8];          % PD-Control parameters 
Par_controlLqr.k = [-2 -5];          % PD-Control parameters 