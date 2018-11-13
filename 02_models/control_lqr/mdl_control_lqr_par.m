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
[~, Ode_lin]        = mdl_quadrotor_dynamicsLin();
Ode_lin.DynamicsLin = mdl_quadrotor_dynamicsLin(x0, Par_mdlQuad);

% Get subsystem that is to be controlled

% Design LQR



% k = [-1 -2];          % PD-Control parameters 
% k = [-1 -2.5];          % PD-Control parameters 
% par_ctrlLqr.k = [-4 -8];          % PD-Control parameters 
Par_controlLqr.k = [-2 -5];          % PD-Control parameters 