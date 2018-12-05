function Par = mdl_quadrotor_par( )
% Par_mdlQuad = mdl_quadrotor_par( )
%
%   Date        : Winter 2018
%
%   Description : Loads the parameters of a physical quadrotor model
% 
%   Parameters  : None
% 
%   Return      : Par -> Struct containing model the parameters
% 
%-------------------------------------------------------------------------%

% Set basic parameters
Par.m     = 0.55;           % Quadrocopter mass [kg]
Par.g     = 9.81;           % Earth acceleration [m*s^-2]
Par.armLength     = 0.086;          % Distance to rotor from center point [m]
Par.F0    = Par.m*Par.g/4;  % Equilibrium point forces [kg*m*s^-2]
km        = 1.5*10^-9;      % Thrust coefficient [N/rpm^2]
kf        = 6.11*10^-8;     % Momentum coefficient [Nm/rpm^2]
Par.gamma = 1e-4;           % Correlation factor between thrust and momentum
Par.gamma = km/kf;          % Correlation factor between thrust and momentum

% Inertia matrix
Par.I     = [2.500e-04     0     2.550e-06;
                 0     2.320e-04     0; 
             2.550e-06     0     3.738e-04];
         
% Inverse of inertia matrix
Par.invI    = Par.I^-1;