function Par_mdlQuad = mdl_quadrotor_par( )
% function Par_mdlQuad = mdl_quadrotor_par( )
%
%   Date        :   Winter 2018
%
%   Description : 	Loads the parameters of a physical quadrotor model and 
%                   its linearizattion
% 
%   Parameters  : 	None
% 
%   Return      : 	Par_mdlQuad -> Struct containing model parameters
% 
%-------------------------------------------------------------------------%

% Define flags
flags_mdl_quadrotor_par.define.printPlots = 0; % TODO: Make different


% Set basic parameters
Par_mdlQuad.m       = 0.55;             % Quadrocopter mass [kg]
Par_mdlQuad.g       = 9.81;             % Earth acceleration [m*s^-2]
Par_mdlQuad.I       = [2.500e-04,         0, 2.550e-06; ...
                       0, 2.320e-04,         0; ...
                       2.550e-06,         0, 3.738e-04];    % Inertia matrix
Par_mdlQuad.gamma   = 1e-4;

Par_mdlQuad.invI    = Par_mdlQuad.I^-1;         % Inverse of inertia matrix
Par_mdlQuad.l       = 0.086;            % Distance to rotor from center point [m]

Par_mdlQuad.F0      = Par_mdlQuad.m*Par_mdlQuad.g/4;    % Equilibrium point forces [kg*m*s^-2]


% Set identified ESC/BLDC maps
load('00_config/staticMap_escBLDC.mat');    % Load identified ESC+BLDC static map
load('05_data\c_identification\bldc_nonLinearHammerWienerstein_v01.mat')

coeffs      = [staticMap_escBLDC.coeffValues{:}];
coeffsRoot  = coeffs-[0 0 0 Par_mdlQuad.m/4];
coeffsDer   = polyder(coeffs);
PWM0        = roots(coeffsRoot);
Par_mdlQuad.PWM0    = PWM0(3)*staticMap_escBLDC.stdx+staticMap_escBLDC.meanx;
linearTerm  = polyval(coeffsDer, PWM0(3))/staticMap_escBLDC.stdx;

Par_mdlQuad.coeffs  = coeffs;
Par_mdlQuad.std     = staticMap_escBLDC.stdx;
Par_mdlQuad.mean    = staticMap_escBLDC.meanx;

x           = 0:1000;
y           = polyval(coeffs, (x-staticMap_escBLDC.meanx)/staticMap_escBLDC.stdx);

if flags_mdl_quadrotor_par.define.printPlots
    figure
    plot(x,y)
    grid on
    hold on
    plot(x, linearTerm*x+(Par_mdlQuad.m/4-linearTerm*Par_mdlQuad.PWM0))
    jo = struct(nlhw1);
    plot(jo.OutputNonlinearity.Breakpoints(1,:)/250, jo.OutputNonlinearity.Breakpoints(2,:))
end

Par_mdlQuad.ca_esc = [                  1                     1                    1                      1; ...
                   Par_mdlQuad.l/Par_mdlQuad.I(1,1)                    0     -Par_mdlQuad.l/Par_mdlQuad.I(1,1)                     0; ...
                                 0       Par_mdlQuad.l/Par_mdlQuad.I(2,2)                    0     -Par_mdlQuad.l/Par_mdlQuad.I(2,2); ...
               Par_mdlQuad.gamma/Par_mdlQuad.I(3,3) -Par_mdlQuad.gamma/Par_mdlQuad.I(3,3) Par_mdlQuad.gamma/Par_mdlQuad.I(3,3) -Par_mdlQuad.gamma/Par_mdlQuad.I(3,3)] ...
               *linearTerm;
Par_mdlQuad.ca = [                  1                     1                    1                      1; ...
                   Par_mdlQuad.l/Par_mdlQuad.I(1,1)                    0     -Par_mdlQuad.l/Par_mdlQuad.I(1,1)                     0; ...
                                 0       Par_mdlQuad.l/Par_mdlQuad.I(2,2)                    0     -Par_mdlQuad.l/Par_mdlQuad.I(2,2); ...
               Par_mdlQuad.gamma/Par_mdlQuad.I(3,3) -Par_mdlQuad.gamma/Par_mdlQuad.I(3,3) Par_mdlQuad.gamma/Par_mdlQuad.I(3,3) -Par_mdlQuad.gamma/Par_mdlQuad.I(3,3)];