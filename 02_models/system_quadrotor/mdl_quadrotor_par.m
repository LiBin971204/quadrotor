function Par = mdl_quadrotor_par( )
% function Par_mdlQuad = mdl_quadrotor_par( )
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

% Define flags
flags_mdl_quadrotor_par.define.printPlots = 1; % TODO: Make different


% Set basic parameters
Par.m     = 0.55;             % Quadrocopter mass [kg]
Par.g     = 9.81;             % Earth acceleration [m*s^-2]
Par.l     = 0.086;            % Distance to rotor from center point [m]
Par.F0    = Par.m*Par.g/4;    % Equilibrium point forces [kg*m*s^-2]
Par.gamma = 1e-4;             % Correlation factor between thrust and momentum

% Inertia matrix
Par.I     = [2.500e-04     0     2.550e-06;
                 0     2.320e-04     0; 
             2.550e-06     0     3.738e-04];
         
% Inverse of inertia matrix
Par.invI    = Par.I^-1;



% Set identified ESC/BLDC maps
load('00_config/staticMap_escBLDC.mat');    % Load identified ESC+BLDC static map
load('05_data\c_identification\bldc_nonLinearHammerWienerstein_v01.mat')

coeffs      = [staticMap_escBLDC.coeffValues{:}];
coeffsRoot  = coeffs-[0 0 0 Par.m/4];
coeffsDer   = polyder(coeffs);
PWM0        = roots(coeffsRoot);
Par.PWM0    = PWM0(3)*staticMap_escBLDC.stdx+staticMap_escBLDC.meanx;
linearTerm  = polyval(coeffsDer, PWM0(3))/staticMap_escBLDC.stdx;

Par.coeffs  = coeffs;
Par.std     = staticMap_escBLDC.stdx;
Par.mean    = staticMap_escBLDC.meanx;

x           = 0:1000;
y           = polyval(coeffs, (x-staticMap_escBLDC.meanx)/staticMap_escBLDC.stdx);

if flags_mdl_quadrotor_par.define.printPlots
    figure
    plot(x,y)
    grid on
    hold on
    plot(x, linearTerm*x+(Par.m/4-linearTerm*Par.PWM0))
    jo = struct(nlhw1);
    plot(jo.OutputNonlinearity.Breakpoints(1,:)/250, jo.OutputNonlinearity.Breakpoints(2,:))
end
Par.linearTerm = linearTerm;
Par.ca_esc = [                  1                     1                    1                      1; ...
                   Par.l/Par.I(1,1)                    0     -Par.l/Par.I(1,1)                     0; ...
                                 0       Par.l/Par.I(2,2)                    0     -Par.l/Par.I(2,2); ...
               Par.gamma/Par.I(3,3) -Par.gamma/Par.I(3,3) Par.gamma/Par.I(3,3) -Par.gamma/Par.I(3,3)] ...
               *linearTerm;
Par.ca = [                  1                     1                    1                      1; ...
                   Par.l/Par.I(1,1)                    0     -Par.l/Par.I(1,1)                     0; ...
                                 0       Par.l/Par.I(2,2)                    0     -Par.l/Par.I(2,2); ...
               Par.gamma/Par.I(3,3) -Par.gamma/Par.I(3,3) Par.gamma/Par.I(3,3) -Par.gamma/Par.I(3,3)];