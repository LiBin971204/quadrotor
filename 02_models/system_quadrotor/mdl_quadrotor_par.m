%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           Model Parmeters                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
par.m       = 0.55;             % Quadrocopter mass [kg]
par.g       = 9.81;             % Earth acceleration [m*s^-2]
par.I       = [2.500e-04,         0, 2.550e-06; ...
                       0, 2.320e-04,         0; ...
               2.550e-06,         0, 3.738e-04];    % Inertia matrix
par.gamma   = 1e-4;
 
par.invI    = par.I^-1;         % Inverse of inertia matrix
par.l       = 0.086;            % Distance to rotor from center point [m]

k           = [-1 -2];          % PD-Control parameters 
par.F0      = par.m*par.g/4;    % Equilibrium point forces [kg*m*s^-2]
%-------------------------------------------------------------------------%




coeffs      = [staticMap_escBLDC.coeffValues{:}];
coeffsRoot  = coeffs-[0 0 0 par.m/4];
coeffsDer   = polyder(coeffs);
PWM0        = roots(coeffsRoot);
par.PWM0    = PWM0(3)*staticMap_escBLDC.stdx+staticMap_escBLDC.meanx;
linearTerm  = polyval(coeffsDer, PWM0(3))/staticMap_escBLDC.stdx;

par.coeffs  = coeffs;
par.std     = staticMap_escBLDC.stdx;
par.mean    = staticMap_escBLDC.meanx;




x           = 0:1000;
y           = polyval(coeffs, (x-staticMap_escBLDC.meanx)/staticMap_escBLDC.stdx);

figure
plot(x,y)
grid on
hold on
plot(x, linearTerm*x+(par.m/4-linearTerm*par.PWM0))
jo = struct(nlhw1);
plot(jo.OutputNonlinearity.Breakpoints(1,:)/250, jo.OutputNonlinearity.Breakpoints(2,:))
 
par.CA      = [                  1                     1                    1                      1; ...
                   par.l/par.I(1,1)                    0     -par.l/par.I(1,1)                     0; ...
                                 0       par.l/par.I(2,2)                    0     -par.l/par.I(2,2); ...
               par.gamma/par.I(3,3) -par.gamma/par.I(3,3) par.gamma/par.I(3,3) -par.gamma/par.I(3,3)] ...
               *linearTerm;

           
% Linearized system
Ap  = [0 1; 0 0];
Bp  = [0 0 0 0; par.l/par.I(1,1) 0 -par.l/par.I(1,1) 0]*linearTerm;

Aq  = [0 1; 0 0];
Bq  = [0 0 0 0; 0 par.l/par.I(2,2) 0 -par.l/par.I(2,2)]*linearTerm;

Ar  = [0 1; 0 0];
Br  = [0 0 0 0; par.gamma/par.I(1,1) -par.gamma/par.I(1,1) par.gamma/par.I(1,1) -par.gamma/par.I(1,1)]*linearTerm;

Az  = [0 1; 0 0];
Bz  = [0 0 0 0; 1 1 1 1]*linearTerm;

C   = [1 0;0 1];
D   = zeros(2,4);