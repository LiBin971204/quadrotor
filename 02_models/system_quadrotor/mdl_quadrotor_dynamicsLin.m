function [mA, mB, mC, mD, varargout] = mdl_quadrotor_dynamicsLin( vx0, Par_mdlQuadrotorLin )
% function [mA, mB, mC, mD, varargout] = mdl_quadrotor_dynamicsLin( vx0, Par_mdlQuadrotorLin )
%
%   Date        : Winter 2018
%
%   Description : Calculates the system dynamics and outputs of a 
%                 physical quadrotor model linearized around x0
% 
%   Parameters  : vx -> Vector containing the system states x
%                 vu -> Vector containing the control inputs u
%                 vx0 -> Vector containing linearization point
%                 Par_mdlQuadrotor -> Struct containing model parameter
% 
%   Return      : DynamicsLin -> A struct containing the system dynamics 
%                                in linear state space form with the 
%                                matrices A, B, C and D
%
%-------------------------------------------------------------------------%

if nargin==0
    % Set system size for query
    Ode.Size.n_systemStates  = 12;
    Ode.Size.n_controlInputs = 4;
    Ode.Size.n_algebraicVars = 0;
    Ode.Size.n_systemOutputs = 6;
    Ode.Size.n_parOn         = 0;
    Ode.Size.n_algebraicEqs  = 0;
    Ode.Info.manipulatedVars = {'thrust_motor1', 'thrust_motor2', ...
                                'thrust_motor3', 'thrust_motor4'};
    Ode.Info.systemStates    = {'x_geodetic', 'y_geodetic', 'z_geodetic', ...
                                'u', 'v', 'w', ...
                                'phi', 'theta', 'psi', ...
                                'p', 'q', 'r'};
    Ode.Info.systemOutputs   = {'p', 'q', 'r', 'uDot', 'vDot', 'wDot'};
    Ode.Scaling.states       = [1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1];
    Ode.Scaling.controls     = [1; 1; 1; 1];
    Ode.Scaling.outputs      = [1; 1];
    Ode.name                 = 'quadrotor_linearized';
    Ode.type                 = 'ode';
    
    % Set output
    vxdot                    = 'Call for system size only';
    vy                       = 'Call for system size only';
    varargout                = {Ode};
    
else 
    % Extract necessary states for system dynamics
    v_rel       = vx(4:6);
    eulerAngles = vx(7:9);
    omegaRel    = vx(10:12);
    
    I = Par_mdlQuadrotorLin.Theta_b;

    % Calculate A
    dfdx_geo = zeros(12, 3);
    dfdv_rel = [cos(theta)*cos(psi) ...
                sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) ...
                cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
                cos(theta)*sin(psi) ...
                sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) ...
                cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
                -sin(theta) ...
                sin(phi)*cos(theta) ...
                cos(phi)*cos(theta);
                0 r -q; ...
                -r 0 p; ...
                q -p 0; ...
                zeros(6,3)];
    dfdeuler = [];
    dfdomega = [zeros(3,3); ...
                0 -w v; ...
                w 0 -u; ...
                -v u 0; ...
                1 sin(phi)*tan(theta) cos(phi)*tan(theta); ...
                0 cos(phi) -sin(phi); ...
                0 sin(phi)/cos(theta) sin(phi)/cos(theta); ...
                  -I(3,1)*q +   I(2,1)*r  -2*I(3,2)*q +  I(2,2)*r    -I(3,3)*q + 2*I(2,3)*r;...
                  -I(1,1)*r + 2*I(3,1)*p    -I(1,2)*r +  I(3,2)*p  -2*I(1,3)*r + I(3,3)*p;...
                -2*I(2,1)*p +   I(1,1)*q    -I(2,2)*p +2*I(1,2)*q    -I(2,3)*p + I(1,3)*q];
    mA = [dfdx_geo dfdv_rel dfdeuler dfdomega];
    
    % Calculate B
    mB = [zeros(5,4); 1/m*ones(1,4); zeros(3,4); 0 l 0 -l; -l 0 l 0; dm1 -dm2 dm3 -dm4];
    
    % Set C
    mC = [zeros(3, 9) eye(3)];

    % Set function outputs
    mD = zeros(3);
end