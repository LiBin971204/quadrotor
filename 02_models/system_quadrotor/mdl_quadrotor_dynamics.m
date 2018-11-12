function [vXdot, vY, varargout] = mdl_quadrotor_dynamics( vX, vU, Par_mdlQuadrotor )
% function [vXdot, vY, varargout] = mdl_quadrotor_dynamics( vX, vU, Par_mdlQuadrotor )
%
%   Date        : Winter 2018
%
%   Description : Calculates the system dynamics and outputs of a 
%                 physical quadrotor model
% 
%   Parameters  : vX -> Vector containing the system states x
%                 vU -> Vector containing the control inputs u
%                 Par_mdlQuadrotor -> Struct containing model parameter
% 
%   Return      : vDotx -> Vector containing state derivatives
%                 vY -> Vector containing system outputs
%                 varargout -> Cell containing information about
%                              dynamic system
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
    Ode.name                 = 'quadrotor';
    Ode.type                 = 'ode';
    
    % Set output
    vXdot                    = 'Call for system size only';
    vY                       = 'Call for system size only';
    varargout                = {Ode};
    
else 
    % Extract necessary states for system dynamics
    v_rel       = vX(4:6);
    eulerAngles = vX(7:9);
    omegaRel    = vX(10:12);

    % Rotate gravity vector to body frame
    g_bodyFrame = rot_euler([0;0;-Par_mdlQuadrotor.g], eulerAngles, 'to-body-frame');

    % Calculate system dynamics
    x_geo_dot   = rot_euler(v_rel, eulerAngles, 'to-inertial-frame');
    v_rel_dot   = g_bodyFrame + [0; 0; 1/m*sum(vU)] - cross(omegaRel, v_rel);
    eulerAngles_dot = rot_karman(omega_rel, eulerAngles, 'to-inertial-frame');
    omegaRel_dot = [l*(F(2) - F(4)); l*(F(3) - F(1)); M(1)-M(2)+M(3)-M(4)] - ...
                    cross(omegaRel, Par_mdlQuadrotor.theta_b * omegaRel);

    % Set function outputs
    vXdot = [x_geo_dot; v_rel_dot; eulerAngles_dot; omegaRel_dot];
    vY    = [omegaRel; v_rel_dot];
    
end