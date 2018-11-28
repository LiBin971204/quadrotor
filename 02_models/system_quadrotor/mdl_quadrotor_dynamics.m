function [x_dot, y, varargout] = mdl_quadrotor_dynamics( x, u, Par_mdlQuad )
% function [x_dot, y, varargout] = mdl_quadrotor_dynamics( x, u, Par_mdlQuad )
%
%   Date        : Winter 2018
%
%   Description : Calculates the system dynamics and outputs of a 
%                 physical quadrotor model
% 
%   Parameters  : x -> Vector containing the system states x
%                 u -> Vector containing the control inputs u
%                 Par_mdlQuad -> Struct containing model parameter
% 
%   Return      : x_dot -> Vector containing state derivatives
%                 y -> Vector containing system outputs
%                 varargout -> Cell containing information about
%                              dynamic system
%
%-------------------------------------------------------------------------%

if nargin==0
    % Set system size for query
    Ode.Size.n_states   = 12;
    Ode.Size.n_inputs   = 4;
    Ode.Size.n_outputs  = 6;
    Ode.Size.n_onlineParameter = 0;
    Ode.Info.inputs     = {'thrust_motor1'; 'thrust_motor2'; ...
                           'thrust_motor3'; 'thrust_motor4'};
    Ode.Info.states     = {'x_geodetic'; 'y_geodetic'; 'z_geodetic'; ...
                           'u_body'; 'v_body'; 'w_body'; ...
                           'phi'; 'theta'; 'psi'; ...
                           'p_body'; 'q_body'; 'r_body'};
    Ode.Info.outputs    = {'p_body'; 'q_body'; 'r_body'; ...
                           'u_body_dot'; 'v_body_dot'; 'w_body_dot'};
    Ode.Scaling.states  = [1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1];
    Ode.Scaling.inputs  = [1; 1; 1; 1];
    Ode.Scaling.outputs = [1; 1];
    Ode.name            = 'quadrotor';
    Ode.type            = 'ode';
    
    % Set output
    msg = 'Call for system size. Add an additional output argument';
    x_dot               = msg;
    y                   = msg;
    varargout           = {Ode};
    
else 
    % Extract necessary states for system dynamics
    M           = zeros(4,1); % TODO: Get info about motor torque
    F           = u; % TODO: Replace by correct correlation
    M           = (u(1)-u(3)+u(2)-u(4))*Par_mdlQuad.gamma;
    v_body      = x(4:6);
    eulerAngles = x(7:9);
    omega_body  = x(10:12);

    % Rotate gravity vector to body frame
    g_bodyFrame = rot_euler([0;0;-Par_mdlQuad.g], eulerAngles, 'to-body-frame');

    % Calculate system dynamics
    x_geo_dot ...
        = rot_euler(v_body, eulerAngles, 'to-inertial-frame');
    
    v_body_dot ...
        = g_bodyFrame + [0; 0; 1/Par_mdlQuad.m*sum(F)] - cross(omega_body, v_body);
    
    eulerAngles_dot ...
        = rot_karman(omega_body, eulerAngles, 'to-inertial-frame');
    
    % TODO: Adapt thrust indices according to notes
    omega_body_dot ...
        = Par_mdlQuad.invI*([Par_mdlQuad.l*(F(1)-F(3)); Par_mdlQuad.l*(F(2)-F(4)); ...
           M] - cross(omega_body, Par_mdlQuad.I*omega_body));

    % Set function outputs
    x_dot = [x_geo_dot; v_body_dot; eulerAngles_dot; omega_body_dot];
    y     = [omega_body; v_body_dot];
    
end