function [xDot, y, varargout] = mdl_quadrotor_dynamics( x, u, ParMdlQuad )
% function [x_dot, y, varargout] = mdl_quadrotor_dynamics( x, u, ParmdlQuad )
%
%   Date        : Winter 2018
%
%   Description : Calculates the system dynamics and outputs of a 
%                 physical quadrotor model
% 
%   Parameters  : x -> Vector containing the system states x
%                 u -> Vector containing the control inputs u
%                 ParMdlQuad -> Struct containing model parameter
% 
%   Return      : xDot -> Vector containing state derivatives
%                 y -> Vector containing system outputs
%                 varargout -> Cell containing information about
%                              dynamic system
%
%-------------------------------------------------------------------------%

if nargin==0
    % Set system size for query
    Ode.Size.states     = 12;
    Ode.Size.inputs     = 4;
    Ode.Size.outputs    = 6;
    Ode.Size.parameters = 0;
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
    xDot               = msg;
    y                   = msg;
    varargout           = {Ode};
    
else 
    % Extract necessary states for system dynamics
    F           = u;
    M           = (u(1)-u(2)+u(3)-u(4))*ParMdlQuad.gamma; % TODO: Get info about motor torque
    L           = ParMdlQuad.armLength;
    vBody       = x(4:6);
    eulerAngles = x(7:9);
    omegaBody   = x(10:12);

    % Rotate gravity vector to body frame
    gBodyFrame = rot_euler([0;0;-ParMdlQuad.g], eulerAngles, 'to-body-frame');

    % Calculate system dynamics
    xGeoDot ...
        = rot_euler(vBody, eulerAngles, 'to-inertial-frame');
    
    vBodyDot ...
        = gBodyFrame + [0; 0; 1/ParMdlQuad.m*sum(F)] - cross(omegaBody, vBody);
    
    eulerAnglesDot ...
        = rot_karman(omegaBody, eulerAngles, 'to-inertial-frame');
    
    % TODO: Adapt thrust indices according to notes
    omegaBodyDot ...
        = ParMdlQuad.invI*([L*(F(2)-F(4)); L*(-F(1)+F(3)); M] ...
          - cross(omegaBody, ParMdlQuad.I*omegaBody));

    % Set function outputs
    xDot = [xGeoDot; vBodyDot; eulerAnglesDot; omegaBodyDot];
    y    = [omegaBody; vBodyDot];
    
end