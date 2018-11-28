function Par = mdl_control_lqr_par( SystemDynamics )
% function Par = mdl_control_lqr_par( systemDynamics )
%
%   Date        : Winter 2018
%
%   Description : Sets the feedback gain matrix obtained via LQR synthesis
% 
%   Parameters  : SystemDynamics -> Struct containing linear state space
%                                   matrices
% 
%   Return      : Par -> Struct containing the feedback gains
%
%-------------------------------------------------------------------------%

A = SystemDynamics.mA;
B = SystemDynamics.mB;

% Get subsystem that is to be controlled (exclude x,y and psi dynamics)
A([1:2 4:5 9 12],:) = [];
A(:,[1:2 4:5 9 12]) = [];
B([1:2 4:5 9 12],:) = [];

% TODO: Scale system
% Design LQR
Q = eye(6);
R = 0.5*eye(4);

[k, S, e] = lqr(A, B, Q, R);
% disp(e);
Par.k = k;
Par.ew = e;
Par.S = S;