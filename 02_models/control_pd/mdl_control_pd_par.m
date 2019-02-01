function Par = mdl_control_pd_par( B, ParMdlQuad )
% function Par = mdl_control_lqr_par( B )
%
%   Last change : Winter 2018
%
%   Description : Sets the feedback gain matrix for the PD-Controller
% 
%   Parameters  : B -> State space input matrix
% 
%                 ParMdlQuad -> Struct containing model parameters
% 
%   Return      : Par -> Struct containing the feedback gains
%
%-------------------------------------------------------------------------%
% Use shorter variables
I    = ParMdlQuad.I;
m    = ParMdlQuad.m;
L    = ParMdlQuad.armLength;
gam  = ParMdlQuad.gamma;

% Get subsystem that is to be controlled (exclude x and y dynamics, see
% documentation)
B([1:5 7:9],:) = [];
Par.B          = B;

% Inverse of input matrix
% Numerically: If model is adapted result will automatically be adapted
Par.Binv = B^-1;
% Analytically: More exact
invE     = 1/4*[1 0 -2 1; 1 2 0 -1; 1 0 2 1; 1 -2 0 -1];
Par.Binv = invE * [1 0 0 0; 0 1/L 0 0; 0 0 1/L 0; 0 0 0 1/gam] * ...
           [m zeros(1,3); zeros(3,1) I];

% PD-Control parameters 
% Par.kp   = 25;
% Par.kd   = 10;
Par.kp   = 12.25;
Par.kd   = 7;

