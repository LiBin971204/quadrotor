function Par = mdl_control_pd_par( B )
% function Par = mdl_control_lqr_par( B )
%
%   Date        : Winter 2018
%
%   Description : Sets the feedback gain matrix for the PD-Controller
% 
%   Parameters  : SystemDynamics -> Struct containing linear state space
%                                   matrices
% 
%   Return      : Par -> Struct containing the feedback gains
%
%-------------------------------------------------------------------------%

% Get subsystem that is to be controlled (exclude x and y dynamics)
B([1:5 7:9],:) = [];

% jo = [1 1 1 1; 1 0 -1 0; 0 1 0 -1; 1 -1 1 -1];
% jo^-1

% k = [-1 -2];                  % PD-Control parameters 
% k = [-1 -2.5];                % PD-Control parameters 
% par_ctrlLqr.k = [-4 -8];      % PD-Control parameters 
Par.k    = [-2 -5];     % PD-Control parameters 
Par.B    = B;
Par.Binv = B^-1;