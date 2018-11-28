function Par = mdl_control_pd_par( )
% function Par = mdl_control_lqr_par( )
%
%   Date        : Winter 2018
%
%   Description : Sets the feedback gain matrix for the PD-Controller
% 
%   Parameters  : None
% 
%   Return      : Par -> Struct containing the feedback gains
%
%-------------------------------------------------------------------------%

% k = [-1 -2];                  % PD-Control parameters 
% k = [-1 -2.5];                % PD-Control parameters 
% par_ctrlLqr.k = [-4 -8];      % PD-Control parameters 
Par.k = [-2 -5];     % PD-Control parameters 