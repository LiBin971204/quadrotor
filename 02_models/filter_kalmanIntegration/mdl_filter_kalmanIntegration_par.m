function Par = mdl_filter_kalmanIntegration_par( )
% function Par = mdl_filter_kalmanIntegration_par( )
%
%   Date        : Winter 2018
%
%   Description : Loads the parameters of the strap down integration Kalman 
%                 filter
% 
%   Parameters  : None
% 
%   Return      : Par -> Struct containing kalman filter parameters
%
%-------------------------------------------------------------------------%

Par.T = 4.0000e-03;
Par.A = [1 -Par.T; 0 1];
Par.B = [Par.T; 0];
Par.C = [1 0];
Par.D = 0;

% Calc Kalman feedback matrix L via LQR
Q                          = [0.01 0; 0 1e-02];
R                          = 20;
[Par.L, ~, ~] = dlqr(Par.A.', ...
                             Par.C.', Q, R);
Par.L         = Par.L.';
clear Q R