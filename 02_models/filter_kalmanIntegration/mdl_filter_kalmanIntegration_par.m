function Par_filterKalman = mdl_filter_kalmanIntegration_par( )
% function Par_filterKalman = mdl_filter_kalmanIntegration_par( )
%
% Date:    Winter 2018
%
% Description: 	Loads the parameters of the strap down integration Kalman 
%               filter
% 
% Parameters : 	None
% 
% Return     : 	Par_filterKalman - a struct which contains the parameters
% 
% Examples of Usage: None
%
%-------------------------------------------------------------------------%

Par_filterKalman.T = 4.0000e-03;
Par_filterKalman.A = [1 -Par_filterKalman.T; 0 1];
Par_filterKalman.B = [Par_filterKalman.T; 0];
Par_filterKalman.C = [1 0];
Par_filterKalman.D = 0;

% Calc Kalman feedback matrix L via LQR
Q                          = [0.01 0; 0 1e-02];
R                          = 20;
[Par_filterKalman.L, ~, ~] = dlqr(Par_filterKalman.A.', ...
                             Par_filterKalman.C.', Q, R);
Par_filterKalman.L         = Par_filterKalman.L.';
clear Q R