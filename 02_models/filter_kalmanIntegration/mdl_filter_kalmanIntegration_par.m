par_filterKalman.T = 1.0000e-02;
par_filterKalman.A = [1 -par_filterKalman.T; 0 1];
par_filterKalman.B = [par_filterKalman.T; 0];
par_filterKalman.C = [1 0];
par_filterKalman.D = 0;

% Calc Kalman feedback matrix L via LQR
Q                          = [0.01 0; 0 0.001];
R                          = 200;
[par_filterKalman.L, ~, ~] = dlqr(par_filterKalman.A.', ...
                             par_filterKalman.C.', Q, R);
par_filterKalman.L         = par_filterKalman.L.';
clear Q R