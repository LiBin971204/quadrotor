filterKalman_par.T = 1.0000e-02;
filterKalman_par.A = [1 -filterKalman_par.T; 0 1];
filterKalman_par.B = [filterKalman_par.T; 0];
filterKalman_par.C = [1 0];
filterKalman_par.D = 0;

% Calc Kalman feedback matrix L via LQR
Q                          = [0.01 0; 0 0.001];
R                          = 200;
[filterKalman_par.L, ~, ~] = dlqr(filterKalman_par.A.', ...
                             filterKalman_par.C.', Q, R);
filterKalman_par.L         = filterKalman_par.L.';
clear Q R