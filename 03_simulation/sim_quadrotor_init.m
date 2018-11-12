clear all; 
close all; 
clc;

% load('00_config/nlhw1.mat');                          % Load identified ESC+BLDC dynamics
load('00_config/bus_init.mat');                         % Load bus definitions

[par_mdlQuad, par_mdlQuadLin] = mdl_quadrotor_par();    % Load quadrotor model parameters
par_filterKalman = mdl_filter_kalmanIntegration_par();  % Load kalman filter parameters
par_filterMadgwick = mdl_filter_madgwick_par();         % Load complementary filter parameters
par_controlLqr = mdl_control_lqr_par();                 % Load complementary filter parameters