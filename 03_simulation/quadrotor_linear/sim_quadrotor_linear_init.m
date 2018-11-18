clear all; 
close all; 
clc;

load('00_config/bus_init.mat');                         % Load bus definitions



Par_mdlQuad = mdl_quadrotor_par();    % Load quadrotor model parameters
% Set linear system dynamics
x0                  = [0;0;1;zeros(9,1)];
u0                  = Par_mdlQuad.m*Par_mdlQuad.g/4*ones(4,1);
[~, Ode_lin]        = mdl_quadrotor_dynamicsLin();
Ode_lin.SystemDynamics = mdl_quadrotor_dynamicsLin(x0, u0, Par_mdlQuad);


Par_filterKalman = mdl_filter_kalmanIntegration_par();  % Load kalman filter parameters
Par_filterMadgwick = mdl_filter_madgwick_par();         % Load complementary filter parameters

Par_control_pd = mdl_control_pd_par();                 % Load complementary filter parameters
Par_control_lqr = mdl_control_lqr_par( Ode_lin.SystemDynamics );                 % Load complementary filter parameters