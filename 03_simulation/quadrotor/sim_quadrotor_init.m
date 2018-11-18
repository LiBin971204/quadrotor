clear all; 
close all; 
clc;

load('00_config/bus_init.mat');                         % Load bus definitions

Par_mdlQuad = mdl_quadrotor_par();    % Load quadrotor model parameters
Par_filterKalman = mdl_filter_kalmanIntegration_par();  % Load kalman filter parameters
Par_filterMadgwick = mdl_filter_madgwick_par();         % Load complementary filter parameters
Par_control_pd = mdl_control_pd_par();                 % Load complementary filter parameters