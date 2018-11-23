clear all; 
close all; 
clc;

% Load bus definitions
load('00_config/bus_init.mat');

% Load quadrotor model parameters
Par_mdlQuad = mdl_quadrotor_par();

% Load kalman filter parameters
Par_filterKalman = mdl_filter_kalmanIntegration_par();

% Load complementary filter parameters
Par_filterMadgwick = mdl_filter_madgwick_par();

% Load complementary filter parameters
Par_control_pd = mdl_control_pd_par();

% Load esc-bldc parameter
Par_actuator = mdl_actuator_thrust_par();