clear all; close all; clc;
load('00_config/nlhw1.mat');    % Load identified ESC+BLDC dynamics
load('00_config/bus_init.mat'); % Load bus definitions
load('00_config/staticMap_escBLDC.mat');    % Load identified ESC+BLDC static map

run('mdl_quadrotor_par');   % Load quadrotor model parameters
run('mdl_filter_kalmanIntegration_par'); % Load kalman filter parameters
run('mdl_filter_complementary_par'); % Load complementary filter parameters