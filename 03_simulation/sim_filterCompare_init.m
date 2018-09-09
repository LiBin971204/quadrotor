clear all; clc; close all;
load('00_config/bus_init.mat'); % Load bus definitions
load('05_data/b_simulation/exampleData') % Load sample data

run('mdl_filter_kalmanIntegration_par'); % Load kalman filter parameters
run('mdl_filter_complementary_par'); % Load complementary filter parameters