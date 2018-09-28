clear all; 
close all;
clc; 
load('00_config/bus_init.mat'); % Load bus definitions

run('mdl_filter_kalmanIntegration_par'); % Load kalman filter parameters
run('mdl_filter_madgwick_par'); % Load complementary filter parameters


% Select test data
selSet = 1;
switch selSet
    case 1
        load('05_data/b_simulation/exampleData') % Load sample data
    case 2
        tmp         = dlmread('05_data/a_measurement/imu07.csv', ';', 2, 0);
        rangeAcc    = 0.061/1000*9.81; % m/s^2
        rangeGyro   = 0.00875/180*pi;    % rad/s
        % rangeAcc = 1;
        % rangeGyro = 1;
% TODO Resample and try again -> why so bad?
        time        = tmp(:,1)/1000; % Time [s]
        y_new interp1(x,y, xnew)
        acc(:,1)    = tmp(:,2)*rangeAcc;
        acc(:,2)    = tmp(:,3)*rangeAcc;
        acc(:,3)    = tmp(:,4)*rangeAcc;
        gyro(:,1)   = tmp(:,5)*rangeGyro;
        gyro(:,2)   = tmp(:,6)*rangeGyro;
        gyro(:,3)   = tmp(:,7)*rangeGyro;
    case 3
        load('05_data/b_simulation/exampleData_noMag') % Load sample data
end