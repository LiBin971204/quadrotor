clear all; 
close all;
clc; 
load('00_config/bus_init.mat'); % Load bus definitions

run('mdl_filter_kalmanIntegration_par'); % Load kalman filter parameters
run('mdl_filter_madgwick_par'); % Load complementary filter parameters


% Select test data
selSet = 4;
switch selSet
    case 1
        load('05_data/b_simulation/exampleData') % Load sample data
    case 2
        tmp         = dlmread('05_data/a_measurement/imu07.csv', ';', 2, 0);
        rangeAcc    = -0.061/1000*9.81; % m/s^2
        rangeGyro   = 0.00875/180*pi;    % rad/s
        % rangeAcc = 1;
        % rangeGyro = 1;
        time        = tmp(:,1)/1000; % Time [s]
        acc(:,1)    = tmp(:,2)*rangeAcc;
        acc(:,2)    = tmp(:,3)*rangeAcc;
        acc(:,3)    = tmp(:,4)*rangeAcc;
        gyro(:,1)   = tmp(:,5)*rangeGyro;
        gyro(:,2)   = tmp(:,6)*rangeGyro;
        gyro(:,3)   = tmp(:,7)*rangeGyro;
        time_rsmpld = (time(1):10e-03:time(end)).';
        acc         = interp1(time, acc, time_rsmpld);
        gyro        = interp1(time, gyro, time_rsmpld);
        time        = time_rsmpld-time_rsmpld(1);
        
    case 3
        tmp         = dlmread('05_data/a_measurement/2018-10-03_imu_checkDir.csv', ';', 0, 0);
        rangeAcc    = -1;
        rangeGyro   = pi/180;
        time        = tmp(:,1)*1e-06; % Time [s]
        acc(:,1)    = tmp(:,2)*rangeAcc;
        acc(:,2)    = tmp(:,3)*rangeAcc;
        acc(:,3)    = tmp(:,4)*rangeAcc;
        gyro(:,1)   = tmp(:,5)*rangeGyro;
        gyro(:,2)   = tmp(:,6)*rangeGyro;
        gyro(:,3)   = tmp(:,7)*rangeGyro;
        time_rsmpld = (time(1):4e-03:time(end)).';
        acc         = interp1(time, acc, time_rsmpld);
        gyro        = interp1(time, gyro, time_rsmpld);
        time        = time_rsmpld-time_rsmpld(1);
        
    case 4
        tmp         = dlmread('05_data/a_measurement/2018-10-03_imu_02.csv', ';', 0, 0);
        rangeAcc    = -1;
        rangeGyro   = pi/180;
        time        = tmp(:,1)*1e-06; % Time [s]
        acc(:,1)    = tmp(:,2)*rangeAcc;
        acc(:,2)    = tmp(:,3)*rangeAcc;
        acc(:,3)    = tmp(:,4)*rangeAcc;
        gyro(:,1)   = tmp(:,5)*rangeGyro;
        gyro(:,2)   = tmp(:,6)*rangeGyro;
        gyro(:,3)   = tmp(:,7)*rangeGyro;
        time_rsmpld = (time(1):10e-03:time(end)).';
        acc         = interp1(time, acc, time_rsmpld);
        gyro        = interp1(time, gyro, time_rsmpld);
        time        = time_rsmpld-time_rsmpld(1);
end