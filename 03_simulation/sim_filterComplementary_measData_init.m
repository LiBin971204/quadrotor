clear all; clc; close all;

tmp = dlmread('imu07.csv', ';', 2, 0);
set = 0;
rangeAcc = 0.061/1000*9.81; % m/s^2
rangeGyro = 0.00875;    % degree/s

if set == 0
    time0   = tmp(:,1)/1000;
    time    = (0:0.006:time0(end)).';
    acc_x   = interp1(time0, tmp(:,2)*rangeAcc, time,'linear', 'extrap');
    acc_y   = interp1(time0, tmp(:,3)*rangeAcc, time,'linear','extrap');
    acc_z   = interp1(time0, tmp(:,4)*rangeAcc, time,'linear','extrap');
    gyro_x  = interp1(time0, tmp(:,5)*rangeGyro, time,'linear','extrap');
    gyro_y  = interp1(time0, tmp(:,6)*rangeGyro, time,'linear', 'extrap');
    gyro_z  = interp1(time0, tmp(:,7)*rangeGyro, time, 'linear', 'extrap');
    
else
    load('ExampleData.mat');
    load('Example_euler.mat');
    euler   = euler/180*pi;
    acc_x   = Accelerometer(:,1)*-9.81;
    acc_y   = Accelerometer(:,2)*-9.81;
    acc_z   = Accelerometer(:,3)*-9.81;
    gyro_x  = Gyroscope(:,1)/180*pi;
    gyro_y  = Gyroscope(:,2)/180*pi;
    gyro_z  = Gyroscope(:,3)/180*pi;
end
w0 = 2;  % Eigenfrequency [rad/s]


w0s = 0.1:0.1:10;

for ii = 1:length(w0s)
    w0 = w0s(ii);
    sim('mdl_complementaryFilter')
    est(:,ii) = estimate;   
end


figure
plot(est)
hold on
plot(signal)
grid on
xlim([0 3000])