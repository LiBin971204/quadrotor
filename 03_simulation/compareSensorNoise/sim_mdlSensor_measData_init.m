% Select dataset
tmp         = dlmread('05_data/a_measurement/imu04.csv', ';', 2, 0);
% rangeAcc    = 0.061/1000*9.81; % m/s^2
% rangeGyro   = 0.00875;    % degree/s
rangeAcc = 1;
rangeGyro = pi/180;
par.g = 9.81;

time        = tmp(:,1)/1000; % Time [s]
acc_x       = tmp(:,2)*rangeAcc;
acc_y       = tmp(:,3)*rangeAcc;
acc_z       = tmp(:,4)*rangeAcc;
gyro_x      = tmp(:,5)*rangeGyro;
gyro_y      = tmp(:,6)*rangeGyro;
gyro_z      = tmp(:,7)*rangeGyro;
