clear; close all; clc;
% Identification of the electronic speed controller (esc) and brushless
% direct current motor (bldc) unit. Measured data: pwm signal and resulting
% Wäge-sensor voltage. Wägesensor calibration data is also stored. 
%
%   Author1     : 
%
%   Date        : Winter 2018
%
%-------------------------------------------------------------------------%

% Set options
Specs.functionName  = 'mdl_actuator_thrust';
Specs.date          = '2018-07-04';
Specs.sampleTime    = 0.004;
Specs.sampleTime    = 0.1;
Dirs.loadData       = ['05_data/a_measurement/ident_esc-bldc_' Specs.date '/'];
Dirs.saveFun        = '02_models/actuator_esc-bldc/';

% Fit options for model mass = f(sensor output)
MatlabCurveFit(1).Options        ...
    = fitoptions( 'Method', 'LinearLeastSquares', ...
                  'Normalize', 'off', ...
                  'Robust', 'Bisquare');

MatlabCurveFit(1).modelType ...
    = fittype( 'poly1' );

% Fit options for static map mass = f(pwm)
MatlabCurveFit(2).Options        ...
    = fitoptions( 'Method', 'LinearLeastSquares', ...
                  'Normalize', 'off', ...
                  'Robust', 'Bisquare');

MatlabCurveFit(2).modelType ...
    = fittype( 'poly3' );

% Fit options for dynamic model mass_dot = f(pwm)
MatlabIdent.modelType = 'Wiener-Hammerstein';
MatlabIdent.inputNonlinearity = poly1d;
MatlabIdent.outputNonlinearity = 'unitgain';
MatlabIdent.modelOrder = [1 1 0]; % n_zeros+1, n_poles, n_delay




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  Fit calibration curve for sensor                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load calibration data 
[Data_sensorCalibration.sensorOutput, Data_sensorCalibration.mass] ...
    = ident_escbldc_sensorCalibration( Specs.date );

% Fit model
MatlabCurveFit(1).fitMatlab ...
    = fit( Data_sensorCalibration.sensorOutput, Data_sensorCalibration.mass, ...
           MatlabCurveFit(1).modelType, ...
           MatlabCurveFit(1).Options );

% Visualize resulting fit
figure;
plot(Data_sensorCalibration.sensorOutput, Data_sensorCalibration.mass,'.')
hold on
plot(0:250, MatlabCurveFit(1).fitMatlab(0:250))
grid on
xlabel('Sensor output in ?')
ylabel('Mass in kg')
legend('Measured data', 'Polynomial fit')
title('Sensor characteristics')

%-------------------------------------------------------------------------%
    




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Fit static map (pwm - mass)                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load measurement data of each bldc motor (recorded using same esc)
for ii = 1:4
    buff = dlmread(['ident' num2str(ii) '4.csv'],';');
    Data_measurement(ii).num_bldc = ii;
    Data_measurement(ii).num_esc  = 1;
    Data_measurement(ii).time     = buff(:,1)/1000;
    Data_measurement(ii).pwm      = buff(:,2)/1000 - 1; % Scale
    Data_measurement(ii).sensorOutput = buff(:,3)/1000;
    Data_measurement(ii).mass_raw = MatlabCurveFit(1).fitMatlab(buff(:,3)/1000);
    
    % Convert sensor signal to mass with identified model and take out
    % measurement imperfections
    offset_outputThrust ...
        = max(MatlabCurveFit(1).fitMatlab(buff(1:20,3)/1000));
    
    Data_measurement(ii).mass     ...
        = MatlabCurveFit(1).fitMatlab(buff(:,3)/1000) - offset_outputThrust;
    
    offset_outputThrust ...
        = max(MatlabCurveFit(1).fitMatlab(buff(end-16:end,3)/1000));
    
    Data_measurement(ii).mass(end-16:end) ...
        = MatlabCurveFit(1).fitMatlab(buff(end-16:end,3)/1000) - offset_outputThrust;
    
    ind = Data_measurement(ii).mass < 0;
    Data_measurement(ii).mass(ind) = 0;
    
end

% Get static map
% Manually set time windows in which step response reached steady state
limits = [ 3  4;  5  6;  7  8;  9 10; 11 12; ...
          13 14; 15 16; 17 18; 19 20; 21 21.5].';

pwm_steadyState = [];
mass_steadyState = [];
for ii = 1:size(limits,2)
    for jj = 1:4
        ind = Data_measurement(jj).time>=limits(1,ii) & Data_measurement(jj).time<=limits(2,ii);
        pwm_steadyState ...
            = [pwm_steadyState; mean(Data_measurement(jj).pwm(ind))];
        mass_steadyState ...
            = [mass_steadyState; mean(Data_measurement(jj).mass(ind))];
    end
end

% Fit steady state data
MatlabCurveFit(2).fitMatlab ...
    = fit( pwm_steadyState, mass_steadyState, ...
           MatlabCurveFit(2).modelType, ...
           MatlabCurveFit(2).Options );

% Visualize
figure
subplot(2,1,1)
plot([Data_measurement.time], [Data_measurement.mass])
hold on
plot([Data_measurement.time], [Data_measurement.mass_raw] )
grid on
title('Original vs. postprocessed data')


subplot(2,1,2)
plot(pwm_steadyState, mass_steadyState, '.')
hold on
plot(0:0.1:1, MatlabCurveFit(2).fitMatlab(0:0.1:1))
grid on
xlabel('PWM in -')
ylabel('Mass in kg')
title('Static map')
legend('Measured data', 'Polynomial fit')

clear pwm_steadyState mass_steadyState ii jj ind limits buff ...
      offset_outputThrust
% TODO: plot original vs postprocessed in 1 figure, 2 subplots
% TODO: Plot static thrust curve and get inverse in one figure
% TODO: Seperate between mdl_act_inputNonlinearity, mdl_act_dynamics
% and mdl_act_staticMapInverse
%-------------------------------------------------------------------------%





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     Fit dynamic system behavior                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Resampling
MatlabIdent.time = 0;
MatlabIdent.pwm  = [];
MatlabIdent.mass = [];
for ii = 1:4
    Data_measurement(ii).time_resampled ...
        = (0:Specs.sampleTime:Data_measurement(ii).time(end)).';

    Data_measurement(ii).pwm_resampled ...
        = interp1(Data_measurement(ii).time, Data_measurement(ii).pwm, ...
                  Data_measurement(ii).time_resampled);

    Data_measurement(ii).mass_resampled ...
        = interp1(Data_measurement(ii).time, Data_measurement(ii).mass, ...
                  Data_measurement(ii).time_resampled);
              
    MatlabIdent.time = [MatlabIdent.time; MatlabIdent.time(end)+Data_measurement(ii).time_resampled];
    MatlabIdent.pwm  = [MatlabIdent.pwm; Data_measurement(ii).pwm_resampled];
    MatlabIdent.mass = [MatlabIdent.mass; Data_measurement(ii).mass_resampled];
end
MatlabIdent.time(1) = [];


% Identify dynamic behavior
MatlabIdent.inputNonlinearity.Coefficients ...
    = [MatlabCurveFit(2).fitMatlab.p1 ...
       MatlabCurveFit(2).fitMatlab.p2 ...
       MatlabCurveFit(2).fitMatlab.p3 ...
       MatlabCurveFit(2).fitMatlab.p4];

MatlabIdent.identifiedModel ...
    = nlhw( iddata(MatlabIdent.mass, MatlabIdent.pwm, Specs.sampleTime), ...
            MatlabIdent.modelOrder, ...
            MatlabIdent.inputNonlinearity, ...
            MatlabIdent.outputNonlinearity );
        
simOut = sim(MatlabIdent.identifiedModel, MatlabIdent.pwm);


% Visualize
figure
aux_plotSize(14.8, 10.5, 25, 1.5)
subplot(2,1,1)

plot(MatlabIdent.time, MatlabIdent.mass)
hold on
grid on
plot(MatlabIdent.time, simOut)
title('Output')
xlabel('Time in s')
ylabel('Thrust in kg')
ylim([0 0.6])

subplot(2,1,2)
plot(MatlabIdent.time, MatlabIdent.pwm)
grid on
title('Input')
xlabel('Time in s')
ylabel('PWM in -')

clear ii simOut

%-------------------------------------------------------------------------%





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Store model as function                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate input nonlinearity function
coefs = MatlabIdent.identifiedModel.InputNonlinearity.Coefficients;

fid = fopen([Dirs.saveFun Specs.functionName '.m'], 'wt');
fprintf(fid, ['function F_static = ' Specs.functionName '( pwm ) \n']);
fprintf(fid, ['%% function F_static = ' Specs.functionName '( pwm ) \n']);
fprintf(fid, '%%\n');
fprintf(fid, '%%   Author1     : \n');
fprintf(fid, '%%\n');
fprintf(fid, '%%   Date        : Winter 2018\n');
fprintf(fid, '%%\n');
fprintf(fid, '%%   Description : Submodel for esc-bldc dynamics\n');
fprintf(fid, '%%\n');
fprintf(fid, '%%   Parameters  : pwm -> Pulse-width modulated signal\n');
fprintf(fid, '%% \n');
fprintf(fid, '%%   Return      : F_static -> Motor thrust from static map\n');
fprintf(fid, '%%\n');
fprintf(fid, '%%   This function was autogenerated by 04_identification/...\n');
fprintf(fid, '%%   ...actuator_esc-bldc/ident_acutator_thrust.m \n');
fprintf(fid, '%%\n');
fprintf(fid, '%%-------------------------------------------------------------------------%%\n\n');
fprintf(fid, 'g = 9.81; %% Gravity [m/s^2]\n');
fprintf(fid, ['F_static = g * (' num2str(coefs(1)) '.*pwm.^3 + ' ...
                            num2str(coefs(2)) '.*pwm.^2 + ', ...
                            num2str(coefs(3)) '.*pwm + ' num2str(coefs(4)) ');']);
fclose(fid);

% Generate parameter function
coefs = getpvec(MatlabIdent.identifiedModel.LinearModel);
fid = fopen([Dirs.saveFun Specs.functionName '_par.m'], 'wt');
fprintf(fid, ['function Par = ' Specs.functionName '_par( ) \n']);
fprintf(fid, ['%% function Par = ' Specs.functionName '_par( ) \n']);
fprintf(fid, '%%\n');
fprintf(fid, '%%   Author1     : \n');
fprintf(fid, '%%\n');
fprintf(fid, '%%   Date        : Winter 2018\n');
fprintf(fid, '%%\n');
fprintf(fid, '%%   Description : Submodel for esc-bldc dynamics\n');
fprintf(fid, '%%\n');
fprintf(fid, '%%   Parameters  : \n');
fprintf(fid, '%% \n');
fprintf(fid, '%%   Return      : Par -> Struct containing dynamic model parameters\n');
fprintf(fid, '%%\n');
fprintf(fid, '%%   This function was autogenerated by 04_identification/...\n');
fprintf(fid, '%%   ...actuator_esc-bldc/ident_acutator_thrust.m \n');
fprintf(fid, '%%\n');
fprintf(fid, '%%-------------------------------------------------------------------------%%\n\n');
fprintf(fid, ['Par.linearModelCoefs = [' num2str(coefs(1)) '; ' ...
                                         num2str(coefs(2)) '];\n']);
fprintf(fid, ['Par.sampleTime = ' num2str(MatlabIdent.identifiedModel.Ts) ';']);
fclose(fid);

figure
aux_plotSize(14.8, 10.5, 25, 10)

plot(0:0.1:1, MatlabCurveFit(2).fitMatlab(0:0.1:1))
hold on
lim_t_inf = 1/(coefs(1)+coefs(2));
plot(0:0.1:1, mdl_actuator_thrust( 0:0.1:1)*lim_t_inf/9.81)
grid on
xlabel('PWM in -')
ylabel('Mass in kg')
title('Static map')
legend('Polynomial fit', 'Estimated input nonlinearity/(a0+a1)/g', 'location', 'northwest')

clear ii fid ans coefs

%-------------------------------------------------------------------------%