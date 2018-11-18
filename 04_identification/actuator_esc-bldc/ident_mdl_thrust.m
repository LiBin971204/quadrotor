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
Specs.functionName                = 'mdl_actuator_thrust';
Specs.date                        = '2018-07-04';
Dirs.loadData                     = ['05_data/a_measurement/ident_esc-bldc_' Specs.date '/'];
Dirs.saveData                     = '05_data/c_identification/ident_esc-bldc/';
Dirs.saveFun                      = '02_models/actuator_esc-bldc/';

Par_matlabCurveFit(1).Options        ...
    = fitoptions( 'Method', 'LinearLeastSquares' );

Par_matlabCurveFit(1).Options.Robust ...
    = 'Bisquare';

Par_matlabCurveFit(1).modelType ...
    = fittype( 'poly1' );



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                Load and preprocess measurement data                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if exist([Dirs.saveData 'identData_actuator_thrust.mat'], 'file')==2  
    load([Dirs.saveData 'identData_actuator_thrust.mat'])
    
else
    % Load calibration
    [SensorCalibration.sensorOutput, SensorCalibration.mass] ...
        = ident_escbldc_sensorCalibration( Specs.date );
    
    Result_matlabCureFit(1).fitMatlab ...
        = fit( SensorCalibration.sensorOutput, SensorCalibration.mass, ...
               Par_matlabCurveFit(1).modelType, ...
               Par_matlabCurveFit(1).Options );
    
    SensorCalibration.figure = figure;
    plot(SensorCalibration.sensorOutput, SensorCalibration.mass,'.')
    hold on
    plot(0:250, Result_matlabCureFit(1).fitMatlab(0:250))
    xlabel('Sensor output in ?')
    ylabel('Mass in kg')
    
    % Load measurement data of eac bldc motor (recorded using same esc)
    offset_outputThrust = [37; 34; 25; 5];
    for ii = 1:4
        buff = dlmread(['ident' num2str(ii) '4.csv'],';');
        Data_measurement(ii).num_bldc = ii;
        Data_measurement(ii).num_esc  = 1;
        Data_measurement(ii).time     = buff(:,1);
        Data_measurement(ii).pwm      = buff(:,2) - 1000; % Substract offset
        Data_measurement(ii).sensorOutput = buff(:,3);
        Data_measurement(ii).mass     = 4.818*buff(:,3)/1000 - 193.4 - offset_outputThrust(ii);
    end

    % Get static map
    limits = [ 3000  4000;  5000  6000;  7000  8000;  9000 10000; 11000 12000; ...
              13000 14000; 15000 16000; 17000 18000; 19000 20000; 21000 21500].';
    
    x = 0;
    y = 0;
    for ii = 1:size(limits,2)
        for jj = 1:4
            ind = Data_measurement(jj).time>=limits(1,ii) & Data_measurement(jj).time<=limits(2,ii);
            x = [x; mean(Data_measurement(jj).pwm(ind))];
            y = [y; mean(Data_measurement(jj).mass(ind))];
        end
    end


    % Visualize
    figure

    subplot(3,1,1)
    grid on
    hold on
    for ii = 1:4
        plot(Data_measurement(ii).time/1000, Data_measurement(ii).pwm)
    end
    legend('Motor1 - ESC4', 'Motor2 - ESC4', 'Motor3 - ESC4', 'Motor4 - ESC4', 'Location', 'Northwest')

    subplot(3,1,2)
    grid on
    hold on
    for ii = 1:4
        plot(Data_measurement(ii).time/1000, Data_measurement(ii).mass)
    end
    legend('Motor1 - ESC4', 'Motor2 - ESC4', 'Motor3 - ESC4', 'Motor4 - ESC4', 'Location', 'Northwest')


    subplot(3,1,3)
    hold on
    grid on
    plot(x, y, 'o--')
    ylim([0 600])

    
%     % Store extracted data
%     save([Dirs.saveData 'identData_actuator_thrust'], 'ParFit');
    
end

%-------------------------------------------------------------------------%

load('staticMap_escBLDC')

esc4bldc1 = dlmread('IDENT14.csv',';');
esc4bldc2 = dlmread('IDENT24.csv',';');
esc4bldc3 = dlmread('IDENT34.csv',';');
esc4bldc4 = dlmread('IDENT44.csv',';');


% Scale signals
esc4bldc1(:,2)       = esc4bldc1(:,2) - 1000;
esc4bldc2(:,2)       = esc4bldc2(:,2) - 1000;
esc4bldc3(:,2)       = esc4bldc3(:,2) - 1000;
esc4bldc4(:,2)       = esc4bldc4(:,2) - 1000;
esc4bldc1(:,3)       = 4.818*esc4bldc1(:,3)/1000-193.4 - 37;
esc4bldc2(:,3)       = 4.818*esc4bldc2(:,3)/1000-193.4 - 34;
esc4bldc3(:,3)       = 4.818*esc4bldc3(:,3)/1000-193.4 - 25;
esc4bldc4(:,3)       = 4.818*esc4bldc4(:,3)/1000-193.4 - 5;


% Resampling
e4b1_resamp(:,1)     = 0:1:esc4bldc1(end,1);
e4b2_resamp(:,1)     = 0:1:esc4bldc2(end,1);
e4b3_resamp(:,1)     = 0:1:esc4bldc3(end,1);
e4b4_resamp(:,1)     = 0:1:esc4bldc4(end,1);

e4b1_resamp(:,2)     = interp1(esc4bldc1(:,1), esc4bldc1(:,2), e4b1_resamp(:,1)).';
e4b1_resamp(:,3)     = interp1(esc4bldc1(:,1), esc4bldc1(:,3), e4b1_resamp(:,1)).';
e4b2_resamp(:,2)     = interp1(esc4bldc2(:,1), esc4bldc2(:,2), e4b2_resamp(:,1)).';
e4b2_resamp(:,3)     = interp1(esc4bldc2(:,1), esc4bldc2(:,3), e4b2_resamp(:,1)).';
e4b3_resamp(:,2)     = interp1(esc4bldc3(:,1), esc4bldc3(:,2), e4b3_resamp(:,1)).';
e4b3_resamp(:,3)     = interp1(esc4bldc3(:,1), esc4bldc3(:,3), e4b3_resamp(:,1)).';
e4b4_resamp(:,2)     = interp1(esc4bldc4(:,1), esc4bldc4(:,2), e4b4_resamp(:,1)).';
e4b4_resamp(:,3)     = interp1(esc4bldc4(:,1), esc4bldc4(:,3), e4b4_resamp(:,1)).';

% Visualize
figure
subplot(3,1,1)
plot(e4b1_resamp(:,1)/1000, e4b1_resamp(:,2))
grid on
hold on
plot(e4b2_resamp(:,1)/1000, e4b2_resamp(:,2))
plot(e4b3_resamp(:,1)/1000, e4b3_resamp(:,2))
plot(e4b4_resamp(:,1)/1000, e4b4_resamp(:,2))


subplot(3,1,2)
plot(e4b1_resamp(:,1)/1000, e4b1_resamp(:,3))
grid on
hold on
plot(e4b2_resamp(:,1)/1000, e4b2_resamp(:,3))
plot(e4b3_resamp(:,1)/1000, e4b3_resamp(:,3))
plot(e4b4_resamp(:,1)/1000, e4b4_resamp(:,3))
ylim([0 600])

u_ident = [e4b1_resamp(:,2); e4b2_resamp(:,2); e4b3_resamp(:,2); e4b4_resamp(:,2)];
y_ident = [e4b1_resamp(:,3); e4b2_resamp(:,3); e4b3_resamp(:,3); e4b4_resamp(:,3)]/1000;


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %                     Preprocess and visualize data                       %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [FilterCoefficients.b, FilterCoefficients.a] = butter(10, 0.2);
% Tbd_filt   = filtfilt(FilterCoefficients.b, FilterCoefficients.a, ParFit.Tbd);
% Yexg_filt  = filtfilt(FilterCoefficients.b, FilterCoefficients.a, ParFit.Yexg);
% 
% figure
% aux_plotSize(15, 10, 32, 15, 2, 2, 1, 1)
% 
% subplot(3,1,1)
% plot(ParFit.cycles, ParFit.alpha_evc)
% grid on
% ylabel('\alpha_{evc, set} in °CA aTDC')
% 
% subplot(3,1,2)
% plot(ParFit.cycles, ParFit.Tbd)
% hold on
% plot(ParFit.cycles, Tbd_filt)
% grid on
% ylabel('T_{bd} in K')
% 
% subplot(3,1,3)
% plot(ParFit.cycles, ParFit.Yexg)
% hold on
% plot(ParFit.cycles, Yexg_filt)
% grid on
% ylabel('Y_{exg} in kg')
% xlabel('Cycle in -')
% 
% % TODO: Scale data
% 
% %-------------------------------------------------------------------------%
% 
% 
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %                              Fit Data                                   %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % NOTE: What is Par_matlabCurveFit.Options.Weights = weightsmRed;
% 
% ParFit.fitMatlab = fit( [ParFit.alpha_evc, ParFit.Tbd], ParFit.Yexg, ...
%                         Par_matlabCurveFit.modelType, ...
%                         Par_matlabCurveFit.Options );
% 
% % Get model, coefficients and names from fit
% ParFit.model             = formula(ParFit.fitMatlab);
% ParFit.coefficientNames  = coeffnames(ParFit.fitMatlab);
% ParFit.coefficientValues = coeffvalues(ParFit.fitMatlab);
% 
% % Adapt model for function
% ParFit.model = strrep(ParFit.model, 'x', 'alpha_evc');
% ParFit.model = strrep(ParFit.model, 'y', 'Tbd');
% 
% % Replace model coefficients by their values
% for ii = 1:length(ParFit.coefficientNames)
%     ParFit.model = strrep(ParFit.model, ParFit.coefficientNames{ii}, ...
%                           num2str(ParFit.coefficientValues(ii)));
% end
% 
% % Generate function
% fid = fopen([Dirs.saveFun Spec.functionName '.m'], 'wt');
% fprintf(fid, ['function F = ' Spec.functionName '( pwm ) \n']);
% fprintf(fid, ['%% function F = ' Spec.functionName '( pwm ) \n']);
% fprintf(fid, '%%\n');
% fprintf(fid, '%%   Author1     : \n');
% fprintf(fid, '%%\n');
% fprintf(fid, '%%   Date        : Winter 2018\n');
% fprintf(fid, '%%\n');
% fprintf(fid, '%%   Description : Submodel for esc-bldc dynamics\n');
% fprintf(fid, '%%\n');
% fprintf(fid, '%%   Parameters  : pwm -> Pulse-width modulated signal\n');
% fprintf(fid, '%% \n');
% fprintf(fid, '%% \n');
% fprintf(fid, '%%   Return      : F -> Motor thrust\n');
% fprintf(fid, '%%\n');
% fprintf(fid, '%%   This function was autogenerated by 04_identification/...\n');
% fprintf(fid, '%%   ...actuator_esc-bldc/ident_acutator_thrust.m \n');
% fprintf(fid, '%%\n');
% fprintf(fid, '%%-------------------------------------------------------------------------%%\n\n');
% fprintf(fid, ['F = ' ParFit.model ';']);
% fclose(fid);
% 
% clear ii fid ans
% 
% %-------------------------------------------------------------------------%
% 
% 
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %                           Visualize result                              %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [xg, yg] = meshgrid(260:1:280, 0:100:2000);
% 
% figure
% aux_plotSize(15, 10, 32, 2, 2, 2, 1, 1)
% plot3(ParFit.alpha_evc, ParFit.Tbd, ParFit.Yexg, '.')
% hold on
% mesh(xg, yg, mdlSub_discreteCycle_Yexg(xg, yg))
% grid on
% xlabel('\alpha_{evc, set} in °CA aTDC')
% ylabel('T_{bd} in K')
% zlabel('Y_{exg} in kg')
% zlim([0 1])
% clear xg yg
% 
% %-------------------------------------------------------------------------%