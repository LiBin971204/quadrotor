clear all; close all; clc;
% Set paths needed by project

% Get current folder dir
projectRootDir = fileparts(mfilename('fullpath'));

% Specify necessary folders
includePaths = { ...
    projectRootDir; ...
    fullfile(projectRootDir, '05_data'); ...
    fullfile(projectRootDir, '01_auxilary'); ...
    fullfile(projectRootDir, '01_auxilary/quaternion_library'); ...
    genpath(fullfile(projectRootDir, '02_models')); ...
    genpath(fullfile(projectRootDir, '03_simulation')); ...
    fullfile(projectRootDir, '10_work'); ...
}; 

% Add specified folders to Matlab path
for ii = 1:size(includePaths,1)
    addpath(includePaths{ii});
end

% Set work directory for compiled and temporary data
workFolder = includePaths{end}; 
Simulink.fileGenControl('set', 'CacheFolder', workFolder, 'CodeGenFolder', workFolder);
save(fullfile(workFolder, 'includePaths.mat'), 'includePaths');
clear projectRootDir workFolder ii includePaths;



%% Busdefinitions
load('00_config/bus_init.mat');