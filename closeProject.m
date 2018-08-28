projectRootDir = fileparts(mfilename('fullpath'));
try 
    load('includePaths.mat');
    for ii = 1:size(includePaths,1)
        rmpath(includePaths{ii});
    end
catch
    disp('Could not find loaded paths');
end
clear all;
close all;
clc;