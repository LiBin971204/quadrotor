function Par = mdl_filter_madgwick_par( )
% function Par = mdl_filter_madgwick_par( )
%
%   Date        : Winter 2018
%
%   Description : Loads the parameters of the Madgwick filter
% 
%   Parameters  : None
% 
%   Return      : Par -> Struct containing filter parameters
% 
%-------------------------------------------------------------------------%

Par.T = 4e-3;
Par.beta = 0.1;