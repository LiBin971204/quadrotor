function Par_filterMadgwick = mdl_filter_madgwick_par( )
% function Par_filterMadgwick = mdl_filter_madgwick_par( )
%
% Date:    Winter 2018
%
% Description: 	Loads the parameters of the Madgwick filter
% 
% Parameters : 	None
% 
% Return     : 	Par_filterMadgwick - a struct which contains the parameters
% 
% Examples of Usage: None
%
%-------------------------------------------------------------------------%

Par_filterMadgwick.T = 4e-3;
Par_filterMadgwick.beta = 0.1;