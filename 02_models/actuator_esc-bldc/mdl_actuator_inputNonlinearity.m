function Ftilde = mdl_actuator_inputNonlinearity( pwm ) 
% function Ftilde = mdl_actuator_inputNonlinearity( pwm ) 
%
%   Author1     : 
%
%   Date        : Winter 2018
%
%   Description : Submodel for esc-bldc dynamics
%
%   Parameters  : pwm -> Pulse-width modulated signal [-]
% 
%   Return      : Ftilde -> Motor thrust after hammerstein input nonlinearity
%
%   This function was autogenerated by 04_identification/...
%   ...actuator_esc-bldc/ident_acutator_thrust.m 
%
%-------------------------------------------------------------------------%

Ftilde = -0.12862.*pwm.^3 + 0.26903.*pwm.^2 + 0.03078.*pwm + -0.002248;