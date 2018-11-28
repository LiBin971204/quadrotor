function [vOut, mRotEuler] = rot_euler( vIn, eulerAngles, direction, convention)
% function [vOut, mRotEuler] = rot_euler( eulerAngle, direction, convenction )
%
%   Author1     : 
% 
%   Date        : Winter 2018
%
%   Description : Rotates the vector vIn by using euler angles with 
%                 specified sequence to body or to inertial frame
%
%   Parameters  : vIn -> Vector that is to be rotated
%                 eulerAngles -> Vector containing euler angles
%                 direction -> Containts information whether rotate to
%                              "to-body-frame" or "to-inertial-frame"
%                 convention -> Containts information which rotation
%                               sequence to use (standard 'xyz')
% 
%   Return      : vOut -> Rotated vector
%                 mRotEuler -> Euler rotation matrix
%
%-------------------------------------------------------------------------%

% Get euler angels
phi     = eulerAngles(1);
theta   = eulerAngles(2);
psi     = eulerAngles(3);

% Set convention to 'xyz' if not specified
if nargin==3
    convention = 'xyz';
end

% Calculate rotation matrix and output vector
switch convention
    case 'xyz'
        mRotEuler_toBody = ...
                     [cos(theta)*cos(psi) ...
                      cos(theta)*sin(psi) ...
                      -sin(theta); ...
                      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) ...
                      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) ...
                      sin(phi)*cos(theta); ...
                      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi) ...
                      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) ...
                      cos(phi)*cos(theta)];
        
        switch direction
            case 'to-body-frame'
                vOut = mRotEuler_toBody * vIn;

            case 'to-inertial-frame'
                vOut = mRotEuler_toBody.' * vIn;
                
        end
        
end