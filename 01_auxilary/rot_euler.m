function [vOut, mRotEuler] = rot_euler( vIn, vEulerAngles, direction, convention)
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
%                 vEulerAngles -> Vector containing euler angles
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
phi     = vEulerAngles(1);
theta   = vEulerAngles(2);
psi     = vEulerAngles(3);

% Set convention to 'xyz' if not specified
if nargin==3
    convention = 'xyz';
end

% Calculate rotation matrix and output vector
switch convention
    case 'xyz'
        switch direction
            case 'to-body-frame'
                mRotEuler = ...
                     [cos(theta)*cos(psi) ...
                      cos(theta)*sin(psi) ...
                      -sin(theta); ...
                      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) ...
                      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) ...
                      sin(phi)*cos(theta); ...
                      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi) ...
                      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) ...
                      cos(phi)*cos(theta)];
                  vOut = mRotEuler * vIn;

            case 'to-inertial-frame'
                 mRotEuler = ...
                     [cos(theta)*cos(psi) ...
                      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) ...
                      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
                      cos(theta)*sin(psi) ...
                      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) ...
                      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
                      -sin(theta) ...
                      sin(phi)*cos(theta)  ...
                      cos(phi)*cos(theta)];
                vOut = mRotEuler * vIn;
                
        end
        
end