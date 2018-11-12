function [vAngularVelOut, mRotKarman] = rot_karman( vAngularVelIn, vEulerAngles, direction)
% function [mRotEuler, mRotKarman] = rot_karman( eulerAngle, direction )
%
%   Author1     : 
% 
%   Date        : Winter 2018
%
%   Description : Rotates the angular velocity vector by using euler angles
%
%   Parameters  : vAnularVelIn -> Vector of angular velocities that is to 
%                                 be rotated
%                 vEulerAngles -> Vector containing euler angles
%                 direction -> Containts information whether rotate to
%                              "to-body-frame" or "to-inertial-frame"
% 
%   Return      : vAngularVelOut -> Rotated vector
%                 mRotKarman -> Euler rotation matrix
%
%-------------------------------------------------------------------------%

% Get euler angels
phi     = vEulerAngles(1);
theta   = vEulerAngles(2);

% Calculate rotation matrix and output vector
switch direction
    case 'to-body-frame'
        mRotKarman = [1          0           -sin(theta); ...
                      0   cos(phi)   sin(phi)*cos(theta); ...
                      0  -sin(phi)   cos(phi)*cos(theta)];
     
        vAngularVelOut = mRotKarman * vAngularVelIn;

    case 'to-inertial-frame'
        mRotKarman = [1  sin(phi)*tan(theta)  cos(phi)*tan(theta); ...
                      0             cos(phi)            -sin(phi); ...
                      0  sin(phi)/cos(theta)  cos(phi)/cos(theta)];
                  
        vAngularVelOut = mRotKarman * vAngularVelIn;

end