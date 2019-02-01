function [vAngularVelOut, mRotKarman] = rot_karman( vAngularVelIn, eulerAngles, direction)
% function [vAngularVelOut, mRotKarman] = rot_karman( vAngularVelIn, eulerAngle, direction )
%
%   Author1     : enu89
% 
%   Last change : Spring 2019
%
%   Description : Rotates the angular velocity vector by using euler angles
%
%   Parameters  : vAngularVelIn -> Vector of angular velocities that is to 
%                                 be rotated
%                 eulerAngles -> Vector containing euler angles
%                 direction -> Containts information whether rotate to
%                              "to-body-frame" or "to-inertial-frame"
% 
%   Return      : vAngularVelOut -> Rotated vector
%                 mRotKarman -> Karman rotation matrix
%
%-------------------------------------------------------------------------%
% Get euler angels
phi     = eulerAngles(1);
theta   = eulerAngles(2);

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