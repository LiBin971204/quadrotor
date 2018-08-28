function A = mdl_rotEuler_zyx_i2b(phi, theta, psi)
 A = [cos(theta)*cos(psi)                            cos(theta)*sin(psi)                            -sin(theta); 
      sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);
      cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
end