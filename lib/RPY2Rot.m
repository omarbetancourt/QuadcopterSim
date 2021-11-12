function bRi = RPY2Rot(angles)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    
    %Z-Y-X Euler angles to build rotation matrix
    Rpsi = [cos(psi) , sin(psi), 0;...
           -sin(psi), cos(psi), 0;...
           0        ,        0, 1];
       
    Rtheta = [cos(theta), 0, -sin(theta);...
              0         , 1,          0;...
              sin(theta), 0,  cos(theta)];
        
    Rphi = [1,         0,        0;...
            0,  cos(phi), sin(phi);...
            0, -sin(phi), cos(phi)];
        
    bRi = Rphi*Rtheta*Rpsi;
end