function gst = ur5FwdKin(q)
    if size(q,1)~=6 || size(q,2)~=1
        error('Please pass in a 6x1 vector');
    end
    % define unit vectors and lengths
    e1 = [1 0 0]';
    e2 = [0 1 0]';
    e3 = [0 0 1]';
    L0 = 0.0892;
    L1 = 0.425;
    L2 = 0.392;
    L3 = 0.1093;
    L4 = 0.09475;
    L5 = 0.0825;
    % compute twist coordinates for all joints
    xi1 = rotScrew([0 0 L0]', e3);
    xi2 = rotScrew([0 0 L0]', e2);
    xi3 = rotScrew([0 0 L0+L1]', e2);
    xi4 = rotScrew([0 0 L0+L1+L2]', e2);
    xi5 = rotScrew([0 L3 L0+L1+L2]', e3);
    xi6 = rotScrew([0 L3+L5 L0+L1+L2+L4]', e2);
    % define the "zero" configuration and transformation
    q0 = [0 -pi/2 0 -pi/2 0 0]';
    g0 = [ROTX(-pi/2) [0 L3+L5 L0+L1+L2+L4]'; 0 0 0 1];
    % get the relative configuration between the input and the "zero"
    q_rel = q-q0;
    % convert all twist and joint angles to transformations
    H1 = expm(xi1*q_rel(1));
    H2 = expm(xi2*q_rel(2));
    H3 = expm(xi3*q_rel(3));
    H4 = expm(xi4*q_rel(4));
    H5 = expm(xi5*q_rel(5));
    H6 = expm(xi6*q_rel(6));
    % obtain forward kinematics through PoE formula
    gst = H1*H2*H3*H4*H5*H6*g0;
end

function xi = rotScrew(q,omega)
    xi = [SKEW3(omega) -cross(omega,q); 0 0 0 0];
end