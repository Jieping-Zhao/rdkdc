function J = ur5BodyJacobian(q)
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
    H1 = expm(screwHat(xi1)*q_rel(1));
    H2 = expm(screwHat(xi2)*q_rel(2));
    H3 = expm(screwHat(xi3)*q_rel(3));
    H4 = expm(screwHat(xi4)*q_rel(4));
    H5 = expm(screwHat(xi5)*q_rel(5));
    H6 = expm(screwHat(xi6)*q_rel(6));
    % compute and gather all transformed twists
    J = [adjoint(H1*H2*H3*H4*H5*H6*g0)\xi1 ...
         adjoint(H2*H3*H4*H5*H6*g0)\xi2 ...
         adjoint(H3*H4*H5*H6*g0)\xi3 ...
         adjoint(H4*H5*H6*g0)\xi4 ...
         adjoint(H5*H6*g0)\xi5 ...
         adjoint(H6*g0)\xi6];
end

% get the 6x1 twist coordinate of a revolute joint
function xi = rotScrew(q,omega)
    xi = [-cross(omega,q); omega];
end

% perform the "hat" operation on a screw
function xihat = screwHat(xi)
    xihat = [SKEW3(xi(4:6)) xi(1:3); 0 0 0 0];
end

% compute the adjoint of a transformation
function adj = adjoint(H)
    adj = [H(1:3,1:3) SKEW3(H(1:3,4))*H(1:3,1:3); zeros(3) H(1:3,1:3)];
end