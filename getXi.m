function xi = getXi(g)
    if(size(g,1)~=4 || size(g,2)~=4 || ~isreal(g))
        error("Please pass in a 4 by 4 homogeneous matrix.");
    end
    % extract rotation and translation
    R = g(1:3,1:3);
    p = g(1:3,4);
    if(R==eye(3)) % if pure rotation, v=p
        xi = [p; zeros(3,1)];
    else
        % compute angle and axis from rotation
        theta = acos((trace(R)-1)/2);
        omega = 1/2/sin(theta)*getXfromSkew(R-R');
        % use formula to obtain v
        A = (eye(3)-expm(SKEW3(omega)*theta))*SKEW3(omega)+omega*omega'*theta;
        v = A\p;
        % un-normalize twist vector
        xi = theta * [v; omega];
    end
end

% obtain the 3x1 vector from a 3x3 skew-symmetric matrix
function x = getXfromSkew(S)
    if(size(S,1)~=3 || size(S,2)~=3)
        error("Please pass in a 3 by 3 matrix.");
    end
    x = [S(3,2); S(1,3); S(2,1)];
end