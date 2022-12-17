% the function is ill-defined when the cosine of the pitch angle (angle
% associated with y-rotations) is zero, leading to infinities that cannot
% be solved from the set of equations. Physically, this means that the roll
% and yaw angles can vary independently, but they only cause effect in 1
% degree-of-freedom--there exists an infinite number of angle tuples that
% will lead to this rotation matrix.
function angles = EULERXYZINV(R)
    if(size(R,1)~=3 || size(R,2)~=3 || ~isreal(R))
        error("Please pass in a 3 by 3 rotation matrix.");
    end
    pitch = atan2(R(1,3), sqrt(R(1,1)^2+R(1,2)^2));
    if cos(pitch)>10^(-6)
        roll = atan2(-R(2,3)/cos(pitch), R(3,3)/cos(pitch));
        yaw = atan2(-R(1,2)/cos(pitch), R(1,1)/cos(pitch));

    else
        disp("Warning! Singularity is reached and there exist infinite" + ...
             " solutions." + newline + "Returning the case where roll=0," + ...
             "but in practice adding any amount to roll" + newline + ...
             "and subtracting the same amount from yaw will give the" + ...
             "same rotation matrix.");
        roll = 0;
        yaw = atan2(R(2,1), R(2,2));
    end
    angles = [roll;pitch;yaw];
end