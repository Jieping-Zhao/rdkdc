function Rz = ROTZ(yaw)
    if(~isscalar(yaw) || ~isreal(yaw))
        error("Please pass in a single real scalar.");
    end
    Rz = [cos(yaw)  -sin(yaw)   0
          sin(yaw)  cos(yaw)    0
          0         0           1];
end