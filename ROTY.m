function Ry = ROTY(pitch)
    if(~isscalar(pitch) || ~isreal(pitch))
        error("Please pass in a single real scalar.");
    end
    Ry = [cos(pitch)    0   sin(pitch)
          0             1   0
          -sin(pitch)   0   cos(pitch)];
end