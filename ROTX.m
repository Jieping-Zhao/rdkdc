function Rx = ROTX(roll)
    if(~isscalar(roll) || ~isreal(roll))
        error("Please pass in a single real scalar.");
    end
    Rx = [1 0           0  
          0 cos(roll)   -sin(roll)
          0 sin(roll)   cos(roll)];
end