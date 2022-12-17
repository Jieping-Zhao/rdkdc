function skew = SKEW3(x)
    if(length(x)~=3)
        error("Please pass in a 3-vector.");
    end
    skew = [0       -x(3)   x(2)
            x(3)    0       -x(1)
            -x(2)   x(1)    0];
end