function R = EULERXYZ(angles)
    if(length(angles)~=3 || ~isreal(angles))
        error("Please pass in a real 3-vector.");
    end
    R = ROTX(angles(1))*ROTY(angles(2))*ROTZ(angles(3));
end