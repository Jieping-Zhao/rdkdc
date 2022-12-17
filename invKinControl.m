function finalerr = invKinControl(gdesired,T_step,ur5,gtg)
    q=ur5.get_current_joints();                  % get current joint angle
    gst=ur5FwdKin(q);                            % compute current transformation
    Jb=ur5BodyJacobian(q);                       % compute current body jacobian
    det_Jb=manipulability(Jb, 'detjac');         % compute manipulability criterion
    gsg = gst*gtg;                               % compute transformation to gripper
    
    if(abs(det_Jb) < 1e-7) % check for singularity
        finalerr = -1;
        warning('Abort: Singularity detected.')
        return % exit if at singularity
    end
    if (abs(q) > 2*pi) % check for joint limits exceded 
        finalerr = -1;
        warning('Abort: Joint limits exceeded.')
        return % exit if joint limits reached
    end  
    if (gsg(3,4) <= 0.03) % check for end effector position
        finalerr = -1;
        warning('Abort: Bottom surface impact detected.')
        return % exit if bottom surface reached
    end

    theta = ur5InvKin(gdesired);            % compute entire set of solutions to inverse kinematics
    diff = zeros(1,size(theta,2));          % initialize container for difference
    for i=1:size(theta,2)                   % for each set of joint angles returned
        diff(i) = norm(theta(:,i)-q);       % compute the difference between it and the current joint angles
    end
    [~,i] = min(diff);                      % obtain index for the set of joint angles that differ from the current by the smallest
    ur5.move_joints(theta(:,i),T_step);     % get that set of joint angles and move there within the inputted T_step
    pause(1.2*T_step);                      % wait for move to finish
    finalerr = 0;
end