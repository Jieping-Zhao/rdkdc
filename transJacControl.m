function finalerr=transJacControl(gdesired,K,ur5,gtg)

% Inputs: There are three inputs to this function
%         gdesired: a homogeneous transform that is the desired end-effector pose, 
%                     i.e. gst? .
%         K: the gain on the controller
%         ur5: the ur5 interface object
% Output: finalerr: if, a failure has occured (i.e. singualrity),-1
%                   otherwise, final positional error in m

    T_step=0.6; % initializing time step as 0.6

    q=ur5.get_current_joints();                 % get current joint angle
    gst=ur5FwdKin(q);                           % compute current transformation
    Jb=ur5BodyJacobian(q);                      % compute current body jacobian
    det_Jb=manipulability(Jb, 'detjac');        % compute manipulability criterion
    gsg = gst*gtg;                              % compute transformation to gripper
    xi_k=getXi(gdesired\gst);                   % calculate error
    v_k = xi_k(1:3);
    w_k = xi_k(4:6);
    
    while (norm(v_k)> 0.01 || norm(w_k) > 0.03)    % Terminates when error is below thresh.    
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
        
        % calculate new position with control law 
        alpha = dot(xi_k,Jb*Jb'*xi_k)/dot(Jb*Jb'*xi_k,Jb*Jb'*xi_k);
        q_dot = -transpose(Jb)*K*alpha*xi_k;
        
        % set speed limit constraint by clipping
        speed_lim = abs(q_dot) > pi*ur5.speed_limit;
        q_dot(speed_lim) = 0.9*sign(q_dot(speed_lim))*pi*ur5.speed_limit;
        q = q + q_dot*T_step;
        
        ur5.move_joints(q, T_step); % tell ur5 to move to new position
        pause(1.2*T_step);

        % update new values for next iteration / while loop checking
        q=ur5.get_current_joints();                 % get current joint angle
        gst=ur5FwdKin(q);                           % compute current transformation
        Jb=ur5BodyJacobian(q);                      % compute current body jacobian
        det_Jb=manipulability(Jb, 'detjac');        % compute manipulability criterion
        gsg = gst*gtg;                              % compute transformation to gripper
        xi_k=getXi(gdesired\gst);                   % calculate error
        v_k = xi_k(1:3);
        w_k = xi_k(4:6);

    end
    err = gdesired\gst;
    finalerr = norm(err(1:3,4)); % return final position error magnitude
    
end
