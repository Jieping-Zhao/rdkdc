% This file can be run section by section, or run as an entire file.
% The first section sets up all the pre-determined values that will be used
% throught this file.
% The second section is the default setting specified in the instructions
% for testing purposes.
% The third section allows for manual setting of the robot start and end
% configurations, as prompted on the command line. Note that running this
% section will overwrite the values in section 2. Skip running this section
% if one wants to use the values in section 2.
% The fourth section is the actual robot execution process, where it first
% moves to the home position, then the start position, going downwards in a
% straight line, back up the same way, moves to the target position, goes
% down and up, and then returns to the home position.
% The fifth section is a utility function that generates a Cartesian
% trajectory in a straight line.
% The sixth section is a utility function that commands the robot to follow
% a trajectory under some control algorithmm.

% Setup
clear; clc;

home = [0 -1.25 -0.44 -1.57 0.25 0]';               % non-singular home position of UR5
gtg = [EULERXYZ([0 0 pi/2]) [0; 0; 0.13]; 0 0 0 1]; % transformation from tool to gripper
descent = 0.1;                                      % descend distance (m)
params = [0.75 1 1.2];                              % T_step for invKin, K for invJac, K for transJac

ur5 = ur5_interface();                              % declare UR5 instance

% generate prompt for control algorithm selection and asks for input
prompt = sprintf("Please enter a value for the control algorithm.\n" + ...
    "\t 1 for inverse kinematics,\n" + ...
    "\t 2 for inverse Jacobian, and\n" + ...
    "\t 3 for transpose Jacobian.\n" + ...
    "Robot will not perform downward motion for any other value.\n" + ...
    "Enter control value here: ");
control = input(prompt);


% Default setting
% sets up the default start (gst1) and end (gst2) configurations
gst1 = [0 -1 0 0.3; -1 0 0 -0.4; 0 0 -1 0.22; 0 0 0 1];
q1 = [-1.1477; -1.0253; 1.4181; 1.1781; 1.5708; 1.9939];
gst2 = [0 -1 0 -0.3; -1 0 0 0.39; 0 0 -1 0.22; 0 0 0 1];
q2 = [2.0025; -1.0432; 1.4467; 1.1673; 1.5708; -1.1391];


% Manual setting

% % prompt user to manually move the robot to the start position
% disp("Move robot to start position. Press any key to continue.");
% waitforbuttonpress % wait for user to finish
% q1 = ur5.get_current_joints(); % record starting joints
% gst1 = ur5FwdKin(q1); % compute starting transformation
% % prompt user to manually move the robot to the target position
% disp("Start position collected.")
% disp("Move robot to target position. Press any key to continue.");
% waitforbuttonpress % wait for user to finish
% q2 = ur5.get_current_joints(); % record target joints
% gst2 = ur5FwdKin(q2); % compute target transformation
% disp("Target position collected.")


% Execution

% move first to non-singular home configuration
ur5.move_joints(home,10);
pause(11);

% move to starting position
ur5.move_joints(q1,10);
pause(11);

% generate a Cartesian trajectory straight down
traj1 = traj_gen(gst1,gst1-[0 0 0 0; 0 0 0 0; 0 0 0 descent; 0 0 0 0]);
% execute the motion along the trajectory and check for errors
if(move_robot(traj1,control,params,ur5,eye(4))==-1)
    rosshutdown
    return
end
pause(1) % wait for a short while
printError(ur5,gst1-[0 0 0 0; 0 0 0 0; 0 0 0 descent; 0 0 0 0]); % prints error

% generate a Cartesian trajectory straight up
traj1_inv = traj_gen(gst1-[0 0 0 0; 0 0 0 0; 0 0 0 descent; 0 0 0 0],gst1);

% execute the motion along the trajectory and check for errors
if(move_robot(traj1_inv,control,params,ur5,eye(4))==-1)
    rosshutdown
    return
end
pause(1) % wait for a short while
printError(ur5,gst1); % prints error

ur5.move_joints(q2,10);
pause(11);

% generate a Cartesian trajectory straight down
traj2 = traj_gen(gst2,gst2-[0 0 0 0; 0 0 0 0; 0 0 0 descent; 0 0 0 0]);
% execute the motion along the trajectory and check for errors
if(move_robot(traj2,control,params,ur5,eye(4))==-1)
    rosshutdown
    return
end
pause(1) % wait for a short while
printError(ur5,gst2-[0 0 0 0; 0 0 0 0; 0 0 0 descent; 0 0 0 0]); % prints error

% generate a Cartesian trajectory straight up
traj2_inv = traj_gen(gst2-[0 0 0 0; 0 0 0 0; 0 0 0 descent; 0 0 0 0],gst2);
% execute the motion along the trajectory and check for errors
if(move_robot(traj2_inv,control,params,ur5,eye(4))==-1)
    rosshutdown
    return
end
pause(1) % wait for a short while
printError(ur5,gst2); % prints error

rosshutdown % terminate


% Trajectory Generation
function traj = traj_gen(g_start, g_end)
% generates a trajectory in Cartesian space with 5 intermediate points
    s = linspace(0,1,5); % 5 points for s between 0 and 1
    p = zeros(3,1,length(s)); % initialize container for translation
    R = zeros(3,3,length(3)); % initialize container for rotation
    for i=1:length(s) % for each value of s
        % compute the appropriate translation
        p(:,:,i) = g_start(1:3,4) + s(i)*(g_end(1:3,4)-g_start(1:3,4));
        % compute the appropriate rotation
        R(:,:,i) = g_start(1:3,1:3) * expm(s(i)*logm(g_start(1:3,1:3)'*g_end(1:3,1:3)));
    end
    % store translation and rotation in transformation matrices and return
    traj = [R,p; zeros(1,3,length(s)),ones(1,1,length(s))];
end


% Robot Motion
function val = move_robot(traj,control,params,ur5,gtg)
% takes the trajectory generated and send UR5 to follow it with a specified
% control algorithm
    val = 0; % initialize return value
    for i=1:size(traj,3) % for each waypoint in the trajectory
        if control == 1 % calls inverse kinematics control to get to the waypoint
            val = invKinControl(traj(:,:,i),params(1),ur5,gtg);
            if(val == -1) % abort in case of error
                return
            end
        elseif control == 2 % calls inverse Jacobian control to get to the waypoint
            val = invJacControl(traj(:,:,i),params(2),ur5,gtg);
            if(val == -1) % abort in case of error
                return
            end
        elseif control == 3 % calls transpose Jacobian control to get to the waypoint
            val = transJacControl(traj(:,:,i),params(3),ur5,gtg);
            if(val == -1) % abort in case of error
                return
            end
        end
    end
end

% Error Reporting
function printError(ur5,gdesired)
% prints out the translational and rotational errors between the current
% end-effector position of the UR5 and the desired position
    gst = ur5.get_current_transformation("base_link","tool0"); % get current transformation
    R = gst(1:3,1:3); % extract current rotation
    Rd = gdesired(1:3,1:3); % extract desired rotation
    p = gst(1:3,4); % extract current translation
    pd = gdesired(1:3,4); % extract desired translation
    dr3 = norm(p-pd); % compute translational error
    dso3 = sqrt(trace((R-Rd)*(R-Rd)')); % compute rotational error
    fprintf("Translational error: %.2f mm\n",dr3*1000); % prints translational error
    fprintf("Rotational error: %.2f degrees\n",dso3/pi*180); % prints rotational error
end