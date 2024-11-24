global Px100;

% Set your port number
Px100.DEVICENAME = 'Enter your port number';

% Initializing the robot
init_robot

% Getting the minimum Z threshold
min_z_thresh = Px100.MIN_Z_THRESH;

% Getting max movement distance
movement_thresh = Px100.MOVEMENT_THRESH;


%% Reaching the Payload
while(true)

    % Getting joint positions
    joint_positions = get_joint_pos();

    % Get the postions of pick , place and obstacle
    [pick_pose_base, drop_pose_base, obstacle_pose_base] = get_pick_and_place_position();  

    % Get the initial end-effector position using forward kinematics
    T_eef= fKpincherx100(joint_positions);
    pick_joint_angles = iKpincherx100(pick_pose_base, "up");

    % Generate end-effector trajectory to reach the payload
   
    % Generate waypoints in joint-space
    joint_space_waypoints = generate_waypoints(joint_positions, pick_joint_angles, num_waypoints);
    % 
    N = length(joint_space_waypoints);
    
    for i=1:N
        % Give position command to robot
        set_joint_pos(waypoints[i]);

        while(true)
            % Break out of the loop when 
            % end-effector reaches the point
        end
    end
end

% Close the gripper
closeGripper();
pause(0.5) % Wait for the gripper to close

%% Transporting Payload
while(true)
    % Getting joint positions
    joint_positions = get_joint_pos();

    % Get the initial end-effector position using forward kinematics

    % Generate end-effector trajectory to transport the payload

    % Generate waypoints in joint-space

    N = length(joint_space_waypoints);
    
    for i=1:N
        % Give position command to robot
        set_joint_pos(waypoints[i]);

        while(true)
            % Break out of the loop when 
            % end-effector reaches the point
        end
    end

    % Open the gripper
end

pause(0.5);

%% Returning to Home config

while(true)
    % Getting joint positions
    joint_positions = get_joint_pos();

    % Get the initial end-effector position using forward kinematics
    
    % Generate end-effector trajectory to reach home config

    % Generate waypoints in joint-space

    N = length(joint_space_waypoints);
    
    for i=1:N
        % Give position command to robot
        set_joint_pos(waypoints[i]);

        while(true)
            % Break out of the loop when 
            % end-effector reaches the point
        end
    end
end