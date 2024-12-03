% Global Robot Structure    
global Px100;
global joint_positions pick_pose_base drop_pose_base;

% Define pick and drop poses
pick_pose_base = trvec2tform([0.20, 0.15, 0.05]);  % Default pick pose ( You can change it)

drop_pose_base = trvec2tform([0.10, -0.15, 0.05]);  % Default drop pose ( You can change it)
global num_waypoints;
num_waypoints =100;
% Initialize simulation environment
init_robot_sim(pick_pose_base, drop_pose_base);

%% Main Loop
try
    % Call main task
    main_task();
catch ME
    disp('Error encountered:');
    disp(ME.message);
end

function main_task()
    global joint_positions Px100 pick_pose_base drop_pose_base num_waypoints;

    %% Reaching the Payload
    while true
        % Get current joint positions (first 4 only)
        joint_positions = get_current_joint_positions();
       
        %Write Inverse Kinematics for PincherX100. The input should take the pick_pose and the output of it should be the pick_joint_config( joint configuration wrt pick_pose)
        pick_pose_angle = ikpincher100(pick_pose_base);
        time_to_travel=3;
        ang_theta1_1 = pick_pose_angle(1,1);
        intermediate_1 = [ang_theta1_1;0;0;0];
        executeJointSpaceTrajectory(joint_positions, intermediate_1, time_to_travel, num_waypoints);
        joint_positions_new = get_current_joint_positions();
        pause(0.01);
        disp("Started journey towards pick pose");
        executeJointSpaceTrajectory(joint_positions_new, pick_pose_angle, time_to_travel, num_waypoints);
        % Attach cylinder at pick pose
        attachCylinder();
        disp("Picked the object");

        pause(0.5);
        break;  % Exit loop after first run for simplicity
    end

    %% Transporting Payload
    while true
        %Write Inverse Kinematics for PincherX100 . The input should take the drop_pose and the output of it should be the drop_joint_config( joint configuration wrt drop_pose)
        drop_pose_angle = ikpincher100(drop_pose_base);
        time_to_travel = 1;
        disp("Reaching above pick pose ");
        ang_theta1_1 = pick_pose_angle(1,1);
        intermediate_1 = [ang_theta1_1;0;0;0];
        executeJointSpaceTrajectory(pick_pose_angle, intermediate_1, time_to_travel, num_waypoints);
        disp("move to drop pose and hover")
        ang_theta1_2 = drop_pose_angle(1,1);
        intermediate_2 = [ang_theta1_2;0;0;0];
        executeJointSpaceTrajectory(intermediate_1, intermediate_2, time_to_travel, num_waypoints);

        disp("Dropping the object")
        executeJointSpaceTrajectory(intermediate_2, drop_pose_angle, time_to_travel, num_waypoints);
        detachCylinder();
        disp("TOUCHDOWN CONFIRMED");
        pause(0.5);
        break;
    end

    %% Returning to Home Configuration
    while true
        % Write the code for trajectory planning from drop_pose to home_pose. The input should be the drop_pose, home.Config . The output should be "joint_space_waypoints" n x 4 matrix.
        num_waypoints_last = num_waypoints;

        time_to_return = 2;
        to_hover=drop_pose_angle(1,1);
        intermediate_3=[to_hover;0;0;0];
        disp("Going back to hover to avoid object-robot collision");
        executeJointSpaceTrajectory( drop_pose_angle,intermediate_3, time_to_return, num_waypoints_last);

        pause(0.01);
        disp("Reached home");
        executeJointSpaceTrajectory( intermediate_3,joint_positions, time_to_return, num_waypoints_last);
        break;
    end
 end
% Setting up simulation environment
function init_robot_sim(pick_pose, drop_pose)
    global Px100;
    Px100.robot = importrobot('px100.urdf');% path to the urdf file
    Px100.robot.DataFormat = 'row';
    Px100.homeConfig = homeConfiguration(Px100.robot);
    Px100.currentConfig = Px100.homeConfig;

    % Display the robot in its home configuration
    figure;
    show(Px100.robot, Px100.homeConfig, 'PreservePlot', false, 'Collisions', 'on');
    hold on;

    % Initialize environment with platform and cylinder
    Px100.env = {};

    % Add platform
    platform = collisionBox(0.6, 1, 0.02);
    platform.Pose = trvec2tform([0, 0, -0.01]);
    Px100.env{1} = platform;

    % Add cylinder at pick pose
    cylinderRadius = 0.03;
    cylinderHeight = 0.05;
    cylinder = collisionCylinder(cylinderRadius, cylinderHeight);
    cylinder.Pose = pick_pose * trvec2tform([0, 0, -cylinderHeight / 2]); % Centered at pick position
    Px100.env{2} = cylinder;

    % Add pick and drop markers
    add_markers(pick_pose, drop_pose);

    % Display environment objects
    for i = 1:length(Px100.env)
        show(Px100.env{i});
    end
end
% Placing markers at Pick and drop position
function add_markers(pickPose, dropPose)
    global Px100;

    % Add pick marker
    pickMarker = collisionSphere(0.01);
    pickMarker.Pose = pickPose;
    Px100.env{3} = pickMarker;

    % Add drop marker
    dropMarker = collisionSphere(0.01);
    dropMarker.Pose = dropPose;
    Px100.env{4} = dropMarker;

    % Display markers
    show(pickMarker);
    show(dropMarker);
end

%Imposing joint limit constraints and minimum Z threshold
function execute_trajectory(waypoints)
    global Px100;

    % Define joint limits
    joint_limits = [
        -1.6, 1.53;  % Joint 1
        -1.68, 1.55; % Joint 2
        -1.68, 1.55; % Joint 3
        -1.86, 2.07  % Joint 4
    ];

    % Minimum allowed z position
    min_z_thresh = 0.02; % 0.02 m

    % Maximum allowed distance between consecutive waypoints
    movement_thresh = 0.1;  % 0.1 m

    for i = 1:size(waypoints, 1)
        % Apply joint limits for each joint
        for j = 1:4
            waypoints(i, j) = max(joint_limits(j, 1), min(joint_limits(j, 2), waypoints(i, j)));
        end

        % Calculate the forward kinematics for the current configuration
        Px100.currentConfig(1:4) = waypoints(i, :);
        eePose = getTransform(Px100.robot, Px100.currentConfig, 'px100/ee_gripper_link');

        % Check the z position constraint
        if eePose(3, 4) < min_z_thresh
            error('Z position constraint violated at waypoint %d. Execution stopped.', i);
        end

        % Check distance between waypoints
        if i > 1
            dist = norm(waypoints(i, :) - waypoints(i - 1, :));
            if dist > movement_thresh
                error('Distance between consecutive waypoints exceeds threshold at waypoint %d. Execution stopped.', i);
            end
        end

        % Show the robot's configuration
        show(Px100.robot, Px100.currentConfig, 'PreservePlot', false, 'Collisions', 'on');
        drawnow;
    end
end

function attachCylinder()
    global Px100;
    if isempty(Px100.env{2})
        disp('Cylinder is already attached or does not exist.');
        return;
    end

    cylinder = Px100.env{2};
    cylinderBody = rigidBody('cylinderBody');
    cylinderJoint = rigidBodyJoint('cylinderJoint', 'fixed');

    % Get end-effector pose at the current config
    eePose = getTransform(Px100.robot, Px100.currentConfig, 'px100/ee_gripper_link');
    setFixedTransform(cylinderJoint, eePose \ cylinder.Pose);
    addCollision(cylinderBody, cylinder, inv(cylinder.Pose));
    cylinderBody.Joint = cylinderJoint;
    addBody(Px100.robot, cylinderBody, 'px100/ee_gripper_link');

    % Remove the cylinder from the environment to indicate it is picked up
    Px100.env{2} = [];
end

function detachCylinder()
    global Px100;

    % Remove cylinder from robot
    removeBody(Px100.robot, 'cylinderBody');

    % Get the end-effector pose at the current configuration
    dropPose = getTransform(Px100.robot, Px100.currentConfig, 'px100/ee_gripper_link');

    % Define cylinder dimensions
    cylinderRadius = 0.03;
    cylinderHeight = 0.05;

    % Create a new cylinder collision object
    cylinder = collisionCylinder(cylinderRadius, cylinderHeight);

    % Adjust cylinder pose relative to the drop pose
    cylinder.Pose = dropPose * trvec2tform([0, 0, -cylinderHeight / 2]);

    % Re-add the cylinder to the environment at the new pose
    Px100.env{2} = cylinder;

    % Display the updated environment for verification
    show(Px100.env{2});
end

function joint_positions = get_current_joint_positions()
    global Px100;
    % Read only the first 4 joint positions
    joint_positions = Px100.currentConfig(1:4);
end

