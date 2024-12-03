function waypoints = JointSpaceWaypoints(start, stop, time, num_waypoints)
    global Px100;
    robot = Px100;
    numberofpoints = num_waypoints;

    % Setting minimum threshold
    MIN_Z_THRESH = robot.MIN_Z_THRESH;
    MOVEMENT_THRESH = robot.MOVEMENT_THRESH;

    % Define encoder and angle range limits
    Px100.angle_min = -pi;
    Px100.angle_max = pi;
    Px100.encoder_min = 0;
    Px100.encoder_max = 4095;

    % Initial and final joint positions
    q_start = start(:); % Ensure column vector
    q_end = stop(:);

    % Velocities at start and end of trajectory
    v_start = zeros(size(q_start));
    v_end = zeros(size(q_end));

    % Time vector
    t = linspace(0, time, numberofpoints);

    % Compute coefficients for cubic trajectory
    a0 = q_start;
    a1 = v_start;
    a2 = (3 * (q_end - q_start) / time^2) - (2 * v_start / time) - (v_end / time);
    a3 = (-2 * (q_end - q_start) / time^3) + ((v_start + v_end) / time^2);

    % Generate the trajectory
    waypoints = zeros(numberofpoints, length(q_start));
    encoder_val = zeros(num_waypoints, length(q_start));
    joint_angles = zeros(num_waypoints, length(q_start));

    for i = 1:numberofpoints
        % Calculate joint space waypoints
        waypoints(i, :) = (a0 + a1 * t(i) + a2 * t(i)^2 + a3 * t(i)^3); % Positions stored row-wise
        
        % Convert joint angles to encoder values
        encoder_val(i, :) = angle_to_encoder(waypoints(i, :));
    end

    % % Verify limits and update joint angles
    % for i = 1:numberofpoints
    %     valid = true;
    % 
    %     for j = 1:4
    %         % Extract encoder value
    %         val_read = encoder_val(i, j);
    %         disp(val_read);
    % 
    %         % Checking for joint limit violations
    %         if val_read < robot.JOINT_LIMITS(j, 1) || val_read > robot.JOINT_LIMITS(j, 2)
    %             valid = false;
    %             fprintf("\n\n==========x==========\n\nWARNING!\nJoint limit exceeded!\n");
    %         end
    % 
    %         % Converting encoder values back to joint angles
    %         joint_angles(i, j) = round(Px100.angle_min + ((val_read - Px100.encoder_min) * (Px100.angle_max - Px100.angle_min) / (Px100.encoder_max - Px100.encoder_min)));
    %     end
    % 
    %     % You may choose to halt the process or notify further based on `valid` here
    % end
    % 
    % % Returning joint_angles is not required but might be helpful for debug
    % joint_angles
end

