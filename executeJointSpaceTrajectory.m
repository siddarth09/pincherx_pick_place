function executeJointSpaceTrajectory(start, stop, time, numberofpoints)
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
    movement_thresh = 0.1; % 0.1 m

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
    for i = 1:numberofpoints
        waypoints(i, :) = (a0 + a1 * t(i) + a2 * t(i)^2 + a3 * t(i)^3)'; % Positions stored row-wise
    end

    % Execute trajectory with constraints and visualization
    for i = 1:numberofpoints
        % Apply joint limits
        for j = 1:size(joint_limits, 1)
            waypoints(i, j) = max(joint_limits(j, 1), min(joint_limits(j, 2), waypoints(i, j)));
        end
        
        % Update robot's joint configuration
        Px100.currentConfig(1:4) = waypoints(i, :);

        % Calculate forward kinematics for end-effector position
        eePose = getTransform(Px100.robot, Px100.currentConfig, 'px100/ee_gripper_link');

        % Check z-position constraint
        if eePose(3, 4) < min_z_thresh
            error('Z position constraint violated at waypoint %d. Execution stopped.', i);
        end

        % Check movement threshold between consecutive waypoints
        if i > 1
            dist = norm(waypoints(i, :) - waypoints(i - 1, :));
            if dist > movement_thresh
                error('Distance between consecutive waypoints exceeds threshold at waypoint %d. Execution stopped.', i);
            end
        end

        % Visualize robot's configuration
        show(Px100.robot, Px100.currentConfig, 'PreservePlot', false, 'Collisions', 'on');
        drawnow;
    end

    % % Plot joint trajectories
    % figure;
    % subplot(2, 1, 1);
    % plot(t, waypoints);
    % title('Joint Position Trajectory');
    % xlabel('Time (s)');
    % ylabel('Position (rad)');
    % legend(arrayfun(@(i) sprintf('Joint %d', i), 1:size(waypoints, 2), 'UniformOutput', false));
    % 
    % subplot(2, 1, 2);
    % % Compute velocities for plotting (finite differences)
    % velocities = diff(waypoints) / (time / numberofpoints);
    % plot(t(1:end-1), velocities);
    % title('Joint Velocity Trajectory');
    % xlabel('Time (s)');
    % ylabel('Velocity (rad/s)');
    % legend(arrayfun(@(i) sprintf('Joint %d', i), 1:size(waypoints, 2), 'UniformOutput', false));
end
