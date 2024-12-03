function waypoints = generate_waypoints(start, goal, num_waypoints)
    movement_thresh = 0.1; % Maximum allowed distance between consecutive waypoints (in meters)
    waypoints_valid = false;

    % Continue generating waypoints until the distance between consecutive waypoints meets the threshold
    while ~waypoints_valid
        % Generate initial waypoints using linear interpolation
        waypoints = zeros(num_waypoints, length(start));
        for i = 1:length(start)
            waypoints(:, i) = linspace(start(i), goal(i), num_waypoints);
        end

        % Check distances between consecutive waypoints
        waypoints_valid = true; % Assume waypoints are valid initially
        for i = 2:num_waypoints
            distance = norm(waypoints(i, :) - waypoints(i - 1, :));
            if distance > movement_thresh
                % If any distance exceeds the threshold, increase the number of waypoints and try again
                num_waypoints = num_waypoints + 1;
                waypoints_valid = false;
                break;
            end
        end
    end

    
end