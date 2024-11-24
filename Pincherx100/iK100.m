function joint_angles = iK100(T, lm,elbow_config)
    % Analytical IK for PincherX 100 Robot
    
    % Robot link lengths
    L1 = 0.0931; % Base to shoulder rotation axis
    L2 = 0.10595; % Shoulder to elbow length
    L3 = 0.2;     % Elbow to wrist center
    L4 = 0.109;   % Wrist to TCP offset
    LM = lm;
    % Elbow configuration
    if elbow_config == "up"
        up = true;
    elseif elbow_config == "down"
        up = false;
    else
        error("Invalid elbow configuration. Use 'up' or 'down'.");
    end

    % Extract target position from transformation matrix
    px = T(1, 4);
    py = T(2, 4);
    pz = T(3, 4);

    % Solve for q1 (Base Joint)
    q1 = atan2(py, px);

    % Wrist center position
    wx = px - L4 * T(1, 3);
    wy = py - L4 * T(2, 3);
    wz = pz - L4 * T(3, 3);

    % Compute intermediate distances
    r = sqrt(wx^2 + wy^2); % Horizontal distance to wrist center
    h = wz - L1;           % Vertical distance to wrist center
    c = sqrt(r^2 + h^2);   % Total distance to wrist center

    % Check if the target is within the reachable workspace
    if c > (L2 + L3) || c < abs(L2 - L3)
        error("Target position is outside the reachable workspace.");
    end

    % Solve for q3 (Elbow Joint)
    cos_q3 = (L2^2 + L3^2 - c^2) / (2 * L2 * L3);
    cos_q3 = clamp(cos_q3, -1, 1); % Clamp to avoid numerical issues
    if up
        q3 = -acos(cos_q3); % Elbow-up
    else
        q3 = acos(cos_q3);  % Elbow-down
    end

    % Solve for q2 (Shoulder Joint)
    gamma = atan2(h, r);
    beta = atan2(L3 * sin(q3), L2 + L3 * cos(q3));
    q2 = gamma - beta;

    % Solve for q4 (Wrist Joint)
    phi = atan2(T(2, 3), T(1, 3));
    q4 = phi - q2 - q3;

    % Return joint angles
    joint_angles = [q1, q2, q3, q4];
end

% Helper function to clamp values
function clamped_value = clamp(value, min_val, max_val)
    clamped_value = max(min_val, min(value, max_val));
end
