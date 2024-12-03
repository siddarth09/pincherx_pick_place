function T_e = fKpincherx100(theta)
    % Extract joint angles
    t1 = theta(1);
    t2 = theta(2) + 1.2341; % Offset for q2
    t3 = theta(3) - 1.2341; % Offset for q3
    t4 = theta(4);

    % DH TABLE (alpha, a, d, theta)
    dhparams = [0,    0,      0.0445, t1;
               -pi/2, 0,      0,      t2;
                0,    0.1010, 0,      t3;
                0,    0.1010, 0,      t4];

    % Tool center point offset (TCP)
    tcp = [1, 0, 0, 0.163;
           0, 0, -1, 0;
           0, 1, 0, 0;
           0, 0, 0, 1];

    % Initialize transformation matrix
    T_e = eye(4);

    % Compute forward kinematics using DH parameters
    for i = 1:size(dhparams, 1)
        a_i = dhparams(i, 2);
        alpha_i = dhparams(i, 1);
        d_i = dhparams(i, 3);
        theta_i = dhparams(i, 4);

        % Transformation matrix for each joint
        T_i = [cos(theta_i), -sin(theta_i), 0, a_i;
               sin(theta_i) * cos(alpha_i), cos(theta_i) * cos(alpha_i), -sin(alpha_i), -sin(alpha_i) * d_i;
               sin(theta_i) * sin(alpha_i), cos(theta_i) * sin(alpha_i), cos(alpha_i), cos(alpha_i) * d_i;
               0, 0, 0, 1];

        % Multiply the current transformation matrix with the previous one
        T_e = T_e * T_i;
    end

    % Apply the tool center point transformation
    T_e = T_e * tcp;
end
