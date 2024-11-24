function J = jacobian_x100(theta)
    % DH Parameters (alpha, a, d, theta)
    dhparams = [0, 0, 0.0931, theta(1);
                -pi/2, 0, 0, theta(2);
                 0, 0.1059, 0, theta(3);
                 0, 0.1, 0, theta(4)];

  
    T = eye(4);
    
   
    T_list = cell(1, 4);
    O = zeros(3, 4);
    Z = zeros(3, 4);

    Z(:,1) = [0; 0; 1]; % Z-axis of the base frame
    O(:,1) = [0; 0; 0]; % Origin of the base frame

   
    for i = 1:4
       
        alpha_i = dhparams(i, 1);
        a_i = dhparams(i, 2);
        d_i = dhparams(i, 3);
        theta_i = dhparams(i, 4);

       
        T_i = [cos(theta_i), -sin(theta_i) * cos(alpha_i), sin(theta_i) * sin(alpha_i), a_i * cos(theta_i);
               sin(theta_i), cos(theta_i) * cos(alpha_i), -cos(theta_i) * sin(alpha_i), a_i * sin(theta_i);
               0, sin(alpha_i), cos(alpha_i), d_i;
               0, 0, 0, 1];
        
       
        T = T * T_i;
        T_list{i} = T;
        
       
        if i < 4
            Z(:, i+1) = T(1:3, 3);
            O(:, i+1) = T(1:3, 4);
        end
    end
    
   
    O_e = T(1:3, 4);
    
   
    J_v = zeros(3, 4);
    J_w = zeros(3, 4);
    
    for i = 1:4
        % Calculate the linear velocity part (position Jacobian)
        J_v(:, i) = cross(Z(:, i), O_e - O(:, i));
        
        % Calculate the angular velocity part (only for revolute joints)
        J_w(:, i) = Z(:, i);
    end
    
   
    J = [J_v; J_w];
end
