global Px100;

% Set your port number
Px100.DEVICENAME = 'COM9';

% Initializing the robot
init_robot
home_config=[0,-1,1,0];
% Getting the minimum Z threshold
min_z_thresh = Px100.MIN_Z_THRESH;
pick_pose_base=trvec2tform([0.15,0.10,0.05]);
% Getting max movement distance
movement_thresh = Px100.MOVEMENT_THRESH;
num_waypoints=500;
time_to_travel=30;
%% Reaching the Payload
reach=true;
while(true)
    closeGripper(false);
    disp("Getting joint positions");
    joint_positions = get_joint_pos();    
    home_config=joint_positions;
    % Get the postions of pick , place and obstacle
    %[pick_pose_base, drop_pose_base, obstacle_pose_base] = get_pick_and_place_position();
    pick_pose_angle=ikpincher100(pick_pose_base);
    intermediate1= [pick_pose_angle(1,1);0;0;0];
    waypoints=JointSpaceWaypoints(home_config,intermediate1,time_to_travel,num_waypoints);
    disp("Trajectory generated for home to pick");
    N = length(waypoints);

    for i=1:N
        set_joint_pos(waypoints(i,:));
    end

    disp("Reached Hovering point");
    reach=false
    break;
end

reach=true;
while(reach)
    disp("Going towards pick positions");   
    waypoints= JointSpaceWaypoints(intermediate1,pick_pose_angle,time_to_travel,num_waypoints);
    N = length(waypoints);
    for i=1:N
        set_joint_pos(waypoints(i,:));
    end   
    disp("Reached Pick pose");
    closeGripper(true);
    reach=false;
    break;
end

reach=true;
while(reach)
    disp("Going to drop pose");

    drop_pose_angle=ikpincher100(drop_pose_base);
    intermediate2= [drop_pose_angle(1,1);0;0;0];
    waypoints=JointSpaceWaypoints(pick_pose_angle,intermediate2,time_to_travel,num_waypoints);
    disp("Trajectory generated for pick to hover at drop");
     for i=1:N
        set_joint_pos(waypoints(i,:));
     end 
     reach=false;
     break;
end
reach=true;
while(reach)
   
    waypoints=JointSpaceWaypoints(intermediate2,drop_pose_angle,time_to_travel,num_waypoints);
    disp("Trajectory generated from hover->drop");
     for i=1:N
        set_joint_pos(waypoints(i,:));
     end 
     reach=false;
     break;
end
closeGripper(false);
disp("TOUCHDOWN CONFIRMED, DROPPED PAYLOAD SUCCESSFULLY")

reach=true;
while(reach)
    disp("GOING HOME");
    waypoints=JointSpaceWaypoints(drop_pose_angle,home_config,num_waypoints);
  
     for i=1:N
        set_joint_pos(waypoints(i,:));
     end 
     reach=false;
     
     break;
     
end
disp("REACHED HOME");
% Close the gripper

% disp("Closing the gripper");
% pause(0.5) % Wait for the gripper to close
% disp("Reaching hovering point");
% %% Transporting Payload
% while(true)
%     % Getting joint positions
%     joint_positions = get_joint_pos();
%     % for i=1:4
%     %    joint_angles(i) = round(Px100.angle_min + ((val_read - Px100.encoder_min) * (Px100.angle_max - Px100.angle_min)/(Px100.encoder_max - Px100.encoder_min)));
%     % end
% 
%     drop_pose_angle=ikpincher100(drop_pose_base);
%     intermediate2= [drop_pose_angle(1,1);0;0;0];
%     waypoints=JointSpaceWaypoints(joint_angles,intermediate2,time_to_travel,num_waypoints);
%     disp("Trajectory generated for pick to drop");
% 
%     % 
%     N = length(waypoints);
% 
%     for i=1:N
%         % Give position command to robot
%         % waypoints(i,:)
%         set_joint_pos(waypoints(i,:));
%     end
%     disp("Reached Hovering point");
%     waypoints= JointSpaceWaypoints(intermediate2,drop_pose_angle,time_to_travel,num_waypoints);
%     N = length(waypoints);
% 
%     for i=1:N
%         % Give position command to robot
%         % waypoints(i,:)
%         set_joint_pos(waypoints(i,:));
%     end
%     disp("Reached Drop pose");
% 
%     % Open the gripper
%     closeGripper(False);
% end
% 
% pause(0.5);
% 
% %% Returning to Home config
% 
% while(true)
%     curr_joint_positions = get_joint_pos();
%     % for i=1:4
%     %    joint_angles(i) = round(Px100.angle_min + ((val_read - Px100.encoder_min) * (Px100.angle_max - Px100.angle_min)/(Px100.encoder_max - Px100.encoder_min)));
%     % end
%     % 
% 
%     intermediate3= [home_config(1,1);0;0;0];
%     waypoints=JointSpaceWaypoints(drop_pose_angle,intermediate3,time_to_travel,num_waypoints);
%     disp("Trajectory generated for HOME");
% 
%     % 
%     N = length(waypoints);
% 
%     for i=1:N
%         % Give position command to robot
%         % waypoints(i,:)
%         set_joint_pos(waypoints(i,:));
%     end
%     disp("Reached Hovering point");
%     waypoints= JointSpaceWaypoints(intermediate3,home_config,time_to_travel,num_waypoints);
%     N = length(waypoints);
% 
%     for i=1:N
%         % Give position command to robot
%         % waypoints(i,:)
%         set_joint_pos(waypoints(i,:));
%     end
%     disp("Reached HOME");
% 
%     % Open the gripper
%     closeGripper(False);
% end