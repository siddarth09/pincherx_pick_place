function encoder_val=angle_to_encoder(joint_angles)
    global Px100;
    robot=Px100;
   
    encoder_val=zeros(1,4);
    
    Px100.angle_min = -pi;
    Px100.angle_max =  pi;
    Px100.encoder_min = 0;
    Px100.encoder_max = 4095;

    for i=1:4
    % Calculate encoder values
        val_read=joint_angles(i);
        encoder_val(i) = round(Px100.encoder_min + ((val_read - Px100.angle_min) * (Px100.encoder_max - Px100.encoder_min) / (Px100.angle_max - Px100.angle_min)));
       
    end
    val_read = zeros(1,4);
    for i=1:4
        temp = groupBulkReadGetData(robot.groupread_num, robot.DXL_ID(i), robot.ADDR_PRESENT_POSITION, robot.LEN_PRESENT_POSITION);
        % Updated part ( Converting encoder values to joint angles)
        temp = Px100.angle_min + ((temp - Px100.encoder_min) * (Px100.angle_max - Px100.angle_min)/(Px100.encoder_max - Px100.encoder_min));
        val_read(i) = temp;
    end

