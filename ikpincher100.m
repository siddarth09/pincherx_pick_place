function joint_angles = ikpincher100(joint_pose)
    
    l1 = 0.0931;
    l2=0.1059481005;
    l3 = 0.1;
    l4=0.1136;

    px = joint_pose(1,4);
    py = joint_pose(2,4);
    pz = joint_pose(3,4);

    m=sqrt(px^2+py^2)-l4;
    n=pz;
    j2_def_ang = atan2(0.035,0.1);
    j3_def_ang= atan2(0.1,0.035);

    theta1 = atan2(py,px);

    costheta3 = (m^2+n^2-l2^2-l3^2)/(2*l2*l3);

    theta3_r = atan2(sqrt(1-costheta3^2),costheta3);

    beta = atan2(n,m);

    psi = atan2(-l3*sin(theta3_r), l2+(l3*costheta3));

    theta2_r = beta+psi;

    theta4_r = -theta2_r-theta3_r;

    theta2 = pi/2 - j2_def_ang+theta2_r;

    theta3 = theta3_r-j3_def_ang;

    theta4 = theta4_r;

    joint_angles = [theta1; theta2; theta3 ; theta4];

end