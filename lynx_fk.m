function [ pos ] = lynx_fk( theta1, theta2, theta3, theta4, theta5, g )
%LYNX_FK The input to the function will be the joint
%    angles of the robot in radians, and the distance between the gripper pads in inches.

    a = 3;
    b=5.75;
    c=7.375;
    d=4.125;
    e=1.125;
    A_1=[0 0 0 1]';
    r=[0 b c 0 0];
    di=[a 0 0 0 d];
    ap=[-pi/2 0 0 -pi/2 0];
    T_1= compute_dh_matrix(r(1,1),ap(1,1), di(1,1), theta1);
    A_2=T_1*A_1;
    T_2= compute_dh_matrix(r(1,2),ap(1,2),di(1,2),theta2-pi/2);
    A_3=T_1*T_2*A_1;
    T_3= compute_dh_matrix(r(1,3),ap(1,3),di(1,3),theta3+pi/2);
    A_4=T_1*T_2*T_3*A_1;
    T_4= compute_dh_matrix(r(1,4),ap(1,4),di(1,4),theta4-pi/2);
    A_5=T_1*T_2*T_3*T_4*A_1;
    pos_1=[A_1(1:3,1)';A_2(1:3,1)';A_3(1:3,1)';A_4(1:3,1)';A_5(1:3,1)';];
    T_5=compute_dh_matrix(r(1,5),ap(1,5),di(1,5),theta5);
    GR=T_1*T_2*T_3*T_4*T_5*A_1;
    GRfinal=GR(1:3,1)';
    A_6=T_1*T_2*T_3*T_4*T_5*([0 0 -e 1]'); 
    A_7=T_1*T_2*T_3*T_4*T_5*([g/2 0 -e 1]');
    A_8=T_1*T_2*T_3*T_4*T_5*([-g/2 0 -e 1]');
    A_9=T_1*T_2*T_3*T_4*T_5*([g/2 0 0 1]');
    A_10=T_1*T_2*T_3*T_4*T_5*([-g/2 0 0 1]');
    pos_2=[A_6(1:3,1)';A_7(1:3,1)';A_8(1:3,1)';A_9(1:3,1)';A_10(1:3,1)'];
    pos=[pos_1;pos_2];
    end
function A = compute_dh_matrix(r, alpha, d, theta)

    A=[cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) r*cos(theta);sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) r*sin(theta);0 sin(alpha) cos(alpha) d;0 0 0 1];
    
end