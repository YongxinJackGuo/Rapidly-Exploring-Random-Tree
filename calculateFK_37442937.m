function [jointPositions,T0e] = calculateFK_37442937(q)
% CALCULATEFK_PENNKEY - Please rename this function using your pennkey in
%   both the function header and the file name. If you are working in
%   pairs, choose ONE pennkey to use - otherwise the grading scripts will
%   get confused.
%
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   q - 1x6 vector of joint inputs [q1,q2,q3,q4,q5,lg]
%
% OUTPUT:
%   jointPositions - 6 x 3 matrix, where each row represents one 
%                    joint along the robot. Each row contains the [x,y,z]
%                    coordinates of the respective joint's center (mm). For
%                    consistency, the first joint should be located at 
%                    [0,0,0]. These values are used to plot the robot.
%   T0e            - a 4 x 4 homogeneous transformation matrix, 
%                    representing the end effector frame expressed in the 
%                    base (0) frame
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your code here

th1 = q(1);
th2 = q(2);
th3 = q(3);
th4 = q(4);
th5 = q(5);

% Define some physical parameters
L1 = 25.4*3;
L2 = 25.4*5.75;
L3 = 25.4*7.375;
L4 = 25.4*3;



% Define a DH table
DH = [   0   -pi/2      L1            th1;...
        L2       0       0    (-pi/2)+th2;...
        L3       0       0     (pi/2)+th3;...
         0   -pi/2       0    (-pi/2)+th4;
         0       0      L4            th5;...
         0       0   28.57              0];
     
% Define the degree of freedom
[ro, co] = size(DH);
dof = ro;


% Calculate the FK

% get the transformation matrix from frame 0 to frame 5
% T05 = getFK(DH,1,dof);
% T05 = simplify(T05);
% disp("The transformation matrix from frame 5 to frame 0, T05 is: " + newline);
% disp(T05);

% % get the transformation matrix from frame 0 to frame 3
% T03 = getFK(DH,1,3);
% T03 = simplify(T03);
% disp("The transformation matrix from frame 3 to frame 0, T03 is: " + newline);
% disp(T03);
% 
% % get the transformation matrix from frame 3 to frame 5
% T35 = getFK(DH,3,5);
% T35 = simplify(T35);
% disp("The transformation matrix from frame 3 to frame 5, T35 is: " + newline);
% disp(T35);


% location of each joint

% Joint 1 (Origin 0)
Location_Joint_1 = [0, 0, 0];

% Joint 2 (Origin 1)
T01 = getFK(DH,1,1);
Location_Joint_2 = T01(1:3,4)';

% Joint 3 (Origin 2)
T02 = getFK(DH,1,2);
Location_Joint_3 = T02(1:3,4)';

% Joint 4 (Origin 3)
T03 = getFK(DH,1,3);
Location_Joint_4 = T03(1:3,4)';

% Joint 5 (Origin 4, should coincide with Joint 4)
T04 = getFK(DH,1,4);
T04_offset = [1  0  0   0;...
              0  1  0   0;...
              0  0  1  34;...
              0  0  0   1];
T04 = T04*T04_offset;
Location_Joint_5 = T04(1:3,4)';

% Gripper center (Origin 5)
T05 = getFK(DH,1,5);
Location_Gripper_center = T05(1:3,4)';

jointPositions = [Location_Joint_1;...
                  Location_Joint_2;...
                  Location_Joint_3;...
                  Location_Joint_4;...
                  Location_Joint_5;...
                  Location_Gripper_center];
         
End_Effector_Frame = getFK(DH,1,6);

% T_tip = [1  0  0     0;...
%          0  1  0     0;...
%          0  0  1 28.57;...
%          0  0  0     1];

% T0e = End_Effector_Frame * T_tip;
T0e = End_Effector_Frame;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

function T = getFK(DH_table, start_row, end_row)

T = eye(4);

for i = start_row : end_row
    % target: i
    
    a     = DH_table(i,1);
    % Link length, distance between Zi-1 and Zi, measured along Xi
    
    alpha = DH_table(i,2);
    % Link twist, angle between Zi-1 and Zi, measured in the plane normal to Xi (RHR)
    
    d     = DH_table(i,3);
    % Link offset, distanc between Xi-1 and Xi, measured along Zi-1
    
    th    = DH_table(i,4);
    % Joint Angle, angle between Xi-1 and Xi, measured in the plane normal to Zi-1 (RHR)
    
    A = [  cos(th),  -sin(th)*cos(alpha),  sin(th)*sin(alpha),  a*cos(th);...
           sin(th),   cos(th)*cos(alpha), -cos(th)*sin(alpha),  a*sin(th); ...
                 0,           sin(alpha),          cos(alpha),          d; ...
                 0,                    0,                   0,          1];
   
    T = T * A;
end

end



