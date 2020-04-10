function [isCollided] = isRobotCollided(q, map, robot)
% ISROBOTCOLLIDED Detect if a configuration of the Lynx is in collision
%   with any obstacles on the map.
%
% INPUTS:
%   q   - a 1x6 configuration of the robot
%   map - a map strucutre containing axis-aligned-boundary-box obstacles
%   robot - a structure containing the robot dimensions and joint limits
%
% OUTPUTS:
%   isCollided - a boolean flag: 1 if the robot is in collision, 0 if not.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This code will model Link 2, Link 3 and Link 4 of Lynx robot as three
% cylinders because Link 1 always stays at the base and won't be taken into
% account for saving computational time.
isCollided = false; % initialize the isCollided variable to be false

% The following codes will model all the links as the cylinder by 
% discretizing the cylinder into a collection of lines
% around the surface of the cylinder. First a collection of points will be
% defined at the top and bottom respectively. By connecting each pair of
% points at the top and bottom of the cylinder, a line segment is created.



%--------model Link 2 using cylindrical coordinates-------
% please note the suffix 2 represents Link 2 variables
r2 = 20; % declare the radius of cyliner as 100mm
resolution2 = 20; % define how many line segments are needed for L2 cylinder.
stepAngle2 = (2 * pi) / resolution2; % defind the step angle based on resolution.
startDepth2 = -1 * robot.a2 - 50; % define the start level for the depth in cylindrical coordinates that corresponds to x-axis of frame 2
endDepth2 = 50; % assign 50mm
% prelocate the size
y2 = zeros(resolution2,1);
x2_start = zeros(resolution2,1);
x2_end = zeros(resolution2,1);
z2 = zeros(resolution2,1);
% start discretizing points around the bottom and top of the cylinder.
for i = 1:resolution2
    y2(i) = r2 * cos(stepAngle2 * i); % define the y-coordinates.
    z2(i) = r2 * sin(stepAngle2 * i); % define the z-coordinates.
    x2_start(i) = startDepth2; % define the x-coordinates for start level, which is the bottom of the cylinder
    x2_end(i) = endDepth2; % define the x-coordinates for end level, which is the top of the cylinder.
    % please note that for the top and bottom level, y and z coordinates
    % are fixed.
end
bottomPoints2 = [x2_start, y2, z2];
topPoints2 = [x2_end, y2, z2];


%--------model Link 3 using cylindrical coordinates-------
% please note the suffix 3 represents Link 3 variables
r3 = 20; % declare the radius of cyliner as 100mm
resolution3 = 20; % define how many line segments are needed for L3 cylinder.
stepAngle3 = (2 * pi) / resolution3; % defind the step angle based on resolution.
startDepth3 = -1 * robot.a3 - 50; % define the start level for the depth in cylindrical coordinates that corresponds to x-axis in frame 3
endDepth3 = 50; % assign 50mm
% prelocate the size
y3 = zeros(resolution3,1);
x3_start = zeros(resolution3,1);
x3_end = zeros(resolution3,1);
z3 = zeros(resolution3,1);
% start discretizing points around the bottom and top of the cylinder.
for i = 1:resolution3
    y3(i) = r3 * cos(stepAngle3 * i); % define the y-coordinates.
    z3(i) = r3 * sin(stepAngle3 * i); % define the z-coordinates.
    x3_start(i) = startDepth3; % define the x-coordinates for start level, which is the bottom of the cylinder
    x3_end(i) = endDepth3; % define the x-coordinates for end level, which is the top of the cylinder.
    % please note that for the top and bottom level, y and z coordinates
    % are fixed.
end
bottomPoints3 = [x3_start, y3, z3];
topPoints3 = [x3_end, y3, z3];




%--------model Link 4 using cylindrical coordinates-------
% please note the suffix 4 represents Link 4 variables
r4 = 20; % declare the radius of cyliner as 100mm
resolution4 = 20; % define how many line segments are needed for L3 cylinder.
stepAngle4 = (2 * pi) / resolution4; % defind the step angle based on resolution.
startDepth4 = -50; % define the start level for the depth in cylindrical coordinates that corresponds to z-axis in frame 4
endDepth4 = robot.d5 + robot.lg; % assign d5 in robot struct to the end level. d5 plust lg is the length of Link L4.
% prelocate the size
x4 = zeros(resolution4,1);
z4_start = zeros(resolution4,1);
z4_end = zeros(resolution4,1);
y4 = zeros(resolution4,1);
% start discretizing points around the bottom and top of the cylinder.
for i = 1:resolution4
    x4(i) = r4 * cos(stepAngle4 * i); % define the x-coordinates.
    y4(i) = r4 * sin(stepAngle4 * i); % define the y-coordinates.
    z4_start(i) = startDepth4; % define the z-coordinates for start level, which is the bottom of the cylinder
    z4_end(i) = endDepth4; % define the z-coordinates for end level, which is the top of the cylinder.
    % please note that for the top and bottom level, x and y coordinates
    % are fixed.
end
bottomPoints4 = [x4, y4, z4_start];
topPoints4 = [x4, y4, z4_end];

% Assign configurations
th1 = q(1);
th2 = q(2);
th3 = q(3);
th4 = q(4);
th5 = q(5);

% Assign robotic link length
L1 = robot.d1;
L2 = robot.a2;
L3 = robot.a3;
L4 = robot.d5;
L5 = robot.lg;

% Define a DH table for one specific configuration
DH = [   0   -pi/2      L1            th1;...
        L2       0       0    (-pi/2)+th2;...
        L3       0       0     (pi/2)+th3;...
         0   -pi/2       0    (-pi/2)+th4;
         0       0      L4            th5;...
         0       0      L5              0];
     
% calculate transformed discrete points in one specific configuration
T02 = getFK(DH,1,2);
T03 = getFK(DH,1,3);
T04 = getFK(DH,1,4);

% compute transformed bottom and top points for L2 cylinder attached to
% frame 2 using T02
for i = 1:resolution2
    transformedBottomPoints2 = T02 * [transpose(bottomPoints2(i,:));1];
    bottomPoints2(i,:) = transpose(transformedBottomPoints2(1:3));
    transformedTopPoints2 = T02 * [transpose(topPoints2(i,:));1];
    topPoints2(i,:) = transpose(transformedTopPoints2(1:3));
end

% compute transformed bottom and top points for L3 cylinder attached to
% frame 3 using T03
for i = 1:resolution3
    transformedBottomPoints3 = T03 * [transpose(bottomPoints3(i,:));1];
    bottomPoints3(i,:) = transpose(transformedBottomPoints3(1:3));
    transformedTopPoints3 = T03 * [transpose(topPoints3(i,:));1];
    topPoints3(i,:) = transpose(transformedTopPoints3(1:3));
end

% compute transformed bottom and top points for L4 cylinder attached to
% frame 4 using T04
for i = 1:resolution4
    transformedBottomPoints4 = T04 * [transpose(bottomPoints4(i,:));1];
    bottomPoints4(i,:) = transpose(transformedBottomPoints4(1:3));
    transformedTopPoints4 = T04 * [transpose(topPoints4(i,:));1];
    topPoints4(i,:) = transpose(transformedTopPoints4(1:3));
end

%----------------start checking collision with obstacles-----------------
[obstacleNum,col] = size(map.obstacles); % get the number of obstacles
aggregateTopPoints = [topPoints2; topPoints3; topPoints4]; % sum up all the top cylinder points
aggregateBottomPoints = [bottomPoints2; bottomPoints3; bottomPoints4]; % sum up all the bottom cylinder points
totalResolution = resolution2 + resolution3 + resolution4; % sum up all the resolution number 
% check three links
isCollided_obstacle = zeros(totalResolution,1); % initialize a vector of all zeros for collision flag
% check all obstacles
for i = 1:obstacleNum
    isCollided_obstacle = isCollided_obstacle | detectCollision(aggregateBottomPoints, aggregateTopPoints, map.obstacles(i,:));% get a vectorize booleans
end
isCollided_obstacle_oneBol = false; % declare a single boolean variable as false initially
for i = 1:totalResolution
    if isCollided_obstacle(i) == 1 
        isCollided_obstacle_oneBol = true; % if one of the booleans are true then change single bol to true and break.
        break;
    end
end


%----------------start checking collision with boundary-----------------
isCollided_boundary_oneBol = false; % declare a single boolean variable as false initially
boundary = map.boundary;
for i = 1:totalResolution
    for j = 1:3
        if (aggregateTopPoints(i,j) < boundary(j)) || (aggregateTopPoints(i,j) > boundary(j+3))
            isCollided_boundary_oneBol = true; % if any point is beyond the max or min xyz coordinates
            break;                             % then it means the arm is hitting the boundary, so break the loop.
        end
    end
    if isCollided_boundary_oneBol == true 
        break; % break outside for loop after breaking the inside one
    end
end

%----------------start checking collision with boundary-----------------
l = 50;
w = 50;
depth = 80; % along x-axis direction in frame 2
start_depth = -30; 

% define xmin ymin zmin and xmax ymax zmax for rectangular in homogenous
% representation
rectMin = [start_depth - depth, -l/2, -w/2, 1];
rectMax = [start_depth, l/2, w/2, 1];

% get transformed rectangular max and min points
rectMin = T02 * transpose(rectMin);
rectMax = T02 * transpose(rectMax);
aggregateRectPoints = [transpose(rectMin(1:3)), transpose(rectMax(1:3))];

% get straight line L3 and L4. L3 is defined in frame 3 and L4 is defined
% in frame 4
L3_endPoint = [-L3, 0, 0];
L3_startPoint = [0, 0, 0];
L4_endPoint = [0, 0, L4 + L5];
L4_startPoint = [0, 0, 0];

% check collision
isCollided_self = detectCollision(L3_endPoint, L3_startPoint, aggregateRectPoints) | detectCollision(L4_endPoint, L4_startPoint, aggregateRectPoints);

% -----end------


isCollided = isCollided_boundary_oneBol | isCollided_obstacle_oneBol | isCollided_self;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

