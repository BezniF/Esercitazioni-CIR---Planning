close all
clear
clc

%% Init. robot connection
Ned = NiryoRosWrapper();

%% Set initial position and calibrate
ReqMsg = rosmessage('niryo_robot_msgs/RobotState');
ReqMsg.Position.X = 0.256000000000000;
ReqMsg.Position.Y = 0.243000000000000;
ReqMsg.Position.Z = 0.434000000000000;

% Ned.set_learning_mode(false)

ReqMsg.Orientation.X = 0;
ReqMsg.Orientation.Y = 0;
ReqMsg.Orientation.Z = 0.479000000000000;
ReqMsg.Orientation.W = 0.878000000000000;
IK = Ned.getIK(ReqMsg)
Ned.move_state(ReqMsg, 5)
Ned.get_state()

Ned.request_new_calibration()
Ned.auto_calibration(1)
Ned.set_learning_mode(false)
Ned.get_id()
Ned.get_learning_mode()
%%
% ReqMsg = rosmessage('niryo_robot_msgs/RobotState');
% ReqMsg.Position.X = 0.256000000000000;
% ReqMsg.Position.Y = 0.443000000000000;
% ReqMsg.Position.Z = 0.4000000000000;
% 
% ReqMsg.Orientation.X = 0;
% ReqMsg.Orientation.Y = 0;
% ReqMsg.Orientation.Z = -0.6000000000000;
% ReqMsg.Orientation.W = 0.878000000000000;
% IK = Ned.getIK(ReqMsg)
% Ned.move_state(ReqMsg, 5)
% Ned.get_state()
% Ned.set_learning_mode(false)
% Ned.get_id()

%% Initial Joint position
startConfiguration = [0.7973, -0.8287, 0.2278, 0.0047, -1.0355, 0.0031]';
Ned.move_joints(startConfiguration, 4)

%% Final Joint position
goalConfiguration = [-0.7078, -0.8297, 0.2263, -0.0030, -1.0187, -0.1043]';
Ned.move_joints(goalConfiguration, 4)

%% Back to initial position (no RRT -> collision)
startConfiguration = [0.7973, -0.8287, 0.2278, 0.0047, -1.0355, 0.0031]';
Ned.move_joints(startConfiguration, 4)

%% Set Environment
% Obstacle pose
obs_pose = [0.3795, 0.0083, 0.16]';

obstacle = collisionSphere(0.1); % A sphere of radius 0.2
obstaclePose = eye(4);
obstaclePose(1:3,4) = obs_pose;
obstacle.Pose = obstaclePose;

% Floor (avoid smashing to the ground)
floor = collisionBox(0.5, 0.5, 0.05); % A box of 0.5X0.5X0.05
floorPose = eye(4);
floorPose(3,4) = - 0.05;
floor.Pose = floorPose;

% Environment
environment = {floor, obstacle};

%% Load robot model
robot = importrobot("ned.urdf");
robot.DataFormat = "column";

clearCollision(robot.Bodies{1, 5})

% show(robot, startConfiguration, 'Collisions', 'off', 'Visuals','on')

% check for self collision
[isColliding, sepDist] = checkCollision(robot, startConfiguration, "Exhaustive", "on");
[b1, b2] = find(isnan(sepDist));
robot.BodyNames{b1};
robot.BodyNames{b2};
%% Invoke RRT
rrt = manipulatorRRT(robot, environment);
rrt.MaxConnectionDistance = 0.2; %Maximum length between planned configurations
rrt.EnableConnectHeuristic = false;

rng('shuffle');
[path, info] = plan(rrt, startConfiguration.', goalConfiguration.');

%% Visualize in Sim
show(robot,startConfiguration);
hold on
show(robot,goalConfiguration);
for i=1:size(path,1)
    robotPose = robot.getTransform(path(i,:).','tool_link');
    scatter3(robotPose(1,4), robotPose(2,4), robotPose(3,4),'r');
end
hold on

zlim([-0.05 1.2]) %% Copy-paste from above
[~, obs_patch] = show(obstacle); 
obs_patch.FaceColor = [0 1 1];
obs_patch.EdgeColor = 'none';
[~, floor_patch] = show(floor);
floor_patch.FaceColor = [0.5 0.1 0.1];

%% Follow the unsmoothed joint trajectory from RRT
motionPeriod = 10.0;

num_waypoints  = size(path, 1);
i = 1;
duration = motionPeriod / num_waypoints;

while i < num_waypoints +1
    q_des = path(i, :)';
    Ned.move_joints(q_des, duration)
    i = i + 1;
end

%% Snab back to reality
rrt = manipulatorRRT(robot, environment);
rrt.MaxConnectionDistance = 0.2; %Maximum length between planned configurations
rrt.EnableConnectHeuristic = false;

rng('shuffle');
[path, info] = plan(rrt, goalConfiguration.', startConfiguration.');

%% Follow the unsmoothed joint trajectory from RRT
num_waypoints  = size(path, 1);
i = 1;
duration = motionPeriod / num_waypoints;

while i < num_waypoints +1
    q_des = path(i, :)';
    Ned.move_joints(q_des, duration)
    i = i + 1;
end

%% Smooth the trajectory - go with RRT again
rrt = manipulatorRRT(robot, environment);
rrt.MaxConnectionDistance = 0.2; %Maximum length between planned configurations
rrt.EnableConnectHeuristic = false;

rng('shuffle');
[path, info] = plan(rrt, startConfiguration.', goalConfiguration.');

%% Then smoothen it out
trajSplinePos = cscvn(path.'); %p(s)
trajSplineVel = fnder(trajSplinePos); %dp(s)
trajSplineAcc = fnder(trajSplineVel); %ddp(s)

trajLength = trajSplinePos.breaks(end);

trajPointNumber = 1000;
trajTime = linspace(0, motionPeriod, trajPointNumber);
[st, dst, ddst, pp] = cubicpolytraj([0, trajLength], [0, motionPeriod], trajTime);

% x(t) = p(s(t))
trajPos = (ppval(trajSplinePos, st)).'; 

% dx(t) = dp(s(t)) * ds(t)
trajVel = (ppval(trajSplineVel, st) .* dst).';

% ddx(t) = ddp(s(t)) * ds(t)^2 + dp(s(t)) * dds(t)
trajAcc = (ppval(trajSplineAcc, st) .* dst .^ 2 + ppval(trajSplineVel, st) .* ddst).';

%% Follow the smoothed trajectory
Ned.move_joints_traj(trajPos, trajTime, trajVel, trajAcc)

%% Initial Joint position
startConfiguration = [0.7973, -0.8287, 0.2278, 0.0047, -1.0355, 0.0031]';
Ned.move_joints(startConfiguration, 4)

%% Get & Print Joint Position
q_msg = Ned.get_joints()
q_pos_act = q_msg.Position

%% Get & Print Cartesian Position
x_msg = Ned.get_state()
x_pos_act = x_msg.Position

%% Hand Guiding
Ned.set_learning_mode(true)

%% Autonomous
Ned.set_learning_mode(false)