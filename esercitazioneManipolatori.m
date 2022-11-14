%% Init
clear();
clc();
close all;

startConfiguration = [-pi/4; pi/5; 0.0; -pi/8; 0.0; pi/2; 0.0];
goalConfiguration =  [pi/4; pi/5; 0.0; -pi/8; 0.0; pi/2; 0.0];
motionPeriod = 10;

%Import the robot
robot = loadrobot("kukaIiwa14");
robot.DataFormat = "column";
robot.Gravity = [0; 0; -9.81]; %% Gravity

%% Create environment
%Floor
floor = collisionBox(0.5, 0.5, 0.05); %A box of 0.5X0.5X0.05
floorPose = eye(4);
floorPose(3,4) = - 0.05;
floor.Pose = floorPose;

%Obstacle
obstacle = collisionSphere(0.3); %A sphere of radius 0.3
obstaclePose = eye(4);
obstaclePose(1:3,4) = [0.6; 0.0; 0.8];
obstacle.Pose = obstaclePose;

%Environment
environment = {floor, obstacle};

show(robot,startConfiguration, 'Frames', 'off');
hold on
zlim([-0.05 1.2])
[~, obs_patch] = show(obstacle);
obs_patch.FaceColor = [0 1 1];
obs_patch.EdgeColor = 'none';
[~, floor_patch] = show(floor);
floor_patch.FaceColor = [0.5 0.1 0.1];
        
%% 
show(robot,goalConfiguration, 'Frames', 'off');
    
%% Planning
close all;

rrt = manipulatorRRT(robot, environment);
rrt.MaxConnectionDistance = 0.2; %Maximum length between planned configurations
rrt.EnableConnectHeuristic = false;

rng('shuffle');
[path, info] = plan(rrt, startConfiguration.', goalConfiguration.');

%Joint viapoints
for i=1:7
    subplot(7,1,i)
    plot(path(:,i),'o');
    xlabel("Index [ad]");
    ylabel("Position [rad]");
    title(strcat("Joint ",num2str(i)));
end
set(gcf, 'Position', get(0, 'Screensize'));
%% 
close all;

%Cartesian viapoints
show(robot,startConfiguration);
hold on
show(robot,goalConfiguration);
for i=1:size(path,1)
    robotPose = robot.getTransform(path(i,:).','iiwa_link_ee_kuka');
    scatter3(robotPose(1,4), robotPose(2,4), robotPose(3,4),'r');
end
hold on

zlim([-0.05 1.2]) %% Copy-paste from above
[~, obs_patch] = show(obstacle); 
obs_patch.FaceColor = [0 1 1];
obs_patch.EdgeColor = 'none';
[~, floor_patch] = show(floor);
floor_patch.FaceColor = [0.5 0.1 0.1];

%% Trajectory
close all;

trajSplinePos = cscvn(path.'); %p(s)
trajSplineVel = fnder(trajSplinePos); %dp(s)
trajSplineAcc = fnder(trajSplineVel); %ddp(s)

trajLength = trajSplinePos.breaks(end);

trajPointNumber = 1000;
trajTime = linspace(0, motionPeriod, trajPointNumber);
[st, dst, ddst, pp] = cubicpolytraj([0, trajLength], [0, motionPeriod], trajTime);

plot(trajTime,st);
xlabel("Time [s]");
ylabel("Normalized curve length [ad]");
pause();
close all;

% x(t) = p(s(t))
trajPos = (ppval(trajSplinePos, st)).'; 

% dx(t) = dp(s(t)) * ds(t)
trajVel = (ppval(trajSplineVel, st) .* dst).';

% ddx(t) = ddp(s(t)) * ds(t)^2 + dp(s(t)) * dds(t)
trajAcc = (ppval(trajSplineAcc, st) .* dst .^ 2 + ppval(trajSplineVel, st) .* ddst).'; 

figure
for i=1:7
    subplot(7,1,i)
    plot(trajTime, trajPos(:,i));
    xlabel("Time [s]")
    ylabel("Position [rad]")
    title(strcat("Joint ",num2str(i)));
end
set(gcf, 'Position', get(0, 'Screensize'));
figure
for i=1:7
    subplot(7,1,i)
    plot(trajTime, trajVel(:,i));
    xlabel("Time [s]")
    ylabel("Velocity [rad/s]")
    title(strcat("Joint ",num2str(i)));
end
set(gcf, 'Position', get(0, 'Screensize'));
figure
for i=1:7
    subplot(7,1,i)
    plot(trajTime, trajAcc(:,i));
    xlabel("Time [s]")
    ylabel("Accel. [rad/s2]")
    title(strcat("Joint ",num2str(i)));
end
set(gcf, 'Position', get(0, 'Screensize'));
%%
close all;

show(robot,startConfiguration);
hold on
show(robot,goalConfiguration);
zlim([-0.05 1.2]) %% Copy-paste from above
[~, obs_patch] = show(obstacle); 
obs_patch.FaceColor = [0 1 1];
obs_patch.EdgeColor = 'none';
[~, floor_patch] = show(floor);
floor_patch.FaceColor = [0.5 0.1 0.1];
for i=1:size(path,1)
    robotPose = robot.getTransform(path(i,:).','iiwa_link_ee_kuka');
    scatter3(robotPose(1,4), robotPose(2,4), robotPose(3,4),'r');
end
for i=1:10:length(trajTime)
    robotPose = robot.getTransform(trajPos(i,:).','iiwa_link_ee_kuka');
    scatter3(robotPose(1,4), robotPose(2,4), robotPose(3,4),10,'b');
end

%% Controller stuff (don't think I have to do any of this)
% %% Controller simulation
% close all;
% 
% %%Parameters
% simulationStepTime = 0.004;
% videoExport = 0;
% pdfExport = 0;
% 
% %%%Controller parameters
% Kp = diag([500 500 500 500 500 500 500]);
% Kd = diag([100 100 100 100 100 100 100]);
% 
% %%%Simulation init
% t = 0;
% q = startConfiguration;
% dq = zeros(7,1);
% 
% %%%Simulation data history
% qHistory = q.';
% dqHistory = dq.';
% tHistory = t;
% qdHistory = [];
% dqdHistory = [];
% ddqdHistory = [];
% 
% 
% while(t < motionPeriod)
%     t
% 
%     %%Desired trajectory
%     qd = interp1(trajTime, trajPos, t).';
%     dqd = interp1(trajTime, trajVel, t).';
%     ddqd = interp1(trajTime, trajAcc, t).';
%     
%     %%Controller
%     M = robot.massMatrix(q);
%     C_dot_dq = robot.velocityProduct(q,dq);
%     n = C_dot_dq + robot.gravityTorque(q);
%     u = n + M * (ddqd + Kd * (dqd - dq) + Kp * (qd - q));
%     
%     %%Simulation update
%     ddq = robot.forwardDynamics(q, dq, u);
%     dq = dq + ddq * simulationStepTime;
%     q = q + dq * simulationStepTime;
%     t = t + simulationStepTime;
% 
%     %%Simulation history
%     qHistory = [qHistory; q.'];
%     dqHistory = [dqHistory; dq.'];
%     tHistory = [tHistory, t];
% 
%     qdHistory = [qdHistory; qd.'];
%     dqdHistory = [dqdHistory; dqd.'];
%     ddqdHistory = [ddqdHistory; ddqd.'];
% 
% end
% 
% %% Simulation video
% figure;
% qHistory30Fps = interp1(tHistory, qHistory, 0:1/30:tHistory(end)).';
% show(robot,startConfiguration);
% hold on
% for i=1:length(environment)
%      show(environment{i})
% end
% for i=1:size(path,1)
%     robotPose = robot.getTransform(path(i,:).','iiwa_link_ee_kuka');
%     scatter3(robotPose(1,4), robotPose(2,4), robotPose(3,4),'r');
% end
% for i=1:1:length(qHistory30Fps)
%     robotPose = robot.getTransform(qHistory30Fps(:,i),'iiwa_link_ee_kuka');
%     scatter3(robotPose(1,4), robotPose(2,4), robotPose(3,4),10,'b');
%     show(robot, qHistory30Fps(:,i),'PreservePlot',false); %display robot with joint position at i
%     axis([-1.5,1.5,-1.5,1.5,-1.5,1.5])
%     F(i) = getframe(gcf); % store the displayed frame for offline reproduction
%     i / length(qHistory30Fps) * 100 % print video percentage
% end
% 
% videoExport = 0;
% if videoExport
%     v = VideoWriter('simulation_video','MPEG-4');
%     open(v);
%     for i=2:length(F)
%         writeVideo(v,F(i));
%     end
%     close(v);
% end
% 
% %% Joint evolution
% jfig = figure;
% for i=1:7
%     subplot(7,1,i)
%     plot(tHistory, qHistory(:,i),'LineWidth',2); 
%     hold on
%     plot(tHistory(1:25:end), qdHistory(1:25:end, i),'o'); 
%     grid on
%     xlabel("Time [s]");
%     ylabel("Position [rad]");
%     title(strcat("Joint ",num2str(i)));
%     legend('Real','Desired','Location','EastOutside');
%     xlim([0,5]);
% end
% 
% pdfExport = 0;
% if pdfExport
%     print(jfig,'joint','-dpdf','-fillpage');
% end