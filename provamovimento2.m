rosshutdown

ipaddress = "http://169.254.200.200:11311";
setenv('ROS_MASTER_URI',ipaddress)
setenv('ROS_IP','169.254.200.201')

rosinit(ipaddress)

% Ned.set_learning_mode(false)

client = rossvcclient('/niryo_robot/kinematics/inverse');
ReqMsg = rosmessage(client);

ReqMsg.Pose.Position.X = 0.256000000000000;
ReqMsg.Pose.Position.Y = 0.243000000000000;
ReqMsg.Pose.Position.Z = 0.434000000000000;

ReqMsg.Pose.Orientation.X = 0;
ReqMsg.Pose.Orientation.Y = 0;
ReqMsg.Pose.Orientation.Z = 0.479000000000000;
ReqMsg.Pose.Orientation.W = 0.878000000000000;

IK = call(client, ReqMsg);

NedState = rossubscriber("/niryo_robot_follow_joint_trajectory_controller/state");

NedCmd = rospublisher("/niryo_robot_follow_joint_trajectory_controller/command");
CmdMsg = rosmessage(NedCmd);

CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
CmdPoint.Positions = IK.Joints; %We get the Joints goal from the IK service
CmdPoint.Velocities = zeros(1,6);
CmdPoint.Accelerations = zeros(1,6);
CmdPoint.Effort = 0.0;
CmdPoint.TimeFromStart = ros.msg.Duration(5);


CmdMsg.JointNames = {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'};
CmdMsg.Points = CmdPoint;

send(NedCmd,CmdMsg);