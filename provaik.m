Ned = NiryoRosWrapper();
%% 
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
% Ned.grip(2)
Ned.pick_from_pose(0.373800000000000,- 0.033100000000000,0.066200000000000)
Ned.place_from_pose(0.225200000000000,  0.267700000000000,0.102200000000000)
Ned.pick_and_place(0.373800000000000,- 0.033100000000000,0.066200000000000,0.225200000000000,  0.260000000000000,0.100000000000000)