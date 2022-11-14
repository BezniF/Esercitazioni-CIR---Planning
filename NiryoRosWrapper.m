classdef NiryoRosWrapper

    properties
        joint_states
        RobotState
        IKclient
        subJointState
        NedJointCmdPub
        learning_state
        stateclient
        calibration
        new_calibration        
        learning_mode_client
        get_id
        state
    end

    methods
        function obj = NiryoRosWrapper()
            rosshutdown
            setenv('ROS_MASTER_URI','http://169.254.200.200:11311')
            setenv('ROS_IP','169.254.200.201')
            ipaddress = "http://169.254.200.200:11311";
            rosinit(ipaddress)
            obj.NedJointCmdPub = rospublisher("/niryo_robot_follow_joint_trajectory_controller/command");
            obj.subJointState = rossubscriber("/joint_states");
            obj.IKclient = rossvcclient('/niryo_robot/kinematics/inverse');
            obj.learning_state = rossubscriber("/niryo_robot/learning_mode/state");
            obj.calibration = rossvcclient("/niryo_robot/joints_interface/calibrate_motors");
            obj.new_calibration = rossvcclient("/niryo_robot/joints_interface/request_new_calibration");            
            obj.learning_mode_client = rossvcclient("/niryo_robot/learning_mode/activate");
            obj.get_id = rossubscriber("/niryo_robot_tools_commander/current_id");
            obj.state = rossubscriber("/niryo_robot/robot_state");

        end

        function joints = get_joints(obj)
            joints = obj.subJointState.LatestMessage;
        end

        function IK = getIK(obj,Pose)
            ReqMsg = rosmessage(obj.IKclient);
            ReqMsg.Pose = Pose;
            IK = call(obj.IKclient, ReqMsg);
        end

        function move_joints(obj,joint_des,duration)
            CmdMsg = rosmessage(obj.NedJointCmdPub);

            CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            CmdPoint.Positions = joint_des'; %We get the Joints goal from the IK service
            CmdPoint.Velocities = zeros(1,6);
            CmdPoint.Accelerations = zeros(1,6);
            CmdPoint.Effort = 0.0;
            CmdPoint.TimeFromStart = ros.msg.Duration(duration);

            CmdMsg.JointNames = {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'};
            CmdMsg.Points = CmdPoint;

            send(obj.NedJointCmdPub,CmdMsg);
            pause(duration);

        end
        
        function move_joints_traj(obj,joint_traj, duration, joint_vel, joint_acc)
            if nargin < 5
                joint_acc = zeros(size(joint_traj));
            end
            if nargin < 4
                joint_vel = zeros(size(joint_traj));
            end
            CmdMsg = rosmessage(obj.NedJointCmdPub);
            
            CmdMsg.JointNames = {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'};
            
            for i = 1:length(joint_traj)
                CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                CmdPoint.Positions = joint_traj(i,:); %We get the Joints goal from the IK service
                CmdPoint.Velocities = joint_vel(i,:);
                CmdPoint.Accelerations = joint_acc(i,:);
                CmdPoint.Effort = zeros(size(joint_traj(i,:)));
                CmdPoint.TimeFromStart = ros.msg.Duration(duration(i));
                CmdMsg.Points = [CmdMsg.Points; CmdPoint];
            end
            
            send(obj.NedJointCmdPub,CmdMsg);
           %pause(duration);

        end

        function move_state(obj,target_state,duration)
            IK = obj.getIK(target_state);
            obj.move_joints(IK.Joints',duration)
        end

        function state=get_learning_mode(obj)
            state = obj.learning_state.LatestMessage;
        end

        function result = set_learning_mode(obj,set_bool)
            LearnMsg = rosmessage(obj.learning_mode_client);
            LearnMsg.Value = set_bool;             
            res = call(obj.learning_mode_client,LearnMsg);                                
            result = res.Status;
        end        

        function result=request_new_calibration(obj)
            RequestMsg=rosmessage(obj.new_calibration);
            res=call(obj.new_calibration,RequestMsg);
            result=res.Status;        
            
        end
        
        function result = auto_calibration(obj,int)
            CalibMsg = rosmessage(obj.calibration);
            CalibMsg.Value = int;
            res = call(obj.calibration,CalibMsg);
            result = res.Status;

        end
        
        function int=setInt(obj,type)
            IntMsg=rosmessage(obj.calibration);
            IntMsg.type=type;
            int = call(obj.calibration,IntMsg);
        end       

        function result=get_current_tool_id(obj) 
            result=obj.get_id.LatestMessage;
        end

        function grip(obj,int)
            [actClient,goalMsg]=rosactionclient('/niryo_robot_tools_commander/action_server');
            waitForServer(actClient);
            tjpoint1=rosmessage('niryo_robot_tools_commander/ToolCommand');
            tjpoint1.ToolId=11;
            tjpoint1.Speed=500;
            tjpoint1.MaxTorquePercentage=100;
            if(int==1)                
                tjpoint1.HoldTorquePercentage=20;
                tjpoint1.CmdType=1;   
            else               
                tjpoint1.HoldTorquePercentage=50;
                tjpoint1.CmdType=2;
            end
            goalMsg.Cmd=[tjpoint1];
            sendGoalAndWait(actClient,goalMsg);            
                   
        end

        function pick_from_pose(obj,x,y,z)            
                                    
            ReqMsg = rosmessage('niryo_robot_msgs/RobotState');
            ReqMsg.Position.X=x;
            ReqMsg.Position.Y=y;
            ReqMsg.Position.Z=z+0.1;            

            ReqMsg.Orientation.X = -0.007100000000000;
            ReqMsg.Orientation.Y = -0.724600000000000;
            ReqMsg.Orientation.Z =  0.007100000000000;
            ReqMsg.Orientation.W = -0.689100000000000; 
            
            obj.move_state(ReqMsg,3);
            
%             obj.grip(1);
            
            ReqMsg1 = rosmessage('niryo_robot_msgs/RobotState');
            ReqMsg1.Position.X=x;
            ReqMsg1.Position.Y=y;
            ReqMsg1.Position.Z=z;
           
            ReqMsg1.Orientation.X = -0.007100000000000;
            ReqMsg1.Orientation.Y = -0.724600000000000;
            ReqMsg1.Orientation.Z =  0.007100000000000;
            ReqMsg1.Orientation.W = -0.689100000000000;  
            
            obj.move_state(ReqMsg1,2);
%             obj.grip(2);

            obj.move_state(ReqMsg,2);                                 

        end

        function place_from_pose(obj,x,y,z)
%             obj.grip(2);
           
            ReqMsg = rosmessage('niryo_robot_msgs/RobotState');
            ReqMsg.Position.X=x;
            ReqMsg.Position.Y=y;
            ReqMsg.Position.Z=z+0.1;            
            
            ReqMsg.Orientation.X = -0.007100000000000;
            ReqMsg.Orientation.Y = -0.724600000000000;
            ReqMsg.Orientation.Z =  0.007100000000000;
            ReqMsg.Orientation.W = -0.689100000000000; 
            obj.move_state(ReqMsg,3);

            ReqMsg1 = rosmessage('niryo_robot_msgs/RobotState');
            ReqMsg1.Position.X=x;
            ReqMsg1.Position.Y=y;
            ReqMsg1.Position.Z=z;
            
            ReqMsg1.Orientation.X = -0.007100000000000;
            ReqMsg1.Orientation.Y = -0.724600000000000;
            ReqMsg1.Orientation.Z =  0.007100000000000;
            ReqMsg1.Orientation.W = -0.689100000000000; 
            
            obj.move_state(ReqMsg1,2);
%             obj.grip(1);
            obj.move_state(ReqMsg,2); 

        end

        function pick_and_place(obj,x1,y1,z1,x2,y2,z2)
            obj.pick_from_pose(x1,y1,z1);
            obj.place_from_pose(x2,y2,z2);

        end

        function pose_state = get_state(obj)
            pose_state = obj.state.LatestMessage();

        end

        
    end
end
