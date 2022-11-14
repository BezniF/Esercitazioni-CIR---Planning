%folfderpath= "~/ned_ros"
%rosgenmsg(folfderpath)

rosshutdown
setenv('ROS_MASTER_URI','http://169.254.200.200:11311')
setenv('ROS_IP','169.254.200.250')
ipaddress = "http://169.254.200.200:11311";
rosinit(ipaddress)

[actClient,goalMsg] = rosactionclient('/niryo_robot_tools_commander/action_server','DataFormat','struct');
waitForServer(actClient);


rosshutdown