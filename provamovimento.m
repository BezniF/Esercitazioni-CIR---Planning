% folfderpath= "~/ned_ros"
% rosgenmsg(folfderpath)
rosshutdown

ipaddress = "http://169.254.200.200:11311";
setenv('ROS_MASTER_URI',ipaddress)
setenv('ROS_IP','169.254.200.201')
rosinit(ipaddress)



% client = rossvcclient('/niryo_robot/kinematics/inverse');
