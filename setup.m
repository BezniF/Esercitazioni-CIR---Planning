%% Re-build custom ROS msgs for Niryo
folderpath = "/home/federico/catkin_ws/src/ned_ros";
rosgenmsg(folderpath)
addpath('/home/federico/catkin_ws/src/ned_ros/matlab_msg_gen_ros1/glnxa64/install/m');
savepath('/home/federico/catkin_ws/src/ned_ros/matlab_msg_gen_ros1/glnxa64/install/m')
% clear classes
% rehash toolboxcache
rosmsg list