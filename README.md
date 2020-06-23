# GridLocalizationBayes

Localized a mobile robot through Bayesian filter using observation from LIDAR sensor and robot's actions
Odometry motion model for a differential drive robot is considered. Therefore, robot actions consist of initial heading, translation, and final heading

# How to run

1. Install ROS (if not already done)
2. Create a ROS catkin workspace if not already done: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
3. Create catkin ros package in the ROS catkin workspace (cmd: catkin_create_pkg ros_pa3 std_msgs rospy roscpp)
4. Download and unzip the repository in the ROS workspace
5. update the workspace (like for python3, command: $ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3)
6. Source the bash file of the workspace
7. make the files executable if required (cmd: chmod +x "file name")
8. Run "pa3.launch" file
