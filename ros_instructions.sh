# TERMINAL 1:
sr1
roscore

# TERMINAL 2:
sr1
srk
roslaunch kinect2_bridge kinect2_bridge.launch depth_method:=opengl reg_method:=cpu publish_tf:=true

# TERMINAL 3:
sr1
sr2
srk
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

# TERMINAL 4:
sr1
# ROS1 PROGRAMS
rviz # Open the kinect.rviz config file

# TERMINAL 4:
sr2
# ROS2 PROGRAMS
source ~/rviz2_ws/src/install/setup.bash
rviz2 # Open the kinect2.rviz config file
