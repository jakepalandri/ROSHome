# TERMINAL 1:
sr1
roscore

# TERMINAL 2:
sr1
srk
roslaunch kinect2_bridge kinect2_bridge.launch depth_method:=opengl publish_tf:=true

# TERMINAL 3:
sr1
sr2
srk
export ROS_MASTER_URI=http://localhost:11311
rosparam load ~/ROSHome/bridge.yaml
ros2 run ros1_bridge parameter_bridge

# TERMINAL 4:
srk
# Kinect Viewer
rosrun kinect2_viewer kinect2_viewer sd cloud # requires kinect2_bridge to be running

# TERMINAL 5:
sr1
# ROS1 PROGRAMS
rviz # Open the kinect.rviz config file

# TERMINAL 6:
sr2
# ROS2 PROGRAMS
rviz2 # Open the kinect2.rviz config file
source ~/rviz2_ws/src/install/setup.bash
source ~/ROSHome/ros2_ws/install/setup.bash 
ros2 run kinect_ascii listener # ros2 run package executable_name
