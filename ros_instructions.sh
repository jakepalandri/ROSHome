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
source /opt/ros/noetic/setup.bash
# ROS1 PROGRAMS
rviz # Set "Fixed Frame" to kinect2_link in Global Options

# TERMINAL 4:
source /opt/ros/foxy/setup.bash
# ROS2 PROGRAMS

