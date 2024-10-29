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

# TERMINAL 4:
sr1
# ROS1 PROGRAMS
rviz # Open the kinect.rviz config file

# TERMINAL 4:
# sr2
# ROS2 PROGRAMS
# rviz2 # Open the kinect2.rviz config file
# srws
# colcon build the package first, ensuring in correct directory
# ros2 run package executable_name
sr2
srws
cd ~/ROSHome/ros2_ws
colcon build --packages-select kinect_pose
ros2 run kinect_pose listener

# TERMINAL 5:
mosquitto_sub -v -h localhost -p 1883 -t '#'

# TERMINAL 6:
sr2
srws
cd ~/ROSHome/ros2_ws
colcon build --packages-select speech_to_text
ros2 run speech_to_text listener

# TERMINAL 7:
cd ~/ROSHome/ros2_ws/web_app/backend
python3 web_server.py

# TERMINAL 8:
cd ~/ROSHome/ros2_ws/web_app/frontend
tsc
live-server