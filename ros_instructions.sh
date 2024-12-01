# TERMINAL 1 (ROS 1 Core):
sr1
roscore

# TERMINAL 2 (ROS 1 - Kinect Bridge):
sr1
srk
roslaunch kinect2_bridge kinect2_bridge.launch depth_method:=opengl publish_tf:=true

# TERMINAL 3 (ROS 1 - ROS 2 Bridge):
sr1
sr2
srk
export ROS_MASTER_URI=http://localhost:11311
rosparam load ~/ROSHome/bridge.yaml
ros2 run ros1_bridge parameter_bridge

# TERMINAL 4 (Kinect Gesture and Voice Command Recognition):
sr2
srws
cd ~/ROSHome/ros2_ws
colcon build --packages-select kinect_pose
ros2 run kinect_pose listener

# TERMINAL 5 (Speech to Text):
sr2
srws
cd ~/ROSHome/ros2_ws
colcon build --packages-select speech_to_text
ros2 run speech_to_text listener

# TERMINAL 6 (Voice Command Web Server):
cd ~/ROSHome/ros2_ws/web_app/backend
python3 web_server.py

# TERMINAL 7 (Voice Command Web App):
cd ~/ROSHome/ros2_ws/web_app/frontend
tsc
live-server