# Install git and download this repository to your machine
sudo apt -y install git
git clone git@github.com:jakepalandri/ROSHome.git

# ROS 1 INSTALLATION
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt -y install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt -y install ros-noetic-desktop-full # this may need a second attempt if it errors
sudo apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo rosdep init
rosdep update

# ROS1 Environment Configuring
alias sr1='source /opt/ros/noetic/setup.bash'
echo "alias sr1='source /opt/ros/noetic/setup.bash'" >> ~/.bashrc
sr1
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo "alias srk='source ~/catkin_ws/devel/setup.bash'" >> ~/.bashrc

# verify installation
roscore
# Ctrl+C to exit

# Expected output:

# ... logging to /home/jake/.ros/log/f02fa19c-abff-11ef-b9f6-552c6127c39a/roslaunch-jake-XPS-15-9510-37823.log
# Checking log directory for disk usage. This may take a while.
# Press Ctrl-C to interrupt
# Done checking log file disk usage. Usage is <1GB.

# started roslaunch server http://jake-XPS-15-9510:34723/ ######################### THIS IS IN BOLD
# ros_comm version 1.17.0


# SUMMARY
# ========

# PARAMETERS
#  * /rosdistro: noetic
#  * /rosversion: 1.17.0

# NODES

# auto-starting new master
# process[master]: started with pid [37833] ######################### THIS IS IN BOLD
# ROS_MASTER_URI=http://jake-XPS-15-9510:11311/ ######################### THIS IS IN BOLD

# setting /run_id to f02fa19c-abff-11ef-b9f6-552c6127c39a ######################### THIS IS IN BOLD
# process[rosout-1]: started with pid [37843] ######################### THIS IS IN BOLD
# started core service [/rosout]

# CUDA INSTALLATION
cd ~
sudo apt update
sudo apt -y install nvidia-cuda-toolkit 

# https://github.com/OpenKinect/libfreenect2/issues/1196
# remove previous gcc-9 and g++-9 by :
sudo update-alternatives --remove gcc /usr/bin/gcc-9

# you need to install gcc-7 and g++-7 :
sudo apt-get -y install gcc-7 g++-7
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 50
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 50
sudo update-alternatives --config gcc
sudo update-alternatives --config g++

wget http://developer.download.nvidia.com/compute/cuda/10.1/Prod/local_installers/cuda_10.1.243_418.87.00_linux.run
sudo sh cuda_10.1.243_418.87.00_linux.run
# Type: "accept" to accept the EULA

# Uncheck
# - Driver
# - CUDA Toolkit 10.1
# - CUDA Demo Suite 10.1
# - CUDA Documentation 10.1
#
# Only leave checked
# - CUDA Samples 10.1

# Select install

# verify installation
nvcc --version

# Expected output:

# nvcc: NVIDIA (R) Cuda compiler driver
# Copyright (c) 2005-2019 NVIDIA Corporation
# Built on Sun_Jul_28_19:07:16_PDT_2019
# Cuda compilation tools, release 10.1, V10.1.243

# remove used run file
rm cuda_10.1.243_418.87.00_linux.run

# Install updated drivers 
sudo apt-get -y install linux-headers-$(uname -r)
sudo apt-key del 7fa2af80

wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb

sudo apt-get -y install nvidia-driver-550-open
sudo apt-get -y install cuda-drivers-550

echo "export LD_LIBRARY_PATH='/usr/local/cuda/lib64:${LD_LIBRARY_PATH}'" >> ~/.bashrc
echo "export PATH='/usr/local/cuda/bin:${PATH}'" >> ~/.bashrc

export CPATH=$CPATH:$HOME/NVIDIA_CUDA-10.1_Samples/common/inc
echo "export CPATH=$CPATH:$HOME/NVIDIA_CUDA-10.1_Samples/common/inc" >> ~/.bashrc

# verify keyring installation
dpkg -l | grep cuda-keyring

# Expected output:
#    |THIS IS RED |
# ii  cuda-keyring                                    1.1-1                                 all          GPG keyring for the CUDA repository

# remove used installer
rm cuda-keyring_1.1-1_all.deb

# LIBFREENECT2 INSTALLATION WITH CHANGE IN CMAKE FOR KINECT2 BRIDGE
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2

sudo apt-get -y install build-essential cmake pkg-config
sudo apt-get -y install libusb-1.0-0-dev
sudo apt-get -y install libturbojpeg0-dev # skip?
sudo apt-get -y install libglfw3-dev
sudo apt-get -y install beignet-dev
sudo apt-get -y install libva-dev libjpeg-dev # do instead?
sudo apt-get -y install libopenni2-dev

git apply ~/ROSHome/libfreenect2_changes.patch

mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -DENABLE_CXX11=ON -DENABLE_CUDA=ON
make
make install

cd ..
cmake -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2
sudo cp platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

# UNPLUG AND REPLUG IN YOUR KINECT

export LIBVA_DRIVER_NAME=i965
echo "export LIBVA_DRIVER_NAME=i965" >> ~/.bashrc

# verify it worked
./build/bin/Protonect

# 4 camera feeds should pop up in a separate window
# return to the terminal and Ctrl+C to close these

# IAI_KINECT2_OPENCV4
cd ~/catkin_ws/src/
git clone https://github.com/paul-shuvo/iai_kinect2_opencv4.git
cd iai_kinect2_opencv4
rosdep install -r --from-paths .
cd ~/catkin_ws
sudo bash -c 'echo -e "set( CMAKE_CXX_STANDARD 14)\n$(cat src/CMakeLists.txt)" > src/CMakeLists.txt'
catkin_make -DCMAKE_BUILD_TYPE="Release"

sudo apt-get -y install nvidia-modprobe opencl-headers
echo "/usr/local/cuda/lib64" | sudo tee /etc/ld.so.conf.d/cuda.conf
sudo ldconfig

# we will verify this worked when starting the ROS bridge as described in usage (not setup) instructions, skip to there if you want to check now

# ROS2 INSTALLATION
locale  # check for UTF-8

# Expected output:

# LANG=en_AU.UTF-8
# LANGUAGE=en_AU:en
# LC_CTYPE="en_AU.UTF-8"
# LC_NUMERIC="en_AU.UTF-8"
# LC_TIME="en_AU.UTF-8"
# LC_COLLATE="en_AU.UTF-8"
# LC_MONETARY="en_AU.UTF-8"
# LC_MESSAGES="en_AU.UTF-8"
# LC_PAPER="en_AU.UTF-8"
# LC_NAME="en_AU.UTF-8"
# LC_ADDRESS="en_AU.UTF-8"
# LC_TELEPHONE="en_AU.UTF-8"
# LC_MEASUREMENT="en_AU.UTF-8"
# LC_IDENTIFICATION="en_AU.UTF-8"
# LC_ALL=

# if not, do this - any UTF8 should be fine
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# verify settings
locale

# Expected Output:

# LANG=en_US.UTF-8
# LANGUAGE=en_AU:en
# LC_CTYPE="en_US.UTF-8"
# LC_NUMERIC="en_US.UTF-8"
# LC_TIME="en_US.UTF-8"
# LC_COLLATE="en_US.UTF-8"
# LC_MONETARY="en_US.UTF-8"
# LC_MESSAGES="en_US.UTF-8"
# LC_PAPER="en_US.UTF-8"
# LC_NAME="en_US.UTF-8"
# LC_ADDRESS="en_US.UTF-8"
# LC_TELEPHONE="en_US.UTF-8"
# LC_MEASUREMENT="en_US.UTF-8"
# LC_IDENTIFICATION="en_US.UTF-8"
# LC_ALL=

sudo apt -y install software-properties-common
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# CHOOSE ONE - DEBIAN - THIS
sudo apt update
sudo apt -y upgrade # this may need a second attempt if it errors
sudo apt -y install ros-foxy-desktop python3-argcomplete
sudo apt -y install ros-dev-tools

alias sr2='source /opt/ros/foxy/setup.bash'
echo "alias sr2='source /opt/ros/foxy/setup.bash'" >> ~/.bashrc

# verify installation
sr2
ros2 topic list

# Expected output:

# /parameter_events
# /rosout

# ROS1 BRIDGE INSTALLATION - DEBIAN - YEP
sudo apt -y install ros-foxy-ros1-bridge

# THIS IS SPECIFIC TO MY SETUP WITH THE ROSHome FOLDER IN ~ 
echo "alias srws='source ~/ROSHome/ros2_ws/install/setup.bash'" >> ~/.bashrc

# install python modules and other required modules for the code to run
sudo apt -y install python3-pip
pip install ultralytics
pip install paho-mqtt
pip install watchdog
pip install playsound
pip install sounddevice
sudo apt-get -y install libportaudio2
pip install vosk

# required for web app
pip install flask
pip install flask_cors
sudo apt -y install node-typescript
sudo apt -y install nodejs npm
sudo npm install -g live-server

# download model from vosk
# alternative models available at https://alphacephei.com/vosk/models
cd ~/ROSHome/ros2_ws/assets && mkdir models && cd models
wget -q --show-progress https://alphacephei.com/vosk/models/vosk-model-en-us-0.42-gigaspeech.zip
unzip -q vosk-model-en-us-0.42-gigaspeech.zip
rm vosk-model-en-us-0.42-gigaspeech.zip

# if using whisper branch
pip install openai-whisper
sudo apt -y install ffmpeg

# if you want to test mqtt commands without Home Assistant set up then:
sudo apt -y install mosquitto-clients

# restart your computer
