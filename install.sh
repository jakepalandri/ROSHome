# Install git and download this repository to your machine
sudo apt -y install git
git clone git@github.com:jakepalandri/ROSHome.git

# ROS 1 INSTALLATION
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt -y install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt -y install ros-noetic-desktop-full

sudo apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt -y install python3-rosdep

sudo rosdep init
rosdep update

# ROS1 Environment Configuring
source /opt/ros/noetic/setup.bash
alias sr1='source /opt/ros/noetic/setup.bash'
echo "alias sr1='source /opt/ros/noetic/setup.bash'" >> ~/.bashrc
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo "alias srk='source ~/catkin_ws/devel/setup.bash'" >> ~/.bashrc

# verify installation
sr1
roscore
# Ctrl+C to exit

# CUDA INSTALLATION
cd ~
sudo apt update
sudo apt install nvidia-cuda-toolkit
# verify installation
nvcc --version

# https://github.com/OpenKinect/libfreenect2/issues/1196
# remove previous gcc-9 and g++-9 by :
sudo update-alternatives --remove gcc /usr/bin/gcc-9

# you need to install gcc-7 and g++-7 :
sudo apt-get install gcc-7 g++-7
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 50
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 50
sudo update-alternatives --config gcc
sudo update-alternatives --config g++

wget http://developer.download.nvidia.com/compute/cuda/10.1/Prod/local_installers/cuda_10.1.243_418.87.00_linux.run
sudo sh cuda_10.1.243_418.87.00_linux.run
# Uncheck
# - Driver
# - CUDA Toolkit 10.1
# - CUDA Demo Suite 10.1
# - CUDA Documentation 10.1
#
# Only leave checked
# - CUDA Samples 10.1
export CPATH=$CPATH:$HOME/NVIDIA_CUDA-10.1_Samples/common/inc
echo "export CPATH=$CPATH:$HOME/NVIDIA_CUDA-10.1_Samples/common/inc" >> ~/.bashrc

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

# IAI_KINECT2_OPENCV4
cd ~/catkin_ws/src/
git clone https://github.com/paul-shuvo/iai_kinect2_opencv4.git
cd iai_kinect2_opencv4
rosdep install -r --from-paths .
cd ~/catkin_ws
sudo bash -c 'echo -e "set( CMAKE_CXX_STANDARD 14)\n$(cat src/CMakeLists.txt)" > src/CMakeLists.txt'
sr1
catkin_make -DCMAKE_BUILD_TYPE="Release"

sudo apt-get install nvidia-modprobe opencl-headers
echo "/usr/local/cuda/lib64" | sudo tee /etc/ld.so.conf.d/cuda.conf
sudo ldconfig

# ROS2 INSTALLATION
locale  # check for UTF-8
# if not, do this - any UTF8 should be fine
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt -y install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# CHOOSE ONE - DEBIAN - THIS
sudo apt update
sudo apt -y upgrade
sudo apt -y install ros-foxy-desktop python3-argcomplete
sudo apt -y install ros-dev-tools

alias sr2='source /opt/ros/foxy/setup.bash'
echo "alias sr2='source /opt/ros/foxy/setup.bash'" >> ~/.bashrc

# verify installation
sr2
ros2 topic list

# ROS1 BRIDGE INSTALLATION - DEBIAN - YEP
sudo apt install ros-foxy-ros1-bridge

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
cd ~/ROSHome/ros2_ws/assets/models
wget -q --show-progress https://alphacephei.com/vosk/models/vosk-model-en-us-0.42-gigaspeech.zip
unzip -q vosk-model-en-us-0.42-gigaspeech.zip
rm vosk-model-en-us-0.42-gigaspeech.zip

# if using whisper branch
pip install openai-whisper
sudo apt -y install ffmpeg

# if you want to test mqtt commands without Home Assistant set up then:
sudo apt -y install mosquitto-clients
