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
echo "alias sr1='source /opt/ros/noetic/setup.bash'" >> ~/.bashrc
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo "alias srk='source ~/catkin_ws/devel/setup.bash'" >> ~/.bashrc

# CUDA INSTALLATION
cd ~
sudo apt-get install linux-headers-$(uname -r)
sudo apt-key del 7fa2af80

wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-toolkit # maybe? idk what ended up working
# might've been this: https://linuxconfig.org/how-to-install-cuda-on-ubuntu-20-04-focal-fossa-linux

# sudo apt-get -y install nvidia-gds

sudo apt-get install -y nvidia-driver-550-open
sudo apt-get install -y cuda-drivers-550

echo "export LD_LIBRARY_PATH='/usr/local/cuda/lib64:${LD_LIBRARY_PATH}'" >> ~/.bashrc
echo "export PATH='/usr/local/cuda/bin:${PATH}'" >> ~/.bashrc

sudo reboot # disable secure boot

nvidia-smi

# LIBFREENECT2 INSTALLATION WITH CHANGE IN CMAKE FOR KINECT2 BRIDGE
cd ~

git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2

sudo apt-get -y install build-essential cmake pkg-config
sudo apt-get -y install libusb-1.0-0-dev
sudo apt-get -y install libturbojpeg0-dev # skip?
sudo apt-get -y install libglfw3-dev
sudo apt-get -y install beignet-dev
sudo apt-get -y install libva-dev libjpeg-dev # do instead?
sudo apt-get -y install libopenni2-dev

mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -DENABLE_CXX11=ON -DENABLE_CUDA=ON
make
make install

cd ..
cmake -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2
sudo cp platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

echo "************************************"
echo "*                                  *"
echo "* UNPLUG AND REPLUG IN YOUR KINECT *"
echo "*                                  *"
echo "************************************"
read -rsp $"Press any key to continue...\n" -n1 key

export LIBVA_DRIVER_NAME=i965
echo "export LIBVA_DRIVER_NAME=i965" >> ~/.bashrc
./build/bin/Protonect
read -rsp $"Press any key to continue...\n" -n1 key

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
# if not, do this
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

echo "alias sr2='source /opt/ros/foxy/setup.bash'" >> ~/.bashrc

# ROS1 BRIDGE INSTALLATION - DEBIAN - YEP
sudo apt install ros-foxy-ros1-bridge


# THIS IS SPECIFIC TO MY SETUP WITH THE ROSHome FOLDER IN ~ 
echo "alias srws='source ~/ROSHome/ros2_ws/install/setup.bash'" >> ~/.bashrc

# install python modules
sudo apt -y install python3-pip
pip install ultralytics
pip install paho-mqtt
pip install watchdog
pip install playsound
pip install sounddevice
sudo apt-get -y install libportaudio2
pip install vosk

# download model from vosk
# alternative models available at https://alphacephei.com/vosk/models
cd ~/ROSHome/ros2_ws/assets/models
wget -q --show-progress https://alphacephei.com/vosk/models/vosk-model-en-us-0.42-gigaspeech.zip
unzip -q vosk-model-en-us-0.42-gigaspeech.zip
rm vosk-model-en-us-0.42-gigaspeech.zip

# if using whisper branch
pip install openai-whisper

# if you want to test mqtt commands without Home Assistant set up then:
sudo apt -y install mosquitto-clients
