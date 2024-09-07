#!/bin/bash

# Store the password (ensure it's secure!)
PASSWORD="271828"

# Define working directory
ws=$(pwd)


# Define commands

#Install ros2
cmd1="echo 'Installing ROS2'
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
"

#Install gazebo
cmd2="echo 'Installing Gazebo'
curl -sSL http://get.gazebosim.org | sh
'"

#Install Px4
cmd3="echo 'Installing PX4 Autopilot'
cd 
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
"


#Install FastDDs
cmd4="echo 'Installing FM'
cd
git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build && cd build
cmake ..
sudo cmake --build . --target install
"

cmd5="echo 'Installing Fast DDS'
cd
git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.2 
apt install libasio-dev
cd Fast-DDS
mkdir build && cd build
cmake .. 
make 
sudo make install
"


#Install QGC
cmd6="echo 'Installing QGC'
cd
usermod -a -G dialout \$USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
"


#Cloning 
cmd7="echo 'Cloning and Building px4_offboard'
cd
cd \"$ws\"
mkdir -p src
cd src
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git
git clone https://github.com/PX4/px4_msgs.git
cd ..
colcon build
"

#Install micro ros agent
cmd8="echo 'Installing micro_ros_agent'
cd
source /opt/ros/humble/setup.bash
mkdir microros_ws && cd microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
"

# Array of command blocks
command_blocks=(
    # "$cmd1"
    # "$cmd2"
    "$cmd3"
    "$cmd4"
    "$cmd5"
    "$cmd6"
    "$cmd7"
    "$cmd8"
)

# Iterate over command blocks
for block in "${command_blocks[@]}"; do
    echo "Executing block..."
    eval "$block"
    
    # Check if the block was successful
    if [ $? -ne 0 ]; then
        echo "A block failed. Exiting..."
        exit 1
    fi
done

echo "All blocks executed successfully!"
