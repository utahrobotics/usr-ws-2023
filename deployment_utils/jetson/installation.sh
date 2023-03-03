sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update -y
sudo apt upgrade
# sudo apt install python3-colcon-common-extensions -y
sudo apt install ros-foxy-desktop python3-argcomplete -y

sudo rosdep init
sudo rosdep update
sudo apt install ros-foxy-realsense2-camera ros-foxy-navigation2 ros-foxy-nav2-bringup '~ros-foxy-turtlebot3-.*' -y --no-install-recommends