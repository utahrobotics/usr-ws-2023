sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
mkdir -p ~/ros2_foxy
cd ~/ros2_foxy
curl -L -o ros2-package-linux-x86_64.tar.bz2 https://github.com/ros2/ros2/releases/download/release-foxy-20221021/ros2-foxy-20221021-linux-focal-arm64.tar.bz2
tar xf ros2-package-linux-x86_64.tar.bz2
rm ros2-package-linux-x86_64.tar.bz2
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init
sudo rosdep update
sudo apt upgrade
sudo apt install -y libpython3-dev python3-pip
sudo pip3 install -U argcomplete
sudo apt install ros-foxy-realsense2-camera ros-foxy-navigation2 ros-foxy-nav2-bringup '~ros-foxy-turtlebot3-.*' -y --no-install-recommends