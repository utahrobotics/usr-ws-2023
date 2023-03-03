apt install software-properties-common -y
add-apt-repository universe
apt update && apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
mkdir -p ~/ros2_foxy
cd ~/ros2_foxy
curl -L -o ros2-package-linux-x86_64.tar.bz2 https://github.com/ros2/ros2/releases/download/release-foxy-20221021/ros2-foxy-20221021-linux-focal-arm64.tar.bz2
tar xf ros2-package-linux-x86_64.tar.bz2
rm ros2-package-linux-x86_64.tar.bz2
apt update
apt install -y python3-rosdep
rosdep init
rosdep update
apt upgrade
apt install -y libpython3-dev python3-pip
pip3 install -U argcomplete