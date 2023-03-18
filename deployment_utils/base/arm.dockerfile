# The base image that will serve as the default workspace for Utah Student Robotics programming members
# This is the Arm64 version of base

FROM arm64v8/ubuntu:focal

# Enable Ubuntu Universe Repository
RUN apt-get update
RUN apt-get install software-properties-common -y --no-install-recommends && \
    add-apt-repository universe
RUN apt-get install curl nano pip -y --no-install-recommends

# Add ROS 2 GPG key then add the repository to sources list
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y

# Install ROS 2
RUN apt-get install ros-foxy-ros-base python3-argcomplete -y --no-install-recommends

# Install GPIO and VESC stuff, and rosdep
RUN pip install Jetson.GPIO pyvesc rosdep
# Install colcon and build essentials
RUN apt-get install python3-colcon-common-extensions build-essential -y --no-install-recommends
# Update rosdep
RUN rosdep init && rosdep update

# install realsense 2 ROS
RUN apt-get install ros-foxy-realsense2-camera -y --no-install-recommends
# Install nav2
RUN apt-get install ros-foxy-navigation2 ros-foxy-nav2-bringup '~ros-foxy-turtlebot3-.*' -y --no-install-recommends
# Install usbutils
RUN apt-get install usbutils -y --no-install-recommends

COPY bashrc_append /
COPY findusbdev.sh /root
RUN chmod +x /root/findusbdev.sh
# Append bashrc
RUN ["/bin/bash", "-c", "cat /bashrc_append >> /root/.bashrc && rm /bashrc_append"]