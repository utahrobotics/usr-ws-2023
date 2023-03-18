# Enable Ubuntu Universe Repository
RUN apt-get update && \
    apt-get install software-properties-common -y --no-install-recommends && \
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
# Install nav2 components as separate layers (to make it easier to upload and cache)
RUN apt-get install ros-foxy-navigation2 -y --no-install-recommends
RUN apt-get install ros-foxy-nav2-bringup '~ros-foxy-turtlebot3-.*' -y --no-install-recommends
# Install usbutils and nano
RUN apt-get install usbutils nano -y --no-install-recommends

COPY base_bashrc_append.sh /bashrc_append
COPY findusbdev.sh /root
RUN chmod +x /root/findusbdev.sh
# Append bashrc
RUN ["/bin/bash", "-c", "cat /bashrc_append >> /root/.bashrc && rm /bashrc_append"]

FROM base as lunabase
# Install shared utils
RUN apt-get install -y --no-install-recommends \
        apt-utils \
        ca-certificates \
        locales \
        net-tools \
        sudo \
        supervisor \
        wget \
        openssh-server

# Install XFCE and terminal
RUN apt-get install -y --no-install-recommends \
        dbus-x11 \
        libexo-1-0 \
        x11-apps \
        x11-xserver-utils \
        xauth \
        xfce4 \
        xfce4-terminal \
        xterm

ENV VNC_PORT=5901 \
    VNC_RESOLUTION=1024x640 \
    DISPLAY=:1 \
    TERM=xterm \
    DEBIAN_FRONTEND=noninteractive \
    HOME=/root \
    PATH=/opt/TurboVNC/bin:$PATH \
    TVNC_WM=xfce4-session \
    PASSWORD=password

EXPOSE $VNC_PORT

# Install TurboVNC
RUN export TVNC_DOWNLOAD_FILE="turbovnc_2.2.2_amd64.deb" && \
    wget -q -O $TVNC_DOWNLOAD_FILE "https://sourceforge.net/projects/turbovnc/files/2.2.2/turbovnc_2.2.2_amd64.deb/download" && \
    dpkg -i $TVNC_DOWNLOAD_FILE && \
    rm -f $TVNC_DOWNLOAD_FILE

# Configure X server
RUN touch ~/.Xauthority && \
    mkdir ~/.vnc

# # Install rviz2
RUN apt-get install -y --no-install-recommends \
    ros-foxy-rviz2 \
    ros-foxy-rviz-visual-tools

COPY lunabase_bashrc_append.sh /bashrc_append
RUN ["/bin/bash", "-c", "cat /bashrc_append >> /root/.bashrc && rm /bashrc_append"]
