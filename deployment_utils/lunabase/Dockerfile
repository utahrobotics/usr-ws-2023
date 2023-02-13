# This image has the ability to present a GUI through a VNC client
# IMPORTANT NOTE: For this to work, port 5901 of the container must be exposed
# The latest build of this can be pulled from manglemix/usr_docker_repo:lunabot_gui

FROM ghcr.io/utahrobotics/lunabotics/base

# Install shared utils and nano
RUN apt-get install -y --no-install-recommends \
        apt-utils \
        ca-certificates \
        locales \
        net-tools \
        sudo \
        supervisor \
        wget \
        openssh-server \
        nano

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
    PASSWORD=password \
    TVNC_VERSION=2.2.2

EXPOSE $VNC_PORT

# Install TurboVNC
RUN export TVNC_DOWNLOAD_FILE="turbovnc_${TVNC_VERSION}_amd64.deb" && \
    wget -q -O $TVNC_DOWNLOAD_FILE "https://sourceforge.net/projects/turbovnc/files/2.2.2/${TVNC_DOWNLOAD_FILE}/download" && \
    dpkg -i $TVNC_DOWNLOAD_FILE && \
    rm -f $TVNC_DOWNLOAD_FILE

# Configure X server
RUN touch ~/.Xauthority && \
    mkdir ~/.vnc

# Install gmock then rviz
RUN mkdir -p ~/rviz2_ws/src && \
    cd ~/rviz2_ws/src && \
    git clone https://github.com/ros2/rviz.git --branch foxy --single-branch && \
    git clone https://github.com/ros/resource_retriever.git --branch foxy --single-branch && \
    cd .. && \
    bash -c "source /opt/ros/foxy/setup.bash && colcon build --merge-install"
    
COPY bashrc_append /
RUN ["/bin/bash", "-c", "cat /bashrc_append >> /root/.bashrc && rm /bashrc_append"]
