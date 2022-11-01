# This is an experimental deployment container that will run on the Jetson Xavier
# The latest build of this can be pulled from manglemix/usr_docker_repo:lunabot

FROM osrf/ros:foxy-desktop

# Set up auto-source of workspace for ros user
ARG WORKSPACE
# Always source ros setup.bash
RUN ["/bin/bash", "-c", "echo \"source /opt/ros/foxy/setup.bash\" >> /root/.bashrc"]
# colcbuild alias, build then source overlay
RUN ["/bin/bash", "-c", "echo \"alias colcbuild='colcon build --symlink-install && source install/setup.bash'\" >> /root/.bashrc"]
# colctest alias, test with output printed to console
RUN ["/bin/bash", "-c", "echo \"alias colctest='colcon test --return-code-on-test-failure --event-handlers console_cohesion+'\" >> /root/.bashrc"]

# Clone workspace
RUN git clone https://github.com/utahrobotics/usr-ws-2023.git

WORKDIR usr-ws-2023

RUN sudo apt update
RUN sudo apt-get install ros-foxy-realsense2-camera -y

