# This is an experimental deployment container that will run on the Jetson Xavier
# The latest build of this can be pulled from manglemix/usr_docker_repo:lunabot

FROM osrf/ros:foxy-desktop

# Always source ros setup.bash
RUN ["/bin/bash", "-c", "echo \"source /opt/ros/foxy/setup.bash\" >> /root/.bashrc"]
# colcbuild alias, build then source overlay
RUN ["/bin/bash", "-c", "echo \"alias colcbuild='colcon build --symlink-install && source install/setup.bash'\" >> /root/.bashrc"]
# colctest alias, test with output printed to console
RUN ["/bin/bash", "-c", "echo \"alias colctest='colcon test --return-code-on-test-failure --event-handlers console_cohesion+'\" >> /root/.bashrc"]
# Initial setup
RUN ["/bin/bash", "-c", "echo -e \"if ! [ -d /usr-ws-2023 ]; then\n    echo \"Performing first time setup...\"\n    cd /\n    git clone https://github.com/utahrobotics/usr-ws-2023.git && cd usr-ws-2023 && colcbuild && echo \"Setup completed! You do not need to rebuild if you just edit Python scripts\"\n    cd ~\nfi\" >> /root/.bashrc"]

RUN sudo apt update

# Install realsense 2 ROS
RUN sudo apt-get install ros-foxy-realsense2-camera -y

# Install Navigation2 Dependencies
RUN sudo apt install python3-pip -y
RUN sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup '~ros-foxy-turtlebot3-.*' -y

# Clone Navigation2
RUN mkdir -p ~/nav2_ws/src && \
    cd ~/nav2_ws/src && \
    git clone https://github.com/ros-planning/navigation2.git --branch foxy-devel && \
    cd ~/nav2_ws && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy

# Build Navigation2
# RUN ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash && cd ~/nav2_ws && colcon build --symlink-install"]

# Install nano
RUN sudo apt install nano