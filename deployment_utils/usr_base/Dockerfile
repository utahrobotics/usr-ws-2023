# The base image that will serve as the default workspace for Utah Student Robotics programming members

FROM osrf/ros:foxy-desktop

    # Update apt
RUN sudo apt update && \
    # Install realsense 2 ROS
    sudo apt-get install ros-foxy-realsense2-camera -y --no-install-recommends && \
    # Install nano and pip and Navigation2 dependencies
    sudo apt install python3-pip ros-foxy-navigation2 ros-foxy-nav2-bringup '~ros-foxy-turtlebot3-.*' -y --no-install-recommends && \
    # Clone Navigation2
    mkdir -p ~/nav2_ws/src && \
    cd ~/nav2_ws/src && \
    git clone https://github.com/ros-planning/navigation2.git --branch foxy-devel && \
    cd ~/nav2_ws && \
    # Get dependencies
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy && \
    # build Navigation2
    bash -c "source /opt/ros/foxy/setup.bash && cd ~/nav2_ws && colcon build --symlink-install"

# ENV DEPS_INITIALIZED=false

COPY bashrc_append /
# Append bashrc
RUN ["/bin/bash", "-c", "cat /bashrc_append >> /root/.bashrc && rm /bashrc_append"]