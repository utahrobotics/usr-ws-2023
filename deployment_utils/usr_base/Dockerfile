# The base image that will serve as the default workspace for Utah Student Robotics programming members

FROM osrf/ros:foxy-desktop

COPY bashrc_append /
RUN ["/bin/bash", "-c", "cat /bashrc_append >> /root/.bashrc && rm /bashrc_append"]

# Update apt
RUN sudo apt update

# Install realsense 2 ROS
RUN sudo apt-get install ros-foxy-realsense2-camera -y

# Install nano and pip and Navigation2 dependencies
RUN sudo apt install python3-pip ros-foxy-navigation2 ros-foxy-nav2-bringup '~ros-foxy-turtlebot3-.*' -y

# Clone Navigation2
RUN mkdir -p ~/nav2_ws/src && \
    cd ~/nav2_ws/src && \
    git clone https://github.com/ros-planning/navigation2.git --branch foxy-devel && \
    cd ~/nav2_ws && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy

# Build Navigation2
RUN ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash && cd ~/nav2_ws && colcon build --symlink-install"]