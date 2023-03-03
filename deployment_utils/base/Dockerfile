# The base image that will serve as the default workspace for Utah Student Robotics programming members

# FROM arm64v8/ros:foxy-ros-base
FROM ros:foxy-ros-base

# Update apt and rosdep
RUN sudo apt update && rosdep update
# install realsense 2 ROS
RUN sudo apt install ros-foxy-realsense2-camera -y --no-install-recommends
# Install nav2
RUN apt install ros-foxy-navigation2 ros-foxy-nav2-bringup '~ros-foxy-turtlebot3-.*' -y --no-install-recommends

COPY bashrc_append /
# Append bashrc
RUN ["/bin/bash", "-c", "cat /bashrc_append >> /root/.bashrc && rm /bashrc_append"]