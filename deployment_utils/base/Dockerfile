# The base image that will serve as the default workspace for Utah Student Robotics programming members

FROM arm64v8/ros:foxy-ros-base

# Update apt and rosdep
RUN sudo apt update && rosdep update
# install pip and realsense 2 ROS
RUN sudo apt install python3-pip ros-foxy-realsense2-camera -y --no-install-recommends

RUN apt install ros-foxy-navigation2 ros-foxy-nav2-bringup '~ros-foxy-turtlebot3-.*' -y

COPY bashrc_append /
# Append bashrc
RUN ["/bin/bash", "-c", "cat /bashrc_append >> /root/.bashrc && rm /bashrc_append"]