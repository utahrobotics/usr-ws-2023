# This container should be a complete program for lunabot
# It shall contain the specific version of the repository and image of lunabotics/base
# for the robot to run

FROM ghcr.io/utahrobotics/lunabotics/base

ARG REPO_COMMIT_SHA

RUN apt-get update
RUN git clone https://github.com/utahrobotics/usr-ws-2023.git
WORKDIR /usr-ws-2023
RUN git checkout $REPO_COMMIT_SHA
RUN bash -c "source /opt/ros/foxy/setup.bash && rosdep install --from-paths src -y --ignore-src && colcon build --symlink-install"
