FROM osrf/ros:foxy-desktop

#Additional Packages

#ENV DEBIAN_FRONTEND=noninteractive
#RUN apt-get update \
#    && apt-get -y install --no-install-recommends <your-package-list-here> \
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
#ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
# Always source ros setup.bash
RUN ["/bin/bash", "-c", "echo \"source /opt/ros/foxy/setup.bash\" >> /root/.bashrc"]
# colcbuild alias, build then source overlay
RUN ["/bin/bash", "-c", "echo \"alias colcbuild='colcon build --symlink-install && source install/setup.bash'\" >> /root/.bashrc"]
# colctest alias, test with output printed to console
RUN ["/bin/bash", "-c", "echo \"alias colctest='colcon test --return-code-on-test-failure --event-handlers console_cohesion+'\" >> /root/.bashrc"]