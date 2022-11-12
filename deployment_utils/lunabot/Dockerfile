# This is an experimental deployment container that will run on the Jetson Xavier
# The latest build of this can be pulled from manglemix/usr_docker_repo:lunabot

FROM manglemix/usr_docker_repo:usr_base

ENV CONTEXT=None

COPY bashrc_append /
RUN ["/bin/bash", "-c", "cat /bashrc_append >> /root/.bashrc && rm /bashrc_append"]
