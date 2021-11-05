FROM frankjoshua/ros2

# ** [Optional] Uncomment this section to install additional packages. **
#
# USER root
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends ros-galactic-desktop \
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

USER ros
# # nvidia-container-runtime
# ENV NVIDIA_VISIBLE_DEVICES \
#     ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENTRYPOINT [ "/bin/bash", "-i", "-c" ]
CMD ["ros2 topic list"]