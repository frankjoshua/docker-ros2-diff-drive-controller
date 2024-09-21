FROM frankjoshua/ros2

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-$ROS_DISTRO-tf-transformations pip \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
RUN pip3 install --break-system-packages transforms3d
ENV DEBIAN_FRONTEND=dialog

# Set the working directory to /root
WORKDIR /root

# Copy your existing ROS2 workspace into the container
COPY ros2_ws ./ros2_ws/

# Install all dependencies for the workspace, including rosbridge
RUN cd ros2_ws \
   && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y \
      && rm -rf /var/lib/apt/lists/*

# Build the workspace using colcon
RUN cd ros2_ws \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build --symlink-install

# Copy the entrypoint script into the container
COPY ros_entrypoint.sh /ros_entrypoint.sh

# Ensure the entrypoint script is executable
RUN chmod +x /ros_entrypoint.sh

CMD ["/bin/bash", "-c", "ros2 run diff_drive_controller diff_drive_controller"]