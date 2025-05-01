# Start with ROS 2 Humble base image
FROM ros:humble

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies and setup Gazebo repository
RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    gnupg \
    sudo \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Add the Gazebo repository and install Gazebo Harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-harmonic

# Setup ROS workspace
RUN mkdir -p /root/ws/src
WORKDIR /root/ws

# Initialize rosdep and install any required ROS dependencies
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
    fi && \
    rosdep update

RUN apt-get update && apt-get install -y \
    ros-humble-ros-gzharmonic \
    ros-humble-nav2-bringup \
    ros-humble-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*
# Set up ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install ROS dependencies for workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Set the entrypoint for the container
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
