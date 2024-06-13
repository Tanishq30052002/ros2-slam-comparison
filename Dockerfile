# Use the official ROS Humble desktop base image
FROM osrf/ros:humble-desktop-full

# Install necessary tools and ROS dependencies
RUN apt-get update && \
    apt-get install -y \
    software-properties-common \
    ros-dev-tools \
    ros-humble-joint-state-publisher \
    ros-humble-tf-transformations \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ament-cmake \
    python3-pip && \
    add-apt-repository universe && \
    rm -rf /var/lib/apt/lists/*

# Install additional Python dependencies
RUN pip install \
    transforms3d \
    opencv-python \
    opencv-contrib-python

# Source ROS Humble environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

# Set up Turtlebot
RUN mkdir -p /root/clutterbot/src

# Build the main workspace
WORKDIR /root/clutterbot
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
RUN echo "source /root/clutterbot/install/setup.bash" >> ~/.bashrc

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]