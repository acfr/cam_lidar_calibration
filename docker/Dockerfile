# Base image with all ROS2 Jazzy dependencies for cam_lidar_calibration
# This image is built infrequently and contains all system-level dependencies

FROM osrf/ros:jazzy-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

LABEL maintainer="ACFR"
LABEL description="Base image for ROS2 Jazzy camera-lidar calibration with perception libraries"

# Install perception and development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS2 perception packages
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-vision-opencv \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    # Python and build tools
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # Development libraries
    libpcl-dev \
    libopencv-dev \
    libeigen3-dev \
    libnlopt-dev \
    libboost-all-dev \
    # Build essentials
    git \
    cmake \
    build-essential \
    gdb \
    # Qt for RViz panels
    qtbase5-dev \
    qtdeclarative5-dev \
    # Utilities
    wget \
    curl \
    vim \
    nano \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for calibration
RUN pip3 install --no-cache-dir \
    pandas \
    scipy \
    numpy \
    matplotlib \
    opencv-python

# Initialize rosdep
RUN if [ ! -d "/etc/ros/rosdep" ]; then rosdep init; fi && \
    rosdep update

# Setup ROS2 environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias cs='source install/setup.bash'" >> /root/.bashrc

# Create workspace directory structure
RUN mkdir -p /ros2_ws/src

WORKDIR /ros2_ws

CMD ["bash"]
