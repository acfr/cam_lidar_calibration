# Base image with all ROS2 dependencies for cam_lidar_calibration
# This image is built infrequently and contains all system-level dependencies

ARG ROS_DISTRO=jazzy

FROM osrf/ros:${ROS_DISTRO}-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# Use Zenoh as middleware
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Set ROS 2 environment variables
ENV ROS_DOMAIN_ID=5

LABEL maintainer="ACFR"
LABEL description="Base image for ROS2 camera-lidar calibration with perception libraries"

# Install perception and development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS2 perception packages
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-ublox-msgs \
    # Python and build tools
    python3-pip \
    python3-venv \
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

# Create Python virtual environment with system site-packages (for ROS2 compatibility)
RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Install Python dependencies for calibration in the venv
RUN /opt/venv/bin/pip install --no-cache-dir \
    pandas \
    scipy \
    numpy \
    matplotlib \
    opencv-python

# Initialize rosdep
RUN if [ ! -d "/etc/ros/rosdep" ]; then rosdep init; fi && \
    rosdep update

# Setup ROS2 environment
RUN echo "source /opt/venv/bin/activate" >> /root/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias cs='source install/setup.bash'" >> /root/.bashrc

# Create workspace directory structure
RUN mkdir -p /ros2_ws/src

WORKDIR /ros2_ws

CMD ["bash"]
