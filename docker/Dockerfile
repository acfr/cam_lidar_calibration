FROM ubuntu:18.04

ARG DEBIAN_FRONTEND=noninteractive

# Install basic packages
RUN apt-get update && apt-get install -y --no-install-recommends \
         curl \
         gnupg2 \
         lsb-core \
         libpng16-16 \
         libjpeg-turbo8 \
         libtiff5 \
         wget && \
     rm -rf /var/lib/apt/lists/*

# Install ROS 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-melodic-desktop \
        python-rosdep
RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Intalling ROS basic tools
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        python-catkin-tools \
        python-pip \
        python-tk \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        software-properties-common && \
     rm -rf /var/lib/apt/lists/*

# Catkin workspace setup
RUN mkdir -p /catkin_ws/src

# Initialise workspace
WORKDIR /catkin_ws/src
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_init_workspace"
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin build;"
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install cam_lidar_calibration specific dependencies
RUN pip install -U pandas scipy

COPY ./dependencies /tmp/dependencies
RUN apt-get update && \
    sed "s/\melodic/melodic/g" "/tmp/dependencies" | xargs apt-get install -y #&& \
    rm -rf /var/lib/apt/lists/* 

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/* 

CMD ["tail", "-f", "/dev/null"]
ADD entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
