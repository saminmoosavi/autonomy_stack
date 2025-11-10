FROM nvidia/cuda:11.4.3-cudnn8-devel-ubuntu20.04

# Args for User
ARG UNAME=user
ARG UID=1000
ARG GID=1000

# Ensure that installs are non-interactive
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
        sudo \
        iputils-ping \
        udev \
        usbutils \
        net-tools \
        wget \
        iproute2 \
        curl \
        nano \
        gedit \
        git \
        lsb-release
        
# Setup sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    
# Install dependencies for building ros packages
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-pip \
    python3-catkin-tools\
    && rm -rf /var/lib/apt/lists/*

# Bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# Install ros noetic
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*    

#RUN sudo apt-get install -y libgazebo11-dev
# Install ros packages  
RUN apt-get update && apt-get install -y \
    ros-noetic-catkin \
    ros-noetic-navigation     \
    ros-noetic-robot-localization   \
    ros-noetic-robot-state-publisher  \
    ros-noetic-pcl-ros             \
    ros-noetic-ros-numpy     \   # for liorf
    ros-noetic-cv-bridge     \   # for liorf 
    && rm -rf /var/lib/apt/lists/*
    
# Install dependencies for Ouster driver 
RUN sudo apt-get update && apt-get install -y --no-install-recommends  \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake 
    
    
# Install gtSAM   
RUN sudo apt update && apt install software-properties-common -y
RUN sudo add-apt-repository ppa:borglab/gtsam-release-4.2    
RUN sudo apt install -y libgtsam-dev libgtsam-unstable-dev    

# Create user
RUN groupadd -g $GID $UNAME
RUN useradd -m -u $UID -g $GID -s /bin/bash $UNAME

# Allow the user to run sudo without a password
RUN echo "$UNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $UNAME

RUN LD_LIBRARY_PATH=/usr/local/cuda/lib64/stubs/:$LD_LIBRARY_PATH

# Get Workspace Dependencies
RUN mkdir -p ~/autonomy_stack/src
COPY src /home/user/autonomy_stack/src
RUN cd ~/autonomy_stack && \
    sudo apt update &&\
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y


# Copy entrypoint
COPY docker/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
