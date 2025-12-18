# Start from CUDA NN Docker Image
FROM nvidia/cuda:11.7.1-cudnn8-devel-ubuntu22.04

# Args for User
ARG UNAME=user
ARG UID=1000
ARG GID=1000

# Ensure that installs are non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 Humble and other dependencies
RUN apt update && apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt -y clean && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

RUN apt update && \
    apt install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && \
    apt install -y  -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" keyboard-configuration && \
    apt install -y ros-humble-desktop && \
    apt install -y python3-colcon-common-extensions && \
    apt install -y ros-humble-v4l2-camera && \
    apt install -y git && \
    apt install -y xterm && \
    apt install -y wget && \
    apt install -y pciutils && \
    apt -y clean && \
    rm -rf /var/lib/apt/lists/*

# Install setup utils and basic dependencies
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
        git \
        python3-pip \
        ros-humble-rqt* \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-tf-transformations \
        ros-humble-ros-gz\
        ros-humble-clearpath-desktop\
        ros-humble-clearpath-nav2-demos\
        ros-humble-clearpath-simulator\
        ros-humble-navigation2 \
	    ros-humble-nav2-bringup \
	    ros-humble-turtlebot3* \
        ros-humble-velodyne-description \
        ros-humble-plotjuggler \
        ros-humble-plotjuggler-ros \
        python3-rosdep \
        ros-humble-ament-cmake-clang-format \
     && apt purge -y --auto-remove \
     && rm -rf /var/lib/apt/lists/*
     
# Python3 Packages required by task allocation
RUN pip install \
    numpy \
    matplotlib \
    transforms3d \
    utm \
    networkx \
    openai \
    ros2_numpy \
    ultralytics
    
RUN pip install \
    opencv-python>=4.8.1.78 \
    typing-extensions>=4.4.0 \
    ultralytics==8.3.168 \
    lap>=0.5.12

RUN apt-get update && apt-get install -y ros-humble-realsense2-*

# install yolo
#RUN pip install -U ultralytics


# Create user
RUN groupadd -g $GID $UNAME
RUN useradd -m -u $UID -g $GID -s /bin/bash $UNAME
# Allow the user to run sudo without a password
RUN echo "$UNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
# Switch to the non-root user for other images
USER $UNAME

RUN LD_LIBRARY_PATH=/usr/local/cuda/lib64/stubs/:$LD_LIBRARY_PATH

# Create workspace
RUN mkdir -p ~/autonomy_stack_ros_humble/src
COPY src /home/user/autonomy_stack_ros_humble/src
RUN cd ~/autonomy_stack_ros_humble && \
    sudo apt update && \
    sudo rosdep init && \
    rosdep update
    
RUN cd ~/autonomy_stack_ros_humble && rosdep install --from-paths src --ignore-src -r -y


# Copy entrypoint
COPY docker/entrypoint.sh /
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
