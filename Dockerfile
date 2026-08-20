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
        ros-humble-navigation2 \
	    ros-humble-nav2-bringup \
	    ros-humble-turtlebot3* \
        ros-humble-velodyne-description \
        ros-humble-plotjuggler \
        ros-humble-plotjuggler-ros \
        python3-rosdep \
        ros-humble-ament-cmake-clang-format \
        ros-humble-message-filters \
     && apt purge -y --auto-remove \
     && rm -rf /var/lib/apt/lists/*

# Python3 Packages required by task allocation
RUN pip install \
    "numpy < 2"\
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

# ==============================================================================
# FIX PYTHON BUILD INCOMPATIBILITY LAYER (ADDED HERE)
# ==============================================================================
RUN pip3 install --upgrade packaging && \
    pip3 install "setuptools<71.0.0"
# ==============================================================================

#RUN apt-get update && apt-get install -y ros-humble-realsense2-*
#####################################
#             Realsense SDK         #
#####################################
#Install realsense sdk
# Ensure the directory exists
RUN apt-get update && apt-get install -y \
    software-properties-common \
    lsb-release

# Add Intel RealSense repository key and source
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | gpg --dearmor > /etc/apt/keyrings/librealsenseai.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" > /etc/apt/sources.list.d/librealsense.list

# Update cache and install utils/dev packages (Omit librealsense2-dkms)
RUN apt-get update && apt-get install -y \
    librealsense2-utils \
    librealsense2-dev
#RUN apt-get install ros-humble-vision-msgs
 #######################################################
 
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-humble-clearpath-desktop \
    ros-humble-clearpath-nav2-demos \
    ros-humble-urg-node 

RUN apt-get install ros-humble-vision-msgs


# Create user
RUN apt update && apt install -y cmake g++ make python3 git
### Instruction for fast downward
#RUN cd ~/autonomy_stack_ros_humble/src && \
#   git clone https://github.com/aibasel/downward.git fast_downward && \
#   cd fast_downward && \
#   ./build.py


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

# ==============================================================================
# EvoSkill toolchain for evolve-stl-pddl (OpenEvolve + Fast Downward + VAL)
# The repo itself is not copied here — it is volume-mounted at
# ~/autonomy_stack_ros_humble/evolve-stl-pddl by docker-compose.
# ==============================================================================
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        cmake \
        g++ \
        make \
        flex \
        bison \
    && sudo rm -rf /var/lib/apt/lists/*

# Fast Downward — PDDL planner + validator fallback used by pddl_evolve/evaluator.py
RUN sudo git clone --depth 1 https://github.com/aibasel/downward.git /opt/fast-downward && \
    sudo chown -R $UID:$GID /opt/fast-downward && \
    cd /opt/fast-downward && ./build.py
ENV FD_PATH=/opt/fast-downward/fast-downward.py

# VAL — evaluator.py prefers `validate` on PATH over the FD fallback
RUN sudo git clone --depth 1 https://github.com/KCL-Planning/VAL.git /opt/VAL && \
    sudo chown -R $UID:$GID /opt/VAL && \
    cd /opt/VAL && \
    cmake -DCMAKE_BUILD_TYPE=Release -B build . && \
    cmake --build build -j"$(nproc)" && \
    sudo ln -s /opt/VAL/build/bin/Validate /usr/local/bin/validate
ENV PATH=/opt/VAL/build/bin:/home/user/.local/bin:$PATH
ENV LD_LIBRARY_PATH=/opt/VAL/build/bin:$LD_LIBRARY_PATH

# OpenEvolve — evolution loop driver. Installed from PyPI: an editable install of
# the git checkout breaks against the setuptools<71 pin above.
RUN pip install --no-cache-dir openevolve==0.3.2 stlpy

# The repo's job scripts invoke openevolve-run.py by path; keep that path working.
RUN sudo mkdir -p /opt/openevolve && \
    printf '%s\n' '#!/usr/bin/env python3' 'from openevolve.cli import main' 'raise SystemExit(main())' \
        | sudo tee /opt/openevolve/openevolve-run.py > /dev/null && \
    sudo chmod +x /opt/openevolve/openevolve-run.py && \
    ln -s /opt/openevolve /home/user/openevolve

# Copy entrypoint
COPY docker/entrypoint.sh /
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
