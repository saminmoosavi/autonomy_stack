# ============================================================
# ROS 2 Jazzy + CUDA 12.6 + cuDNN + Ubuntu 24.04
# ============================================================
FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04

ARG UNAME=user
ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=jazzy


# ============================================================
# Locale + basic Ubuntu tools
# ============================================================
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    ca-certificates \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Add ROS 2 repository
# ============================================================
RUN curl -sSL \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list


# ============================================================
# ROS 2 Jazzy + development tools
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    g++ \
    make \
    git \
    sudo \
    nano \
    gedit \
    xterm \
    wget \
    curl \
    pciutils \
    usbutils \
    udev \
    net-tools \
    iproute2 \
    iputils-ping \
    libeigen3-dev \
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev \
    libomp-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# ROS 2 utilities / middleware
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-rqt* \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-tf-transformations \
    ros-jazzy-message-filters \
    ros-jazzy-pcl-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-ament-cmake-clang-format \
    ros-jazzy-v4l2-camera \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Gazebo Harmonic / ROS-GZ
#
# Jazzy uses modern Gazebo, NOT Gazebo Classic.
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-dev-tools \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Nav2
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Clearpath Jazzy
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-clearpath-desktop \
    ros-jazzy-clearpath-config-live \
    ros-jazzy-clearpath-nav2-demos \
    ros-jazzy-clearpath-simulator \
    ros-jazzy-topic-tools \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Additional robot / sensor packages
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlebot3* \
    ros-jazzy-velodyne-description \
    ros-jazzy-plotjuggler \
    ros-jazzy-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Intel RealSense
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-realsense2-camera \
    ros-jazzy-realsense2-description \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# MoveIt 2
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-moveit \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# GTSAM
# ============================================================
RUN apt-get update && apt-get install -y \
    libgtsam-dev \
    libeigen3-dev \
    libpcl-dev \
    libomp-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Redis
# ============================================================
RUN apt-get update && apt-get install -y \
    python3-redis \
    && rm -rf /var/lib/apt/lists/*
    
# ============================================================
# Python dependencies managed by Ubuntu
# ============================================================
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-matplotlib \
    python3-transforms3d \
    python3-networkx \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Python packages not provided by Ubuntu
# ============================================================

# OpenAI and UTM:
# --ignore-installed prevents pip from trying to uninstall
# Debian-managed dependencies such as idna/typing_extensions.
RUN python3 -m pip install \
    --break-system-packages \
    --ignore-installed \
    --no-cache-dir \
    utm \
    openai



# ============================================================
# NumPy compatibility with ROS 2 Jazzy cv_bridge
# ============================================================
RUN python3 -m pip install \
    --break-system-packages \
    --ignore-installed \
    --no-cache-dir \
    "numpy==1.26.4"

# ============================================================
# PyTorch - NVIDIA GB10
# Match known-working Humble environment
# ============================================================
RUN python3 -m pip install \
    --break-system-packages \
    --no-cache-dir \
    "torch==2.13.0" \
    torchvision \
    torchaudio \
    --index-url https://download.pytorch.org/whl/cu130

# ============================================================
# Ultralytics dependencies
# ============================================================
RUN python3 -m pip install \
    --break-system-packages \
    --no-cache-dir \
    "numpy==1.26.4" \
    pyyaml \
    requests \
    scipy \
    pillow \
    psutil \
    py-cpuinfo \
    tqdm \
    seaborn


# ============================================================
# LAP
# ============================================================
RUN python3 -m pip install \
    --break-system-packages \
    --no-cache-dir \
    --no-deps \
    "lap>=0.5.12"


# ============================================================
# Ultralytics
# ============================================================
RUN python3 -m pip install \
    --break-system-packages \
    --no-cache-dir \
    --no-deps \
    ultralytics



RUN apt update && apt install ros-jazzy-rmw-zenoh-cpp
RUN apt update && apt install -y ros-jazzy-urg-node
# ============================================================
# Install sudo
# ============================================================
RUN apt-get update && apt-get install -y \
    sudo \
    && rm -rf /var/lib/apt/lists/*
# ============================================================
# Create user
# ============================================================

ARG UNAME=user
ARG UID=1000
ARG GID=1000

RUN set -eux; \
    \
    existing_user="$(getent passwd "${UID}" | cut -d: -f1 || true)"; \
    if [ -n "${existing_user}" ]; then \
        userdel -r "${existing_user}" 2>/dev/null || userdel "${existing_user}" || true; \
    fi; \
    \
    existing_group="$(getent group "${GID}" | cut -d: -f1 || true)"; \
    if [ -z "${existing_group}" ]; then \
        groupadd --gid "${GID}" "${UNAME}"; \
        existing_group="${UNAME}"; \
    fi; \
    \
    useradd \
        --uid "${UID}" \
        --gid "${existing_group}" \
        --create-home \
        --shell /bin/bash \
        "${UNAME}"; \
    \
    echo "${UNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${UNAME}; \
    chmod 0440 /etc/sudoers.d/${UNAME}

# ============================================================
# ROS environment
# ============================================================
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${UNAME}/.bashrc

ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64/stubs:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}


# ============================================================
# Create Jazzy workspace
# ============================================================
USER ${UNAME}

RUN mkdir -p /home/${UNAME}/autonomy_stack_ros_jazzy/src

WORKDIR /home/${UNAME}/autonomy_stack_ros_jazzy


# ============================================================
# Copy source tree
# ============================================================
COPY --chown=${UID}:${GID} src \
    /home/${UNAME}/autonomy_stack_ros_jazzy/src


# ============================================================
# rosdep
# ============================================================
RUN sudo rosdep init || true

RUN rosdep update


# ============================================================
# Install workspace dependencies
# ============================================================
RUN sudo apt-get update \
    && . /opt/ros/jazzy/setup.sh \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro jazzy \
        -r \
        -y \
    && sudo rm -rf /var/lib/apt/lists/*

# ============================================================
# Entrypoint
# ============================================================
COPY docker/entrypoint.sh /entrypoint.sh

RUN sudo chmod +x /entrypoint.sh


ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
