# ============================================================
# ~/.bashrc
# ROS 2 Jazzy + Autonomy Stack
# Ubuntu 24.04
# ============================================================

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac


# ============================================================
# Standard Bash configuration
# ============================================================

HISTCONTROL=ignoreboth
shopt -s histappend

HISTSIZE=10000
HISTFILESIZE=20000

shopt -s checkwinsize

# Enable colored prompt
if [ "$TERM" != "dumb" ]; then
    color_prompt=yes
fi

if [ "$color_prompt" = yes ]; then
    PS1='\[\033[01;32m\]\u@jazzy-docker\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='\u@jazzy-docker:\w\$ '
fi

unset color_prompt


# ============================================================
# Useful aliases
# ============================================================

alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

alias ..='cd ..'
alias ...='cd ../..'

alias c='clear'

# Git
alias gs='git status'
alias gb='git branch'
alias gl='git log --oneline --graph --decorate -20'


# ============================================================
# ROS 2 Jazzy
# ============================================================

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

export ROS_DISTRO=jazzy


# ============================================================
# ROS 2 workspace
# ============================================================

export AUTONOMY_WS=/home/user/autonomy_stack_ros_jazzy

if [ -f "${AUTONOMY_WS}/install/setup.bash" ]; then
    source "${AUTONOMY_WS}/install/setup.bash"
fi


# ============================================================
# ROS 2 DDS configuration
# ============================================================

# Default Jazzy middleware:
# Leave RMW_IMPLEMENTATION unset unless you specifically
# want to force a particular DDS implementation.

# ------------------------------------------------------------
# CycloneDDS
# ------------------------------------------------------------
# Uncomment to force CycloneDDS:
#
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
#export ZENOH_SESSION_CONFIG_URI=~/autonomy_stack_ros_jazzy/zenoh_config.json5   # uncommnet on host and ros2 run rmw_zenoh_cpp rmw_zenohd
#export ZENOH_SESSION_CONFIG_URI=~/autonomy_stack_ros_jazzy/zenoh_session.json5 # uncommnet on client 
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.50.214:7447"]'
# Optional custom CycloneDDS configuration:
#
# export CYCLONEDDS_URI=file:///home/user/autonomy_stack_ros_jazzy/docker/cyclonedds.xml


# ============================================================
# ROS networking
# ============================================================

# Allow communication outside this machine/container.
#
# Do NOT set ROS_LOCALHOST_ONLY=1 when communicating with
# Warthog, Jackal, GB10s, or other ROS 2 computers.

export ROS_LOCALHOST_ONLY=0


# ============================================================
# ROS Domain ID
# ============================================================

# Your Clearpath systems commonly use Domain ID 5.
export ROS_DOMAIN_ID=0


# ============================================================
# CUDA
# ============================================================

export CUDA_HOME=/usr/local/cuda

export PATH="${CUDA_HOME}/bin:${PATH}"

export LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}"


# ============================================================
# Gazebo Harmonic / ROS-GZ
# ============================================================

# Add custom Gazebo models/resources here if required.
#
# Example:
#
# export GZ_SIM_RESOURCE_PATH="${AUTONOMY_WS}/src/simulation/models:${GZ_SIM_RESOURCE_PATH}"

# ROS-GZ uses modern Gazebo rather than Gazebo Classic.
# Do NOT use old GAZEBO_MODEL_PATH unless a legacy package
# specifically requires it.


# ============================================================
# Qt / GUI support
# ============================================================

export QT_X11_NO_MITSHM=1


# ============================================================
# Python
# ============================================================

export PYTHONUNBUFFERED=1


# ============================================================
# Colcon
# ============================================================

# Make colcon output easier to read.
export RCUTILS_COLORIZED_OUTPUT=1

alias cb='cd ${AUTONOMY_WS} && colcon build --symlink-install'

alias cbs='cd ${AUTONOMY_WS} && \
           colcon build --symlink-install && \
           source install/setup.bash'

alias cbp='cd ${AUTONOMY_WS} && colcon build --symlink-install --packages-select'

alias cbup='cd ${AUTONOMY_WS} && colcon build --symlink-install --packages-up-to'

source /opt/ros/jazzy/setup.bash
alias sc='. ~/autonomy_stack_ros_jazzy/install/setup.bash'
alias build='cd ~/autonomy_stack_ros_jazzy && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && sc'

# ============================================================
# ROS aliases
# ============================================================

alias rt='ros2 topic'
alias rtl='ros2 topic list'
alias rnl='ros2 node list'
alias rsl='ros2 service list'
alias ral='ros2 action list'

alias rtf='ros2 run tf2_ros tf2_echo'

alias rosenv='env | grep -E "ROS|RMW|CYCLONE|GZ_|CUDA" | sort'


# ============================================================
# Workspace aliases
# ============================================================

alias ws='cd ${AUTONOMY_WS}'
alias src='cd ${AUTONOMY_WS}/src'

alias sw='source ${AUTONOMY_WS}/install/setup.bash'

alias sj='source /opt/ros/jazzy/setup.bash'


# ============================================================
# rosdep helper
# ============================================================

alias rosdep_ws='cd ${AUTONOMY_WS} && \
    rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro jazzy \
    -r -y'


# ============================================================
# Clearpath helpers
# ============================================================

alias cpconfig='cd /home/user/clearpath'

alias jackal='cd /home/user/jackal_setup'

alias warthog='cd /home/user/warthog_setup'

alias warthogxarm='cd /home/user/warthog_xarm_setup'


# ============================================================
# Useful system / ROS diagnostics
# ============================================================

alias gpu='nvidia-smi'

alias roscheck='echo "
ROS_DISTRO        = $ROS_DISTRO
ROS_DOMAIN_ID     = $ROS_DOMAIN_ID
ROS_LOCALHOST_ONLY= $ROS_LOCALHOST_ONLY
RMW_IMPLEMENTATION= ${RMW_IMPLEMENTATION:-default}
AUTONOMY_WS       = $AUTONOMY_WS
CUDA_HOME         = $CUDA_HOME
"'


# ============================================================
# Startup message
# ============================================================

echo ""
echo "======================================================"
echo " ROS 2 Jazzy Autonomy Development Container"
echo "======================================================"
echo " ROS_DISTRO    : ${ROS_DISTRO}"
echo " ROS_DOMAIN_ID : ${ROS_DOMAIN_ID}"
echo " Workspace     : ${AUTONOMY_WS}"
echo " RMW           : ${RMW_IMPLEMENTATION:-default}"
echo "======================================================"
echo ""

# ============================================================
# Nav2 launch
# ============================================================
alias sim='ros2 launch clearpath_gz simulation.launch.py'
alias nav2='ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=/home/user/jackal_setup/'
alias slam='ros2 launch clearpath_nav2_demos slam.launch.py  setup_path:=/home/user/jackal_setup/'
alias rviz='ros2 launch clearpath_viz view_navigation.launch.py namespace:=/j100_0612'
alias yolo='ros2 launch yolo_bringup yolo-world.launch.py input_image_topic:=/camera/camera_0/color/image_raw'
