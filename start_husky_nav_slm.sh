#!/bin/bash

set -e

# ============================================================
# Clearpath Husky Simulation + SLAM + Nav2
# ROS 2 Jazzy
# ============================================================

SETUP_PATH="/home/user/clearpath"
ROS_DOMAIN_ID=5

echo "============================================"
echo " Clearpath Husky Simulation"
echo " Setup path: ${SETUP_PATH}"
echo " ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "============================================"

# ------------------------------------------------------------
# ROS 2 environment
# ------------------------------------------------------------
source /opt/ros/jazzy/setup.bash

# Source workspace if available
if [ -f "/home/user/autonomy_stack_ros_jazzy/install/setup.bash" ]; then
    source /home/user/autonomy_stack_ros_jazzy/install/setup.bash
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

# ------------------------------------------------------------
# Cleanup function
# ------------------------------------------------------------
cleanup()
{
    echo ""
    echo "Stopping simulation, SLAM, and Nav2..."

    kill ${SIM_PID} 2>/dev/null || true
    kill ${SLAM_PID} 2>/dev/null || true
    kill ${NAV2_PID} 2>/dev/null || true

    wait 2>/dev/null || true

    echo "Done."
}

trap cleanup SIGINT SIGTERM EXIT


# ============================================================
# 1. Start Clearpath Husky Gazebo simulation
# ============================================================

echo ""
echo "[1/3] Starting Clearpath Gazebo simulation..."

ros2 launch clearpath_gz simulation.launch.py \
    setup_path:=${SETUP_PATH} &

SIM_PID=$!

echo "Simulation PID: ${SIM_PID}"

# Give Gazebo and robot time to initialize
sleep 10


# ============================================================
# 2. Start SLAM
# ============================================================

echo ""
echo "[2/3] Starting Clearpath SLAM..."

ros2 launch clearpath_nav2_demos slam.launch.py \
     setup_path:=${SETUP_PATH} &

SLAM_PID=$!

echo "SLAM PID: ${SLAM_PID}"

sleep 5


# ============================================================
# 3. Start Nav2
# ============================================================

echo ""
echo "[3/3] Starting Clearpath Nav2..."

ros2 launch clearpath_nav2_demos nav2.launch.py \
     setup_path:=${SETUP_PATH} &

NAV2_PID=$!

echo "Nav2 PID: ${NAV2_PID}"

sleep 5


# ============================================================
# 4. RViz
# ============================================================

echo ""
echo "[4/4] Starting RViz..."

ros2 launch clearpath_nav2_demos view_navigation.launch.py \
    setup_path:=${SETUP_PATH}   &

RVIZ_PID=$!

echo "RViz PID: ${RVIZ_PID}"



# ============================================================
# Running
# ============================================================

echo ""
echo "============================================"
echo " Warthog simulation is running"
echo ""
echo " Gazebo : RUNNING"
echo " SLAM   : RUNNING"
echo " Nav2   : RUNNING"
echo ""
echo " Press Ctrl+C to stop everything."
echo "============================================"

wait
