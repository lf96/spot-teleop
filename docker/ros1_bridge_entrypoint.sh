#!/bin/bash
set -e

echo "[ROS BRIDGE] Sourcing ROS1..."
source /opt/ros/noetic/setup.bash

echo "[ROS BRIDGE] Waiting for ROS Master (ROS1)..."
until rostopic list >/dev/null 2>&1; do
  sleep 1
done
echo "[ROS BRIDGE] ROS1 master detected"
rosparam load /home/spot-teleop/ros-bridge_ws/config/bridge.yaml

echo "[ROS BRIDGE] Sourcing ROS2..."
source /opt/ros2_ws/install/setup.bash

echo "[ROS BRIDGE] Starting parameterized ros1_bridge"

exec ros2 run ros1_bridge parameter_bridge
