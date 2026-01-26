# 4 - Moveit planning start
#!/usr/bin/env bash

echo "=============================="
echo "[PLANNING] Bringing up NVBLOX mapping"
echo "=============================="

docker-compose up -d spot-ros2

echo "[PLANNING] Waiting for SPOT-ROS2 container..."
rqt_interface/scripts/utils/wait_for_container.sh spot-ros2

echo "[PLANNING] SPOT-ROS2 Container is running"

docker-compose exec spot-ros2 bash -c "
    export ROS_DISTRO=humble && \
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \

    source /opt/ros/\$ROS_DISTRO/setup.bash && \
    source /home/spot-teleop/spot-ros2_ws/install/setup.bash && \

    ros2 launch spot_moveit_config spot_moveit_all.launch.py 
    "