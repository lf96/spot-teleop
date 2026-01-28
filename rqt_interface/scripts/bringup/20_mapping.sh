# 3 - Nvblox mapping start
#!/usr/bin/env bash

echo "=============================="
echo "[MAPPING] Bringing up NVBLOX mapping"
echo "=============================="

docker-compose up -d zed

echo "[MAPPING] Waiting for ZED container..."
scripts/utils/wait_for_container.sh zed

echo "[MAPPING] ZED Container is running"

docker-compose exec zed bash -c "
    export ROS_DISTRO=humble && \
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \

    source /opt/ros/\$ROS_DISTRO/setup.bash && \
    source /home/spot-teleop/zed_ws/install/setup.bash && \

    ros2 launch nvblox_examples_bringup nvblox_isaac_zed.launch.py
    "