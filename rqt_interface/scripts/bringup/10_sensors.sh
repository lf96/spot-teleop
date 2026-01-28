# 2 - ZED camera start
#!/usr/bin/env bash

echo "=============================="
echo "[SENSORS] Bringing up ZED camera"
echo "=============================="

docker-compose up -d zed

echo "[SENSORS] Waiting for ZED container..."
scripts/utils/wait_for_container.sh zed

echo "[SENSORS] ZED Container is running"

docker-compose exec zed bash -c "
    export ROS_DISTRO=humble && \
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \

    source /opt/ros/\$ROS_DISTRO/setup.bash && \
    source /home/spot-teleop/zed_ws/install/setup.bash && \

    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx sim_mode:=true use_sim_time:=true
    "