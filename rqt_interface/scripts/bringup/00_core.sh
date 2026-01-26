## Start bringup for Spot robot in Docker container
# 1 - ISAAC Sim start
#!/usr/bin/env bash
set -e

echo "=============================="
echo "[CORE] Bringing up Isaac Sim"
echo "=============================="

docker-compose up -d isaac-sim

echo "[CORE] Waiting for Isaac container..."
rqt_interface/scripts/utils/wait_for_container.sh isaac-sim

echo "[CORE] Isaac Container is running"

docker-compose exec isaac-sim bash -c "
    export ROS_DISTRO=humble && \
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
    export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib && \

    cd /isaac-sim && \
    ./kit/kit \
        ./apps/isaacsim.exp.full.kit \
        --headless \
        --no-window \
        --allow-root \
        --ext-folder /workspace/zed-isaac-sim/exts \
        --enable_cameras \
        --enable usd.autoplay \

"

echo "[CORE] Isaac Sim bringup complete"