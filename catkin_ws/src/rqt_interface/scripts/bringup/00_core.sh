## Start bringup for Spot robot in Docker container
# 1 - ISAAC Sim start
#!/usr/bin/env bash
set -e

echo "=============================="
echo "[CORE] Bringing up Isaac Sim"
echo "=============================="

docker-compose up -d isaac-sim

echo "[CORE] Waiting for Isaac container..."
/home/nexus/spot-teleop/catkin_ws/src/rqt_interface/scripts/utils/wait_for_container.sh isaac-sim

echo "[CORE] Isaac Container is running"

READY_FILE="/tmp/isaac_core_ready"
COMPLETE_FILE="/tmp/isaac_core_complete"

docker-compose exec isaac-sim rm -f "$READY_FILE" || true
rm -f "$COMPLETE_FILE" || true

docker-compose exec -d isaac-sim bash -c "
    export ROS_DISTRO=humble && \
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
    export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib && \

    cd /isaac-sim && \
    ./kit/kit \
        ./apps/isaacsim.exp.full.kit \
        --headless \
        --no-window \
        --allow-root \
        --ext-folder /workspace/scripts/exts \
        --enable_cameras \
        --enable usd.autoplay \
        --exts/”omni.kit.widget.cache_indicator”/check_updates = false \

"  &

ISAAC_PID=$!

echo "[CORE] Isaac Sim PID: $ISAAC_PID"

echo "[CORE] Waiting for Isaac Sim readiness signal..."

while ! docker-compose exec -T isaac-sim test -f "$READY_FILE"; do
    sleep 1
done

echo "[CORE] Isaac Sim bringup complete"

echo "[CORE] Finished CORE bringup"

touch "$COMPLETE_FILE"

docker-compose wait isaac-sim
