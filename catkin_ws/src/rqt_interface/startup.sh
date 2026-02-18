#!/usr/bin/env bash
set -e

COMPOSE="docker-compose"
STACK_SERVICES=("rqt-interface" "ros-bridge")

HOST_RUNNER_CMD="python3 -u /home/nexus/spot-teleop/catkin_ws/src/rqt_interface/scripts/utils/host_pipeline_runner.py"

echo "=============================="
echo " [GUI] Start GUI Bringup "
echo "=============================="

cleanup() {
    echo ""
    echo "=============================="
    echo " [GUI] Shutdown sequence "
    echo "=============================="

    if [ -n "$HOST_RUNNER_PID" ]; then
        echo "[GUI] Stopping host pipeline runner (PID $HOST_RUNNER_PID)..."
        kill -TERM "$HOST_RUNNER_PID" 2>/dev/null || true
        wait "$HOST_RUNNER_PID" 2>/dev/null || true
    fi

    echo "[GUI] Stopping GUI ROS nodes..."
    $COMPOSE exec -T rqt-interface bash -lc "pkill -f roslaunch" || true
    $COMPOSE exec -T rqt-interface bash -lc "pkill -f rosrun" || true
    $COMPOSE exec -T rqt-interface bash -lc "pkill -f roscore" || true

    echo "[GUI] Stopping containers..."
    for svc in "${STACK_SERVICES[@]}"; do
        $COMPOSE stop $svc || true
    done
    /home/nexus/spot-teleop/catkin_ws/src/rqt_interface/scripts/shutdown/00_all.sh

    echo "[GUI] Shutdown complete"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "[GUI] Starting rqt container..."
$COMPOSE up -d rqt-interface
/home/nexus/spot-teleop/catkin_ws/src/rqt_interface/scripts/utils/wait_for_container.sh rqt-interface

echo "[GUI] Checking ROS master..."
if $COMPOSE exec -T rqt-interface bash -lc "source /opt/ros/noetic/setup.bash && rostopic list >/dev/null 2>&1"; then
    echo "[GUI] ROS master already running"
else
    echo "[GUI] Starting roscore..."
    $COMPOSE exec -d rqt-interface bash -lc "source /opt/ros/noetic/setup.bash && roscore"
    sleep 5
fi

echo "[GUI] Starting ROS bridge..."
$COMPOSE up -d ros-bridge
/home/nexus/spot-teleop/catkin_ws/src/rqt_interface/scripts/utils/wait_for_container.sh ros-bridge

echo "[GUI] Launching description..."
$COMPOSE exec -d rqt-interface bash -lc "
source /opt/ros/noetic/setup.bash &&
source /home/spot-teleop/catkin_ws/devel/setup.bash &&
roslaunch spot_description spot_launch_description.launch
"

echo "[GUI] Launching interactive marker..."
$COMPOSE exec -d rqt-interface bash -lc "
source /opt/ros/noetic/setup.bash &&
source /home/spot-teleop/catkin_ws/devel/setup.bash &&
rosrun rqt_interface ee_marker_node
"

sleep 5

echo "[GUI] Starting host pipeline runner..."

$HOST_RUNNER_CMD > /tmp/host_runner.log 2>&1 &
HOST_RUNNER_PID=$!

echo "[GUI] Host pipeline runner PID: $HOST_RUNNER_PID"

echo "[GUI] Launching RQT interface..."
$COMPOSE exec rqt-interface bash -lc "
source /opt/ros/noetic/setup.bash &&
source /home/spot-teleop/catkin_ws/devel/setup.bash &&
rosrun rqt_interface rqt_interface > /dev/null 2>&1
"

cleanup
