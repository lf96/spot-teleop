## Start bringup for Spot robot in Docker container
# 1 - ISAAC Sim start
#!/usr/bin/env bash
set -e

echo "=============================="
echo " [GUI] Start GUI Bringup "
echo "=============================="

docker-compose up -d rqt-interface

echo "[GUI] Waiting for RQT container..."
/home/nexus/spot-teleop/catkin_ws/src/rqt_interface/scripts/utils/wait_for_container.sh rqt-interface

echo "[GUI] RQT Container is running"

if docker-compose exec -T rqt-interface bash -lc "source /opt/ros/noetic/setup.bash && rostopic list >/dev/null 2>&1"; then
    echo "[GUI] ROS master is already running, skipping roscore"
else
    echo "[GUI] ROS master not running, starting roscore"
    docker-compose exec -d rqt-interface bash -lc "source /opt/ros/noetic/setup.bash && roscore"
    sleep 5 
fi

echo "[GUI] Starting ROS 1 bridge Container"

docker-compose up -d ros-bridge

echo "[GUI] Waiting for ROS bridge container..."
/home/nexus/spot-teleop/catkin_ws/src/rqt_interface/scripts/utils/wait_for_container.sh ros-bridge

echo "[GUI] ROS bridge Container is running"

docker-compose exec -d rqt-interface bash -lc "source /opt/ros/noetic/setup.bash \
 && source /home/spot-teleop/catkin_ws/devel/setup.bash \
 && roslaunch spot_description spot_launch_description.launch
 "

sleep 5

docker-compose exec -d rqt-interface bash -lc "source /opt/ros/noetic/setup.bash \
 && source /home/spot-teleop/catkin_ws/devel/setup.bash \
 && rosrun rqt_interface rqt_interface
 "

echo "[GUI] Started User Interface"

echo "[GUI] Starting host pipeline runner for GUI"

exec python3 /home/nexus/spot-teleop/catkin_ws/src/rqt_interface/scripts/utils/host_pipeline_runner.py

echo "[GUI] Finished GUI bringup"

