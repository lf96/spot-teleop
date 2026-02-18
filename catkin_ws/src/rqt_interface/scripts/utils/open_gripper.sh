# Open Gripper
#!/usr/bin/env bash

docker-compose exec -d spot-ros2 bash -c "

    source /opt/ros/humble/setup.bash && \
    source /home/spot-teleop/spot-ros2_ws/install/setup.bash && \

    ros2 topic pub --once /gripper/command std_msgs/Float64 'data: -1.57' 
    "