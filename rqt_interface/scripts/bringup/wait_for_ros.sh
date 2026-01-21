until ros2 node list | grep -q robot_state_publisher; do
  sleep 1
done
