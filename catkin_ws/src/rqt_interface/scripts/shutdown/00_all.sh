## Kill all services
#!/usr/bin/env bash
set -e

echo "=============================="
echo "[SHUTDOWN] Shutting down all services"
echo "=============================="

docker-compose kill isaac-sim zed spot-ros2