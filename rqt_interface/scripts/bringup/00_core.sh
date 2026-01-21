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

docker-compose exec isaac-sim bash -c cd /isaac-sim && ./run_isaac.sh --headless --allow-root --/app/exec=/workspace/scripts/load_and_play.py

echo "[CORE] Isaac Sim bringup complete"

