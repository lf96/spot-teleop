#!/usr/bin/env bash
set -e

CONTAINER_NAME="$1"
TIMEOUT="${2:-120}"   # segundos (default: 120s)
INTERVAL=2

if [ -z "$CONTAINER_NAME" ]; then
  echo "[wait_for_container] ERROR: container name not provided"
  echo "Usage: wait_for_container.sh <container_name> [timeout_s]"
  exit 1
fi

echo "[wait_for_container] Waiting for container '$CONTAINER_NAME' (timeout=${TIMEOUT}s)..."

START_TIME=$(date +%s)

while true; do
  # Verifica se container existe
  if ! docker inspect "$CONTAINER_NAME" &>/dev/null; then
    sleep $INTERVAL
    continue
  fi

  STATE=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME")
  HEALTH=$(docker inspect -f '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' "$CONTAINER_NAME")

  if [ "$STATE" == "running" ]; then
    if [ "$HEALTH" == "healthy" ] || [ "$HEALTH" == "none" ]; then
      echo "[wait_for_container] Container '$CONTAINER_NAME' is ready (state=$STATE, health=$HEALTH)"
      exit 0
    fi
  fi

  NOW=$(date +%s)
  ELAPSED=$((NOW - START_TIME))

  if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
    echo "[wait_for_container] ERROR: Timeout waiting for container '$CONTAINER_NAME'"
    echo "State=$STATE Health=$HEALTH"
    docker logs --tail 50 "$CONTAINER_NAME" || true
    exit 1
  fi

  sleep $INTERVAL
done
