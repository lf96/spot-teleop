#!/usr/bin/env bash
set -e

SERVICE_NAME="$1"
TIMEOUT="${2:-180}"   # segundos
INTERVAL=2

if [ -z "$SERVICE_NAME" ]; then
  echo "[wait_for_service] ERROR: service name not provided"
  echo "Usage: wait_for_service.sh <service_name> [timeout_s]"
  exit 1
fi

echo "[wait_for_service] Waiting for docker-compose service '$SERVICE_NAME' (timeout=${TIMEOUT}s)..."

START_TIME=$(date +%s)

while true; do
  # pega container ID do serviço
  CONTAINER_ID=$(docker compose ps -q "$SERVICE_NAME")

  if [ -n "$CONTAINER_ID" ]; then
    STATE=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_ID")
    HEALTH=$(docker inspect -f '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' "$CONTAINER_ID")

    if [ "$STATE" == "running" ]; then
      if [ "$HEALTH" == "healthy" ] || [ "$HEALTH" == "none" ]; then
        echo "[wait_for_service] Service '$SERVICE_NAME' is ready"
        echo "  container=$CONTAINER_ID state=$STATE health=$HEALTH"
        exit 0
      fi
    fi
  fi

  NOW=$(date +%s)
  ELAPSED=$((NOW - START_TIME))

  if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
    echo "[wait_for_service] ERROR: Timeout waiting for service '$SERVICE_NAME'"
    if [ -n "$CONTAINER_ID" ]; then
      docker logs --tail 50 "$CONTAINER_ID" || true
    fi
    exit 1
  fi

  sleep $INTERVAL
done
