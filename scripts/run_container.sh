#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
./scripts/sync_robot_descriptions.sh
mkdir -p logs
docker compose up --build -d
docker compose ps

