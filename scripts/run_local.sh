#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

HOST="${FMS_HOST:-127.0.0.1}"
PORT="${FMS_PORT:-8787}"

echo "Humanoid FMS"
echo "URL: http://${HOST}:${PORT}"
echo "Scope: local-only"

exec python3 -m uvicorn fms_server:app --host "$HOST" --port "$PORT"

