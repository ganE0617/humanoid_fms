#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

HOST="${FMS_HOST:-127.0.0.1}"
PORT="${FMS_PORT:-8787}"

if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
  CYCLONE_INTERFACE="${FMS_CYCLONEDDS_INTERFACE:-}"
  if [[ -z "$CYCLONE_INTERFACE" ]] && command -v ip >/dev/null 2>&1; then
    for candidate in eno1 eth0; do
      if ip -o link show "$candidate" >/dev/null 2>&1 \
        && ip -o link show "$candidate" | grep -q "state UP" \
        && ip -o -4 addr show "$candidate" | grep -q " inet "; then
        CYCLONE_INTERFACE="$candidate"
        break
      fi
    done
  fi
  if [[ -z "$CYCLONE_INTERFACE" ]] && command -v ip >/dev/null 2>&1; then
    CYCLONE_INTERFACE="$(ip -o -4 route get 224.0.0.1 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "dev") {print $(i+1); exit}}')"
  fi
  if [[ -n "$CYCLONE_INTERFACE" ]]; then
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${CYCLONE_INTERFACE}</NetworkInterfaceAddress><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>"
  else
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>"
  fi
fi

echo "Humanoid FMS"
echo "URL: http://${HOST}:${PORT}"
echo "Scope: local-only"
echo "CycloneDDS: ${FMS_CYCLONEDDS_INTERFACE:-auto}"

exec python3 -m uvicorn fms_server:app --host "$HOST" --port "$PORT"
