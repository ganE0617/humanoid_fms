#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

FMS_HUMANOID_MSGS_PREFIX="${FMS_HUMANOID_MSGS_PREFIX:-/opt/fms_humanoid_msgs}"
if [[ -d "$FMS_HUMANOID_MSGS_PREFIX" ]]; then
  export AMENT_PREFIX_PATH="$FMS_HUMANOID_MSGS_PREFIX:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$FMS_HUMANOID_MSGS_PREFIX/lib:$FMS_HUMANOID_MSGS_PREFIX/lib/python3.12/site-packages/humanoid_msgs:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$FMS_HUMANOID_MSGS_PREFIX/lib/python3.12/site-packages:${PYTHONPATH:-}"
fi

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

if [[ "${FMS_WEB_VIDEO_ENABLED:-1}" != "0" ]] && command -v ros2 >/dev/null 2>&1; then
  WEB_VIDEO_PORT="${FMS_WEB_VIDEO_PORT:-8080}"
  if ros2 pkg executables web_video_server >/dev/null 2>&1; then
    mapfile -t WEB_VIDEO_PIDS < <(pgrep -f '/opt/ros/.*/web_video_server|ros2 run web_video_server web_video_server' || true)
    if (( ${#WEB_VIDEO_PIDS[@]} > 0 )); then
      kill "${WEB_VIDEO_PIDS[@]}" 2>/dev/null || true
      sleep 1
      mapfile -t WEB_VIDEO_PIDS < <(pgrep -f '/opt/ros/.*/web_video_server|ros2 run web_video_server web_video_server' || true)
      if (( ${#WEB_VIDEO_PIDS[@]} > 0 )); then
        kill -KILL "${WEB_VIDEO_PIDS[@]}" 2>/dev/null || true
      fi
    fi
    echo "web_video_server: http://127.0.0.1:${WEB_VIDEO_PORT}"
    (
      while true; do
        {
          echo "[$(date -Is)] starting web_video_server on ${WEB_VIDEO_PORT}"
          ros2 run web_video_server web_video_server \
            --ros-args \
            -r __node:=fms_web_video_server \
            -p port:="${WEB_VIDEO_PORT}" \
            -p address:=0.0.0.0 \
            -p server_threads:="${FMS_WEB_VIDEO_SERVER_THREADS:-8}" \
            -p ros_threads:="${FMS_WEB_VIDEO_ROS_THREADS:-4}" \
            -p default_stream_type:="${FMS_WEB_VIDEO_STREAM_TYPE:-mjpeg}" \
            -p image_transport:=compressed
          rc=$?
          echo "[$(date -Is)] web_video_server exited with code ${rc}; restarting in 1s"
        } >> /tmp/fms_web_video_server.log 2>&1
        sleep 1
      done
    ) &
  else
    echo "web_video_server: package not installed"
  fi
fi

exec python3 -m uvicorn fms_server:app --host "$HOST" --port "$PORT" --no-access-log
