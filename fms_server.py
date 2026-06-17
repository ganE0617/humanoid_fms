from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

import cv2
from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles


ROOT = Path(__file__).resolve().parent
CONFIG_DIR = ROOT / "config"
WEB_DIR = ROOT / "web"

app = FastAPI(title="Humanoid FMS", version="0.1.0")
_STATUS_CACHE: dict[str, Any] = {"time": 0.0, "data": None}


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as fp:
        return json.load(fp)


def run(cmd: list[str], timeout: float = 3.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, text=True, capture_output=True, timeout=timeout, check=False)


def sh(command: str, timeout: float = 3.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, shell=True, text=True, capture_output=True, timeout=timeout, check=False)


def camera_config() -> list[dict[str, Any]]:
    return load_json(CONFIG_DIR / "cameras.json")


def robot_configs() -> dict[str, dict[str, Any]]:
    robots = {}
    for path in sorted((CONFIG_DIR / "robots").glob("*.json")):
        data = load_json(path)
        robots[data["id"]] = data
    return robots


def resolve_device(path: str) -> str | None:
    device = Path(path)
    if not device.exists():
        return None
    try:
        return str(device.resolve())
    except OSError:
        return path


def capture_source(path: str) -> str | int:
    resolved = resolve_device(path) or path
    name = Path(resolved).name
    if name.startswith("video") and name[5:].isdigit():
        return int(name[5:])
    return resolved


def v4l2_summary(device: str) -> dict[str, str]:
    if not Path(device).exists():
        return {}
    proc = run(["v4l2-ctl", "-d", device, "--info"], timeout=2)
    summary: dict[str, str] = {}
    for line in proc.stdout.splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            key = key.strip().lower().replace(" ", "_")
            if key in {"driver_name", "card_type", "bus_info", "driver_version"}:
                summary[key] = value.strip()
    return summary


def process_users(device: str) -> list[dict[str, str]]:
    if not Path(device).exists():
        return []
    users: list[dict[str, str]] = []
    proc = sh(f"fuser -v {device} 2>&1 || true", timeout=2)
    for line in proc.stdout.splitlines():
        if line.strip().startswith("/"):
            continue
        parts = line.split()
        if len(parts) >= 3 and parts[0].isdigit():
            users.append({"pid": parts[0], "user": parts[1], "access": parts[2], "command": " ".join(parts[3:])})
    lsof = sh(f"lsof {device} 2>/dev/null || true", timeout=2)
    for line in lsof.stdout.splitlines()[1:]:
        parts = line.split()
        if len(parts) >= 2 and not any(item.get("pid") == parts[1] for item in users):
            users.append({"pid": parts[1], "user": parts[2] if len(parts) > 2 else "", "access": "", "command": parts[0]})
    return users


def docker_containers() -> list[dict[str, Any]]:
    proc = sh("docker ps --format '{{json .}}' 2>/dev/null || true", timeout=4)
    containers = []
    for line in proc.stdout.splitlines():
        if not line.strip():
            continue
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        containers.append(
            {
                "id": item.get("ID", ""),
                "name": item.get("Names", ""),
                "status": item.get("Status", ""),
                "ports": item.get("Ports", ""),
            }
        )
    return containers


def docker_device_access() -> list[dict[str, Any]]:
    script = r"""
docker ps -q 2>/dev/null | while read id; do
  name=$(docker inspect -f '{{.Name}}' "$id" | sed 's#^/##')
  devices=$(docker inspect -f '{{json .HostConfig.Devices}}' "$id" 2>/dev/null)
  binds=$(docker inspect -f '{{json .HostConfig.Binds}}' "$id" 2>/dev/null)
  if printf '%s %s\n' "$devices" "$binds" | grep -qE '/dev/video|/dev/logitech|/dev/realsense|/dev/v4l|/dev/bus/usb|/dev'; then
    printf '%s\t%s\t%s\t%s\n' "$id" "$name" "$devices" "$binds"
  fi
done
"""
    proc = sh(script, timeout=5)
    access = []
    for line in proc.stdout.splitlines():
        parts = line.split("\t", 3)
        if len(parts) == 4:
            access.append({"id": parts[0], "name": parts[1], "devices": parts[2], "binds": parts[3]})
    return access


def ros_topics() -> dict[str, Any]:
    candidates = [
        "[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash >/dev/null 2>&1 && command -v ros2 >/dev/null && timeout 1.2 ros2 topic list -t",
        "[ -f /opt/ros/foxy/setup.bash ] && source /opt/ros/foxy/setup.bash >/dev/null 2>&1 && command -v ros2 >/dev/null && timeout 1.2 ros2 topic list -t",
        "command -v ros2 >/dev/null && timeout 1.2 ros2 topic list -t",
    ]
    for command in candidates:
        proc = sh(f"bash -lc {json.dumps(command)}", timeout=2)
        if proc.returncode == 0 and proc.stdout.strip():
            topics = []
            for line in proc.stdout.splitlines():
                line = line.strip()
                if not line:
                    continue
                if " [" in line and line.endswith("]"):
                    name, typ = line.rsplit(" [", 1)
                    topics.append({"name": name, "type": typ[:-1]})
                else:
                    topics.append({"name": line, "type": ""})
            return {"available": True, "topics": topics[:80], "count": len(topics)}
    return {"available": False, "topics": [], "count": 0}


def camera_status() -> list[dict[str, Any]]:
    status = []
    for cam in camera_config():
        device = cam["device"]
        resolved = resolve_device(device)
        users = process_users(device)
        status.append(
            {
                **cam,
                "exists": Path(device).exists(),
                "resolved": resolved,
                "busy": len(users) > 0,
                "users": users,
                "v4l2": v4l2_summary(device),
                "stream": f"/stream/{cam['id']}",
            }
        )
    return status


@app.get("/api/config")
def get_config() -> JSONResponse:
    return JSONResponse(
        {
            "cameras": camera_config(),
            "robots": robot_configs(),
            "defaultRobot": "unitree",
            "runtime": {"hostOnly": True, "port": int(os.getenv("FMS_PORT", "8787"))},
        }
    )


@app.get("/api/status")
def get_status() -> JSONResponse:
    now = time.time()
    if _STATUS_CACHE["data"] is not None and now - float(_STATUS_CACHE["time"]) < 8.0:
        return JSONResponse(_STATUS_CACHE["data"])
    data = {
        "time": now,
        "cameras": camera_status(),
        "docker": {
            "containers": docker_containers(),
            "deviceAccess": docker_device_access(),
        },
        "ros": ros_topics(),
    }
    _STATUS_CACHE.update({"time": now, "data": data})
    return JSONResponse(data)


def open_capture(cam: dict[str, Any]):
    candidates = cam.get("captureCandidates") or [cam["device"]]
    width, height = cam.get("resolution", [640, 360])
    fps = max(1, int(cam.get("fps", 15)))
    attempted = []
    for candidate in candidates:
        if not Path(candidate).exists():
            attempted.append(f"{candidate}:missing")
            continue
        device = capture_source(candidate)
        attempted.append(str(device))
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        if cap.isOpened():
            return cap
        cap.release()
    raise HTTPException(status_code=503, detail=f"camera unavailable: {cam['device']} attempted {attempted}")


def mjpeg_frames(cam: dict[str, Any], cap):
    fps = max(1, int(cam.get("fps", 15)))
    delay = 1.0 / fps
    lock = threading.Lock()
    try:
        while True:
            with lock:
                ok, frame = cap.read()
            if not ok:
                time.sleep(delay)
                continue
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
            if not ok:
                continue
            yield b"--frame\r\nContent-Type: image/jpeg\r\nCache-Control: no-store\r\n\r\n" + encoded.tobytes() + b"\r\n"
            time.sleep(delay)
    finally:
        cap.release()


@app.get("/stream/{camera_id}")
def stream_camera(camera_id: str) -> StreamingResponse:
    cams = {cam["id"]: cam for cam in camera_config()}
    if camera_id not in cams:
        raise HTTPException(status_code=404, detail="unknown camera")
    cam = cams[camera_id]
    if not Path(cam["device"]).exists():
        raise HTTPException(status_code=404, detail=f"camera device missing: {cam['device']}")
    cap = open_capture(cam)
    return StreamingResponse(mjpeg_frames(cam, cap), media_type="multipart/x-mixed-replace; boundary=frame")


app.mount("/", StaticFiles(directory=WEB_DIR, html=True), name="web")
