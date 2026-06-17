from __future__ import annotations

import json
import os
import socket
import subprocess
import threading
import time
import http.client
from pathlib import Path
from typing import Any
from urllib.parse import quote

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles


ROOT = Path(__file__).resolve().parent
CONFIG_DIR = ROOT / "config"
WEB_DIR = ROOT / "web"
VENDOR_DIR = ROOT / "vendor"
DOCKER_SOCKET = Path("/var/run/docker.sock")

app = FastAPI(title="Humanoid FMS", version="0.1.0")
_STATUS_CACHE: dict[str, Any] = {"time": 0.0, "data": None}
_ROS_MONITOR: "RosMonitor | None" = None
VENDOR_DIR.mkdir(exist_ok=True)


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


def safe_repo_path(path: str) -> Path:
    candidate = (ROOT / path).resolve()
    if ROOT not in candidate.parents and candidate != ROOT:
        raise HTTPException(status_code=400, detail="path outside app root")
    return candidate


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


class V4L2CtlCapture:
    def __init__(self, device: str, width: int, height: int, pixel_format: str = "YUYV"):
        self.device = str(resolve_device(device) or device)
        self.width = width
        self.height = height
        self.pixel_format = pixel_format
        self.frame_path = Path(f"/tmp/humanoid-fms-{os.getpid()}-{abs(hash((self.device, width, height))) % 100000}.raw")

    def read(self):
        command = [
            "v4l2-ctl",
            "-d",
            self.device,
            f"--set-fmt-video=width={self.width},height={self.height},pixelformat={self.pixel_format}",
            "--stream-mmap",
            "--stream-count=1",
            f"--stream-to={self.frame_path}",
        ]
        proc = run(command, timeout=2.5)
        if proc.returncode != 0 or not self.frame_path.exists():
            return False, None
        raw = self.frame_path.read_bytes()
        expected = self.width * self.height * 2
        if len(raw) < expected:
            return False, None
        frame = np.frombuffer(raw[:expected], dtype=np.uint8).reshape((self.height, self.width, 2))
        code = cv2.COLOR_YUV2BGR_UYVY if self.pixel_format == "UYVY" else cv2.COLOR_YUV2BGR_YUYV
        return True, cv2.cvtColor(frame, code)

    def release(self):
        try:
            self.frame_path.unlink(missing_ok=True)
        except Exception:
            pass


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
            if int(parts[0]) == os.getpid():
                continue
            users.append({"pid": parts[0], "user": parts[1], "access": parts[2], "command": " ".join(parts[3:])})
    lsof = sh(f"lsof {device} 2>/dev/null || true", timeout=2)
    for line in lsof.stdout.splitlines()[1:]:
        parts = line.split()
        if len(parts) >= 2 and parts[1].isdigit() and int(parts[1]) == os.getpid():
            continue
        if len(parts) >= 2 and not any(item.get("pid") == parts[1] for item in users):
            users.append({"pid": parts[1], "user": parts[2] if len(parts) > 2 else "", "access": "", "command": parts[0]})
    return users


def docker_socket_json(path: str) -> Any:
    if not DOCKER_SOCKET.exists():
        raise RuntimeError("docker socket missing")

    class UnixHTTPConnection(http.client.HTTPConnection):
        def connect(self):
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.settimeout(2.5)
            self.sock.connect(str(DOCKER_SOCKET))

    conn = UnixHTTPConnection("docker")
    try:
        conn.request("GET", path)
        response = conn.getresponse()
        if response.status >= 400:
            raise RuntimeError(f"docker socket returned {response.status}")
        return json.loads(response.read().decode("utf-8"))
    finally:
        conn.close()


def docker_containers() -> list[dict[str, Any]]:
    try:
        items = docker_socket_json("/containers/json?all=false")
        return [
            {
                "id": item.get("Id", "")[:12],
                "name": (item.get("Names") or [""])[0].lstrip("/"),
                "status": item.get("Status", ""),
                "ports": ", ".join(
                    f"{port.get('IP', '')}:{port.get('PublicPort', '')}->{port.get('PrivatePort', '')}/{port.get('Type', '')}"
                    for port in item.get("Ports", [])
                    if port.get("PublicPort")
                ),
            }
            for item in items
        ]
    except Exception:
        pass

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
    try:
        containers = docker_socket_json("/containers/json?all=false")
        access = []
        for item in containers:
            cid = item.get("Id", "")
            detail = docker_socket_json(f"/containers/{quote(cid)}/json")
            host_config = detail.get("HostConfig", {})
            devices = json.dumps(host_config.get("Devices"))
            binds = json.dumps(host_config.get("Binds"))
            combined = f"{devices} {binds}"
            if any(token in combined for token in ["/dev/video", "/dev/logitech", "/dev/realsense", "/dev/v4l", "/dev/bus/usb", "/dev"]):
                access.append(
                    {
                        "id": cid[:12],
                        "name": (item.get("Names") or [""])[0].lstrip("/"),
                        "devices": devices,
                        "binds": binds,
                    }
                )
        return access
    except Exception:
        pass

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


class RosMonitor:
    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.lock = threading.Lock()
        self.source = "starting"
        self.error = ""
        self.last_joint_time = 0.0
        self.last_tf_time = 0.0
        self.joints: dict[str, float] = {}
        self.transforms: list[dict[str, Any]] = []
        self.thread = threading.Thread(target=self._run, name="fms-ros-monitor", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "source": self.source,
                "error": self.error,
                "robotId": self.robot_id,
                "lastJointTime": self.last_joint_time,
                "lastTfTime": self.last_tf_time,
                "joints": self.joints,
                "transforms": self.transforms[-200:],
            }

    def _run(self) -> None:
        if os.getenv("FMS_ENABLE_ROS", "1") == "0":
            with self.lock:
                self.source = "disabled"
            return
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
            from tf2_msgs.msg import TFMessage
        except Exception as exc:
            with self.lock:
                self.source = "unavailable"
                self.error = f"{type(exc).__name__}: {exc}"
            return

        robots = robot_configs()
        robot = robots.get(self.robot_id) or robots.get("unitree") or {}
        topics = robot.get("topics", {})
        joint_topic = topics.get("jointStates", "/joint_states")
        tf_topic = topics.get("tf", "/tf")
        tf_static_topic = topics.get("tfStatic", "/tf_static")

        try:
            rclpy.init(args=None)

            class FmsRosNode(Node):
                def __init__(node_self):
                    super().__init__("humanoid_fms_monitor")
                    node_self.create_subscription(JointState, joint_topic, node_self.on_joint, 20)
                    node_self.create_subscription(TFMessage, tf_topic, node_self.on_tf, 20)
                    node_self.create_subscription(TFMessage, tf_static_topic, node_self.on_tf, 10)

                def on_joint(node_self, msg):
                    now = time.time()
                    data = {name: float(position) for name, position in zip(msg.name, msg.position)}
                    with self.lock:
                        self.joints.update(data)
                        self.last_joint_time = now
                        self.source = "live"

                def on_tf(node_self, msg):
                    now = time.time()
                    transforms = []
                    for transform in msg.transforms:
                        transforms.append(
                            {
                                "parent": transform.header.frame_id,
                                "child": transform.child_frame_id,
                                "translation": [
                                    transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z,
                                ],
                                "rotation": [
                                    transform.transform.rotation.x,
                                    transform.transform.rotation.y,
                                    transform.transform.rotation.z,
                                    transform.transform.rotation.w,
                                ],
                            }
                        )
                    with self.lock:
                        self.transforms.extend(transforms)
                        self.transforms = self.transforms[-400:]
                        self.last_tf_time = now
                        if self.source != "live":
                            self.source = "tf"

            node = FmsRosNode()
            with self.lock:
                self.source = "waiting"
            rclpy.spin(node)
        except Exception as exc:
            with self.lock:
                self.source = "error"
                self.error = f"{type(exc).__name__}: {exc}"
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass


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


@app.get("/api/robots/{robot_id}/urdf")
def get_robot_urdf(robot_id: str) -> JSONResponse:
    robots = robot_configs()
    if robot_id not in robots:
        raise HTTPException(status_code=404, detail="unknown robot")
    robot = robots[robot_id]
    urdf_path = robot.get("urdf", {}).get("path", "")
    if not urdf_path:
        raise HTTPException(status_code=404, detail="robot has no URDF path configured")
    path = safe_repo_path(urdf_path)
    if not path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"URDF missing: {urdf_path}. Run ./scripts/sync_robot_descriptions.sh",
        )
    xml = path.read_text(encoding="utf-8", errors="replace")
    return JSONResponse(
        {
            "robotId": robot_id,
            "label": robot.get("label", robot_id),
            "model": robot.get("model", robot.get("label", robot_id)),
            "source": robot.get("urdf", {}).get("source", robot.get("repo", "")),
            "path": urdf_path,
            "rootFrame": robot.get("urdf", {}).get("rootFrame", ""),
            "xml": xml,
        }
    )


@app.get("/api/robots/{robot_id}/assets")
def get_robot_assets(robot_id: str) -> JSONResponse:
    robots = robot_configs()
    if robot_id not in robots:
        raise HTTPException(status_code=404, detail="unknown robot")
    base_path = safe_repo_path(robots[robot_id].get("urdf", {}).get("path", "")).parent
    if not base_path.exists():
        return JSONResponse({"robotId": robot_id, "assets": []})
    assets = []
    for pattern in ("*.urdf", "*.xacro", "*.stl", "*.dae", "*.obj"):
        for path in base_path.rglob(pattern):
            assets.append(str(path.relative_to(ROOT)))
    return JSONResponse({"robotId": robot_id, "assets": sorted(assets)[:400]})


@app.get("/api/robot-state")
def get_robot_state() -> JSONResponse:
    if _ROS_MONITOR is None:
        return JSONResponse({"source": "not-started", "joints": {}, "transforms": []})
    return JSONResponse(_ROS_MONITOR.snapshot())


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
    fallback_devices = []
    for candidate in candidates:
        if not Path(candidate).exists():
            attempted.append(f"{candidate}:missing")
            continue
        device = capture_source(candidate)
        fallback_devices.append(candidate)
        attempted.append(str(device))
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        if cap.isOpened():
            return cap
        cap.release()
    for candidate in fallback_devices:
        for pixel_format in ("YUYV", "UYVY"):
            fallback = V4L2CtlCapture(candidate, width, height, pixel_format)
            ok, frame = fallback.read()
            attempted.append(f"v4l2-ctl:{candidate}:{pixel_format}")
            if ok and frame is not None:
                return fallback
            fallback.release()
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


@app.on_event("startup")
def start_ros_monitor() -> None:
    global _ROS_MONITOR
    robot_id = os.getenv("FMS_ROBOT", "unitree")
    _ROS_MONITOR = RosMonitor(robot_id)
    _ROS_MONITOR.start()


if VENDOR_DIR.exists():
    app.mount("/vendor", StaticFiles(directory=VENDOR_DIR, html=False), name="vendor")

app.mount("/", StaticFiles(directory=WEB_DIR, html=True), name="web")
