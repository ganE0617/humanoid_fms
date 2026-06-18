from __future__ import annotations

import json
import os
import socket
import subprocess
import threading
import time
import http.client
import importlib
from pathlib import Path
from typing import Any
from urllib.parse import quote

import cv2
import numpy as np
from fastapi import Body, FastAPI, HTTPException
from fastapi.responses import JSONResponse, Response, StreamingResponse
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


class Z16DepthCapture:
    def __init__(self, device: str, width: int, height: int):
        self.device = str(resolve_device(device) or device)
        self.width = width
        self.height = height
        self.frame_path = Path(f"/tmp/humanoid-fms-depth-{os.getpid()}-{abs(hash((self.device, width, height))) % 100000}.z16")

    def read(self):
        command = [
            "v4l2-ctl",
            "-d",
            self.device,
            f"--set-fmt-video=width={self.width},height={self.height},pixelformat=Z16 ",
            "--stream-mmap",
            "--stream-count=1",
            f"--stream-to={self.frame_path}",
        ]
        proc = run(command, timeout=3.0)
        if proc.returncode != 0 or not self.frame_path.exists():
            return False, None
        raw = self.frame_path.read_bytes()
        expected = self.width * self.height * 2
        if len(raw) < expected:
            return False, None
        frame = np.frombuffer(raw[:expected], dtype=np.uint16).reshape((self.height, self.width))
        return True, frame.copy()

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


def depth_points_from_z16(frame: np.ndarray, config: dict[str, Any]) -> tuple[list[list[float]], float, dict[str, Any]]:
    height, width = frame.shape[:2]
    max_points = max(100, min(80000, int(config.get("maxPoints", 8000))))
    scale = float(config.get("depthScaleMeters", 0.001))
    near = float(config.get("nearMeters", 0.18))
    far = float(config.get("farMeters", 3.0))
    point_far = min(far, float(config.get("focusFarMeters", config.get("pointFarMeters", far))))
    hfov = np.deg2rad(float(config.get("horizontalFovDeg", 87)))
    vfov = np.deg2rad(float(config.get("verticalFovDeg", 58)))
    fx = width / (2.0 * np.tan(hfov / 2.0))
    fy = height / (2.0 * np.tan(vfov / 2.0))
    cx = (width - 1) / 2.0
    cy = (height - 1) / 2.0

    depth_m = frame.astype(np.float32) * scale
    valid = (frame > 0) & (frame < 65535) & (depth_m >= near) & (depth_m <= point_far)
    vs, us = np.nonzero(valid)
    count = int(vs.size)
    if count > max_points:
        keep = np.linspace(0, count - 1, max_points, dtype=np.int64)
        vs = vs[keep]
        us = us[keep]

    z = depth_m[vs, us].astype(np.float32) if vs.size else np.array([], dtype=np.float32)
    x = ((us.astype(np.float32) - cx) * z / fx) if vs.size else z
    y = ((vs.astype(np.float32) - cy) * z / fy) if vs.size else z
    distances = np.sqrt(x * x + y * y + z * z) if vs.size else np.array([], dtype=np.float32)
    nearest = float(np.min(distances)) if distances.size else 0.0
    points = np.column_stack((x, y, z)).round(4).astype(float).tolist() if vs.size else []

    step = max(2, min(32, int(config.get("surfaceStep", 8))))
    rows = int(np.ceil(height / step))
    cols = int(np.ceil(width / step))
    surface_points: list[list[float] | None] = []
    for row in range(rows):
        v0 = row * step
        v1 = min(height, v0 + step)
        for col in range(cols):
            u0 = col * step
            u1 = min(width, u0 + step)
            patch_depth = depth_m[v0:v1, u0:u1]
            patch_valid = valid[v0:v1, u0:u1]
            if not np.any(patch_valid):
                surface_points.append(None)
                continue
            zc = float(np.median(patch_depth[patch_valid]))
            uc = u0 + (u1 - u0 - 1) * 0.5
            vc = v0 + (v1 - v0 - 1) * 0.5
            xc = (uc - cx) * zc / fx
            yc = (vc - cy) * zc / fy
            surface_points.append([round(float(xc), 4), round(float(yc), 4), round(float(zc), 4)])

    surface = {
        "width": cols,
        "height": rows,
        "points": surface_points,
        "maxDepthDelta": float(config.get("surfaceMaxDepthDelta", 0.18)),
    }
    return points, nearest, surface


def depth_preview_from_z16(frame: np.ndarray, config: dict[str, Any]) -> tuple[bytes, dict[str, Any]]:
    scale = float(config.get("depthScaleMeters", 0.001))
    near = float(config.get("nearMeters", 0.18))
    far = float(config.get("farMeters", 3.0))
    depth_m = frame.astype(np.float32) * scale
    valid = (frame > 0) & (frame < 65535) & (depth_m >= near) & (depth_m <= far)
    clipped = np.clip(depth_m, near, far)
    inverse = ((far - clipped) / max(0.001, far - near) * 255.0).astype(np.uint8)
    color = cv2.applyColorMap(inverse, cv2.COLORMAP_TURBO)
    color[~valid] = (13, 18, 24)

    height, width = frame.shape[:2]
    cx = width // 2
    cy = height // 2
    roi = max(12, min(width, height) // 14)
    patch = depth_m[max(0, cy - roi):min(height, cy + roi), max(0, cx - roi):min(width, cx + roi)]
    patch_valid = valid[max(0, cy - roi):min(height, cy + roi), max(0, cx - roi):min(width, cx + roi)]
    center = float(np.median(patch[patch_valid])) if np.any(patch_valid) else 0.0
    valid_depths = depth_m[valid]
    nearest = float(np.min(valid_depths)) if valid_depths.size else 0.0
    median = float(np.median(valid_depths)) if valid_depths.size else 0.0
    coverage = float(np.count_nonzero(valid)) / float(frame.size or 1)

    preview_width = int(config.get("previewWidth", 360))
    preview_height = max(1, round(preview_width * height / width))
    preview = cv2.resize(color, (preview_width, preview_height), interpolation=cv2.INTER_AREA)
    sx = preview_width / width
    sy = preview_height / height
    pcx = int(cx * sx)
    pcy = int(cy * sy)
    proi = int(roi * sx)
    cv2.line(preview, (pcx - 14, pcy), (pcx + 14, pcy), (245, 250, 255), 1, cv2.LINE_AA)
    cv2.line(preview, (pcx, pcy - 14), (pcx, pcy + 14), (245, 250, 255), 1, cv2.LINE_AA)
    cv2.rectangle(preview, (pcx - proi, pcy - proi), (pcx + proi, pcy + proi), (245, 250, 255), 1, cv2.LINE_AA)
    ok, encoded = cv2.imencode(".png", preview, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    if not ok:
        return b"", {}
    return encoded.tobytes(), {
        "centerMeters": round(center, 3) if center else 0.0,
        "nearestMeters": round(nearest, 3) if nearest else 0.0,
        "medianMeters": round(median, 3) if median else 0.0,
        "coverage": round(coverage, 3),
        "nearMeters": near,
        "farMeters": far,
    }


def depth_view_from_z16(frame: np.ndarray, config: dict[str, Any]) -> bytes:
    scale = float(config.get("depthScaleMeters", 0.001))
    near = float(config.get("nearMeters", 0.18))
    far = min(float(config.get("farMeters", 3.0)), float(config.get("depthViewFarMeters", 1.4)))
    depth_m = frame.astype(np.float32) * scale
    valid = (frame > 0) & (frame < 65535) & (depth_m >= near) & (depth_m <= far)

    height, width = frame.shape[:2]
    target_aspect = 16 / 9
    crop_height = min(height, int(round(width / target_aspect)))
    crop_y = max(0, (height - crop_height) // 2)
    depth_crop = depth_m[crop_y:crop_y + crop_height, :]
    valid_crop = valid[crop_y:crop_y + crop_height, :]

    clipped = np.clip(depth_crop, near, far)
    inverse = ((far - clipped) / max(0.001, far - near) * 255.0).astype(np.uint8)
    inverse = cv2.medianBlur(inverse, 3)
    color = cv2.applyColorMap(inverse, cv2.COLORMAP_TURBO)
    color[~valid_crop] = (10, 14, 19)

    target_width, target_height = config.get("depthViewResolution", [640, 360])
    view = cv2.resize(color, (int(target_width), int(target_height)), interpolation=cv2.INTER_CUBIC)
    cv2.putText(view, f"{near:.2f}m", (10, int(target_height) - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (245, 250, 255), 1, cv2.LINE_AA)
    cv2.putText(view, f"{far:.2f}m", (int(target_width) - 58, int(target_height) - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (245, 250, 255), 1, cv2.LINE_AA)
    ok, encoded = cv2.imencode(".jpg", view, [int(cv2.IMWRITE_JPEG_QUALITY), 86])
    return encoded.tobytes() if ok else b""


def depth_assist_overlay_from_z16(frame: np.ndarray, config: dict[str, Any]) -> tuple[bytes, dict[str, Any]]:
    scale = float(config.get("depthScaleMeters", 0.001))
    near = float(config.get("nearMeters", 0.18))
    far = float(config.get("farMeters", 3.0))
    desired = float(config.get("assistDesiredMeters", 0.45))
    tolerance = float(config.get("assistToleranceMeters", 0.12))
    depth_m = frame.astype(np.float32) * scale
    height, width = frame.shape[:2]

    target_aspect = float(config.get("assistAspect", 16 / 9))
    crop_height = min(height, int(round(width / target_aspect)))
    crop_y = max(0, (height - crop_height) // 2)
    crop = depth_m[crop_y:crop_y + crop_height, :]
    crop_raw = frame[crop_y:crop_y + crop_height, :]
    valid = (crop_raw > 0) & (crop_raw < 65535) & (crop >= near) & (crop <= far)
    ch, cw = crop.shape[:2]

    roi_w = int(cw * float(config.get("assistRoiWidth", 0.24)))
    roi_h = int(ch * float(config.get("assistRoiHeight", 0.34)))
    roi_cx = int(cw * float(config.get("assistCenterX", 0.5)))
    roi_cy = int(ch * float(config.get("assistCenterY", 0.52)))
    x0 = max(0, roi_cx - roi_w // 2)
    x1 = min(cw, roi_cx + roi_w // 2)
    y0 = max(0, roi_cy - roi_h // 2)
    y1 = min(ch, roi_cy + roi_h // 2)
    roi_depth = crop[y0:y1, x0:x1]
    roi_valid = valid[y0:y1, x0:x1]
    roi_values = roi_depth[roi_valid]

    target = float(np.median(roi_values)) if roi_values.size else 0.0
    roi_nearest = float(np.min(roi_values)) if roi_values.size else 0.0
    roi_coverage = float(np.count_nonzero(roi_valid)) / float(roi_valid.size or 1)
    all_values = crop[valid]
    nearest = float(np.min(all_values)) if all_values.size else 0.0

    if not target:
        guidance = "NO DEPTH"
        delta = 0.0
    else:
        delta = target - desired
        if target < desired - tolerance:
            guidance = "BACK UP"
        elif target > desired + tolerance:
            guidance = "APPROACH"
        else:
            guidance = "GRASP RANGE"

    overlay = np.zeros((ch, cw, 4), dtype=np.uint8)
    if target:
        band = max(0.06, min(0.22, target * 0.08))
        pad_x = int(roi_w * float(config.get("assistOverlayPadX", 1.25)))
        pad_y = int(roi_h * float(config.get("assistOverlayPadY", 0.95)))
        sx0 = max(0, x0 - pad_x)
        sx1 = min(cw, x1 + pad_x)
        sy0 = max(0, y0 - pad_y)
        sy1 = min(ch, y1 + pad_y)
        scope = np.zeros_like(valid, dtype=bool)
        scope[sy0:sy1, sx0:sx1] = True
        target_surface = scope & valid & (np.abs(crop - target) <= band)
        too_close = roi_valid & (roi_depth < max(near, target - band * 1.8))
        overlay[target_surface] = (255, 226, 105, 105)
        roi_overlay = overlay[y0:y1, x0:x1]
        roi_overlay[too_close] = (72, 88, 255, 72)

    cv2.rectangle(overlay, (x0, y0), (x1, y1), (255, 255, 255, 235), 2, cv2.LINE_AA)
    cv2.line(overlay, (roi_cx - 22, roi_cy), (roi_cx + 22, roi_cy), (255, 255, 255, 235), 2, cv2.LINE_AA)
    cv2.line(overlay, (roi_cx, roi_cy - 22), (roi_cx, roi_cy + 22), (255, 255, 255, 235), 2, cv2.LINE_AA)
    cv2.circle(overlay, (roi_cx, roi_cy), 5, (255, 255, 255, 235), 1, cv2.LINE_AA)

    target_width, target_height = config.get("assistOverlayResolution", [640, 360])
    overlay = cv2.resize(overlay, (int(target_width), int(target_height)), interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(".png", overlay, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    if not ok:
        return b"", {}
    return encoded.tobytes(), {
        "targetMeters": round(target, 3) if target else 0.0,
        "targetNearestMeters": round(roi_nearest, 3) if roi_nearest else 0.0,
        "targetCoverage": round(roi_coverage, 3),
        "globalNearestMeters": round(nearest, 3) if nearest else 0.0,
        "desiredMeters": desired,
        "deltaMeters": round(delta, 3) if target else 0.0,
        "guidance": guidance,
        "targetRoi": {
            "x": round(x0 / cw, 4),
            "y": round(y0 / ch, 4),
            "w": round((x1 - x0) / cw, 4),
            "h": round((y1 - y0) / ch, 4),
        },
    }


class RosMonitor:
    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.lock = threading.Lock()
        self.source = "starting"
        self.error = ""
        self.last_joint_time = 0.0
        self.last_tf_time = 0.0
        self.last_depth_time = 0.0
        self.joints: dict[str, float] = {}
        self.transforms: list[dict[str, Any]] = []
        self.depth_frame = ""
        self.depth_points: list[list[float]] = []
        self.depth_surface: dict[str, Any] = {}
        self.depth_preview_png = b""
        self.depth_view_jpg = b""
        self.depth_assist_overlay_png = b""
        self.depth_stats: dict[str, Any] = {}
        self.depth_error = ""
        self.depth_total = 0
        self.depth_nearest = 0.0
        self.depth_source = ""
        self.mission_seq = 0
        self.mission_stage = "idle"
        self.mission_queue: list[dict[str, Any]] = []
        self.mission_events: list[dict[str, Any]] = []
        self.thread = threading.Thread(target=self._run, name="fms-ros-monitor", daemon=True)
        self.depth_thread = threading.Thread(target=self._run_depth_fallback, name="fms-depth-fallback", daemon=True)

    def start(self) -> None:
        self.thread.start()
        self.depth_thread.start()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "source": self.source,
                "error": self.error,
                "robotId": self.robot_id,
                "defaultJoints": robot_configs().get(self.robot_id, {}).get("defaultJointPositions", {}),
                "lastJointTime": self.last_joint_time,
                "lastTfTime": self.last_tf_time,
                "joints": self.joints,
                "transforms": self.transforms[-200:],
            }

    def depth_snapshot(self) -> dict[str, Any]:
        robot = robot_configs().get(self.robot_id, {})
        depth = robot.get("depthSensor", {})
        with self.lock:
            return {
                "source": "live" if self.last_depth_time else "waiting",
                "error": self.depth_error,
                "frameId": self.depth_frame or depth.get("fallbackOpticalFrame", ""),
                "lastDepthTime": self.last_depth_time,
                "points": self.depth_points,
                "surface": self.depth_surface,
                "stats": self.depth_stats,
                "previewUrl": f"/api/depth-preview.png?t={int(self.last_depth_time * 1000)}" if self.depth_preview_png else "",
                "depthViewUrl": f"/api/depth-view.jpg?t={int(self.last_depth_time * 1000)}" if self.depth_view_jpg else "",
                "assistOverlayUrl": f"/api/depth-assist-overlay.png?t={int(self.last_depth_time * 1000)}" if self.depth_assist_overlay_png else "",
                "total": self.depth_total,
                "nearestMeters": self.depth_nearest,
                "dataSource": self.depth_source,
                "config": depth,
            }

    def mission_snapshot(self) -> dict[str, Any]:
        topic = robot_configs().get(self.robot_id, {}).get("topics", {}).get("missionEvents", "/fms/mission_events")
        with self.lock:
            return {
                "stage": self.mission_stage,
                "seq": self.mission_seq,
                "topic": topic,
                "events": self.mission_events[-8:],
            }

    def send_mission_signal(self, signal: str) -> dict[str, Any]:
        normalized = str(signal or "").strip().lower()
        if normalized not in {"start", "complete", "ok", "reset"}:
            raise HTTPException(status_code=400, detail="signal must be start, complete, ok, or reset")
        labels = {
            "start": "START",
            "complete": "COMPLETE",
            "ok": "OK_SIGN",
            "reset": "RESET",
        }
        stages = {
            "start": "started",
            "complete": "completed",
            "ok": "ok",
            "reset": "idle",
        }
        with self.lock:
            self.mission_seq += 1
            self.mission_stage = stages[normalized]
            event = {
                "seq": self.mission_seq,
                "signal": normalized,
                "label": labels[normalized],
                "time": time.time(),
                "robotId": self.robot_id,
            }
            self.mission_events.append(event)
            self.mission_events = self.mission_events[-40:]
            self.mission_queue.append(event)
        return self.mission_snapshot()

    def _run_depth_fallback(self) -> None:
        while True:
            try:
                robot = robot_configs().get(self.robot_id, {})
                depth = robot.get("depthSensor", {})
                device = depth.get("fallbackDevice", "")
                if not device or not Path(device).exists():
                    time.sleep(1.0)
                    continue
                with self.lock:
                    recent_ros_points = self.depth_source == "ros" and self.last_depth_time and time.time() - self.last_depth_time < 1.2
                if recent_ros_points:
                    time.sleep(0.5)
                    continue

                width, height = depth.get("fallbackResolution", [640, 480])
                width = int(width)
                height = int(height)
                capture = Z16DepthCapture(device, width, height)
                ok, frame = capture.read()
                capture.release()
                if not ok or frame is None:
                    with self.lock:
                        if not self.depth_source:
                            self.depth_error = f"depth device unavailable: {device}"
                    time.sleep(1.0)
                    continue

                points, nearest, surface = depth_points_from_z16(frame, depth)
                preview_png, depth_stats = depth_preview_from_z16(frame, depth)
                depth_view_jpg = depth_view_from_z16(frame, depth)
                assist_overlay_png, assist_stats = depth_assist_overlay_from_z16(frame, depth)
                with self.lock:
                    self.depth_points = points
                    self.depth_surface = surface
                    self.depth_preview_png = preview_png
                    self.depth_view_jpg = depth_view_jpg
                    self.depth_assist_overlay_png = assist_overlay_png
                    self.depth_stats = {**depth_stats, **assist_stats}
                    self.depth_frame = depth.get("fallbackOpticalFrame", "camera_depth_optical_frame")
                    self.last_depth_time = time.time()
                    self.depth_total = int(frame.size)
                    self.depth_nearest = round(nearest, 3) if nearest else 0.0
                    self.depth_source = "v4l2-z16"
                    self.depth_error = ""
                time.sleep(1.0 / max(0.5, float(depth.get("fallbackFps", 2))))
            except Exception as exc:
                with self.lock:
                    self.depth_error = f"depth fallback {type(exc).__name__}: {exc}"
                time.sleep(1.0)

    def _run(self) -> None:
        if os.getenv("FMS_ENABLE_ROS", "1") == "0":
            with self.lock:
                self.source = "disabled"
            return
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import JointState
            from tf2_msgs.msg import TFMessage
        except Exception as exc:
            with self.lock:
                self.source = "unavailable"
                self.error = f"{type(exc).__name__}: {exc}"
            return

        point_cloud2 = None
        PointCloud2 = None
        String = None
        try:
            from sensor_msgs.msg import PointCloud2 as RosPointCloud2
            from sensor_msgs_py import point_cloud2 as ros_point_cloud2

            PointCloud2 = RosPointCloud2
            point_cloud2 = ros_point_cloud2
        except Exception as exc:
            with self.lock:
                self.depth_error = f"PointCloud2 unavailable: {type(exc).__name__}: {exc}"
        try:
            from std_msgs.msg import String as RosString

            String = RosString
        except Exception:
            String = None

        robots = robot_configs()
        robot = robots.get(self.robot_id) or robots.get("unitree") or {}
        topics = robot.get("topics", {})
        joint_topic = topics.get("jointStates", "/joint_states")
        tf_topic = topics.get("tf", "/tf")
        tf_static_topic = topics.get("tfStatic", "/tf_static")
        low_state_topic = topics.get("lowState", "")
        low_state_lf_topic = topics.get("lowStateLf", "")
        low_state_joint_names = [name for name in robot.get("lowStateJointNames", []) if name]
        depth_config = robot.get("depthSensor", {})
        point_cloud_topic = depth_config.get("pointCloudTopic") or topics.get("pointCloud", "")
        mission_topic = topics.get("missionEvents", "/fms/mission_events")
        max_depth_points = max(100, min(80000, int(depth_config.get("maxPoints", 2400))))
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        tf_static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        low_state_msg_type = None
        low_state_error = ""
        for type_path in ("unitree_go.msg.LowState", "unitree_go.msg.dds_.LowState_"):
            try:
                module_name, class_name = type_path.rsplit(".", 1)
                low_state_msg_type = getattr(importlib.import_module(module_name), class_name)
                break
            except Exception as exc:
                low_state_error = f"{type(exc).__name__}: {exc}"

        try:
            rclpy.init(args=None)

            class FmsRosNode(Node):
                def __init__(node_self):
                    super().__init__("humanoid_fms_monitor")
                    node_self.create_subscription(JointState, joint_topic, node_self.on_joint, sensor_qos)
                    node_self.create_subscription(TFMessage, tf_topic, node_self.on_tf, sensor_qos)
                    node_self.create_subscription(TFMessage, tf_static_topic, node_self.on_tf, tf_static_qos)
                    if point_cloud_topic and PointCloud2 is not None:
                        node_self.create_subscription(PointCloud2, point_cloud_topic, node_self.on_point_cloud, sensor_qos)
                    node_self.mission_pub = node_self.create_publisher(String, mission_topic, 10) if String else None
                    node_self.create_timer(0.1, node_self.publish_mission_events)
                    if low_state_msg_type and low_state_joint_names:
                        for topic in {low_state_topic, low_state_lf_topic} - {""}:
                            node_self.create_subscription(low_state_msg_type, topic, node_self.on_low_state, sensor_qos)

                def on_joint(node_self, msg):
                    now = time.time()
                    data = {name: float(position) for name, position in zip(msg.name, msg.position)}
                    with self.lock:
                        self.joints.update(data)
                        self.last_joint_time = now
                        self.source = "live"
                        self.error = ""

                def on_low_state(node_self, msg):
                    now = time.time()
                    motor_state = getattr(msg, "motor_state", None) or getattr(msg, "motorState", None) or []
                    data = {}
                    for name, motor in zip(low_state_joint_names, motor_state):
                        q = getattr(motor, "q", None)
                        if q is not None:
                            data[name] = float(q)
                    if not data:
                        return
                    with self.lock:
                        self.joints.update(data)
                        self.last_joint_time = now
                        self.source = "lowstate"
                        self.error = ""

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
                        self.error = ""

                def on_point_cloud(node_self, msg):
                    now = time.time()
                    total = int(msg.width or 0) * int(msg.height or 0)
                    stride = max(1, total // max_depth_points) if total else 1
                    points: list[list[float]] = []
                    nearest = 0.0
                    try:
                        for index, point in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
                            if index % stride:
                                continue
                            x, y, z = float(point[0]), float(point[1]), float(point[2])
                            if not np.isfinite([x, y, z]).all():
                                continue
                            distance = float((x * x + y * y + z * z) ** 0.5)
                            if distance <= 0.05 or distance > 8.0:
                                continue
                            nearest = distance if nearest <= 0.0 else min(nearest, distance)
                            points.append([round(x, 4), round(y, 4), round(z, 4)])
                            if len(points) >= max_depth_points:
                                break
                        with self.lock:
                            self.depth_points = points
                            self.depth_surface = {}
                            self.depth_stats = {"nearestMeters": round(nearest, 3)}
                            self.depth_frame = msg.header.frame_id
                            self.last_depth_time = now
                            self.depth_total = total
                            self.depth_nearest = round(nearest, 3)
                            self.depth_source = "ros"
                            self.depth_error = ""
                    except Exception as exc:
                        with self.lock:
                            self.depth_error = f"{type(exc).__name__}: {exc}"

                def publish_mission_events(node_self):
                    if not node_self.mission_pub:
                        return
                    with self.lock:
                        events = self.mission_queue[:]
                        self.mission_queue.clear()
                    for event in events:
                        msg = String()
                        msg.data = json.dumps(event, separators=(",", ":"))
                        node_self.mission_pub.publish(msg)

            node = FmsRosNode()
            with self.lock:
                self.source = "waiting"
                self.error = ""
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
            "frameAliases": robot.get("urdf", {}).get("frameAliases", {}),
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


@app.get("/api/depth-state")
def get_depth_state() -> JSONResponse:
    if _ROS_MONITOR is None:
        return JSONResponse({"source": "not-started", "points": [], "config": {}})
    return JSONResponse(_ROS_MONITOR.depth_snapshot())


@app.get("/api/depth-preview.png")
def get_depth_preview() -> Response:
    if _ROS_MONITOR is None:
        raise HTTPException(status_code=404, detail="depth monitor not started")
    with _ROS_MONITOR.lock:
        content = _ROS_MONITOR.depth_preview_png
    if not content:
        raise HTTPException(status_code=404, detail="depth preview unavailable")
    return Response(content=content, media_type="image/png", headers={"Cache-Control": "no-store"})


@app.get("/api/depth-view.jpg")
def get_depth_view() -> Response:
    if _ROS_MONITOR is None:
        raise HTTPException(status_code=404, detail="depth monitor not started")
    with _ROS_MONITOR.lock:
        content = _ROS_MONITOR.depth_view_jpg
    if not content:
        raise HTTPException(status_code=404, detail="depth view unavailable")
    return Response(content=content, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/api/depth-assist-overlay.png")
def get_depth_assist_overlay() -> Response:
    if _ROS_MONITOR is None:
        raise HTTPException(status_code=404, detail="depth monitor not started")
    with _ROS_MONITOR.lock:
        content = _ROS_MONITOR.depth_assist_overlay_png
    if not content:
        raise HTTPException(status_code=404, detail="depth assist overlay unavailable")
    return Response(content=content, media_type="image/png", headers={"Cache-Control": "no-store"})


@app.get("/api/mission-state")
def get_mission_state() -> JSONResponse:
    if _ROS_MONITOR is None:
        return JSONResponse({"stage": "not-started", "events": [], "topic": "/fms/mission_events"})
    return JSONResponse(_ROS_MONITOR.mission_snapshot())


@app.post("/api/mission-signal")
def post_mission_signal(payload: dict[str, Any] = Body(default={})) -> JSONResponse:
    if _ROS_MONITOR is None:
        raise HTTPException(status_code=503, detail="ROS monitor not started")
    return JSONResponse(_ROS_MONITOR.send_mission_signal(str(payload.get("signal", ""))))


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
