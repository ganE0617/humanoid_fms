#!/usr/bin/env bash
set -euo pipefail

echo "--- stable camera aliases ---"
ls -l /dev/logitech1_camera /dev/logitech2_camera /dev/realsense_camera /dev/insta360_camera 2>/dev/null || true

echo
echo "--- video devices ---"
ls -l /dev/video* 2>/dev/null || true

echo
echo "--- device users ---"
for dev in /dev/logitech1_camera /dev/logitech2_camera /dev/realsense_camera /dev/insta360_camera /dev/video*; do
  [ -e "$dev" ] || continue
  echo "### $dev"
  fuser -v "$dev" 2>&1 || true
  lsof "$dev" 2>/dev/null || true
done

echo
echo "--- containers with broad device access ---"
docker ps -q 2>/dev/null | while read -r id; do
  name="$(docker inspect -f '{{.Name}}' "$id" | sed 's#^/##')"
  devices="$(docker inspect -f '{{json .HostConfig.Devices}}' "$id" 2>/dev/null)"
  binds="$(docker inspect -f '{{json .HostConfig.Binds}}' "$id" 2>/dev/null)"
  if printf '%s %s\n' "$devices" "$binds" | grep -qE '/dev/video|/dev/logitech|/dev/realsense|/dev/v4l|/dev/bus/usb|/dev'; then
    echo "### $name ($id)"
    echo "devices=$devices"
    echo "binds=$binds"
  fi
done

