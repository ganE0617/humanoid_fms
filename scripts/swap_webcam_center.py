#!/usr/bin/env python3
from pathlib import Path
from datetime import datetime
import re
import shutil

root = Path("/home/robotis/humanoid_2026/humanoid_fms")
app_path = root / "web/app.js"
index_path = root / "web/index.html"

backup = root / "backups" / f"pre-webcam-center-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
backup.mkdir(parents=True, exist_ok=False)

for rel in ["web/index.html", "web/app.js"]:
    shutil.copy2(root / rel, backup / rel.replace("/", "__"))

right_old = '  realsense_right: { slot: "lower-center", order: 3, badgeIndex: 3, size: "secondary" },'
right_new = '  realsense_right: { slot: "lower-right", order: 4, badgeIndex: 4, size: "secondary" },'
logitech_old = '  logitech_c922: { slot: "lower-right", order: 4, badgeIndex: 4, size: "secondary" },'
logitech_new = '  logitech_c922: { slot: "lower-center", order: 3, badgeIndex: 3, size: "secondary" },'

original = app_path.read_text()
app = original.replace("sie:", "size:")
index_original = index_path.read_text()
index = re.sub(r'app\.js\?v=[^"]+', 'app.js?v=20260702-webcam-center', index_original)

if right_old in app and logitech_old in app:
    app = app.replace(right_old, logitech_new, 1).replace(logitech_old, right_new, 1)
    print("swapped right/logitech slots")
elif logitech_new in app and right_new in app:
    pass
else:
    raise SystemExit("camera layout lines not found")

if app != original:
    app_path.write_text(app)

if index != index_original:
    index_path.write_text(index)

if app != original or index != index_original:
    print("PATCHED webcam center")

print("backup", backup)
