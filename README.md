# Humanoid FMS

Local-only fleet/teleop monitoring page for the Spacebank AGX Orin.

The first target robot is Unitree. AI Worker support is isolated behind robot
configuration files so the topic map, URDF path, and camera map can be swapped
without redesigning the UI.

## Run On Orin With Docker

```bash
cd /home/spacebank/humanoid_fms
./scripts/sync_robot_descriptions.sh
docker compose up --build -d
```

Open:

```text
http://127.0.0.1:8787
```

If you are accessing from the Mac over SSH:

```bash
ssh -L 8787:127.0.0.1:8787 spacebank@100.70.9.77
```

Then open `http://127.0.0.1:8787` on the Mac.

For host-only debugging without Docker:

```bash
./scripts/run_local.sh
```

## Camera Slots

The 2x2 camera grid is fixed so the fourth camera can be added without a layout
change.

| Slot | Default alias |
| --- | --- |
| 1 | `/dev/logitech1_camera` |
| 2 | `/dev/logitech2_camera` |
| 3 | `/dev/realsense_camera` |
| 4 | `/dev/insta360_camera` |

Run a read-only camera check:

```bash
./scripts/check_cameras.sh
```

## Safety Boundaries

- This app does not stop, remove, restart, or mutate existing containers.
- Docker data is read through the Docker socket API or `docker ps`/`inspect`.
- Camera ownership is reported through `fuser`, `lsof`, and container bind
  inspection.
- Teleop controls are UI-gated and do not publish robot commands yet.

## Robot Descriptions

Robot URDF assets are intentionally cloned into `vendor/` instead of copied into
this repo:

```bash
./scripts/sync_robot_descriptions.sh
```

Default URDF targets:

- Unitree: `vendor/unitree_ros/robots/go2_description/urdf/go2_description.urdf`
- AI Worker: `vendor/ai_worker/ffw_description/urdf/ffw_bg2_rev4_follower/ffw_bg2_follower.urdf`

The web scene parses the URDF joint tree and applies live `/joint_states` data
when ROS 2 is visible inside the container. If no ROS messages have arrived yet,
the scene stays in a clearly marked waiting/demo preview state.
