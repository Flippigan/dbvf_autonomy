# Hardmount Camera — Implementation Status

**Status:** Implemented (2026-03-26)

## Motivation

The physical drone will have a hard-mounted bottom-facing camera (no gimbal). The simulation was updated to match the real hardware so that AprilTag detection development and testing is representative of actual flight behavior.

Key implication: drone tilt = camera tilt. The camera is body-fixed, so during approach the drone must be roughly level for reliable AprilTag detection.

## What was done

### New files created
| File | Repo | Branch |
|------|------|--------|
| `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm` | ardupilot_gazebo | ros2 |
| `src/ardupilot_gazebo/models/iris_with_hardmount_camera/model.config` | ardupilot_gazebo | ros2 |
| `src/ardupilot_gazebo/models/iris_with_hardmount_camera/model.sdf` | ardupilot_gazebo | ros2 |
| `src/ardupilot_gz/ardupilot_gz_bringup/config/iris_hardmount_bridge.yaml` | ardupilot_gz | humble |

### Modified files
| File | Change |
|------|--------|
| `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf` | Model URI → `iris_with_hardmount_camera` |
| `src/ardupilot_gz/ardupilot_gz_bringup/launch/robots/iris.launch.py` | Param, model SDF, and bridge config → hardmount variants |

### Unchanged (verified)
- `circle_detector.py` — subscribes to `/camera/image` and `/camera/camera_info` (ROS topic names unchanged)
- `iris.rviz` — displays `/camera/image` (unchanged)
- Original gimbal files preserved for future use

## Lesson learned: Gazebo camera pitch

The camera sensor pose uses **positive** pitch to point downward: `0 0 0 0 1.5708 0`. The original plan specified negative pitch (`-1.5708`) which pointed the camera upward. In Gazebo SDF, positive pitch (+π/2) = downward for camera sensors.

## Rollback

To revert to gimbal mode, change these four references back:
1. `iris.launch.py:89` → `"gazebo-iris-gimbal.parm"`
2. `iris.launch.py:119` → `"iris_with_gimbal", "model.sdf"`
3. `iris.launch.py:144` → `"iris_bridge.yaml"`
4. `iris_runway.sdf:76` → `model://iris_with_gimbal`

## Multi-drone extension (not yet done)

When extending to `iris_multi_uav.launch.py` (5 drones), each drone needs its own bridge config with model-specific Gazebo topic paths (`/world/map/model/iris_900N/link/camera_link/sensor/camera/image`).
