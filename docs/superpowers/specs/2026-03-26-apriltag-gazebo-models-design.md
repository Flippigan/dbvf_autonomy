# AprilTag Gazebo Models for Precision Landing at WA

**Date:** 2026-03-26
**Status:** Approved
**Scope:** Place dual-scale AprilTag models in Gazebo simulation for future precision landing at waypoint WA (autonomous payload pickup zone)

---

## Context

This is for the VFS DBVF 2025-2026 wildfire response eVTOL competition. The drone must autonomously pick up payloads at waypoint WA (a 20x20 ft zone) during Flight Mission 3. GPS coordinates guide the drone to WA at transit altitude (30 ft AGL), and AprilTags provide precision positioning for the final descent and payload pickup. Full autonomy at FM-3 earns a x3 point multiplier and +50 bonus points.

The simulation camera is 640x480 at 114° HFOV (2.0 radians). This represents a worst-case resolution; the real hardware camera may be higher resolution, making the tags even more effective.

## Design

### Approach: Dual-Scale Tags

Two tag36h11 AprilTag models placed at the center of WA, stacked vertically:

| Property | Primary Tag (ID 0) | Secondary Tag (ID 1) |
|----------|-------------------|---------------------|
| Total model size | 0.6m x 0.6m x 0.001m | 0.15m x 0.15m x 0.001m |
| Detection size (inner 8/10ths) | 0.48m | 0.12m |
| Usable detection range (640x480/114°) | ~6m (~20 ft) | ~1.5m (~5 ft) |
| Min detection range (20px threshold) | ~16 ft | ~4 ft |
| Tag family | tag36h11 | tag36h11 |
| Z position | 0.001m (above ground) | 0.002m (above primary) |

The primary tag provides acquisition and coarse positioning during descent from ~20 ft. The secondary tag provides precision alignment below ~5 ft for payload pickup. GPS handles the approach to WA at transit altitude; tags take over for the precision phase only.

### Camera Math

With 640x480 resolution and 114° HFOV:
- Focal length: ~207.8 pixels
- Pixels/meter at altitude h: 207.8 / h

| Altitude | px/m | Primary (0.6m) pixels | Secondary (0.15m) pixels |
|----------|------|----------------------|-------------------------|
| 6.1m (20 ft) | 34.1 | 20.4 px (detectable) | 5.1 px (too small) |
| 4.6m (15 ft) | 45.2 | 27.1 px (reliable) | 6.8 px (marginal) |
| 3.0m (10 ft) | 69.3 | 41.6 px (good pose) | 10.4 px (detectable) |
| 1.5m (5 ft) | 138.5 | 83.1 px (excellent) | 20.8 px (reliable) |
| 0.5m (1.6 ft) | 415.6 | fills FOV (lost) | 62.3 px (excellent) |

### Gazebo Models

Two new models in `src/ardupilot_gazebo/models/`:

```
Apriltag36_11_00000/
  model.config
  model.sdf
  materials/textures/
    tag36_11_00000.png      # 2048x2048 nearest-neighbor upscaled

Apriltag36_11_00001/
  model.config
  model.sdf
  materials/textures/
    tag36_11_00001.png      # 2048x2048 nearest-neighbor upscaled
```

Each model is a static thin flat box with PBR albedo texture. SDF version 1.9, following existing workspace conventions.

**model.sdf structure (example for primary tag):**
```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="Apriltag36_11_00000">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box><size>0.6 0.6 0.001</size></box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>materials/textures/tag36_11_00000.png</albedo_map>
              <roughness>0.5</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box><size>0.6 0.6 0.001</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

Secondary tag is identical except: name `Apriltag36_11_00001`, size `0.15 0.15 0.001`, and texture `tag36_11_00001.png`.

### World File Placement

In `src/ardupilot_gazebo/worlds/iris_multiuav.sdf`:

```xml
<!-- AprilTag landing target at WA (payload pickup zone) -->
<include>
  <name>apriltag_wa_primary</name>
  <uri>model://Apriltag36_11_00000</uri>
  <pose>WA_X WA_Y 0.001 0 0 0</pose>
</include>

<include>
  <name>apriltag_wa_secondary</name>
  <uri>model://Apriltag36_11_00001</uri>
  <pose>WA_X WA_Y 0.002 0 0 0</pose>
</include>
```

Tags are placed on top of the grass model at a location within the sim. The exact X/Y coordinates are wherever WA is positioned in the sim world. The GPS coordinates of that sim position will be noted for the drone's navigation waypoints.

### Texture Generation

Source images from [AprilRobotics/apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs) `tag36h11/` directory (10x10 pixel PNGs). Upscale to 2048x2048 using nearest-neighbor interpolation only:

```bash
convert tag36_11_00000.png -filter point -resize 2048x2048 tag36_11_00000.png
convert tag36_11_00001.png -filter point -resize 2048x2048 tag36_11_00001.png
```

Never use bilinear or bicubic interpolation — it blurs edges and ruins detection.

### Tag Sizing Rationale

- **Primary 0.6m**: Sized for worst-case camera (640x480/114° HFOV). Yields ~20 pixels at 6.1m (20 ft) — the minimum for reliable detection. GPS handles approach above this altitude. A higher-res real camera only improves this.
- **Secondary 0.15m**: Provides ~20 pixels at 1.5m (5 ft) for precision during payload pickup. Stays fully in the 114° FOV down to ~0.15m altitude. Covers the gap where the primary tag fills/exceeds the FOV below ~0.5m.
- **tag36h11 family**: Industry standard for drone landing. Hamming distance 11 (tolerates 5 bit errors). 587 unique IDs. Supported by apriltag_ros and ArduPilot companion landing.

## Out of Scope

- ROS2 apriltag_ros detection pipeline (future work)
- ArduPilot precision landing parameters (PLND_TYPE=1 companion integration)
- Tag switching logic during descent
- Tags at other waypoints (H, L, WM, F1, F2) — GPS only for those
- apriltag_params.yaml configuration

## Future Integration Path

When the detection pipeline is added later:
1. Bridge camera image via ros_gz_bridge (already configured for iris cameras)
2. Run `apriltag_ros` node subscribed to camera image/camera_info
3. Custom companion node converts detections to MAVLink `LANDING_TARGET` messages
4. ArduPilot `PLND_TYPE=1` consumes landing target for precision descent
5. Tag switching logic: use largest visible tag, fall back to smaller when primary exits FOV
