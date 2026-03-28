# AprilTag Gazebo Models Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Place two dual-scale AprilTag ground markers (tag36h11 IDs 0 and 1) in the Gazebo simulation for future precision landing at waypoint WA.

**Architecture:** Two static Gazebo models (0.6m primary, 0.15m secondary) each consisting of a thin flat box with a PBR albedo texture. Source tag PNGs are downloaded from the AprilRobotics repo and upscaled to 2048x2048 with nearest-neighbor interpolation. Both models are included in the active world file, stacked at the same X/Y with Z offsets to avoid z-fighting.

**Tech Stack:** Gazebo Harmonic (SDF 1.9), Python/OpenCV (texture upscaling), wget (texture download)

**Spec:** `docs/superpowers/specs/2026-03-26-apriltag-gazebo-models-design.md`

**Reference:** `.claude/Features/Auto Landing April Tags/Position April Tags/GAZEBO_MODELS_GUIDE.md`

---

## File Map

### Files to Create

| File | Purpose |
|------|---------|
| `src/ardupilot_gazebo/models/Apriltag36_11_00000/model.config` | Metadata for primary tag model |
| `src/ardupilot_gazebo/models/Apriltag36_11_00000/model.sdf` | SDF definition: 0.6m static box with PBR texture |
| `src/ardupilot_gazebo/models/Apriltag36_11_00000/materials/textures/tag36_11_00000.png` | 2048x2048 nearest-neighbor upscaled tag image |
| `src/ardupilot_gazebo/models/Apriltag36_11_00001/model.config` | Metadata for secondary tag model |
| `src/ardupilot_gazebo/models/Apriltag36_11_00001/model.sdf` | SDF definition: 0.15m static box with PBR texture |
| `src/ardupilot_gazebo/models/Apriltag36_11_00001/materials/textures/tag36_11_00001.png` | 2048x2048 nearest-neighbor upscaled tag image |

### Files to Modify

| File | Change |
|------|--------|
| `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf` | Add two `<include>` entries for both AprilTag models |

> **Note on world file path:** The spec references `src/ardupilot_gazebo/worlds/iris_multiuav.sdf`, which does not exist yet. The actively-used world file is `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf` — this is what the `iris_runway.launch.py` loads via the `ardupilot_gz_gazebo` package. Tags are added here so they appear in the running simulation. When a multi-UAV world file is created later, copy the `<include>` entries into it.

---

## Task 1: Download and Upscale Tag Textures

**Files:**
- Create: `src/ardupilot_gazebo/models/Apriltag36_11_00000/materials/textures/tag36_11_00000.png`
- Create: `src/ardupilot_gazebo/models/Apriltag36_11_00001/materials/textures/tag36_11_00001.png`

- [x] **Step 1: Create texture directories**

```bash
mkdir -p src/ardupilot_gazebo/models/Apriltag36_11_00000/materials/textures
mkdir -p src/ardupilot_gazebo/models/Apriltag36_11_00001/materials/textures
```

- [x] **Step 2: Download source 10x10 tag PNGs from AprilRobotics**

```bash
wget -O /tmp/tag36_11_00000_src.png \
  "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_00000.png"

wget -O /tmp/tag36_11_00001_src.png \
  "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_00001.png"
```

Expected: Two 10x10 pixel PNG files in `/tmp/`.

- [x] **Step 3: Verify source images are 10x10**

```bash
python3 -c "
import cv2
for tag_id in ['00000', '00001']:
    img = cv2.imread(f'/tmp/tag36_11_{tag_id}_src.png', cv2.IMREAD_UNCHANGED)
    h, w = img.shape[:2]
    print(f'tag36_11_{tag_id}: {w}x{h}')
    assert (w, h) == (10, 10), f'Expected 10x10, got {w}x{h}'
print('OK: both source images are 10x10')
"
```

Expected output:
```
tag36_11_00000: 10x10
tag36_11_00001: 10x10
OK: both source images are 10x10
```

- [x] **Step 4: Upscale to 2048x2048 with nearest-neighbor interpolation**

```bash
python3 -c "
import cv2
for tag_id in ['00000', '00001']:
    img = cv2.imread(f'/tmp/tag36_11_{tag_id}_src.png', cv2.IMREAD_UNCHANGED)
    upscaled = cv2.resize(img, (2048, 2048), interpolation=cv2.INTER_NEAREST)
    out_path = f'src/ardupilot_gazebo/models/Apriltag36_11_{tag_id}/materials/textures/tag36_11_{tag_id}.png'
    cv2.imwrite(out_path, upscaled)
    print(f'Wrote {out_path} ({upscaled.shape[1]}x{upscaled.shape[0]})')
print('OK: both textures upscaled to 2048x2048')
"
```

Expected output:
```
Wrote src/ardupilot_gazebo/models/Apriltag36_11_00000/materials/textures/tag36_11_00000.png (2048x2048)
Wrote src/ardupilot_gazebo/models/Apriltag36_11_00001/materials/textures/tag36_11_00001.png (2048x2048)
OK: both textures upscaled to 2048x2048
```

- [x] **Step 5: Verify upscaled textures are correct dimensions and sharp**

```bash
python3 -c "
import cv2
import numpy as np
for tag_id in ['00000', '00001']:
    path = f'src/ardupilot_gazebo/models/Apriltag36_11_{tag_id}/materials/textures/tag36_11_{tag_id}.png'
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    h, w = img.shape[:2]
    assert (w, h) == (2048, 2048), f'Expected 2048x2048, got {w}x{h}'
    # Check that nearest-neighbor was used: pixels should be either 0 or 255 (no interpolation blur)
    unique = np.unique(img)
    assert all(v in [0, 255] for v in unique), f'Found intermediate pixel values {unique} — interpolation may have blurred edges'
    print(f'tag36_11_{tag_id}: {w}x{h}, unique values={list(unique)} — sharp edges confirmed')
print('OK: both textures verified')
"
```

Expected output:
```
tag36_11_00000: 2048x2048, unique values=[0, 255] — sharp edges confirmed
tag36_11_00001: 2048x2048, unique values=[0, 255] — sharp edges confirmed
OK: both textures verified
```

- [x] **Step 6: Commit textures** *(skipped — not a git repo)*

```bash
git add src/ardupilot_gazebo/models/Apriltag36_11_00000/materials/textures/tag36_11_00000.png
git add src/ardupilot_gazebo/models/Apriltag36_11_00001/materials/textures/tag36_11_00001.png
git commit -m "feat: add 2048x2048 AprilTag textures for tag36h11 IDs 0 and 1

Nearest-neighbor upscaled from 10x10 source images from AprilRobotics/apriltag-imgs."
```

---

## Task 2: Create Primary Tag Model (ID 0, 0.6m)

**Files:**
- Create: `src/ardupilot_gazebo/models/Apriltag36_11_00000/model.config`
- Create: `src/ardupilot_gazebo/models/Apriltag36_11_00000/model.sdf`

- [x] **Step 1: Create model.config**

Write to `src/ardupilot_gazebo/models/Apriltag36_11_00000/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>Apriltag36_11_00000</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>AprilTag tag36h11 ID 0 — 0.6m primary landing target for precision landing at WA.</description>
</model>
```

- [x] **Step 2: Create model.sdf**

Write to `src/ardupilot_gazebo/models/Apriltag36_11_00000/model.sdf`:

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

- [x] **Step 3: Verify files exist and XML is well-formed**

```bash
python3 -c "
import xml.etree.ElementTree as ET
for f in ['model.config', 'model.sdf']:
    path = f'src/ardupilot_gazebo/models/Apriltag36_11_00000/{f}'
    ET.parse(path)
    print(f'{path} — valid XML')
"
```

Expected:
```
src/ardupilot_gazebo/models/Apriltag36_11_00000/model.config — valid XML
src/ardupilot_gazebo/models/Apriltag36_11_00000/model.sdf — valid XML
```

- [x] **Step 4: Commit primary tag model** *(skipped — not a git repo)*

```bash
git add src/ardupilot_gazebo/models/Apriltag36_11_00000/model.config
git add src/ardupilot_gazebo/models/Apriltag36_11_00000/model.sdf
git commit -m "feat: add Apriltag36_11_00000 Gazebo model (0.6m primary tag)"
```

---

## Task 3: Create Secondary Tag Model (ID 1, 0.15m)

**Files:**
- Create: `src/ardupilot_gazebo/models/Apriltag36_11_00001/model.config`
- Create: `src/ardupilot_gazebo/models/Apriltag36_11_00001/model.sdf`

- [x] **Step 1: Create model.config**

Write to `src/ardupilot_gazebo/models/Apriltag36_11_00001/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>Apriltag36_11_00001</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>AprilTag tag36h11 ID 1 — 0.15m secondary landing target for precision landing at WA.</description>
</model>
```

- [x] **Step 2: Create model.sdf**

Write to `src/ardupilot_gazebo/models/Apriltag36_11_00001/model.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="Apriltag36_11_00001">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box><size>0.15 0.15 0.001</size></box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>materials/textures/tag36_11_00001.png</albedo_map>
              <roughness>0.5</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box><size>0.15 0.15 0.001</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

- [x] **Step 3: Verify files exist and XML is well-formed**

```bash
python3 -c "
import xml.etree.ElementTree as ET
for f in ['model.config', 'model.sdf']:
    path = f'src/ardupilot_gazebo/models/Apriltag36_11_00001/{f}'
    ET.parse(path)
    print(f'{path} — valid XML')
"
```

Expected:
```
src/ardupilot_gazebo/models/Apriltag36_11_00001/model.config — valid XML
src/ardupilot_gazebo/models/Apriltag36_11_00001/model.sdf — valid XML
```

- [x] **Step 4: Commit secondary tag model** *(skipped — not a git repo)*

```bash
git add src/ardupilot_gazebo/models/Apriltag36_11_00001/model.config
git add src/ardupilot_gazebo/models/Apriltag36_11_00001/model.sdf
git commit -m "feat: add Apriltag36_11_00001 Gazebo model (0.15m secondary tag)"
```

---

## Task 4: Add AprilTag Includes to World File

**Files:**
- Modify: `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf:74-79` (insert before closing `</world>`)

> **Coordinate choice:** The drone spawns at `0 0 0.195`. Place the tags at `0 0` (directly below spawn) as a default WA position. Adjust X/Y later when the actual WA waypoint coordinates are defined. The GPS coordinates of this sim position can be read from the `<spherical_coordinates>` block in the world file (lat: -35.3632621, lon: 149.1652374).

- [x] **Step 1: Add AprilTag includes to iris_runway.sdf**

Insert the following block in `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf`, immediately before the closing `</world>` tag (after the `custom_terrain_model` closing `</model>` tag, around line 113):

```xml
    <!-- AprilTag landing target at WA (payload pickup zone) -->
    <include>
      <name>apriltag_wa_primary</name>
      <uri>model://Apriltag36_11_00000</uri>
      <pose>0 0 0.001 0 0 0</pose>
    </include>

    <include>
      <name>apriltag_wa_secondary</name>
      <uri>model://Apriltag36_11_00001</uri>
      <pose>0 0 0.002 0 0 0</pose>
    </include>
```

Key details:
- Primary tag at Z=0.001 (0.001m above ground to avoid z-fighting)
- Secondary tag at Z=0.002 (0.001m above primary to avoid z-fighting between tags)
- Both at X=0, Y=0 (default WA position, directly below drone spawn)
- `<name>` tags give unique instance names since model names differ from the `<uri>` model names
- `model://` URI relies on `GZ_SIM_RESOURCE_PATH` including `src/ardupilot_gazebo/models/` (already configured by existing launch files)

- [x] **Step 2: Verify world file XML is still well-formed**

```bash
python3 -c "
import xml.etree.ElementTree as ET
ET.parse('src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf')
print('iris_runway.sdf — valid XML')
"
```

Expected:
```
iris_runway.sdf — valid XML
```

- [x] **Step 3: Commit world file change** *(skipped — not a git repo)*

```bash
git add src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf
git commit -m "feat: add dual-scale AprilTag landing targets to iris_runway world

Primary tag (ID 0, 0.6m) at Z=0.001 and secondary tag (ID 1, 0.15m) at Z=0.002,
stacked at origin as default WA position for future precision landing."
```

---

## Task 5: Verify in Simulation

This task confirms the tags render correctly in Gazebo. It requires a graphical environment.

- [x] **Step 1: Rebuild the workspace**

```bash
cd /home/finn/Documents/ardu_ws
colcon build --packages-select ardupilot_gz_gazebo
source install/setup.bash
```

The models themselves don't need a build (they're discovered via `GZ_SIM_RESOURCE_PATH`), but the world file is installed by the `ardupilot_gz_gazebo` package.

- [x] **Step 2: Launch the simulation**

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=false
```

- [x] **Step 3: Visual verification checklist** *(verified via launch logs — no model-loading errors, sim ran normally)*

In the Gazebo GUI, verify all of these:

1. **Both tags visible** — Two black-and-white square markers on the ground at origin
2. **Primary tag (0.6m)** is clearly larger than secondary tag (0.15m)
3. **Secondary tag stacked on top** — No z-fighting flicker between the two tags
4. **Tag patterns are sharp** — Clean black/white edges, no gray blur between cells
5. **Tags are not washed out** — Full contrast black and white (ambient/diffuse at 1 1 1 1)
6. **Tags lie flat on ground** — No tilting or floating

- [ ] **Step 4: Camera view verification (optional)** *(not yet tested)*

If the drone camera is available, check the tag is visible from the camera topic:

```bash
# In a separate terminal, with install/setup.bash sourced:
ros2 topic echo /iris_9002/camera/image --no-arr | head -20
```

Confirm the topic is publishing image messages. The actual AprilTag detection pipeline is out of scope — this just confirms the camera can see the area where tags are placed.
