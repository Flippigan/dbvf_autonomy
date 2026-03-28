# Gazebo Model Integration Guide

Reference for adding 3D models (including AprilTags) to the multi-drone Gazebo simulation.

---

## 1. Where Models Go

### Directory Structure

Place new models in `src/ardupilot_gazebo/models/`:

```
src/ardupilot_gazebo/models/
  my_model/
    model.config          # Required: metadata
    model.sdf             # Required: model definition
    meshes/               # Optional: DAE/STL/OBJ mesh files
      my_mesh.dae
    materials/            # Optional: textures
      textures/
        albedo.png
        normal.png
```

Both `model.config` and `model.sdf` are **required**. The `meshes/` and `materials/` directories are only needed if the model uses external mesh files or textures.

### Model Discovery

Models are found via `GZ_SIM_RESOURCE_PATH`. The launch files in this workspace already set this, but you can add custom paths:

```bash
export GZ_SIM_RESOURCE_PATH=/path/to/my/models:$GZ_SIM_RESOURCE_PATH
```

Once on the path, reference models by name: `model://my_model` or `package://ardupilot_gazebo/models/my_model/...`

---

## 2. Supported Formats

### Mesh Formats

| Format | Extension | Best For |
|--------|-----------|----------|
| **COLLADA** | `.dae` | **Preferred.** Supports materials, textures, animations. Used by all existing models in this workspace. |
| **OBJ** | `.obj` | Good alternative. Supports materials via `.mtl` sidecar. |
| **STL** | `.stl` | Geometry only (no materials/colors). Fine for collision meshes. |
| **glTF / GLB** | `.gltf`/`.glb` | Supported in newer Gazebo. Less battle-tested. |

### SDF Version

This workspace uses **SDF 1.9**. All model.sdf files should declare `<sdf version="1.9">`.

---

## 3. Creating a Model

### model.config (metadata)

```xml
<?xml version="1.0"?>
<model>
  <name>My Model</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>you@example.com</email>
  </author>
  <description>Brief description of the model.</description>
</model>
```

### model.sdf (static object - e.g., a landing pad, obstacle, building)

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="my_model">
    <static>true</static>

    <link name="link">
      <visual name="visual">
        <geometry>
          <mesh><uri>meshes/my_mesh.dae</uri></mesh>
          <!-- OR use primitives: -->
          <!-- <box><size>1 1 1</size></box> -->
          <!-- <cylinder><radius>0.5</radius><length>1.0</length></cylinder> -->
          <!-- <sphere><radius>0.5</radius></sphere> -->
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>materials/textures/albedo.png</albedo_map>
              <normal_map>materials/textures/normal.png</normal_map>
              <roughness>0.6</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>

      <collision name="collision">
        <geometry>
          <mesh><uri>meshes/my_mesh.dae</uri></mesh>
          <!-- Use simplified geometry for better performance: -->
          <!-- <box><size>1 1 1</size></box> -->
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

### model.sdf (dynamic object - needs physics)

Add `<inertial>` and omit or set `<static>false</static>`:

```xml
<link name="link">
  <inertial>
    <mass>1.0</mass>
    <inertia>
      <ixx>0.083</ixx><ixy>0</ixy><ixz>0</ixz>
      <iyy>0.083</iyy><iyz>0</iyz>
      <izz>0.083</izz>
    </inertia>
  </inertial>
  <!-- visual and collision as above -->
</link>
```

### Mesh URI Patterns (all used in this workspace)

```xml
<!-- Package-relative (recommended for ROS2 packages) -->
<uri>package://ardupilot_gazebo/models/my_model/meshes/mesh.dae</uri>

<!-- Model-relative (works with GZ_SIM_RESOURCE_PATH) -->
<uri>model://my_model/meshes/mesh.dae</uri>

<!-- Plain relative (for textures within the same model) -->
<uri>materials/textures/albedo.png</uri>
```

---

## 4. Positioning Models in the World

### Pose Format

```xml
<pose>x y z roll pitch yaw</pose>
```
- **x, y, z** — position in meters
- **roll, pitch, yaw** — rotation in radians (or use `degrees="true"`)

### Adding to the World File

Edit `src/ardupilot_gazebo/worlds/iris_multiuav.sdf`:

```xml
<world name="iris_runway">
  <!-- ... existing content ... -->

  <!-- Add a model at specific coordinates -->
  <include>
    <uri>model://my_model</uri>
    <pose>5.0 3.0 0 0 0 1.5708</pose>
  </include>

  <!-- Multiple instances need unique names -->
  <include>
    <name>my_model_2</name>
    <uri>model://my_model</uri>
    <pose>10.0 0 0 0 0 0</pose>
  </include>

  <!-- Using degrees instead of radians -->
  <include>
    <uri>model://my_model</uri>
    <pose degrees="true">5 3 0 0 0 90</pose>
  </include>
</world>
```

### Common Orientations

| Placement | Pose Example |
|-----------|-------------|
| Flat on ground | `<pose>x y 0 0 0 0</pose>` |
| On a wall (facing +X) | `<pose>x y z 0 1.5708 0</pose>` |
| On a wall (facing +Y) | `<pose>x y z -1.5708 0 0</pose>` |
| Elevated (e.g., on table) | `<pose>x y table_height 0 0 0</pose>` |

---

## 5. Placing Models on Top of / Attaching to Other Models

### Method A: Absolute Pose Offsets (simplest)

Stack models by calculating world-space positions:

```xml
<!-- Table at ground level -->
<include>
  <name>table</name>
  <uri>model://table</uri>
  <pose>5 0 0 0 0 0</pose>
</include>

<!-- Object sitting on top of the 0.75m-tall table -->
<include>
  <name>cup</name>
  <uri>model://cup</uri>
  <pose>5 0 0.75 0 0 0</pose>
</include>
```

### Method B: Nested Models with Relative Poses

Embed one model inside another — child pose is **relative to parent**:

```xml
<model name="table_with_objects">
  <static>true</static>

  <include merge="true">
    <uri>model://table</uri>
  </include>

  <include>
    <name>cup</name>
    <uri>model://cup</uri>
    <pose>0 0 0.75 0 0 0</pose>  <!-- relative to table origin -->
  </include>
</model>
```

### Method C: Fixed Joints (rigid attachment)

Use a `<joint type="fixed">` to permanently attach models together (like the `iris_with_gimbal` model does in this workspace):

```xml
<model name="assembly">
  <include merge="true">
    <uri>model://base_model</uri>
    <name>base</name>
  </include>

  <include merge="true">
    <uri>model://attachment</uri>
    <name>sensor</name>
    <pose>0 0 0.1 0 0 0</pose>
  </include>

  <joint name="attach_joint" type="fixed">
    <parent>base_link</parent>
    <child>sensor_link</child>
  </joint>
</model>
```

This is how `iris_with_gimbal/model.sdf` attaches the gimbal to the drone — see it for a real example.

### Key Notes

- **`<static>true</static>`** means the model won't move. Other objects still collide with it. Use for landscape, buildings, markers.
- Without a joint, nested models can move independently.
- Scoped names: child links are referenced as `nested_model_name::link_name`.
- **Z-fighting**: Offset co-planar surfaces by 0.001m to avoid visual flickering.

---

## 6. AprilTag Models

### Quick Start: Use a Generator

For **Gazebo Harmonic / gz-sim** (what this workspace uses):

```bash
git clone -b harmonic https://github.com/rickarmstrong/gazebo_apriltag.git
cd gazebo_apriltag
git submodule update --init
pip install -r requirements.txt   # needs opencv-python
python generate.py                # generates tags

# Copy into your workspace models directory
cp -r models/Apriltag36_11_* ~/Documents/ardu_ws/src/ardupilot_gazebo/models/
```

Alternative generators:
- [benediktkreis/apriltag_to_ignition_gazebo](https://github.com/benediktkreis/apriltag_to_ignition_gazebo) — supports configurable tag size (`-s`) and white border (`-w`)
- [TAMS-Group/tams_apriltags](https://github.com/TAMS-Group/tams_apriltags) — provides URDF/xacro models

### Manual AprilTag Model

An AprilTag is a **thin flat box with a texture**:

```
Apriltag36_11_00000/
  model.config
  model.sdf
  materials/textures/
    tag36_11_00000.png      # upscaled tag image
```

**model.sdf:**
```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="Apriltag36_11_00000">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box><size>0.2 0.2 0.001</size></box>
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
          <box><size>0.2 0.2 0.001</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

### Creating Tag Textures

Get source images from [AprilRobotics/apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs), then upscale with **nearest-neighbor** interpolation (never bilinear/bicubic — it blurs edges and ruins detection):

```bash
# ImageMagick
convert tag36_11_00000.png -filter point -resize 2048x2048 tag_large.png

# Python/OpenCV
import cv2
img = cv2.imread('tag36_11_00000.png', 0)
img = cv2.resize(img, (2048, 2048), interpolation=cv2.INTER_NEAREST)
cv2.imwrite('tag_large.png', img)
```

### Positioning AprilTags

| Surface | Pose |
|---------|------|
| Floor (facing up) | `<pose>x y 0.001 0 0 0</pose>` |
| Wall (facing +X) | `<pose>x y z 0 1.5708 0</pose>` |
| Wall (facing -X) | `<pose>x y z 0 -1.5708 0</pose>` |
| On top of object | `<pose>x y object_height+0.001 0 0 0</pose>` |
| Ceiling (facing down) | `<pose>x y z 3.14159 0 0</pose>` |

Always offset by 0.001m from the surface to avoid z-fighting.

### Common Pitfalls

| Problem | Fix |
|---------|-----|
| Tag looks gray/washed out | Set `<ambient>` and `<diffuse>` to `1 1 1 1` |
| Tag falls through floor | Add `<static>true</static>` |
| Texture not found | Use `model://` URI; verify `GZ_SIM_RESOURCE_PATH` |
| Z-fighting flicker | Offset tag 0.001m from surface |
| Poor detection rate | Ensure nearest-neighbor upscale; check camera resolution |

### Detection Size Note

The "tag size" parameter in detection software refers to the **inner black border distance**, not the overall model size. For tag36h11, the detectable region is the inner 8/10ths. A 0.2m model has a ~0.16m detectable tag size.

---

## 7. ROS 2 AprilTag Detection (for simulation)

### Install

```bash
sudo apt install ros-${ROS_DISTRO}-apriltag-ros
```

### Pipeline

```
Gazebo Camera → ros_gz_bridge → image_proc (rectify) → apriltag_ros → /tf + /detections
```

### Launch

```bash
ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/iris_9002/camera/image \
    -r camera_info:=/iris_9002/camera/camera_info \
    --params-file apriltag_params.yaml
```

### Configuration (apriltag_params.yaml)

```yaml
apriltag:
  ros__parameters:
    family: 36h11
    size: 0.16
    max_hamming: 0
    detector:
      threads: 1
      decimate: 2.0
      blur: 0.0
      refine: 1
      sharpening: 0.25
    tag:
      ids: [0, 1, 2, 3]
      sizes: [0.16, 0.16, 0.16, 0.16]
```

---

## Quick Reference: Existing Workspace Patterns

These patterns are already used in `src/ardupilot_gazebo/models/` — follow them for consistency:

- **SDF version**: 1.9
- **Mesh format**: COLLADA (.dae)
- **Materials**: PBR with albedo, normal, roughness, AO maps
- **Model composition**: `<include merge="true">` + fixed joints (see `iris_with_gimbal`)
- **Static terrain**: `<static>true</static>` with plane/mesh collision (see `runway`)
- **URI style**: `package://ardupilot_gazebo/models/...` or `model://model_name/...`
