# Mesh Rotation & Course Alignment Design

**Date:** 2026-03-29
**Status:** Draft
**Related:** `docs/Features/Course orrientation/mesh-orientation-analysis.md`

---

## Problem

The photogrammetry mesh (`meshes/VFS Model v1 - Closed Squares/Untitled.dae`) is a diagonal strip ~64m wide and ~215m long, running at bearing ~33° (NNE). The competition course runs due East (+X) and is 152.4m long. At the course centerline (Y=0), the mesh only covers X = 4m to 61m — zones F1 (121.9m) and F2 (152.4m) are beyond the mesh edge.

### Current State

- **Mesh pose in SDF:** `<pose>30.48 0.92 30.2 0 0 0</pose>` (no rotation)
- **Mesh coverage at Y=0:** ~61m eastward from origin
- **Course requirement:** 152.4m eastward (H to F2)
- **Misalignment:** Strip bearing (~33°) vs course bearing (90°) = ~57° offset

## Solution

Rotate the mesh ~57° clockwise (yaw ≈ -0.995 rad) in the SDF model pose to align the strip's long axis (~215m) with +X (East). Recalculate the translation so that the drone spawn (world origin) sits on good terrain near the start of coverage and F2 (152.4m east) falls within the far end.

## Design

### 1. Computation Script

A standalone Python script at `src/dbvf_autonomy/scripts/compute_mesh_rotation.py` that deterministically computes all values needed for the SDF update.

**Inputs:**
- DAE mesh file path (`meshes/VFS Model v1 - Closed Squares/Untitled.dae`)
- Target yaw rotation (default: -57°, configurable)
- Course zone world coordinates:
  - H: (0, 0)
  - WA: (50.0, 1.0)
  - L: (91.4, 0)
  - F1: (121.9, 0)
  - F2: (152.4, 0)
- AprilTag XY positions: primary (50.0, 1.0), secondary (50.5, 1.0)
- AprilTag Z offset above mesh surface: 0.01m

**Processing:**
1. Load DAE mesh using `trimesh`
2. Apply yaw rotation (Z-axis rotation) to all vertices
3. Find optimal X/Y translation such that:
   - World origin (0, 0) is on the mesh surface with surface Z above the mesh median elevation (i.e. not in a valley)
   - Course centerline (Y=0) runs through the strip's width (within the central 80% of strip width)
   - F2 at (152.4, 0) is within the mesh boundary (ray cast returns a hit)
   - Strategy: sweep candidate translations along the rotated strip's long axis, score each by: (a) all 5 zones have ray-cast hits, (b) H surface Z is above median, (c) maximize margin between F2 and the mesh edge
4. Ray-cast vertical rays at each course zone XY to sample mesh surface Z
5. Compute AprilTag Z from mesh surface at WA + 0.01m offset

**Outputs (printed, human-readable):**
- SDF model pose: `X Y Z 0 0 YAW` (translation in meters, yaw in radians)
- Course zone elevation table:

  | Zone | World (X, Y) | Surface Z | Coverage Status |
  |------|-------------|-----------|-----------------|
  | H    | (0, 0)      | ...       | ...             |
  | WA   | (50.0, 1.0) | ...       | ...             |
  | L    | (91.4, 0)   | ...       | ...             |
  | F1   | (121.9, 0)  | ...       | ...             |
  | F2   | (152.4, 0)  | ...       | ...             |

- AprilTag poses:
  - `apriltag_wa_primary`: `50.0 1.0 Z 0 0 0`
  - `apriltag_wa_secondary`: `50.5 1.0 Z 0 0 0`

**Dependency:** `trimesh` Python package (`pip install trimesh`).

### 2. SDF Updates

Manually update `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf` with values from the script:

1. **Mesh model pose** — replace `<pose>30.48 0.92 30.2 0 0 0</pose>` on `custom_terrain_model` with computed `<pose>X Y Z 0 0 YAW</pose>`
2. **AprilTag primary pose** — update `apriltag_wa_primary` `<pose>` with `50.0 1.0 Z_NEW 0 0 0`
3. **AprilTag secondary pose** — update `apriltag_wa_secondary` `<pose>` with `50.5 1.0 Z_NEW 0 0 0`

No other SDF changes. Drone spawn, camera config, and other models are unaffected.

### 3. Mission Parameters Verification

After SDF update, verify `src/dbvf_autonomy/config/mission_params.yaml`:

- **GPS coordinates** — these map to world XY positions independent of mesh placement. They should not need changes unless a zone falls off the mesh edge or onto extreme terrain.
- **Altitude parameters** — `transit_altitude_ft` (35.0) and other altitude values are AGL from ArduPilot home, not mesh-relative. No changes expected.
- **Zone feasibility** — if the script reveals a zone on extreme slope or off-mesh, that zone's GPS waypoint would need adjustment. The script's elevation table makes this obvious.
- **Known limitation** — terrain elevation differences between zones affect baro-altitude vs AGL accuracy in SITL without terrain data. Documented here, not fixed.

### 4. Test Verification

Run `colcon test --packages-select dbvf_autonomy` to confirm no regressions. The existing 38 unit tests are pure-function tests (mode maps, debounce, angles, tag selection, FSM transitions) that do not depend on mesh geometry. This is a sanity check.

### 5. Documentation Update

Append post-rotation values and the zone elevation table to `src/dbvf_autonomy/docs/Features/Course orrientation/mesh-orientation-analysis.md` for reference.

## Deliverables

| # | Deliverable | Path |
|---|------------|------|
| 1 | Computation script | `src/dbvf_autonomy/scripts/compute_mesh_rotation.py` |
| 2 | Updated world SDF | `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf` |
| 3 | Updated mission params (if needed) | `src/dbvf_autonomy/config/mission_params.yaml` |
| 4 | Updated analysis doc | `src/dbvf_autonomy/docs/Features/Course orrientation/mesh-orientation-analysis.md` |
| 5 | Test run confirmation | Existing tests pass |

## Out of Scope

- Blender re-export or mesh file modifications
- New Gazebo validation nodes
- Changes to autonomy node code (precision_landing, mavlink_interface, tag_detector_adapter)
- Changes to `sim_params.yaml` precision landing parameters (tag sizes, timeouts, descent rates)
