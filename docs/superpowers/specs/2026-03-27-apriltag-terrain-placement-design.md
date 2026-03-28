# AprilTag Terrain Placement Design

**Goal:** Move the dual-scale AprilTag landing targets from the world origin onto the surface of the VFS terrain model, so the grass texture serves as a realistic background for camera-based detection testing.

**Approach:** Update the `<pose>` values of the two existing AprilTag `<include>` entries in the world file. No model changes needed.

---

## Location Selection

The VFS terrain mesh ("VFS Model v1 - Closed Squares") was analyzed for flat patches suitable for tag placement. The mesh was divided into 5m x 5m cells and ranked by Z standard deviation.

**Chosen location: (50.0, 1.0)** in world coordinates.

| Property | Value |
|----------|-------|
| World XY | (50.0, 1.0) |
| Terrain surface Z (ray-cast) | 2.388m |
| Max surface Z in 2m footprint | 2.447m |
| Vertex density | high |

Originally (59.5, 1.0) was selected from vertex std-dev analysis, but visual inspection showed it was at the terrain mesh edge. Moved to (50.0, 1.0) which is well within the mesh. Final Z values determined by trimesh ray-cast sampling a 2m grid and using the max surface height + 10mm margin.

---

## Pose Values

| Tag | Original Pose | Final Pose | Z Offset Above Max Surface |
|-----|--------------|------------|---------------------------|
| Primary (ID 0, 0.6m) | `0 0 0.001 0 0 0` | `50.0 1.0 2.457 0 0 0` | +10mm |
| Secondary (ID 1, 0.15m) | `0 0 0.002 0 0 0` | `50.0 1.0 2.458 0 0 0` | +11mm |

Z-offsets of 10mm/11mm above the max surface height in the tag footprint prevent z-fighting with irregular terrain mesh vertices.

---

## Scope

**What changes:**
- `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf` — two `<pose>` values updated

**What does NOT change:**
- AprilTag model files (Apriltag36_11_00000, Apriltag36_11_00001)
- Drone spawn position (remains at origin)
- Any other world file content

**Rebuild:** `ardupilot_gz_gazebo` package only (world file is installed by this package).

---

## Verification

After the change, visually confirm in Gazebo that:
1. Both tags are visible on the grass surface of the terrain model
2. No z-fighting flicker between tags and terrain
3. Tag patterns remain sharp and readable against the grass background
4. Tags lie flat on the terrain (no tilting or floating)
