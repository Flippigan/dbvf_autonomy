# Photogrammetry Mesh Orientation & Coverage Analysis

**Date:** 2026-03-29
**Mesh:** `meshes/VFS Model v1 - Closed Squares/Untitled.dae`
**World file:** `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf`

---

## Mesh Placement in World

The mesh is placed via the `custom_terrain_model` model in `iris_runway.sdf`:

```xml
<model name="custom_terrain_model">
  <static>true</static>
  <pose>30.48 0.92 30.2 0 0 0</pose>  <!-- no rotation -->
  ...
  <scale>1.00 1.00 1.00</scale>
</model>
```

- **Translation:** X+30.48, Y+0.92, Z+30.2 (compensates for mesh centroid offset)
- **Rotation:** None (0, 0, 0)
- **Scale:** 1:1

---

## Key Finding: Mesh is a Diagonal Strip, Not a Rectangle

The DAE file (exported from Blender, Z_UP, units in meters) has a **bounding box** of ~156m (X) x ~201m (Y), but the actual vertex coverage is a **diagonal photogrammetry strip** approximately **64m wide and 215m long**, running from SW to NE at **bearing ~33° (NNE)**.

### Bounding Box (World Coordinates)

| Axis | Min | Max | Span |
|------|-----|-----|------|
| X (East-West) | -44.2m | 111.5m | 155.7m |
| Y (North-South) | -101.7m | 99.2m | 200.9m |
| Z (Up) | -4.4m | 2.7m | 7.1m |

### Actual Vertex Coverage (World Coordinates)

The mesh is a ~64m-wide strip whose center shifts eastward as Y increases:

| Y Band | X Min | X Max | X Center | Width | Median Z |
|--------|-------|-------|----------|-------|----------|
| -90m | -43.8 | 2.1 | -20.9 | 45.9m | -2.96m |
| -80m | -44.2 | 8.9 | -17.6 | 53.0m | -2.20m |
| -70m | -44.2 | 15.9 | -14.1 | 60.0m | -1.29m |
| -60m | -40.5 | 22.9 | -8.8 | 63.4m | -0.43m |
| -50m | -33.9 | 29.9 | -2.0 | 63.8m | 0.40m |
| -40m | -27.0 | 36.9 | 4.9 | 63.9m | 0.91m |
| -30m | -20.1 | 43.8 | 11.9 | 64.0m | 1.47m |
| -20m | -13.4 | 50.7 | 18.7 | 64.1m | 1.92m |
| -10m | -6.5 | 57.8 | 25.7 | 64.3m | 2.25m |
| 0m | 0.4 | 64.7 | 32.5 | 64.3m | 2.34m |
| +10m | 7.3 | 71.8 | 39.6 | 64.5m | 2.39m |
| +20m | 14.2 | 78.7 | 46.5 | 64.5m | 2.34m |
| +30m | 21.0 | 85.6 | 53.3 | 64.6m | 2.16m |
| +40m | 28.0 | 92.7 | 60.3 | 64.6m | 1.94m |
| +50m | 35.0 | 99.6 | 67.3 | 64.6m | 1.73m |
| +60m | 41.8 | 106.6 | 74.2 | 64.8m | 1.26m |
| +70m | 48.9 | 111.4 | 80.2 | 62.5m | 0.80m |
| +80m | 55.8 | 111.5 | 83.7 | 55.7m | 0.32m |
| +90m | 62.8 | 111.5 | 87.2 | 48.7m | -0.07m |

**Strip centerline equation:** `X_center = 0.65 * Y + 32.8`

---

## Drone & Course Layout

- **Drone spawn:** World origin (0, 0, 0.195), facing North (yaw = 90°)
- **AprilTag (WA zone):** World (50.0, 1.0, 2.457)
- **Course direction:** East (+X), confirmed by AprilTag placement ~50m east of origin
- **Course length:** 152.4m (H to F2)

### Course Zone Positions (Along +X at Y=0)

| Zone | Distance from H | World (X, Y) | Mesh Surface Z | Coverage Status |
|------|-----------------|-------------|----------------|-----------------|
| H | 0m | (0, 0) | 1.79m | Edge of mesh |
| WA/WM | 45.7m | (45.7, 0) | 2.45m | Covered |
| L | 91.4m | (91.4, 0) | ~2.21m (sparse) | Barely covered (30m sample radius needed) |
| F1 | 121.9m | (121.9, 0) | **NO DATA** | Beyond mesh |
| F2 | 152.4m | (152.4, 0) | **NO DATA** | Beyond mesh |

### AprilTag Elevation Check

- Mesh surface at (50, 1): Z = **2.35m**
- AprilTag placed at Z = **2.457m** (~0.1m above surface — reasonable for a pad)

---

## Coverage Problem

The mesh strip runs at bearing ~33° (NNE), but the course runs due East (+X). These are **misaligned by ~57°**.

At Y = 0 (the course centerline), the mesh only spans **X = 4m to 61m** — roughly 61m of east coverage from the origin. The course requires 152.4m.

```
Mesh strip (diagonal, bearing 33°)
          ╱                          ╲
         ╱  ~64m wide, ~215m long     ╲
        ╱                              ╲
       ╱         NE end                 ╲
      ╱      (111, +90)                  ╲
     ╱          ╱                         ╲
    ╱          ╱                           ╲
              ╱
  Drone ★ ──────────── Course (+X) ──────────── F2 (152m)
            ╱          ↑ mesh ends ~61m
           ╱
          ╱
         ╱  SW end
        ╱  (-44, -90)

→ F1 and F2 are 60–90m beyond the mesh edge
```

---

## Chosen Fix: Rotate the Mesh

**Decision:** Rotate the mesh ~57° clockwise by adding yaw ≈ -57° to the model `<pose>` in `iris_runway.sdf`. This aligns the strip's long axis (~215m) with +X (East), easily covering the 152.4m course.

After rotation, the model pose translation (X, Y, Z) will also need recalculating to keep H (the drone spawn point at origin) positioned correctly on the mesh surface.

### Required Follow-up Work

- **Recalculate model pose translation** — the current (30.48, 0.92, 30.2) offset compensates for the unrotated mesh centroid. After adding yaw rotation, the centroid shifts in world space and the translation must be recomputed.
- **Reposition AprilTags** — the existing tags (`apriltag_wa_primary` at (50, 1, 2.457) and `apriltag_wa_secondary` at (50.5, 1, 2.457)) were placed for the unrotated mesh. After rotation, the mesh surface and Z elevation at those positions will change. Tags must be repositioned to sit on the new mesh surface at the correct course zone locations.
- **Verify mesh surface Z at each course zone** — after rotation the terrain profile along +X will differ from the current analysis. Re-sample surface elevations at H, WA/WM, L, F1, and F2 to set correct tag/pad heights.

### Rejected Alternatives

- **Re-orient the course** to follow the strip's diagonal (bearing ~33°). Would require moving AprilTags, updating GPS waypoints, and changing the drone's heading — more disruptive than rotating the mesh.
- **Shift the model pose only** (no rotation). The strip is 64m wide and the course is 152m long; translation alone cannot fix the coverage gap.

---

## Terrain Elevation Profile

The mesh has significant elevation variation (~7m total range). Along the strip from SW to NE:

- **SW end (Y≈-90):** Z ≈ -3.0m (lowest)
- **Center (Y≈0):** Z ≈ 2.3m (highest, near H/WA)
- **NE end (Y≈+90):** Z ≈ -0.1m

This represents a hill/ridge peaking near the middle of the strip. Any course routing must account for this terrain profile.

---

## Post-Rotation Results (2026-03-29)

**Applied rotation:** -57 deg (-0.994838 rad) clockwise about Z axis
**New SDF model pose:** `<pose>44.7831 3.9526 28.9273 0 0 -0.994838</pose>`
**Previous SDF model pose:** `<pose>30.48 0.92 30.2 0 0 0</pose>`

### Course Zone Coverage After Rotation

| Zone | World (X, Y) | Surface Z | Coverage Status |
|------|-------------|-----------|-----------------|
| H    | (0, 0)      | 0.000m    | COVERED         |
| WA   | (50.0, 1.0) | 1.272m    | COVERED         |
| L    | (91.4, 0)   | 0.916m    | COVERED         |
| F1   | (121.9, 0)  | -0.055m   | COVERED         |
| F2   | (152.4, 0)  | -1.421m   | COVERED         |

All 5 zones previously limited to 61m of coverage now have full mesh surface data up to 152.4m.

### AprilTag Poses After Rotation

| Tag | Pose |
|-----|------|
| apriltag_wa_primary   | `<pose>50.0 1.0 1.282 0 0 0</pose>` |
| apriltag_wa_secondary | `<pose>50.5 1.0 1.290 0 0 0</pose>` |

### Verification

- All 5 competition zones have mesh surface coverage
- F1 and F2 (previously 60-90m beyond mesh edge) are now within the rotated strip
- Existing unit tests pass — no regressions
- Mission parameters (GPS coords, altitudes) unchanged
- Computation script: `src/dbvf_autonomy/scripts/compute_mesh_rotation.py`
