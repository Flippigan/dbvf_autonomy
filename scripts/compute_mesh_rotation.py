#!/usr/bin/env python3
"""Compute mesh rotation and translation for VFS DBVF course alignment.

Loads the VFS photogrammetry DAE mesh, applies a yaw rotation to align the
strip's long axis with +X (East), finds the optimal translation so all 5
competition zones are covered, and outputs SDF pose values + AprilTag poses.

Usage:
    python3 compute_mesh_rotation.py
    python3 compute_mesh_rotation.py --yaw -60
    python3 compute_mesh_rotation.py --mesh /path/to/other.dae
"""

import argparse
import math
import sys

import numpy as np
import trimesh

# --- Constants ---

DEFAULT_MESH_PATH = (
    "/home/finn/Documents/ardu_ws/meshes/"
    "VFS Model v1 - Closed Squares/Untitled.dae"
)
DEFAULT_YAW_DEG = -57.0

# Competition zone world coordinates (X = East, Y = North)
ZONES = {
    "H":  (0.0, 0.0),
    "WA": (50.0, 1.0),
    "L":  (91.4, 0.0),
    "F1": (121.9, 0.0),
    "F2": (152.4, 0.0),
}

# AprilTag XY positions and Z offset above mesh surface
APRILTAG_POSITIONS = {
    "apriltag_wa_primary":   (50.0, 1.0),
    "apriltag_wa_secondary": (50.5, 1.0),
}
APRILTAG_Z_OFFSET = 0.01  # meters above mesh surface


def load_mesh(path: str) -> trimesh.Trimesh:
    """Load a DAE mesh file and return a single consolidated Trimesh."""
    loaded = trimesh.load(path)
    if isinstance(loaded, trimesh.Scene):
        meshes = [
            g for g in loaded.geometry.values()
            if isinstance(g, trimesh.Trimesh)
        ]
        if not meshes:
            print("ERROR: No triangle meshes found in DAE file.", file=sys.stderr)
            sys.exit(1)
        mesh = trimesh.util.concatenate(meshes)
    elif isinstance(loaded, trimesh.Trimesh):
        mesh = loaded
    else:
        print(
            f"ERROR: Unexpected type from trimesh.load: {type(loaded)}",
            file=sys.stderr,
        )
        sys.exit(1)
    return mesh


def apply_yaw_rotation(mesh: trimesh.Trimesh, yaw_deg: float) -> None:
    """Apply a Z-axis yaw rotation to the mesh vertices (in-place)."""
    yaw_rad = math.radians(yaw_deg)
    rot = trimesh.transformations.rotation_matrix(yaw_rad, [0, 0, 1])
    mesh.apply_transform(rot)


def ray_cast_z(mesh: trimesh.Trimesh, x: float, y: float) -> float | None:
    """Cast a vertical ray downward at (x, y), return highest surface Z hit."""
    ray_origin = np.array([[x, y, 10000.0]])
    ray_direction = np.array([[0.0, 0.0, -1.0]])
    locations, _, _ = mesh.ray.intersects_location(
        ray_origins=ray_origin,
        ray_directions=ray_direction,
    )
    if len(locations) == 0:
        return None
    return float(locations[:, 2].max())


def find_optimal_translation(
    mesh: trimesh.Trimesh,
    zones: dict[str, tuple[float, float]],
) -> tuple[float, float, dict[str, float]]:
    """Find (tx, ty) so all zone points land on the rotated mesh surface.

    The SDF pose (tx, ty, tz, 0, 0, yaw) means:
        world_point = R_yaw * local_point + (tx, ty, tz)
    Since we pre-rotated the mesh for analysis, the relationship simplifies to:
        world_point = rotated_local_point + (tx, ty, tz)
    So a zone at world (zx, zy) maps to rotated-mesh coords (zx - tx, zy - ty).

    Strategy:
    1. Coarse sweep (2m steps) over candidate (tx, ty).
    2. For each candidate, bounds-check then ray-cast all 5 zones.
    3. Score by: (a) all zones hit, (b) H surface Z above median,
       (c) maximize F2 margin from mesh edge.
    4. Fine sweep (0.5m steps) around the coarse winner.

    Returns (tx, ty, zone_elevations_dict).
    """
    bounds = mesh.bounds  # [[xmin, ymin, zmin], [xmax, ymax, zmax]]
    x_min, y_min = bounds[0][0], bounds[0][1]
    x_max, y_max = bounds[1][0], bounds[1][1]

    median_z = float(np.median(mesh.vertices[:, 2]))
    f2_x = zones["F2"][0]

    def score_candidate(
        tx: float, ty: float
    ) -> tuple[float, dict[str, float]]:
        """Return (score, zone_z_dict). Negative score = invalid."""
        zone_z: dict[str, float] = {}

        # Quick bounds check for all zones
        for name, (zx, zy) in zones.items():
            mx, my = zx - tx, zy - ty
            if mx < x_min or mx > x_max or my < y_min or my > y_max:
                return -1.0, {}

        # Detailed ray-cast check
        for name, (zx, zy) in zones.items():
            mx, my = zx - tx, zy - ty
            z = ray_cast_z(mesh, mx, my)
            if z is None:
                return -1.0, {}
            zone_z[name] = z

        # Scoring
        h_above_median = 1.0 if zone_z["H"] > median_z else 0.0
        f2_mx = f2_x - tx
        f2_margin = min(f2_mx - x_min, x_max - f2_mx)

        return h_above_median * 1000.0 + f2_margin, zone_z

    # --- Coarse sweep ---
    coarse_step = 2.0
    tx_range = np.arange(f2_x - x_max, -x_min, coarse_step)
    ty_range = np.arange(-y_max + 5, -y_min - 5, coarse_step)

    print(
        f"  Coarse sweep: {len(tx_range)} x {len(ty_range)} = "
        f"{len(tx_range) * len(ty_range)} candidates (step={coarse_step}m)"
    )

    best_score = -float("inf")
    best_tx, best_ty = 0.0, 0.0
    best_zone_z: dict[str, float] = {}

    for tx in tx_range:
        for ty in ty_range:
            s, zz = score_candidate(tx, ty)
            if s > best_score:
                best_score = s
                best_tx, best_ty = tx, ty
                best_zone_z = zz

    if best_score < 0:
        print(
            "ERROR: No valid translation found. All 5 zones cannot "
            "be covered simultaneously. Try a different --yaw angle.",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"  Coarse best: tx={best_tx:.1f}, ty={best_ty:.1f}, score={best_score:.1f}")

    # --- Fine sweep around coarse best ---
    fine_step = 0.5
    fine_range = 5.0
    tx_fine = np.arange(best_tx - fine_range, best_tx + fine_range, fine_step)
    ty_fine = np.arange(best_ty - fine_range, best_ty + fine_range, fine_step)

    for tx in tx_fine:
        for ty in ty_fine:
            s, zz = score_candidate(tx, ty)
            if s > best_score:
                best_score = s
                best_tx, best_ty = tx, ty
                best_zone_z = zz

    print(f"  Fine best:   tx={best_tx:.1f}, ty={best_ty:.1f}, score={best_score:.1f}")

    return best_tx, best_ty, best_zone_z
