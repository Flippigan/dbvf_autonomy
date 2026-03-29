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
