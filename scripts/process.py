"""
Process URDF robot descriptions into web-optimized GLB meshes.

For each robot in robots.yaml:
1. Import URDF from robot_descriptions (auto-clones upstream repo)
2. Parse URDF XML to find mesh references
3. Load and decimate meshes with trimesh
4. Export as GLB
5. Rewrite URDF mesh paths to relative GLB references
6. Generate manifest.json
"""

import importlib
import json
import shutil
import subprocess
import sys
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import trimesh
import yaml

ROOT = Path(__file__).resolve().parent.parent
DIST = ROOT / "dist"
MODELS_DIR = DIST / "models"

LOD_TARGETS = {
    "low": 5_000,
    "medium": 25_000,
}


def load_catalog() -> list[dict]:
    with open(ROOT / "robots.yaml") as f:
        data = yaml.safe_load(f)
    return data["robots"]


def get_description_paths(module_name: str) -> tuple[Path, Path, Path]:
    """Import a robot_descriptions module and return (URDF_PATH, PACKAGE_PATH, REPOSITORY_PATH)."""
    mod = importlib.import_module(f"robot_descriptions.{module_name}")
    return Path(mod.URDF_PATH), Path(mod.PACKAGE_PATH), Path(mod.REPOSITORY_PATH)


def find_mesh_references(urdf_path: Path, visual_only: bool = True) -> list[tuple[ET.Element, str]]:
    """Parse URDF and return (element, filename) pairs for mesh references.

    If visual_only is True, only meshes inside <visual> elements are returned
    (skipping <collision> meshes which are low-poly approximations).
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    refs = []
    if visual_only:
        for visual in root.iter("visual"):
            for mesh_el in visual.iter("mesh"):
                filename = mesh_el.get("filename")
                if filename:
                    refs.append((mesh_el, filename))
    else:
        for mesh_el in root.iter("mesh"):
            filename = mesh_el.get("filename")
            if filename:
                refs.append((mesh_el, filename))
    return refs


def resolve_mesh_path(filename: str, urdf_path: Path, package_path: Path, repo_path: Path) -> Path | None:
    """Resolve a URDF mesh filename (possibly with package://) to a local path."""
    if filename.startswith("package://"):
        rel = filename.replace("package://", "")
        # package:// URIs are relative to a ROS package root.
        # Try resolving against: repo parent (for multi-package repos),
        # repo root, package root, and URDF parent.
        candidates = [
            repo_path.parent / rel,
            repo_path / rel,
            package_path / rel,
            urdf_path.parent / rel,
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        # Try stripping the first path component (the package name)
        parts = rel.split("/", 1)
        if len(parts) == 2:
            stripped = parts[1]
            for base in [package_path, repo_path, urdf_path.parent]:
                candidate = base / stripped
                if candidate.exists():
                    return candidate
        return None
    elif filename.startswith("file://"):
        return Path(filename.replace("file://", ""))
    else:
        return urdf_path.parent / filename


def load_mesh(mesh_path: Path) -> trimesh.Trimesh | None:
    """Load a mesh file, returning a single Trimesh (merging scenes if needed)."""
    try:
        loaded = trimesh.load(mesh_path, force="mesh")
        if isinstance(loaded, trimesh.Scene):
            meshes = [g for g in loaded.geometry.values() if isinstance(g, trimesh.Trimesh)]
            if not meshes:
                return None
            return trimesh.util.concatenate(meshes)
        return loaded
    except Exception as e:
        print(f"  WARNING: Failed to load mesh {mesh_path}: {e}")
        return None


def decimate_mesh(mesh: trimesh.Trimesh, target_faces: int) -> trimesh.Trimesh:
    """Decimate a mesh to approximately target_faces triangles."""
    if len(mesh.faces) <= target_faces:
        return mesh
    try:
        decimated = mesh.simplify_quadric_decimation(face_count=target_faces)
        if decimated is not None and len(decimated.faces) > 0:
            return decimated
    except Exception as e:
        print(f"    Decimation failed: {e}")
    return mesh


def export_glb(mesh: trimesh.Trimesh, output_path: Path) -> None:
    """Export a mesh as GLB (binary glTF)."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(output_path, file_type="glb")


def apply_draco_compression(glb_path: Path) -> bool:
    """Apply Draco compression to a GLB file using gltf-transform CLI."""
    if not shutil.which("gltf-transform"):
        return False
    try:
        subprocess.run(
            ["gltf-transform", "draco", str(glb_path), str(glb_path)],
            capture_output=True,
            check=True,
        )
        return True
    except subprocess.CalledProcessError:
        return False


def rewrite_urdf(urdf_path: Path, output_path: Path, mesh_map: dict[str, str]) -> None:
    """
    Copy URDF to output_path, rewriting mesh filenames to their GLB equivalents.
    mesh_map: original_filename -> new_relative_path
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for mesh_el in root.iter("mesh"):
        filename = mesh_el.get("filename")
        if filename and filename in mesh_map:
            mesh_el.set("filename", mesh_map[filename])

    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, xml_declaration=True, encoding="unicode")


def count_non_fixed_joints(urdf_path: Path) -> int:
    """Count non-fixed joints in a URDF file."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    count = 0
    for joint in root.iter("joint"):
        jtype = joint.get("type", "fixed")
        if jtype != "fixed":
            count += 1
    return count


def process_robot(robot: dict, use_draco: bool = True) -> dict | None:
    """
    Process a single robot entry. Returns a manifest entry dict, or None on failure.
    """
    robot_id = robot["id"]
    module_name = robot["module"]

    print(f"\n{'='*60}")
    print(f"Processing: {robot['brand']} {robot['name']} ({robot_id})")
    print(f"  Module: {module_name}")

    # Step 1: Get paths from robot_descriptions
    try:
        urdf_path, package_path, repo_path = get_description_paths(module_name)
    except Exception as e:
        print(f"  ERROR: Failed to import {module_name}: {e}")
        return None

    print(f"  URDF: {urdf_path}")
    print(f"  Package: {package_path}")
    print(f"  Repo: {repo_path}")

    if not urdf_path.exists():
        print(f"  ERROR: URDF file does not exist")
        return None

    # Step 2: Find mesh references
    refs = find_mesh_references(urdf_path)
    print(f"  Found {len(refs)} mesh references")

    # Step 3: Process each mesh at each LOD
    model_dir = MODELS_DIR / robot_id
    mesh_map_by_lod: dict[str, dict[str, str]] = {}
    processed_lods: list[str] = []
    seen_meshes: dict[str, Path] = {}  # original filename -> resolved path

    # First pass: resolve all mesh paths
    for _, filename in refs:
        if filename not in seen_meshes:
            resolved = resolve_mesh_path(filename, urdf_path, package_path, repo_path)
            if resolved and resolved.exists():
                seen_meshes[filename] = resolved
            else:
                print(f"  WARNING: Cannot resolve mesh: {filename}")

    # Second pass: process meshes at each LOD
    for lod_name, target_faces in LOD_TARGETS.items():
        mesh_map: dict[str, str] = {}
        all_ok = True

        for original_filename, resolved_path in seen_meshes.items():
            # Create a deterministic output filename from the mesh path
            mesh_stem = resolved_path.stem
            # Avoid collisions by including parent dir name
            parent_name = resolved_path.parent.name
            glb_filename = f"{parent_name}_{mesh_stem}.glb"
            glb_rel_path = f"meshes/{lod_name}/{glb_filename}"
            glb_abs_path = model_dir / glb_rel_path

            if not glb_abs_path.exists():
                mesh = load_mesh(resolved_path)
                if mesh is None:
                    print(f"  WARNING: Failed to load {resolved_path.name} — skipping")
                    all_ok = False
                    continue

                decimated = decimate_mesh(mesh, target_faces)
                export_glb(decimated, glb_abs_path)

                if use_draco:
                    apply_draco_compression(glb_abs_path)

                print(f"  {lod_name}: {resolved_path.name} — {len(mesh.faces)} -> {len(decimated.faces)} faces -> {glb_abs_path.name}")

            mesh_map[original_filename] = glb_rel_path

        if all_ok or len(mesh_map) > 0:
            mesh_map_by_lod[lod_name] = mesh_map
            processed_lods.append(lod_name)

    if not processed_lods:
        print(f"  ERROR: No LODs were successfully processed")
        return None

    # Step 4: Write rewritten URDFs (one per LOD)
    for lod_name in processed_lods:
        urdf_output = model_dir / f"robot_{lod_name}.urdf"
        rewrite_urdf(urdf_path, urdf_output, mesh_map_by_lod[lod_name])

    # Step 5: Validate
    actual_dof = count_non_fixed_joints(urdf_path)
    expected_dof = robot.get("dof")
    if expected_dof is not None and actual_dof != expected_dof:
        print(f"  WARNING: DOF mismatch — expected {expected_dof}, got {actual_dof}")

    # Step 6: Build manifest entry
    entry = {
        "id": robot_id,
        "brand": robot["brand"],
        "name": robot["name"],
        "tipLinks": robot["tipLinks"],
        "category": robot.get("category", "arm"),
        "lods": processed_lods,
        "dof": actual_dof,
        "upstream": module_name,
    }

    # Optional fields
    for field in ["reach", "weight", "payload", "dataSheet", "productPage"]:
        if field in robot and robot[field] is not None:
            entry[field] = robot[field]

    # Add URDF paths per LOD
    entry["urdfs"] = {
        lod: f"models/{robot_id}/robot_{lod}.urdf" for lod in processed_lods
    }

    print(f"  OK — {len(seen_meshes)} meshes, {len(processed_lods)} LODs, {actual_dof} DOF")
    return entry


def generate_manifest(entries: list[dict]) -> None:
    """Write manifest.json to dist/."""
    manifest = {
        "version": 1,
        "generated": datetime.now(timezone.utc).isoformat(),
        "models": entries,
    }
    manifest_path = DIST / "manifest.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"\nManifest written to {manifest_path} ({len(entries)} models)")


def main():
    catalog = load_catalog()
    print(f"Loaded {len(catalog)} robots from robots.yaml")

    use_draco = bool(shutil.which("gltf-transform"))
    if use_draco:
        print("gltf-transform found — Draco compression enabled")
    else:
        print("gltf-transform not found — Draco compression disabled (install with: npm i -g @gltf-transform/cli)")

    # Clean previous output
    if DIST.exists():
        shutil.rmtree(DIST)

    entries: list[dict] = []
    failures: list[str] = []

    for robot in catalog:
        entry = process_robot(robot, use_draco=use_draco)
        if entry:
            entries.append(entry)
        else:
            failures.append(f"{robot['brand']} {robot['name']} ({robot['id']})")

    generate_manifest(entries)

    print(f"\n{'='*60}")
    print(f"DONE: {len(entries)} succeeded, {len(failures)} failed")
    if failures:
        print(f"\nFailed robots:")
        for f in failures:
            print(f"  - {f}")


if __name__ == "__main__":
    main()
