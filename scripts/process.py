"""
Process URDF robot descriptions for web delivery.

For each robot in robots.yaml:
1. Import URDF from robot_descriptions (auto-clones upstream repo)
2. Parse URDF XML to find visual mesh references
3. Copy original mesh files (preserving materials and textures)
4. Rewrite URDF mesh paths to relative references
5. Generate manifest.json
"""

import importlib
import json
import re
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parent.parent
DIST = ROOT / "dist"
MODELS_DIR = DIST / "models"


def load_catalog() -> list[dict]:
    with open(ROOT / "robots.yaml") as f:
        data = yaml.safe_load(f)
    return data["robots"]


def get_description_paths(module_name: str) -> tuple[Path, Path, Path]:
    """Import a robot_descriptions module and return (URDF_PATH, PACKAGE_PATH, REPOSITORY_PATH)."""
    mod = importlib.import_module(f"robot_descriptions.{module_name}")
    return Path(mod.URDF_PATH), Path(mod.PACKAGE_PATH), Path(mod.REPOSITORY_PATH)


def clone_repo(url: str, branch: str, name: str) -> Path:
    """Clone a git repo to the robot_descriptions cache. Returns the repo path."""
    cache = Path.home() / ".cache" / "robot_descriptions" / name
    if not cache.exists():
        subprocess.run(
            ["git", "clone", "--depth", "1", "-b", branch, url, str(cache)],
            capture_output=True, check=True,
        )
        # Pull LFS files if the repo uses them
        subprocess.run(
            ["git", "lfs", "pull"],
            cwd=cache, capture_output=True,
        )
    return cache


def start_xacro_container() -> bool:
    """Start the persistent xacro Docker container. Returns True on success."""
    result = subprocess.run(
        [str(ROOT / "scripts" / "render_xacro.sh"), "start"],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(f"WARNING: Failed to start xacro container: {result.stderr[:200]}")
        return False
    print(result.stdout.strip())
    return True


def stop_xacro_container() -> None:
    """Stop and remove the persistent xacro Docker container."""
    subprocess.run(
        [str(ROOT / "scripts" / "render_xacro.sh"), "stop"],
        capture_output=True,
    )


def render_xacro(repo_dir: Path, xacro_rel_path: str, output_path: Path,
                  xacro_args: dict[str, str] | None = None) -> bool:
    """Render a xacro file to URDF using the persistent Docker container."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    args_str = " ".join(f"{k}:={v}" for k, v in xacro_args.items()) if xacro_args else ""
    cmd = [str(ROOT / "scripts" / "render_xacro.sh"), "render", str(repo_dir), xacro_rel_path, str(output_path)]
    if args_str:
        cmd.append(args_str)
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"  ERROR: xacro rendering failed: {result.stderr[:200]}")
        return False
    return output_path.exists() and output_path.stat().st_size > 0


def parse_urdf(urdf_path: Path) -> ET.ElementTree:
    """Parse a URDF file, stripping <gazebo> blocks that may contain invalid XML namespaces."""
    text = urdf_path.read_text()
    # Gazebo blocks can contain unbound namespace prefixes (e.g. sensor:camera)
    # which crash the XML parser.  Strip them — they're not needed for mesh processing.
    text = re.sub(r"<gazebo[\s>].*?</gazebo>", "", text, flags=re.DOTALL)
    return ET.ElementTree(ET.fromstring(text))


def find_mesh_references(urdf_path: Path) -> list[tuple[ET.Element, str]]:
    """Parse URDF and return (element, filename) pairs for all mesh references (visual + collision)."""
    tree = parse_urdf(urdf_path)
    root = tree.getroot()
    refs = []
    for mesh_el in root.iter("mesh"):
        filename = mesh_el.get("filename")
        if filename:
            refs.append((mesh_el, filename))
    return refs


def resolve_mesh_path(filename: str, urdf_path: Path, package_path: Path, repo_path: Path) -> Path | None:
    """Resolve a URDF mesh filename (possibly with package://) to a local path."""
    if filename.startswith("package://"):
        rel = filename.replace("package://", "")
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
        # Cross-package reference: search for the package directory in the repo tree
        if parts:
            pkg_name = parts[0]
            remaining = parts[1] if len(parts) == 2 else ""
            for base in [repo_path, repo_path.parent]:
                for pkg_dir in base.rglob(pkg_name):
                    if pkg_dir.is_dir():
                        candidate = pkg_dir / remaining
                        if candidate.exists():
                            return candidate
        return None
    elif filename.startswith("file://"):
        return Path(filename.replace("file://", ""))
    else:
        # Relative path — try URDF dir, its ancestors, package_path, and repo_path
        candidates = [
            urdf_path.parent / filename,
            urdf_path.parent.parent / filename,
            package_path / filename,
            repo_path / filename,
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return None


def rewrite_urdf(urdf_path: Path, output_path: Path, mesh_map: dict[str, str],
                  tip_frames: dict[str, dict] | None = None) -> None:
    """Copy URDF to output_path, rewriting mesh filenames and injecting tip frames."""
    tree = parse_urdf(urdf_path)
    root = tree.getroot()

    for mesh_el in root.iter("mesh"):
        filename = mesh_el.get("filename")
        if filename and filename in mesh_map:
            mesh_el.set("filename", mesh_map[filename])

    # Normalize whitespace in numeric attributes (leading spaces break some parsers)
    for tag in ("origin", "axis"):
        for el in root.iter(tag):
            for attr in ("xyz", "rpy"):
                val = el.get(attr)
                if val:
                    el.set(attr, " ".join(val.split()))

    # Zero out the mounting offset on the fixed joint from "world" to the robot base
    # (some xacros place the robot above the origin to simulate a table/pedestal)
    for joint_el in root.iter("joint"):
        if joint_el.get("type") == "fixed":
            parent_el = joint_el.find("parent")
            if parent_el is not None and parent_el.get("link") == "world":
                origin_el = joint_el.find("origin")
                if origin_el is not None:
                    origin_el.set("xyz", "0 0 0")
                    origin_el.set("rpy", "0 0 0")

    # Convert floating joints to fixed — unbounded 6-DOF bases aren't meaningful
    # in the viewer and cause the IK solver to send the robot to infinity
    for joint_el in root.iter("joint"):
        if joint_el.get("type") == "floating":
            joint_el.set("type", "fixed")

    # Convert continuous joints to revolute with ±π limits so the viewer can
    # generate bounded sliders (continuous joints have no limits to display)
    import math
    for joint_el in root.iter("joint"):
        if joint_el.get("type") == "continuous":
            joint_el.set("type", "revolute")
            limit_el = joint_el.find("limit")
            if limit_el is None:
                limit_el = ET.SubElement(joint_el, "limit")
            if not limit_el.get("lower"):
                limit_el.set("lower", str(-math.pi))
            if not limit_el.get("upper"):
                limit_el.set("upper", str(math.pi))
            if not limit_el.get("effort"):
                limit_el.set("effort", "100")
            if not limit_el.get("velocity"):
                limit_el.set("velocity", "100")

    # Normalize material alpha to 1.0 (some upstream URDFs use low alpha values)
    for color_el in root.iter("color"):
        rgba = color_el.get("rgba")
        if rgba:
            parts = rgba.split()
            if len(parts) == 4 and float(parts[3]) < 1.0:
                parts[3] = "1.0"
                color_el.set("rgba", " ".join(parts))

    # Inject fixed tip frames for hands that lack them in the upstream URDF
    if tip_frames:
        for link_name, spec in tip_frames.items():
            if root.find(f".//link[@name='{link_name}']") is not None:
                continue  # already exists
            ET.SubElement(root, "link", name=link_name)
            joint = ET.SubElement(root, "joint", name=f"{link_name}_joint", type="fixed")
            ET.SubElement(joint, "origin", xyz=spec["xyz"], rpy="0 0 0")
            ET.SubElement(joint, "parent", link=spec["parent"])
            ET.SubElement(joint, "child", link=link_name)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, xml_declaration=True, encoding="unicode")


def count_non_fixed_joints(urdf_path: Path) -> int:
    """Count non-fixed joints in a URDF file."""
    tree = parse_urdf(urdf_path)
    root = tree.getroot()
    count = 0
    for joint in root.iter("joint"):
        jtype = joint.get("type", "fixed")
        if jtype != "fixed":
            count += 1
    return count


def process_robot(robot: dict) -> dict | None:
    """
    Process a single robot entry. Returns a manifest entry dict, or None on failure.

    Copies original mesh files to preserve materials/textures/colors.
    """
    robot_id = robot["id"]
    module_name = robot.get("module")

    print(f"\n{'='*60}")
    print(f"Processing: {robot['brand']} {robot['name']} ({robot_id})")

    # Step 1: Get URDF path — either from robot_descriptions or from a git repo + xacro
    if "git_repo" in robot:
        # Clone the repo and render xacro to URDF
        repo_info = robot["git_repo"]
        repo_path = clone_repo(repo_info["url"], repo_info["branch"], repo_info["name"])
        package_path = repo_path / robot.get("package", "")

        if "xacro" in robot:
            # Render xacro via Docker
            xacro_rel = robot["xacro"]
            xacro_args = robot.get("xacro_args")
            print(f"  Xacro: {xacro_rel}")
            if xacro_args:
                print(f"  Xacro args: {xacro_args}")
            tmp_urdf = DIST / "tmp" / f"{robot_id}.urdf"
            if not render_xacro(repo_path, xacro_rel, tmp_urdf, xacro_args):
                return None
            urdf_path = tmp_urdf
        elif "urdf" in robot:
            urdf_path = repo_path / robot["urdf"]
        else:
            print(f"  ERROR: git_repo entry needs 'xacro' or 'urdf' field")
            return None

        print(f"  URDF: {urdf_path}")
    else:
        # Use robot_descriptions Python package
        print(f"  Module: {module_name}")
        try:
            urdf_path, package_path, repo_path = get_description_paths(module_name)
        except Exception as e:
            print(f"  ERROR: Failed to import {module_name}: {e}")
            return None
        print(f"  URDF: {urdf_path}")

    if not urdf_path.exists():
        print(f"  ERROR: URDF file does not exist")
        return None

    # Step 2: Find all mesh references (visual + collision)
    try:
        refs = find_mesh_references(urdf_path)
    except ET.ParseError as e:
        print(f"  ERROR: Failed to parse URDF XML: {e}")
        return None
    print(f"  Found {len(refs)} mesh references")

    # Step 3: Resolve and copy mesh files
    model_dir = MODELS_DIR / robot_id
    meshes_dir = model_dir / "meshes"
    mesh_map: dict[str, str] = {}  # original filename -> new relative path
    seen_meshes: dict[str, Path] = {}  # original filename -> resolved source path
    copied_names: set[str] = set()  # track output filenames to avoid collisions

    for _, filename in refs:
        if filename in mesh_map:
            continue  # already processed

        resolved = resolve_mesh_path(filename, urdf_path, package_path, repo_path)
        if not resolved or not resolved.exists():
            print(f"  WARNING: Cannot resolve mesh: {filename}")
            continue

        seen_meshes[filename] = resolved

        # Build a unique output filename: parentdir_filename.ext
        out_name = resolved.name
        if out_name in copied_names:
            parent_name = resolved.parent.name
            out_name = f"{parent_name}_{resolved.name}"
        copied_names.add(out_name)

        out_path = meshes_dir / out_name
        out_path.parent.mkdir(parents=True, exist_ok=True)

        # Copy the mesh file
        shutil.copy2(resolved, out_path)

        # Copy textures referenced by DAE files
        if resolved.suffix.lower() == ".dae":
            # 1) Copy sibling textures (same directory)
            for sibling in resolved.parent.iterdir():
                if sibling.suffix.lower() in (".png", ".jpg", ".jpeg", ".tga", ".bmp"):
                    tex_out = meshes_dir / sibling.name
                    if not tex_out.exists():
                        shutil.copy2(sibling, tex_out)
            # 2) Parse DAE for <init_from> texture references and copy them.
            #    Use string replacement to rewrite paths — ET.write corrupts Collada namespaces.
            try:
                dae_tree = ET.parse(resolved)
                dae_root = dae_tree.getroot()
                ns = dae_root.tag.split("}")[0] + "}" if "}" in dae_root.tag else ""
                replacements: dict[str, str] = {}
                for init_from in dae_root.iter(f"{ns}init_from"):
                    tex_ref = init_from.text
                    if not tex_ref or "/" not in tex_ref:
                        continue
                    tex_path = (resolved.parent / tex_ref).resolve()
                    if tex_path.exists() and tex_path.suffix.lower() in (".png", ".jpg", ".jpeg", ".tga", ".bmp"):
                        tex_out = meshes_dir / tex_path.name
                        if not tex_out.exists():
                            shutil.copy2(tex_path, tex_out)
                        replacements[tex_ref] = tex_path.name
                    elif not tex_path.exists():
                        # Create a 1x1 placeholder texture to avoid 404s
                        placeholder_name = Path(tex_ref).name
                        placeholder_out = meshes_dir / placeholder_name
                        if not placeholder_out.exists():
                            # Minimal valid 1x1 white PNG (69 bytes)
                            import base64
                            placeholder_out.write_bytes(base64.b64decode(
                                "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAIAAACQd1PeAAAADElE"
                                "QVR4nGP4//8/AAX+Av4N70a4AAAAAElFTkSuQmCC"
                            ))
                            print(f"  WARNING: texture not found, created placeholder: {placeholder_name}")
                        replacements[tex_ref] = placeholder_name
                dae_text = out_path.read_text()
                needs_write = False
                for old_ref, new_ref in replacements.items():
                    dae_text = dae_text.replace(old_ref, new_ref)
                    needs_write = True
                # 3) Force Y_UP so Three.js ColladaLoader doesn't add an
                #    extra -90° X rotation (the URDF root already handles Z→Y)
                dae_text = re.sub(
                    r"<up_axis>Z_UP</up_axis>",
                    "<up_axis>Y_UP</up_axis>", dae_text
                )
                needs_write = True
                # 4) Zero out high emission values that wash out textures
                def _fix_emission(m: re.Match) -> str:
                    vals = m.group(1).split()
                    if len(vals) == 4 and any(float(v) > 0.1 for v in vals[:3]):
                        return m.group(0).replace(m.group(1), "0 0 0 1")
                    return m.group(0)
                fixed = re.sub(
                    r"<emission>\s*<color[^>]*>([\d.\s]+)</color>\s*</emission>",
                    _fix_emission, dae_text
                )
                if fixed != dae_text:
                    dae_text = fixed
                    needs_write = True
                if needs_write:
                    out_path.write_text(dae_text)
            except ET.ParseError:
                pass

        # Copy MTL and textures referenced by OBJ files.
        # The companion MTL is always written with the same stem as the OBJ
        # (e.g. body.obj → body.mtl) so viewers that derive the MTL path from
        # the OBJ filename can find it (the mtllib directive inside the OBJ is
        # rewritten to match).
        elif resolved.suffix.lower() == ".obj":
            try:
                obj_text = resolved.read_text()
            except UnicodeDecodeError:
                obj_text = ""
            companion_mtl = out_path.with_suffix(".mtl").name  # e.g. "body.mtl"
            obj_needs_rewrite = False
            obj_replacements: dict[str, str] = {}
            for line in obj_text.splitlines():
                if line.strip().startswith("mtllib "):
                    mtl_name = line.strip().split(None, 1)[1]
                    mtl_src = resolved.parent / mtl_name
                    if not mtl_src.exists():
                        continue
                    # Copy as companion MTL (same stem as the OBJ)
                    mtl_out = meshes_dir / companion_mtl
                    if not mtl_out.exists():
                        shutil.copy2(mtl_src, mtl_out)
                    if mtl_name != companion_mtl:
                        obj_replacements[f"mtllib {mtl_name}"] = f"mtllib {companion_mtl}"
                        obj_needs_rewrite = True
                    # Parse MTL for texture map references
                    try:
                        mtl_text = mtl_src.read_text()
                    except UnicodeDecodeError:
                        continue
                    for mtl_line in mtl_text.splitlines():
                        parts = mtl_line.strip().split(None, 1)
                        if len(parts) == 2 and parts[0] in (
                            "map_Kd", "map_Ks", "map_Ka", "map_Ns",
                            "map_d", "map_bump", "bump", "disp", "decal",
                        ):
                            tex_name = parts[1].strip()
                            tex_src = mtl_src.parent / tex_name
                            if tex_src.exists():
                                tex_out = meshes_dir / tex_src.name
                                if not tex_out.exists():
                                    shutil.copy2(tex_src, tex_out)
            # Rewrite mtllib directive to point to the companion MTL
            if obj_needs_rewrite:
                obj_out_text = out_path.read_text()
                for old_ref, new_ref in obj_replacements.items():
                    obj_out_text = obj_out_text.replace(old_ref, new_ref)
                out_path.write_text(obj_out_text)

        mesh_map[filename] = f"meshes/{out_name}"
        print(f"  {resolved.name} ({resolved.suffix}) -> meshes/{out_name}")

    if not mesh_map and refs:
        print(f"  ERROR: No meshes could be resolved")
        return None

    # Step 4: Write rewritten URDF
    urdf_output = model_dir / "robot.urdf"
    rewrite_urdf(urdf_path, urdf_output, mesh_map, robot.get("tipFrames"))

    # Step 5: Validate (count DOF from the rewritten URDF since we modify joint types)
    actual_dof = count_non_fixed_joints(urdf_output)
    expected_dof = robot.get("dof")
    if expected_dof is not None and actual_dof != expected_dof:
        print(f"  WARNING: DOF mismatch — expected {expected_dof}, got {actual_dof}")

    # Validate tipLinks exist in the URDF
    tree = parse_urdf(urdf_path)
    link_names = {link.get("name") for link in tree.getroot().iter("link") if link.get("name")}
    for tip in robot.get("tipLinks", []):
        if tip not in link_names:
            print(f"  WARNING: tipLink \"{tip}\" not found in URDF")

    # Step 6: Build manifest entry
    entry = {
        "id": robot_id,
        "brand": robot["brand"],
        "name": robot["name"],
        "tipLinks": robot["tipLinks"],
        "category": robot.get("category", "arm"),
        "dof": actual_dof,
        "upstream": module_name or robot.get("git_repo", {}).get("name", ""),
        "urdf": f"models/{robot_id}/robot.urdf",
    }

    for field in ["reach", "weight", "payload", "dataSheet", "productPage"]:
        if field in robot and robot[field] is not None:
            entry[field] = robot[field]

    print(f"  OK — {len(seen_meshes)} meshes, {actual_dof} DOF")
    return entry


def generate_manifest(entries: list[dict]) -> None:
    """Write manifest.json to dist/."""
    manifest = {
        "version": 2,
        "generated": datetime.now(timezone.utc).isoformat(),
        "models": entries,
    }
    manifest_path = DIST / "manifest.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"\nManifest written to {manifest_path} ({len(entries)} models)")


def main():
    import sys
    filter_ids = set(sys.argv[1:])

    catalog = load_catalog()
    if filter_ids:
        catalog = [r for r in catalog if r["id"] in filter_ids]
        print(f"Processing {len(catalog)} robot(s): {', '.join(r['id'] for r in catalog)}")
    else:
        print(f"Loaded {len(catalog)} robots from robots.yaml")

    # Clean previous output
    if DIST.exists():
        shutil.rmtree(DIST)

    # Start persistent xacro container if any robots need it
    needs_xacro = any("xacro" in r for r in catalog)
    if needs_xacro:
        start_xacro_container()

    entries: list[dict] = []
    failures: list[str] = []

    try:
        for robot in catalog:
            entry = process_robot(robot)
            if entry:
                entries.append(entry)
            else:
                failures.append(f"{robot['brand']} {robot['name']} ({robot['id']})")
    finally:
        if needs_xacro:
            stop_xacro_container()

    generate_manifest(entries)

    print(f"\n{'='*60}")
    print(f"DONE: {len(entries)} succeeded, {len(failures)} failed")
    if failures:
        print(f"\nFailed robots:")
        for f in failures:
            print(f"  - {f}")


if __name__ == "__main__":
    main()
