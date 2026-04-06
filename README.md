# robot-viewer-models

Pipeline to process URDF robot descriptions into web-optimized GLB meshes for [robot-viewer](https://github.com/ferrolho/robot-viewer).

## Why GLB?

Upstream URDF models reference meshes in formats like STL, DAE (COLLADA), and OBJ. These are not ideal for web delivery:

- **STL** has no compression and no material/color data
- **DAE (COLLADA)** is XML-based (verbose) and poorly supported by modern web 3D tooling
- **OBJ** is text-based and requires separate material files

**GLB** (binary glTF) is the web-native 3D format — compact binary encoding, supported by Three.js and all major browsers, and can be further compressed with [Draco](https://google.github.io/draco/). Converting upstream meshes to GLB with mesh decimation typically reduces file sizes by 5-10x while preserving visual quality.

## How it works

1. Pulls URDF descriptions from upstream repos via [robot_descriptions](https://github.com/robot-descriptions/robot_descriptions.py)
2. Decimates meshes to multiple LODs using [trimesh](https://trimesh.org/)
3. Converts meshes to GLB (binary glTF), optionally with [Draco compression](https://github.com/donmccurdy/glTF-Transform)
4. Rewrites URDF mesh references to point to the processed GLB files
5. Generates a `manifest.json` with model metadata

Output is published as GitHub Releases and served via [jsDelivr CDN](https://www.jsdelivr.com/).

## Usage

### Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .
```

Optionally install `gltf-transform` CLI for Draco compression:

```bash
npm install -g @gltf-transform/cli
```

### Process models

```bash
python -m scripts.process
```

Output goes to `dist/`.

### Configuration

Edit `robots.yaml` to add, remove, or configure robots in the pipeline.

## Output structure

```
dist/
  manifest.json
  models/
    universal_robot_ur5/
      robot.urdf
      meshes/
        low/
          base_link.glb
          shoulder_link.glb
          ...
        medium/
          base_link.glb
          shoulder_link.glb
          ...
    franka_panda/
      ...
```

## Branch structure

- **`main`** — Source code: Python scripts, robot catalog (`robots.yaml`), CI workflow
- **`dist`** — Build output only: processed GLB meshes, rewritten URDFs, `manifest.json`. This branch is force-pushed by CI on each release. Binary assets are kept off `main` to avoid bloating clones.

## CDN URLs

jsDelivr serves files from the `dist` branch via tagged versions. After a tagged release, models are available at:

```
https://cdn.jsdelivr.net/gh/ferrolho/robot-viewer-models@v0.1.0/manifest.json
https://cdn.jsdelivr.net/gh/ferrolho/robot-viewer-models@v0.1.0/models/{id}/robot_{lod}.urdf
```
