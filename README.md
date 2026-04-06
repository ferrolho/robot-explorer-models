# robot-viewer-models

Pipeline to process URDF robot descriptions for [robot-viewer](https://github.com/ferrolho/robot-viewer). Currently serves 81 robots from 35+ brands, sourced from [robot_descriptions](https://github.com/robot-descriptions/robot_descriptions.py) and [ROS-Industrial](https://github.com/ros-industrial) packages.

## How it works

1. Pulls URDF descriptions from upstream repos via [robot_descriptions](https://github.com/robot-descriptions/robot_descriptions.py)
2. Copies original mesh files (STL, DAE, OBJ) alongside textures to preserve materials
3. Rewrites URDF mesh references to relative paths
4. Generates a `manifest.json` with model metadata

Output is served via [jsDelivr CDN](https://www.jsdelivr.com/).

## Usage

### Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .
```

Some robots (ABB, KUKA) use ROS xacro files that require a ROS Noetic Docker container to render:

```bash
docker pull ros:noetic-ros-core
```

### Process models

```bash
python -m scripts.process
```

Output goes to `dist/`. Robots sourced from `robot_descriptions` are processed directly. Robots with xacro files are rendered via Docker automatically.

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
        base.dae
        shoulder.dae
        ...
    franka_panda/
      robot.urdf
      meshes/
        link0.dae
        link1.dae
        ...
```

## Branch structure

- **`main`** — Source code: Python scripts, robot catalog (`robots.yaml`), CI workflow
- **`dist`** — Build output only: processed meshes, rewritten URDFs, `manifest.json`. This branch is force-pushed by CI on each release. Binary assets are kept off `main` to avoid bloating clones.

## CDN URLs

jsDelivr serves files from the `dist` branch:

```
https://cdn.jsdelivr.net/gh/ferrolho/robot-viewer-models@dist/manifest.json
https://cdn.jsdelivr.net/gh/ferrolho/robot-viewer-models@dist/models/{id}/robot.urdf
```

## Acknowledgments

This project builds on top of [robot_descriptions.py](https://github.com/robot-descriptions/robot_descriptions.py) by Stéphane Caron and contributors, which provides a unified Python interface to 179+ robot descriptions from the robotics community. All robot models are sourced from their respective upstream repositories — this pipeline simply repackages them for web delivery.

## Roadmap

- **GLB conversion with material preservation**: The pipeline currently copies original mesh files (STL, DAE, OBJ) to preserve materials and textures. A future improvement is to convert these to GLB (binary glTF) using a tool that preserves materials (e.g., Blender headless or `assimp`), enabling Draco compression and significantly smaller download sizes (5-10x reduction). This would also allow multi-LOD support (low/medium/high mesh detail).
- **Expand catalog**: 5 robots currently fail processing (Fetch has invalid URDF XML; iiwa7, Mini Cheetah, Sigmaban, Skydio X2 have unresolvable mesh paths). Fix these as upstream descriptions are updated.
- **Automated validation**: CI step to verify each model loads correctly in a headless Three.js/urdf-loader environment.
- **CI pipeline**: Automate the full build (including Docker xacro rendering) in GitHub Actions so releases can be cut without local processing.
