# CLAUDE.md

## Project Overview

Pipeline to process URDF robot descriptions into web-optimized GLB meshes for [robot-viewer](https://github.com/ferrolho/robot-viewer). Pulls from the [robot_descriptions](https://github.com/robot-descriptions/robot_descriptions.py) Python package (179+ robots), decimates meshes, converts to GLB with optional Draco compression, and publishes via jsDelivr CDN.

## Commands

- **Process models:** `python -m scripts.process`
- **Setup:** `python -m venv .venv && source .venv/bin/activate && pip install -e .`
- **Draco compression:** requires `npm i -g @gltf-transform/cli`

## Architecture

- `robots.yaml` — curated catalog of robots to process (id, upstream module, metadata)
- `scripts/process.py` — main pipeline: URDF import, mesh decimation, GLB export, URDF rewriting, manifest generation
- `dist/` — build output (gitignored), published to `dist` branch by CI
- `.github/workflows/build-release.yml` — CI: tag push triggers processing and release

## Key Design Decisions

- Two LODs: low (~5k triangles) and medium (~25k triangles)
- One rewritten URDF per LOD (`robot_low.urdf`, `robot_medium.urdf`) with mesh paths pointing to the corresponding GLB directory
- `manifest.json` is the contract between this repo and robot-viewer
- Draco compression is optional (applied if gltf-transform CLI is available)
