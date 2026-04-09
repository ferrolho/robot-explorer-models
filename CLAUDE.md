# CLAUDE.md

## Project Overview

Pipeline to process URDF robot descriptions into web-optimized GLB meshes for [Robot Explorer](https://github.com/ferrolho/robot-explorer). Pulls from the [robot_descriptions](https://github.com/robot-descriptions/robot_descriptions.py) Python package (179+ robots), decimates meshes, converts to GLB with optional Draco compression, and publishes via jsDelivr CDN.

## Commands

- **Process all models:** `python -m scripts.process`
- **Process specific robots:** `python -m scripts.process atlas_v4 booster_t1`
- **Setup:** `python -m venv .venv && source .venv/bin/activate && pip install -e .`
- **Draco compression:** requires `npm i -g @gltf-transform/cli`

## Local Development

To iterate without waiting for CI, process locally and serve with CORS:

```bash
python -m scripts.process atlas_v4        # process one robot
python3 scripts/serve.py                  # serve dist/ on :8081 with CORS
```

Then in the robot-explorer repo:

```bash
VITE_MODELS_BASE_URL=http://localhost:8081/ npm run dev
```

## Releasing

CI only triggers on tag pushes (or manual dispatch). After pushing commits to `main`, tag and push to trigger the build.

## Architecture

- `robots.yaml` — curated catalog of robots to process (id, upstream module, metadata)
- `scripts/process.py` — main pipeline: URDF import, mesh decimation, GLB export, URDF rewriting, manifest generation
- `dist/` — build output (gitignored), published to `dist` branch by CI
- `.github/workflows/build-release.yml` — CI: tag push triggers processing and release

## Key Design Decisions

- Two LODs: low (~5k triangles) and medium (~25k triangles)
- One rewritten URDF per LOD (`robot_low.urdf`, `robot_medium.urdf`) with mesh paths pointing to the corresponding GLB directory
- `manifest.json` is the contract between this repo and robot-explorer
- Draco compression is optional (applied if gltf-transform CLI is available)
