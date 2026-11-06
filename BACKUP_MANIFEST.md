# Dingyue LivSurf Research Archive

This private repository is a source-and-results snapshot of
`/home/vuser/2026livsurf_project/probing` prepared before the original virtual
machine is cleaned up.

## Included

- Material-probing Python source and launch files
- Franka FR3 MoveIt integration and continuous Cartesian/Pilz support
- MuJoCo models and probe primitives
- Docker and ROS 2 workspace helpers
- Documentation and unit tests
- CSV telemetry and PNG plots under `probing/output/`

## Intentionally excluded

- Original `.git` metadata and Codex/agent metadata
- Python bytecode and virtual environments
- ROS/colcon generated `build`, `install`, and `log` directories
- IDE settings and machine-local caches

The tracked `probing/output/` directory is intentional: it contains the summer
experiment results and is part of this archive even though the project's
working `.gitignore` normally excludes runtime output.

Archive owner and commit identity: `Dingyue04`.
