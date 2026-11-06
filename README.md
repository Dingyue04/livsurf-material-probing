# LivSurf Material Probing Research

Private archive of Dingyue's 2026 summer LivSurf material-probing work using
MuJoCo and a Franka FR3 through ROS 2 / MoveIt.

The runnable Python package, launch files, models, documentation, tests, and
experiment results are under [`probing/`](probing/). See
[`probing/README.md`](probing/README.md) for setup and usage instructions, and
[`BACKUP_MANIFEST.md`](BACKUP_MANIFEST.md) for the archive scope.

From the repository root, the package layout supports commands such as:

```bash
python3 -m probing.scripts.run_single_probe_mujoco \
  --primitive micro_compression
```

The tracked `probing/output/` directory contains the summer CSV telemetry and
PNG trajectory plots.
