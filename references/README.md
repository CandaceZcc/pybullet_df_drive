# Reference Repositories

The project uses external repositories as reading references only. Their code is not mixed into the main package.

Run:

```bash
bash scripts/sync_references.sh
```

The script clones the repositories listed in `references/manifest.yml` into `references/repos/` and checks out fixed commits. `references/repos/` is ignored by git so large external trees do not pollute this project.

Current references include PyBullet differential-drive basics, path planning, mobile robot examples, official Bullet examples, slope/terrain examples, and `padawanabhi/pybullet_sim` for sensor streams, obstacle environments, navigation, and RL-ready environment structure.

For tracked-proxy V2, prioritize Bullet's `snake.py` for `anisotropicFriction`, Bullet's `racecar_differential.py` for multi-joint linkage/control patterns, and `padawanabhi/pybullet_sim` for sensor/navigation structure. The main project still treats these repositories as references only; do not vendor their code into `slope_sim/`.
