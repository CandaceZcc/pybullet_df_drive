# Reference Repositories

The project uses external repositories as reading references only. Their code is not mixed into the main package.

Run:

```bash
bash scripts/sync_references.sh
```

The script clones the five repositories listed in `references/manifest.yml` into `references/repos/` and checks out fixed commits. `references/repos/` is ignored by git so large external trees do not pollute this project.

