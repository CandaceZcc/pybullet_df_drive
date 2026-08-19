# Reference Repositories

The project uses external repositories as reading references only. Their code is not mixed into the main package.

Synchronize every pinned checkout declared by the manifest:

```bash
bash scripts/sync_references.sh
```

Verify the existing checkouts without fetching or changing them:

```bash
bash scripts/sync_references.sh --check
```

Both modes are driven exclusively by `references/manifest.yml`. Synchronization materializes each repository under `references/repos/` at its exact pinned commit and rejects checkout/root/Git-metadata symlinks, redirected Git layouts or object stores, unapproved executable local Git configuration, replace metadata, inherited repository-selection variables, unsafe `assume-unchanged`/`skip-worktree` index flags, and modified, untracked, or ignored local content. Git hooks, fsmonitor commands, replacement objects, and lazy promisor fetches are disabled for every subprocess. The read-only check validates every checkout's local config, clean worktree, `HEAD`, and `origin`; for `stage: 4` entries it also proves every declared focus and license path belongs to the pinned Git object before checking the materialized path. `references/repos/` is ignored by git so large external trees do not pollute this project.

Current references include PyBullet differential-drive basics, path planning, mobile robot examples, official Bullet examples, slope/terrain examples, and `padawanabhi/pybullet_sim` for sensor streams, obstacle environments, navigation, and RL-ready environment structure.

For tracked-proxy V2, prioritize Bullet's `snake.py` for `anisotropicFriction`, Bullet's `racecar_differential.py` for multi-joint linkage/control patterns, and `padawanabhi/pybullet_sim` for sensor/navigation structure. The main project still treats these repositories as references only; do not vendor their code into `slope_sim/`.

## Stage 4 admitted references

The following repositories were verified on the observation dates recorded in `manifest.yml` from the official GitHub API, the exact branch ref returned by `git ls-remote`, and every declared first-party and third-party license file at the same commit. The Star count is an observation-time maturity signal only. Each repository is pinned in `manifest.yml` with `stage: 4`; none of these reference source trees is vendored into the production package or release archive. Earlier reference entries intentionally keep their legacy schema. Separately version-locked and license-audited build outputs, such as the optional Livox ROS overlay, follow the release dependency locks rather than this reading manifest.

| Repository | Pinned reading snapshot | License | Stars | Intended reference scope |
|---|---|---|---:|---|
| `bulletphysics/bullet3` | `63c4d67e337017f9d8b298c900e9aabdb69296e7` | Zlib | 14,673 | Official PyBullet `rayTestBatch` execution pattern and collision-result semantics |
| `eclipse-ecal/ecal` | `e9ca7cf7f3e39696f79c6506eb116f61f8948bc7` | Apache-2.0 | 1,033 | Raw publisher/subscriber, `SDataTypeInformation`, callback ownership, peer count and recorder patterns |
| `protocolbuffers/protobuf` | `ea6ec8d5b1601c6d4b43c512d36901856ff378e1` | BSD-3-Clause | 71,666 | Deterministic serialization, descriptor sets and Python/C++ wire compatibility |
| `foxglove/mcap` | `58db4435c68e92c58713d22938639ebe8c0ed2fd` | MIT | 1,022 | C++ writer/reader, raw Protobuf channels, indexes, CRC and file rotation |
| `facebook/zstd` | `5c7b7bad26808e6b40ac3b3d0075466e27738a9d` | BSD-3-Clause OR GPL-2.0-only | 27,491 | Reproducible source build, deterministic compression and ABI checks |
| `Livox-SDK/livox_ros_driver2` | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` | MIT for the project-owned files; bundled third-party notices also apply | 821 | `CustomMsg/CustomPoint`, ROS 2 Jazzy conversion and RViz configuration |
| `Livox-SDK/Livox-SDK2` | `68ae1e1dc77f61f03c95d7c2809831e198d0aedd` | MIT for the project-owned files; bundled third-party notices also apply | 490 | MID-360 packet/device definitions and the official driver build chain |
| `PointCloudLibrary/pcl` | `6a4f535faad59bb22afdcb4becafc2e35e99c711` | BSD-3-Clause | 11,078 | Independent PCD/PLY readers, writers and conversion smoke tests |
| `Livox-SDK/livox_laser_simulation` | `1cce1073633a062b92e30243a4c2920e45551bb5` | MIT | 338 | Official time-ordered MID-360 scan pattern and ray-plugin implementation |
| `hku-mars/FAST_LIO` | `7cc4175de6f8ba2edf34bab02a42195b141027e9` | GPL-2.0-only | 5,061 | Livox per-point timing, motion undistortion, scan-to-map and global PCD accumulation |
| `ANYbotics/elevation_mapping` | `f4b082c64a3e660980da53b33c7936a8f2a2ea22` | BSD-3-Clause | 1,847 | Point-cloud and pose fusion into a probabilistic terrain elevation map |

Two additional official repositories were evaluated but not admitted as source checkouts: `ros2/rosbag2` duplicates the direct MCAP C++ recording path, while `ros2/rviz` is consumed as the Ubuntu 24.04/Jazzy system application rather than built from source. The Livox Driver 2 runtime dependency on the system `rosbag2` package remains mandatory in `ros2-dependencies.lock`; “not admitted” does not remove that package dependency. If source-level diagnosis becomes necessary, use their `jazzy` branches rather than the default `rolling` branch. The observed Jazzy snapshots were `5fb00aaff1ec6ccaad926aa62006272bfa4300e9` and `52d11538aa5a30d8e4d3bc9befe63e3366904e76`, respectively.

For the MID-360 terrain-mapping follow-up, use Bullet's `examples/pybullet/examples/batchRayCast.py` as the PyBullet batch-ray boundary, then use the three new references as separate domain layers rather than transplanting a ROS/Gazebo stack into PyBullet. `livox_laser_simulation` is the scan-direction and timing oracle, `FAST_LIO` is the motion-compensated global 3D mapping reference, and `elevation_mapping` is the terrain-height fusion reference. The `inkccc/mid360_simulation` ROS 2 port was evaluated but not admitted because its `scan_mode/mid360.csv` is byte-identical at the Git blob level to the official Livox pattern already retained here.

Reading snapshots are not release dependency locks. The stage four build must separately pin the versions required by the approved ABI: eCAL `v6.1.1` (`bf0bc5734dd31c6315ebad907c92c2bb1edc1851`), Protobuf `v33.6` (`6e1998413a5bca7c058b85999667893f167434bc`), MCAP C++ `releases/cpp/v1.4.0` (`9e7838c3ea51336d84141a80e2ffb15c589d2f54`), Zstd `v1.5.7` (`f8745da6ff1ad1e7bab384bd1f9d742439278e99`) and PCL `pcl-1.14.0` (`f62c018b4fc7df3dc2c096918a8462a190f28bb8`).

Admission rules:

1. For every `stage: 4` entry, use the canonical `https://github.com/OWNER/REPOSITORY.git` URL, verify the repository through its official GitHub page/API, record a nonnegative Star count with a UTC observation timestamp, and provide exact `license_scope: first_party` plus nonempty `license`, `license_files`, `purpose`, and `focus` fields. Inspect every path in `license_files` and `third_party_license_files` at the same resolved commit; third-party notices cannot replace the repository's own `license_files`. The `first_party` scope must not be promoted into a whole-archive distribution license, and a multi-license expression must list every first-party file needed to support it. Star count is an advisory maturity signal, not an admission substitute for relevance, license, and reproducibility.
2. Resolve an exact branch ref with `git ls-remote --heads URL refs/heads/BRANCH`; store the exact 40-character SHA in `manifest.yml` and the current synchronization entry. A bare branch pattern is invalid because it can suffix-match another ref.
3. Record only the files needed for the stated scope. Never vendor repository code or assets into the production package without a separate license and maintenance review.
4. `references/repos/` remains ignored and is never included in the Ubuntu release archive.
