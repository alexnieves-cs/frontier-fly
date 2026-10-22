# Intel Research Lab (Seattle) laser log

Real robot data for the indoor pose estimator (`src/loc`). `npm run study:intel` replays it and writes [docs/intel-lab.json](../../docs/intel-lab.json); the write-up is in [docs/REPORT.md](../../docs/REPORT.md#real-data-intel-research-lab).

| File | What | Changes |
| --- | --- | --- |
| `intel.flaser.gz` | The 13,631 `FLASER` lines of `intel.clf`: one 180-beam SICK LMS sweep each (1° steps, right to left, 81.83 m = no return) with the wheel-odometry pose and timestamps | Only the `FLASER` lines kept (the `ODOM` and `PARAM` lines dropped), gzipped. Lines unchanged. |
| `intel.relations` | 2,984 manually verified relations between pairs of sweeps: `t1 t2 x y z roll pitch yaw`, the pose at `t2` in the frame of the pose at `t1` | None |

## Source and license

- The log was recorded by Dirk Hähnel (University of Washington / Intel Research Seattle, 2003) and published on [Radish, the Robotics Data Set Repository](https://dspace.mit.edu/handle/1721.1/62287). The MIT DSpace record of the Radish item states [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/).
- `intel.clf` and `intel.relations` are the copies in the [Freiburg SLAM benchmark](http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php) by R. Kümmerle, B. Steder, C. Dornhege, M. Ruhnke, G. Grisetti, C. Stachniss and A. Kleiner, "On measuring the accuracy of SLAM algorithms", *Autonomous Robots* 27(4):387–407, 2009. That page states: the Radish datasets "are available under the Creative Commons Attribution License" ([CC BY 1.0](http://creativecommons.org/licenses/by/1.0/)), and "the manually obtained relations for all the datasets are available under the Creative Commons Attribution License" ([CC BY 3.0](http://creativecommons.org/licenses/by/3.0/)).

## Fetching it again

`bash scripts/loc/fetch-intel.sh` downloads both files from the benchmark, checks their SHA-256 and rewrites this folder:

```
8db0ad6231b48bad4118e6a2dc78e83213b2c306be3b4ed80a90969891abfe7b  intel.clf
330378f153eeae652d6756c1746f335801f66c3e097d90611548e6c4f593c359  intel.relations
```

The copies here were made that way on GitHub Actions on 2026-10-22 (the development container's network can't reach the host).

## Format notes

- The sweeps' ipc timestamps are not monotonic: 631 of them step back, by up to 0.9 s. File order is the order the sweeps happened in (the odometry is continuous in file order and jumps when sorted by stamp), so the replay keeps file order, takes time steps from the logger clock, and the relations are matched to sweeps by exact stamp.
- Every one of the 1,791 distinct stamps in the relations names a sweep of the log.
