# Real floor plans

Every number elsewhere in this repository comes from buildings the generator made. This page measures the same pilots on 60 real apartments from the Swiss Dwellings dataset, next to generated buildings of the same size.

![Three real apartments and their sim grids](media/real-plans.svg)

## Result

Under the crash rule ([REPORT.md](REPORT.md#the-crash-rule)), real apartments are where the learned policy breaks. The sensor-only planner covers 85.5% of a real apartment, 8 points less than on a generated building of the same size, and never crashes. The policy alone (the checkpoint retrained under the crash rule, see [REPORT.md](REPORT.md#retraining-under-the-crash-rule)) covers 36.1% against 92.9% on the same-size generated buildings: 170 of its 180 real-apartment flights end in a crash. The lidar guard cuts that to 30 crashes and lifts it to 67.0%, still 18 points behind the planner.

| Pilot | Real, day (95% CI) | Real, night | Crashes, day (of 180) | Generated, same size, day (95% CI) | Crashes (of 180) | Real − generated, paired (95% CI) |
| --- | --- | --- | --- | --- | --- | --- |
| Teacher (privileged frontier planner) | 71.9% (64.5–78.7) | 71.9% | 75 | 82.3% (74.1–89.3) | 6 | -10.4 (-19.1 to -1.1) |
| Sensor-only planner | 85.5% (82.1–88.4) | 85.7% | 0 | 93.8% (92.8–94.7) | 0 | -8.3 (-11.3 to -5.9) |
| Learned policy | 36.1% (30.6–41.6) | 37.9% | 170 | 92.9% (90.5–94.7) | 21 | -56.8 (-62.5 to -50.9) |
| Policy + lidar guard | 67.0% (62.5–71.3) | 68.9% | 30 | 93.4% (91.6–94.8) | 9 | -26.4 (-30.8 to -22.0) |
| Waypoint follower | 22.5% (18.3–27.3) | 22.5% | 177 | 39.3% (33.8–44.8) | 174 | -16.8 (-22.8 to -10.8) |
| Lidar heuristic | 9.9% (7.9–11.9) | 11.0% | 0 | 26.1% (23.9–28.2) | 0 | -16.2 (-19.2 to -13.3) |

Coverage is the share of reachable 0.5 m voxels flown within 2,400 steps (the stage-3 protocol), counted until the flight ends. A crash is any contact faster than 0.5 m/s into a surface, and it ends the flight. Each of the 60 apartments is flown three times per lighting with different env seeds, and so is its generated control. Intervals are bootstrap intervals over apartments (each apartment's mean over its three flights); the last column pairs each apartment with its own generated control. Night uses the `night-dark` preset. The teacher and the waypoint follower read the true map, so their night numbers equal their day numbers.

Against the targets: the bare policy misses the 60% hard-building coverage goal on real apartments (36.1%, interval 30.6–41.6%); the guarded policy meets it (67.0%, interval 62.5–71.3%). Night is 105% of day for the policy and 103% for the guarded policy.

Earlier versions of this page, measured before the crash rule, let the drone slide along walls at any speed. Under those rules the previous checkpoint covered 74.3% alone and 80.7% guarded, and the teacher 88.0%. Those numbers are not comparable with the table above: the rule also ends 75 of the teacher's 180 flights. FrontierBench's apartment task still leaves contacts free, and there the retrain cost a few points: 72.8% to 68.8% for the policy alone, 79.4% to 72.4% guarded ([LEADERBOARD.md](LEADERBOARD.md)).

One flight per apartment was not enough. With the DAgger policy, the first run, with one flight each, put it at 60.4% on one seed block and 52.3% on another; the three-flight means per repeat were 52.3%, 55.4% and 55.7%. The numbers above are from the three-flight run only.

### Why real apartments are harder

A generated building the size of an apartment is about two long rooms joined by 2 m doors. A real apartment of the same footprint has three times as many rooms, and one cell in ten is a 1 m door or passage:

| Per building (means over the 60) | Real apartments | Generated, same size |
| --- | --- | --- |
| Rooms | 6.4 | 2.1 |
| Walkable cells | 88 | 63 |
| Walkable cells with walls on both sides (1 m passages and doorways) | 10.4% | 0% |
| Furniture pieces | 12.8 | 6.6 |

The policy was trained only on generated layouts, and generated doors and halls are at least 2 m wide: in 30 hard (stage-3, 20 × 20) buildings no walkable cell has walls on both sides. A 1 m door is new to it. The planner, which plans paths on its own lidar map, loses less. The teacher's difference is within noise (its interval includes zero).

## Dataset and license

[Swiss Dwellings](https://zenodo.org/records/7788422) v3.0.0 by Archilyse AG: about 45,000 apartments in about 3,100 Swiss buildings, digitized from architects' plans with walls, doors, windows and typed rooms. It is published under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/), which allows redistribution with attribution and a note of changes. Both travel inside the data file (`dataset.attribution` and `dataset.changes`) and are listed in [CREDITS.md](../CREDITS.md).

> Standfest, M., Franzen, M., Schröder, Y., Gonzalez Medina, L., Villanueva Hernandez, Y., Buck, J. H., Tan, Y.-L., Niedzwiecka, M., Colmegna, R. (2023). Swiss Dwellings: A large dataset of apartment models. Archilyse AG. Zenodo. doi:10.5281/zenodo.7788422

Licenses were checked on the publisher's record before anything was downloaded. Not used:

- CubiCasa5K is CC BY-NC-SA 4.0 (Zenodo record 2613548). The non-commercial clause does not fit an MIT repository.
- Modified Swiss Dwellings is CC BY 4.0 but is derived from the same source, so it adds nothing here.

## Pipeline

1. **Import** (`scripts/plans/import-swiss-dwellings.ts`, run once offline on the 1.1 GB `geometries.csv`). Keep residential apartments on one floor with no stairs and at least 3 rooms (42,081 of 45,175). Order them by a hash of the apartment id, keep one per building, and split buildings into dev (one in four, by a hash of the building id) and eval. Rotate each plan so its walls run along the axes, move it to the origin, round to 1 cm, and keep rooms, walls, doors and windows. Balconies, loggias, stairs, shafts and fixtures (sinks, bathtubs) are dropped.
2. **Convert** (`src/plans/convert.ts`, at load time in the app and the study). The sim's walls are whole 1 m cells, and real walls are 10-30 cm, so a plain 1 m raster either closes 1.2 m corridors or loses walls. The converter cuts each axis into cells by dynamic programming over a 5 cm lattice, minimizing Σ (length − 1)². Every wall lies inside one cell. Walls that face each other always keep a wall-free cell between them, so no room between them disappears. A cell centre inside a door's width earns a credit, so doors survive. Each cut cell becomes one 1 m sim cell, classified by sampling the plan at the cell centre and along the wall centre lines it holds. Doors that lead outside mark the entrance, and the drone starts just inside it. Each room becomes its largest inscribed rectangle for room bookkeeping; the rest of an L-shaped room is still walkable and counted. Rooms are furnished with the same seeded props as generated buildings.
3. **Select**. A plan is kept when it is at least 97% axis-aligned, converts into a building where every room is reachable at flight altitude, keeps all but at most one of its rooms, and fits in 28 × 28 cells (stage 3 is 20 × 20). Of the 126 plans examined to fill 20 dev and 60 eval slots, 29 were not axis-aligned enough, 15 lost more than one room, and 2 had fewer than 3 rooms left. The data file records these counts.

Converted apartments come out close to metric: across the 20 dev plans the walkable area is 1.14 times the real floor area, and they keep 124 of 132 rooms.

## Protocol

- Dev: 20 apartments, env seeds 74001+, matched controls 75001+. The converter was developed and checked on these only.
- Eval: 60 apartments from other buildings, env seeds 72001+ (repeat r adds 100 r), matched controls 73001+. Measured after the converter was frozen. Nothing was tuned on them.
- Controls: for each apartment, a stage-3 generated building with the same grid size.
- Pilots and checkpoint are the shipped ones (`public/checkpoints/pretrained.bin`), flown exactly as in [BENCHMARKS.md](BENCHMARKS.md).

```bash
npm run study:plans                                  # docs/plans-bench.json (4,320 flights, about 20 min on 4 cores)
vite-node scripts/plans/figure.ts                    # docs/media/real-plans.svg
# Rebuilding the plan set needs the dataset (0.9 GB zip):
curl -L -o sd.zip https://zenodo.org/api/records/7788422/files/swiss-dwellings-v3.0.0.zip/content
unzip sd.zip 'swiss-dwellings-v3.0.0/geometries.csv'
vite-node scripts/plans/import-swiss-dwellings.ts -- swiss-dwellings-v3.0.0/geometries.csv
```

In the browser, `sim.html?plan=0` (through `?plan=59`) flies an eval apartment, and `?plan=<id>` picks one by id.

## Limits

- Single floors only. Maisonettes and stairs are excluded.
- Walls are 1 m thick in the sim, as in every generated building. Rooms lose up to about half a meter per wall, and crowded corners (a bathroom between two closets) are stretched rather than dropped.
- Ceilings are the sim's 3 m, not the plans' 2.6 m.
- Furniture is the sim's generic seeded props, not the real fixtures. Bathtubs and kitchen counters sit below flight altitude anyway.
- 23% of examined plans were rejected as not axis-aligned and 12% for losing rooms, so the set leans toward rectilinear layouts.
- Three flights per apartment and lighting. Single flights vary a lot (see above), so per-apartment numbers in `docs/plans-bench.json` are noisy.
