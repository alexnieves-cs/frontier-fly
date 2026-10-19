# Observation layout

`OBS_VERSION = 2`, `OBS_DIM = 1088`. Every value is a float, scaled to roughly `[-1, 1]` or `[0, 1]` so an unnormalized MLP can read it. Offsets are exported as `OBS` from `src/shared/constants.ts` (re-exported by `src/sim/sensors.ts`); index the observation through `OBS`, never through literals.

This layout is the contract for checkpoints (`Checkpoint.obsVersion` and `obsDim`), the SharedArrayBuffer rollout stride (`src/workers/sab.ts`), and the SIMD gemm scratch (`src/rl/gemm.ts`, stride sized from `OBS_DIM`, batch ≤ 64). Checkpoints without `obsVersion` are v1. The app refuses a checkpoint whose `obsVersion` or `obsDim` differs (`checkpointMismatch` in `src/rl/checkpoint.ts`) and flies the lidar heuristic instead. A later change to any slot below needs `OBS_VERSION = 3`, not edits in place.

| Index | Count | `OBS.` | Signal |
| --- | --- | --- | --- |
| 0–31 | 32 | `lidar` | Horizontal lidar, yaw + `i * 2π/32`, range / 8 m (1 = miss or dropout) |
| 32–39 | 8 | `tilt` | Tilted lidar, ±0.45 rad elevation, every 45°, range / 8 m |
| 40–42 | 3 | `vel` | Body-frame velocity forward / left / up, divided by 2, 2, 1 m/s |
| 43 | 1 | `yawRate` | Yaw rate / (π/2 rad/s), noisy in quad mode |
| 44–45 | 2 | `sinYaw`, `cosYaw` | sin(yaw), cos(yaw) |
| 46–47 | 2 | `alt`, `ceil` | Distance to floor and ceiling / floor height, clamped to [0, 2] |
| 48–49 | 2 | `roll`, `pitch` | Radians. Zero in kinematic mode |
| 50 | 1 | `floor` | Floor index / (floors − 1) |
| 51 | 1 | `time` | Fraction of episode steps remaining |
| 52–53 | 2 | `wpDir` | Body-frame unit vector (forward, left) to the BFS lookahead waypoint, 0 when no frontier |
| 54 | 1 | `pathLen` | BFS path length to the nearest frontier voxel / 20 m, clamped; 1 when none |
| 55 | 1 | `hasFrontier` | 1 when the BFS reached a frontier voxel |
| 56–57 | 2 | `clusterDir` | Body-frame unit vector to the best coarse frontier cluster's centroid (straight line), 0 when none |
| 58 | 1 | `clusterDist` | Straight-line distance to that centroid / 32 m, clamped; 1 when none |
| 59 | 1 | `clusterMass` | That cluster's frontier voxels / 128, clamped |
| 60 | 1 | `stall` | Steps since the last new visited voxel / 50, clamped |
| 61 | 1 | `bfsTruncated` | 1 when the BFS hit its expansion cap before finding a frontier |
| 62 | 1 | `frontierTotal` | Frontier voxels on this floor / 512, clamped |
| 63 | 1 | `frontierDist` | Straight-line distance to the BFS target voxel / 8 m, clamped; 1 when none |
| 64–319 | 256 | `localVisited` | Egocentric 16×16 at 0.5 m (8 m): visited voxel, 0/1 |
| 320–575 | 256 | `localFrontier` | Same cells: seen-free and not visited, 0/1 |
| 576–831 | 256 | `coarseVisited` | Egocentric 16×16 at 2 m (32 m): fraction of the 4×4 voxel block visited |
| 832–1087 | 256 | `coarseFrontier` | Same cells: fraction seen-free and not visited |

Indices 0–47 keep their v1 slots, so lidar-only code (`src/sim/heuristic.ts`) reads the same indices under both versions. Velocity and yaw rate are now scaled (v1 had raw m/s and rad/s); the yaw rate is wrapped so a ±π crossing no longer reads as a 2π spike.

## Maps

Egocentric cell `(ix, iz)` is stored at `iz * 16 + ix`. `ix` runs along body forward and `iz` along body left, both centered on the drone (the drone sits on the corner shared by cells 7 and 8). The local map samples one voxel per cell center. The coarse map takes the 4×4 voxel block nearest each rotated 2 m cell center through summed-area tables; it is exact when yaw is a multiple of 90° and approximate in between. Cells outside the building read 0 in both channels, the same as walls and never-seen space: `localFrontier` / `coarseFrontier` are the channels that say "walkable, not yet flown".

## Seen-free map (sensor only)

`src/sim/seenMap.ts`. Each env step, before the observation is written, the 32 horizontal lidar readings (after noise) are traced with a voxel DDA from the drone to 0.15 m short of the return, on the drone's current floor. Voxels crossed are marked seen-free. Readings at max range mark nothing: they are either misses or dropouts (glass, night), and a dropout through a glass wall would otherwise paint the wall free. The grid uses the visited-grid layout `f * vw * vd + vy * vw + vx` (0.5 m voxels, `vw = floor0.w * 2`), lives in the env, and is cleared on `reset`. `env.seenGrid` exposes it. `observeAt` reads it without marking.

A frontier voxel is seen-free and not visited. Traversable voxels for the BFS are seen-free or visited.

## Navigation block

`src/sim/frontierFeatures.ts`, typed-array scratch reused across steps.

- BFS from the drone voxel over traversable voxels, 8-connected, diagonal moves only when both orthogonal neighbors are traversable (no corner cutting). It is level-synchronous: among the frontier voxels at the smallest hop count, the one nearest the drone wins. It pops at most 2048 voxels (a 20×20 m floor has 1600); hitting the cap sets `bfsTruncated`.
- The waypoint is the first path voxel at least 1.25 m along the path, or the target when the path is shorter.
- Coarse clusters: frontier voxels are binned into 2 m cells over the whole floor, and 8-connected cells form clusters. The chosen cluster maximizes `mass / (1 + distance / 8 m)`, so it is the largest cluster with a mild preference for near ones. Its direction complements the BFS, which always picks the nearest frontier.

## Cost

Measured on stage-3 seeds 4242 and 5150, driven by the frontier teacher (4 shared cores): env.step was 0.16–0.20 ms on v1 and is 0.18–0.24 ms on v2. The maps cost about 15 µs and the BFS plus clusters about 5–10 µs. Constants these loops read are copied into module locals, because under vite-node and vitest every read of an imported binding is a getter call.

## Future options (not in v2)

- Memory: a GRU over the observation or a small action/obs history. Out of scope for v2. The coarse 32 m map and the navigation block stand in for memory for now.
- A third map channel for "seen occupied" (lidar returns) to tell walls from unknown.
- Multi-floor frontier features (shaft voxels as targets on the floor above and below).

Actions are `[vx, vy, vz, yawRate]` in `[-1, 1]`, scaled inside the env to 2 m/s, 2 m/s, 1 m/s, and 90°/s. `vx` is forward, `vy` is left, `vz` is up.
