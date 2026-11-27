# Joint GPS-denied mission: treeline to last room

One simulated mission, scored end to end. The Denied Signal Stack (DSS) flies the ingress with no GPS and hands its
position to frontier-fly at the treeline; frontier-fly flies the building search from there to the last room on its own
pose estimate. Nothing here flew a real drone.

## The pieces

- **Ingress (DSS).** DSS's footage lab renders a 60 to 90 s nadir pass from Spain's PNOA 2013 orthophotos and matches
  it against the 2009 flight of the same tile, vision only (GPS never fused). Its error at the end of the leg is the
  handoff error, and its protection level there says whether DSS vouches for it. 23 Madrid tiles: the 11 DSS was tuned
  on and the 12 it was then tested on fresh (DSS's `docs/footage/REAL-ORTHO.md`). DSS's `tools/footage/handoff.py`
  writes them to a `dss-handoff@1` file. PNOA © Instituto Geográfico Nacional de España, CC BY 4.0.
- **Search (frontier-fly).** The four bench scenarios (cabin-day, compound-clear, backpack-night,
  compound-occupied), 30 seeds each, in `sensorQuadLoc` mode: indoors the drone localises by lidar scan matching, IMU
  and optical flow. `RunnerOptions.outdoorPoseError` adds the outdoor error to everything the drone believes.
- **Arms**, paired on the same worlds and seeds (`scripts/joint/joint-mission.ts`):
  - *control*: no outdoor error, what GNSS would give.
  - *dss*: the handoff error, held constant.
  - *anchor*: the handoff error plus visual-inertial drift (1 % of distance flown) until the drone is within 20 m of
    a building; from then on it flies on its offset to that building's facade, 0.15 m per axis, wandering with a 5 s
    time constant. This step is **modelled, not simulated from lidar rays**, and 0.15 m is an assumption.
  - Each of *dss* and *anchor* is scored twice: alert ignored, and alert limit enforced (any handoff whose protection
    level is over 10 m holds at the treeline instead of flying).

## Result (120 missions per row)

| Arm | Expected outcome | 95 % CI | Held at treeline | Collisions | False clears | Indoor coverage |
|---|---|---|---|---|---|---|
| control | 117 (97.5 %) | 92.9 to 99.1 % | 0 | 0 | 0 | 94.6 % |
| DSS handoff, alert ignored | 3 (2.5 %) | 0.9 to 7.1 % | 0 | 87 | 0 | 6.2 % |
| DSS handoff, alert enforced | 3 (2.5 %) | 0.9 to 7.1 % | 51 | 61 | 0 | 8.9 % |
| DSS + facade re-anchor, alert ignored | 87 (72.5 %) | 63.9 to 79.7 % | 0 | 5 | 0 | 75.9 % |
| DSS + facade re-anchor, alert enforced | 49 (40.8 %) | 32.5 to 49.8 % | 51 | 4 | 0 | 73.7 % |

By scenario, re-anchor with the alert ignored: cabin-day 23/30, compound-clear 18/30, backpack-night 23/30,
compound-occupied 23/30.

What it says:

- **A constant metre or two of outdoor error is enough to crash the indoor search.** The median handoff error is
  3.3 m, and frontier-fly's building inspection flies about a metre off the walls. No absolute navigator without RTK
  holds that, so the handoff alone fails (2.5 %), and enforcing DSS's alert only turns crashes into holds.
- **Referencing the facade recovers most of it** (72.5 %), with no false clears in any arm. That is the design the
  joint mission points to: DSS gets the drone to the building, and the building gets it inside.
- **With re-anchoring, the 10 m alert limit at the treeline is conservative.** 38 of the 51 flights it held would have
  succeeded in the re-anchor arm, because re-anchoring forgives the handoff error as long as the drone reaches a
  building. A better rule would be "within 20 m of a building", not "within 10 m of truth".

## Limits

- The re-anchor is modelled. A real one would match lidar returns against the building outline; its error and
  failure rate are unmeasured.
- The entry planner scores openings from the opening's believed position but casts its rays against the true
  geometry, so a pose error that only shifts the frame can still make the planner judge a usable opening unsafe
  (cabin-day seed 47103: a 0.66 m window scores 0.23, "never", against 0.59, "ask", with the true pose). This
  probably understates the re-anchor arm. Fixing it changes the shared planner and FrontierBench numbers, so it is
  left as follow-up.
- The handoff is static: the drone starts the search where DSS left it, with no transition flight.

## Watch it

`joint.html` replays a DSS ingress leg in 3D over the 2013 imagery: the drone, what the camera sees, its map match,
the protection level, and a ghost that trusts GPS while it is spoofed. Each frame is a pure function of time, so
`?record=1` exposes `window.__joint.render(i)` for frame-exact capture. The leg data is generated, not committed:

```
# in DSS
PYTHONPATH=src python tools/footage/export_leg.py runs/0582_15-03 pairs/0582_15-03 clips/0582_15-03 \
    --out ../frontier-fly/public/joint/0582_15-03
# optional: the 24 tiles around it, dimmed, so wide shots see real ground to the horizon
PYTHONPATH=src python tools/footage/pnoa_context.py ../frontier-fly/public/joint/0582_15-03 --tile 0582_15-03
# in frontier-fly
npx vite --open /joint.html?leg=0582_15-03
```

## Reproduce

```
npx vite-node scripts/joint/joint-mission.ts -- --handoff handoff.json --part 0 --of 3 --out out/joint/j-0.jsonl
npx vite-node scripts/joint/joint-mission.ts -- --summary --handoff handoff.json out/joint/j-*.jsonl --out score.json
```
