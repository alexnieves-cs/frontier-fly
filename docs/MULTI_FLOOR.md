# Multi-floor flight

The report's multi-floor number (about 40% flown) is a time limit, not a flying limit. FrontierBench's `floors` task gives a two- or three-floor building the 120 s of one floor, and the metric counts 0.5 m voxels flown through, so at 2 m/s no pilot can fly more than about half of such a building ([REPORT.md](REPORT.md#climbing-and-map-prediction)). This page measures what happens when that limit is lifted, and does it on real buildings as well as generated ones.

**In short.** On the airframe's own battery the planner flies 95.0% of 30 fresh generated multi-floor buildings and 94.8% of 60 real two- and three-storey homes (the env stops a flight at 95%), reaching every floor of every building, in about 5 minutes on about a third of the pack. On 12 held-out real homes flown once it flies 95.0%, every floor in 12 of 12, with no contact in any flight. The view-first planner already brings 97.2% of the generated buildings and 95.3% of the real homes within 3 m of its sensor inside the old 120 s. A shaft search in the climber lifts the published methods most: NBVP's 120 s footprint on generated buildings goes from 57.6% to 94.8% and nearest frontier's from 42.0% to 90.4%.

## What was measured

Five pilots fly three building sets under three budgets. Every pilot is a sensor-only bench baseline ([`src/bench/baselines.ts`](../src/bench/baselines.ts)): the planner, the view-first planner, and the three published methods (boustrophedon BA\*, NBVP, nearest frontier), all on the kinematic airframe in daylight, all switching floors through the same climber ([`src/sim/climber.ts`](../src/sim/climber.ts)).

**Buildings.**

- *Generated*: 30 fresh two- and three-floor buildings from FrontierBench's `floors` generator, seeds 88101–88130.
- *Real homes*: 60 two- and three-storey homes (maisonettes) from [Swiss Dwellings](https://zenodo.org/records/7788422) (Archilyse AG, CC BY 4.0), env seeds 88201–88260. The importer ([`scripts/plans/import-swiss-homes.ts`](../scripts/plans/import-swiss-homes.ts)) keeps residential homes of two or three storeys at consecutive elevations with a staircase on every storey, one per building. The converter ([`src/plans/convert.ts`](../src/plans/convert.ts) `floorsToBuilding`) cuts every storey on the same axis cuts, so cells stack, and opens a shaft wherever a staircase on one storey overlaps the staircase on the next. The stair flight itself is not modelled: the drone flies an open column where the real stairs are.
- *Held-out homes*: the 12 further homes the same selection rule yields (none in the first set, none from the same buildings), env seeds 88801–88812, flown once after the last code change (see [the stairwell fix](#the-stairwell-fix)).

**Budgets.**

- *120 s*: FrontierBench's budget, whatever the number of floors.
- *120 s per floor*: the budget the bench's single-floor tasks give each floor.
- *Battery*: the airframe's own pack ([`src/shared/airframe.ts`](../src/shared/airframe.ts), 4S 35.7 Wh, energy from [`src/mission/autonomy/battery.ts`](../src/mission/autonomy/battery.ts)). The drone explores until 70% of the usable energy is spent, which holds 30% back for the way home and a landing reserve, or until the flight ends on its own: the env stops a flight at 95% flown coverage, after 25 s with nothing new flown, or on a crash. No flight here came near the energy line.

**Metrics.** *Flown* is the env's coverage, the share of reachable 0.5 m voxels the drone flew through. *Seen* is footprint coverage, the share of the reachable building that came within 3 m of the drone by ground-truth rays at its altitude ([`src/eval/footprint.ts`](../src/eval/footprint.ts)), the way exploration papers count it. *Every floor reached* counts flights that flew at least 8 voxels on every floor. A contact is any touch of a surface; the crash rule ends a flight on one faster than 0.5 m/s.

Development used FrontierBench's dev split (81401–81420) and the 20 dev homes (env seeds 88601+) only. Seeds are recorded in the project's seed ledger. The study is [`scripts/study/storeys-study.ts`](../scripts/study/storeys-study.ts) and its results are [`storeys-study.json`](storeys-study.json).

## Results

<!-- generated:storeys:start -->
**30 generated multi-floor buildings**

| Pilot | 120 s: flown | 120 s: seen | 120 s per floor: flown | 120 s per floor: seen | Battery: flown | Battery: seen (95% CI) | Battery: minutes, share of pack | Every floor reached (battery) | Flights with a contact (battery) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Sensor-only planner | 42.9% | 76.9% | 89.8% | 100.0% | 95.0% | 100.0% (100.0–100.0) | 5.4 min, 36% | 30 of 30 | 0 |
| Planner, view first | 43.3% | 97.2% | 89.4% | 100.0% | 95.0% | 100.0% (100.0–100.0) | 5.5 min, 35% | 30 of 30 | 0 |
| Boustrophedon (BA*) | 41.5% | 74.4% | 89.4% | 99.8% | 95.0% | 100.0% (100.0–100.0) | 5.4 min, 34% | 30 of 30 | 0 |
| NBVP (Bircher et al. 2016) | 25.7% | 94.8% | 26.0% | 96.7% | 26.0% | 96.7% (95.4–97.9) | 1.7 min, 11% | 30 of 30 | 0 |
| Nearest frontier (Yamauchi 1997) | 21.4% | 90.4% | 21.4% | 90.7% | 21.4% | 90.7% (88.7–92.7) | 1.5 min, 9% | 30 of 30 | 0 |

**60 real multi-storey homes**

| Pilot | 120 s: flown | 120 s: seen | 120 s per floor: flown | 120 s per floor: seen | Battery: flown | Battery: seen (95% CI) | Battery: minutes, share of pack | Every floor reached (battery) | Flights with a contact (battery) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Sensor-only planner | 51.0% | 74.0% | 85.2% | 93.9% | 94.8% | 99.5% (99.0–99.8) | 4.6 min, 30% | 60 of 60 | 0 |
| Planner, view first | 43.6% | 95.3% | 80.4% | 99.3% | 94.7% | 100.0% (100.0–100.0) | 5.1 min, 33% | 60 of 60 | 0 |
| Boustrophedon (BA*) | 50.2% | 74.8% | 85.2% | 94.0% | 94.9% | 99.8% (99.7–100.0) | 4.6 min, 29% | 60 of 60 | 0 |
| NBVP (Bircher et al. 2016) | 31.6% | 92.3% | 34.2% | 95.1% | 34.8% | 95.1% (92.7–96.9) | 2.7 min, 18% | 58 of 60 | 0 |
| Nearest frontier (Yamauchi 1997) | 19.4% | 79.2% | 19.6% | 79.7% | 19.6% | 79.7% (75.7–83.4) | 1.3 min, 8% | 52 of 60 | 0 |

**12 held-out real homes**

| Pilot | 120 s: flown | 120 s: seen | 120 s per floor: flown | 120 s per floor: seen | Battery: flown | Battery: seen (95% CI) | Battery: minutes, share of pack | Every floor reached (battery) | Flights with a contact (battery) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Sensor-only planner | 46.0% | 69.5% | 81.2% | 91.7% | 95.0% | 99.9% (99.7–100.0) | 5.0 min, 33% | 12 of 12 | 0 |
| Planner, view first | 39.0% | 89.9% | 77.0% | 100.0% | 95.0% | 100.0% (100.0–100.0) | 5.6 min, 36% | 12 of 12 | 0 |
| Boustrophedon (BA*) | 44.5% | 65.5% | 81.5% | 94.1% | 95.0% | 99.9% (99.7–100.0) | 5.0 min, 31% | 12 of 12 | 0 |
| NBVP (Bircher et al. 2016) | 29.7% | 87.7% | 35.1% | 93.9% | 33.5% | 87.6% (77.6–96.8) | 3.0 min, 20% | 9 of 12 | 0 |
| Nearest frontier (Yamauchi 1997) | 21.8% | 77.2% | 22.7% | 77.9% | 22.7% | 77.9% (72.6–83.3) | 1.6 min, 10% | 9 of 12 | 0 |
<!-- generated:storeys:end -->

## What changed in the climber

Two changes, both picked on the dev split.

**Shaft search.** The climber finds a shaft only with the up and down range finders, so only by flying under or over it. The planner flies every voxel and finds the stairs on the way. NBVP and nearest frontier map a floor from its doorways: they finish the floor without passing under the shaft, and used to hover there until the flight ended. The climber now flies a shaft search when the explorer is done and a floor it has never entered remains: the plain planner sweeping lanes 1 m apart (`SensorPlannerOptions.lanes`), which passes under every opening at least 1 m across, until a shaft turns up. **Faster climbs.** The climber climbs at the airframe's vertical cap, 1 m/s, easing over the last third of a metre (it was 0.9 m/s, easing over the last half metre).

Main's code flew the same buildings as a paired control:

<!-- generated:storeys-control:start -->
| Pilot | Buildings | 120 s: seen, main → this | 120 s: every floor reached, main → this | Battery: seen, main → this | Battery: every floor reached, main → this |
| --- | --- | --- | --- | --- | --- |
| Sensor-only planner | Generated | 77.0% → 76.9% | 26 → 26 of 30 | 100.0% → 100.0% | 30 → 30 of 30 |
| Planner, view first | Generated | 97.7% → 97.2% | 28 → 28 of 30 | 100.0% → 100.0% | 30 → 30 of 30 |
| Boustrophedon (BA*) | Generated | 74.5% → 74.4% | 24 → 25 of 30 | 100.0% → 100.0% | 30 → 30 of 30 |
| NBVP (Bircher et al. 2016) | Generated | 57.6% → 94.8% | 8 → 29 of 30 | 57.6% → 96.7% | 8 → 30 of 30 |
| Nearest frontier (Yamauchi 1997) | Generated | 42.0% → 90.4% | 4 → 30 of 30 | 42.0% → 90.7% | 4 → 30 of 30 |
| Sensor-only planner | Real homes | 72.4% → 74.0% | 50 → 53 of 60 | 96.7% → 99.5% | 56 → 60 of 60 |
| Planner, view first | Real homes | 95.3% → 95.3% | 53 → 53 of 60 | 96.5% → 100.0% | 55 → 60 of 60 |
| Boustrophedon (BA*) | Real homes | 73.5% → 74.8% | 54 → 56 of 60 | 97.1% → 99.8% | 56 → 60 of 60 |
| NBVP (Bircher et al. 2016) | Real homes | 74.9% → 92.3% | 34 → 56 of 60 | 74.9% → 95.1% | 32 → 58 of 60 |
| Nearest frontier (Yamauchi 1997) | Real homes | 52.0% → 79.2% | 16 → 50 of 60 | 52.0% → 79.7% | 16 → 52 of 60 |
| Sensor-only planner | Held-out homes | 67.6% → 69.5% | 10 → 10 of 12 | 100.0% → 99.9% | 12 → 12 of 12 |
| Planner, view first | Held-out homes | 89.8% → 89.9% | 9 → 9 of 12 | 100.0% → 100.0% | 12 → 12 of 12 |
| Boustrophedon (BA*) | Held-out homes | 66.5% → 65.5% | 11 → 11 of 12 | 99.9% → 99.9% | 12 → 12 of 12 |
| NBVP (Bircher et al. 2016) | Held-out homes | 81.8% → 87.7% | 7 → 9 of 12 | 77.4% → 87.6% | 6 → 9 of 12 |
| Nearest frontier (Yamauchi 1997) | Held-out homes | 43.6% → 77.2% | 1 → 9 of 12 | 43.6% → 77.9% | 1 → 9 of 12 |
<!-- generated:storeys-control:end -->

For the planners the change is small, because they already passed under the stairs. On the real homes their battery rows rise from about 96.5% seen to 99.5–100% and from 55–56 to 60 of 60 homes with every floor reached; most of that is [the stairwell fix](#the-stairwell-fix), since main has neither change. For NBVP and nearest frontier the search is the difference between mapping one floor and mapping the building. Nearest frontier still stops short on some homes (52 of 60 and 9 of 12 with every floor reached): those flights end early (about 1.3 minutes on average), and why the shaft search does not start or does not find the stairs on them is not diagnosed yet. NBVP flies few voxels by design (it views rooms from their doorways), so its *flown* column is low under every budget; *seen* is the column that compares it fairly.

On the quadrotor the search costs crashes. On FrontierBench Hard's `floors-quad` dev split (92401–92420), NBVP's camera footprint rises from 56.7% to 80.0% and nearest frontier's from 34.0% to 56.0%, but crashes rise from 0 to 4 and from 6 to 8 of 20 flights: instead of hovering once the ground floor is done, they now fly the rest of the building. The view-first planner rises from 87.0% to 88.1% with its 1 crash unchanged. Published FrontierBench results stay valid at the commits they name; a re-score after this change will move the published methods' multi-floor cells up.

## The stairwell fix

The first report run (commit 131f7d7) found a bug that generated buildings never show. A generated shaft is a straight column, so the climber climbs from the mean of its marked voxels. A real staircase can mark an L- or a U-shaped patch, and the mean of an L lies in the wall the stairs wrap around. On 4 of the 60 real homes every pilot flew to the stairs and stalled there, and none reached the upper storey. The climber now keeps the mean only when the marked patch surrounds it, and otherwise climbs from the most enclosed marked voxel. Dev results did not change. Those 4 homes are no longer held out for this fix, so the real-home rows above are a second look at the same 60 homes, and the 12 held-out homes are the clean check:

<!-- generated:storeys-first:start -->
| Pilot | Buildings | Battery: flown, first run → final | Battery: every floor reached, first run → final |
| --- | --- | --- | --- |
| Sensor-only planner | Generated | 95.0% → 95.0% | 30 → 30 of 30 |
| Planner, view first | Generated | 95.0% → 95.0% | 30 → 30 of 30 |
| Boustrophedon (BA*) | Generated | 95.0% → 95.0% | 30 → 30 of 30 |
| NBVP (Bircher et al. 2016) | Generated | 26.0% → 26.0% | 30 → 30 of 30 |
| Nearest frontier (Yamauchi 1997) | Generated | 21.5% → 21.4% | 30 → 30 of 30 |
| Sensor-only planner | Real homes | 92.3% → 94.8% | 56 → 60 of 60 |
| Planner, view first | Real homes | 92.2% → 94.7% | 56 → 60 of 60 |
| Boustrophedon (BA*) | Real homes | 92.4% → 94.9% | 56 → 60 of 60 |
| NBVP (Bircher et al. 2016) | Real homes | 34.8% → 34.8% | 57 → 58 of 60 |
| Nearest frontier (Yamauchi 1997) | Real homes | 19.6% → 19.6% | 52 → 52 of 60 |
<!-- generated:storeys-first:end -->

## Limits

- The stair flight is not modelled. A real drone climbing a stairwell has steps, banisters and a turning flight under it; here it has an open column of the staircase's footprint. Shafts are where the staircase polygons of two storeys overlap, which in some homes is one or two 1 m cells.
- The real homes are small: 2–3 storeys and about 140 m² on average. The converter stretches thin walls into 1 m cells, so walkable area is a little larger than the real one (as for the single-floor plans, [REAL_PLANS.md](REAL_PLANS.md)).
- Kinematic airframe and daylight only. The quadrotor numbers above are FrontierBench Hard dev flights, not a report run.
- The battery budget holds 30% of the pack back but does not fly the way home; on these buildings the explore leg used about a third of the pack, so the return is not what limits it.
- The learned policy is not in this study: it still has no way to change floors on purpose.
