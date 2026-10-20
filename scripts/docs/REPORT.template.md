# Searching a building without its floor plan

A technical report on frontier-fly's indoor autonomy: how much of it works from the drone's own sensors, what the learned policy adds, and where it still fails. Every number here is rendered from a JSON file a script in this repository writes; the scripts are listed under [Reproduce](#reproduce).

## Summary

- **Nothing the mission flies indoors reads the floor plan any more.** Before this work, the mission's indoor planner, its viewpoint checks and its exit routes all read the building's true cell grid. They now run on a map the drone builds from its lidar, its up and down range finders, and its own estimate of where the camera has looked. The floor-plan planner is kept only as an *oracle* reference.
- **Missions did not get worse without the floor plan.** On {{R_M_FLIGHTS}} fresh mission worlds per explorer, the sensor-only explorer meets the expected outcome in {{R_M_SENSOR}} of flights, the oracle planner in {{R_M_ORACLE}}, and the policy with the sensor-only fallback in {{R_M_POLICY}}; the intervals overlap. {{R_M_COLL}} of the {{R_M_ALL}} flights touched anything. No flight declared a building clear while a target was inside it, and none confirmed a decoy.
- **A classical sensor-only planner is the stronger indoor coverage pilot.** On 30 fresh hard buildings it covers {{R_PLANNER_HARD}} with no collisions. The learned policy covers {{R_POLICY_HARD}} and averages {{R_POLICY_COLL}} collisions per episode; under a lidar guard it keeps its coverage ({{R_GUARD_HARD}}) at {{R_GUARD_COLL}} collisions per episode. The oracle teacher it imitates covers {{R_TEACHER_HARD}}.
- **What the policy learns from is the frontier waypoint and the local maps.** Removing the navigation block costs 14 points of coverage and removing the local maps 7, with twice the collisions; the coarse maps and an anchored PPO fine-tune change little. DAgger adds 11 points over behavior cloning.
- **Outdoors, a probability search finds a lost person much sooner than the lane sweep the drone used to fly**: {{R_SAR_POA_MEAN}} restricted mean time against {{R_SAR_LANES_MEAN}} over {{R_SAR_SEEDS}} fresh daytime scenes, a paired difference of {{R_SAR_D_LANES}}. Most of that gain is not the prior: lanes spaced at the computed sweep width with buildings left for last differ from it by only {{R_SAR_D_SW}}, and the lost-person map features add nothing measurable over a bare distance ring. When the person is placed with no regard to the prior, the behavior prior costs time ({{R_SAR_UNI_D_SW}} against the sweep-width lanes).
- **Two open problems are measured, not hidden.** No pilot in the coverage environment climbs to a second floor (the mission explorer does, through shafts), and every pilot collapses on quadrotor dynamics with sensor noise ({{R_PLANNER_QUAD}} for the planner, {{R_POLICY_QUAD}} for the policy, {{R_TEACHER_QUAD}} for the oracle teacher).

## Setup

**Coverage environment** (`src/sim/env.ts`). A drone explores a generated building: stage 0 is one room, stage 3 has rooms, hallways and loops, stage 4 adds a second floor joined by shafts. Furniture is solid. Coverage is the fraction of reachable 0.5 m voxels the drone has flown through. An episode ends at 95% coverage, after 2400 steps (1000 on stage 0) at 20 Hz, after 500 steps without new coverage, or after 16 collisions. Dynamics are kinematic with a 0.15 s velocity lag unless stated; the quadrotor condition uses rigid-body dynamics with randomized mass, thrust, drag, motor lag, latency and sensor noise.

**Missions** (`src/mission/autonomy`). A drone takes off outside, sweeps the area, finds buildings, inspects their envelope for openings, enters through one, searches inside, and either confirms the targets it was sent for, declares buildings clear, or reports that it could not. Three scenarios: a daytime cabin search, a two-building safety sweep in which one building has two floors, and a night search for a backpack with every light off and only the drone's flashlight, next to a look-alike decoy. The camera detector misses with distance, occlusion, darkness and motion blur, confuses classes, and fires on decoys (`src/perception`).

### What each component may read

| Component | Reads | Privileged? |
| --- | --- | --- |
| Oracle teacher (`src/sim/frontier.ts`) | True floor plan and furniture | Yes. Used only to label training data, and as a reference row. |
| Learned policy | Observation v2: lidar ring, tilt rays, range finders, velocity, and maps built from what the drone has seen and flown | No |
| Sensor-only planner (`src/sim/sensorPlanner.ts`) | Its lidar, its seen and flown voxels, lidar occupancy evidence | No |
| Mission indoor explorer (`SensorExplorer`) | The above, plus up and down range finders (shafts) and its own estimate of what the camera has looked at | No |
| Oracle mission planner (`indoor: 'oracle'`) | True floor plan | Yes. Reference only. |
| Confirm-clear (`clearStatus`) | Ground-truth cells the camera saw | Partly. It decides whether a building may be declared clear and whether a safety sweep flies a second pass, so it tells the drone one thing per building: whether its look was complete. It never routes the drone. |

## Methods

**Policy.** A 512×256 MLP maps the 1088-float observation to `[vx, vy, vz, yawRate]`. It is trained by behavior cloning on the oracle teacher, then DAgger: the policy flies, the teacher labels every state it reaches, and the round kept is the one that scores best on held-out validation buildings (seeds 9001–9166), never the evaluation seeds. An anchored PPO fine-tune is implemented and tested in the ablations.

**Sensor-only planner.** Dijkstra over voxels the drone has flown or seen free and not occupied, with a cost for passing near unknown or occupied space. The goal is the frontier voxel with the best distance minus a bonus for unknown neighbours, kept with hysteresis and banned after 60 steps without progress. It flies a pure-pursuit point on the path at constant speed and 1.2 m above the floor, between the 0.9 m and 1.4 m furniture heights so the lidar ring sees the taller pieces. Every command passes a lidar velocity filter: along each ray with a return, the approach speed is capped so the drone can stop before the surface.

**Lidar occupancy.** Lidar that passes through a small window hole paints the wall behind it free. Each voxel counts returns and pass-throughs, and is occupied when its returns exceed half its pass-throughs. That closes window holes and keeps doorways, whose jambs only graze a few rays, open.

**Lidar guard for the policy.** The policy's action goes through the same velocity filter with a tighter margin, and its altitude is pulled to 1.15 m above the floor. The guard does not descend while a wall is within 0.6 m, because a sill under the lidar ring is invisible to it.

**Mission explorer.** Per floor: *sweep* (the planner), then *look* (fly to the viewpoint that shows the most voxels the camera has not looked at, per metre of travel, and turn a full circle), then take a shaft that the up or down range finder found to a floor not yet searched, and finally return to the entry floor. At night the look estimate models the flashlight cone around the tilted camera, so the floor right under the drone does not count as looked at. The policy can fly the sweep instead; on a stall or collision it hands over to the planner, which always does the look phase.

**Confirmation.** A contact is confirmed from at least three bearings whose best confidences average above the mission threshold. Indoors on a young map, re-views wait until the map offers enough bearings, plan one spare bearing, narrow the spacing for a target in a corner, skip bearings that added nothing, re-view the weakest bearing first, and hold longer in the dark.

## Results

### Coverage on fresh buildings

{{STUDY_NOTE}}

{{STUDY_HEADLINE}}

<details>
<summary>Every condition and pilot</summary>

{{STUDY_TABLE}}

</details>

### Missions on fresh worlds

{{MISSION_BENCH_NOTE}}

{{MISSION_BENCH_TABLE}}

The night search is where the explorers differ most: the sensor-only explorer finds the backpack in {{R_M_NIGHT_SENSOR}} of worlds and the oracle planner in {{R_M_NIGHT_ORACLE}}. The oracle knows the floor plan, but the night changes (the flashlight-cone look estimate, closer re-views and a longer dwell) key off the drone's own sensing and do not apply to it. The compound sweep succeeds in {{R_M_COMPOUND}} of worlds for every explorer; the misses are worlds where no opening into one building is safe enough to enter, so the drone correctly reports the sweep incomplete. The policy hands {{R_M_HANDOFF}} of buildings to the planner, all after a stall and most of them in the compound sweep.

### Ablations

{{ABLATION_NOTE}}

{{ABLATION_TABLE}}

What the ablations show, on day coverage without the guard:

- **DAgger is worth about 11 points over behavior cloning** (61.5% vs 50.5%), and BC alone collides more. Imitating the teacher only on the teacher's own states leaves the policy lost once it drifts off them.
- **The navigation block carries the most information.** Without the frontier waypoint and its distance (observation 52–63) the policy drops to 47.2%, the largest loss of any arm, and its validation curve starts lowest.
- **Local maps matter for safety as much as for coverage.** Without them the policy loses 7 points by day and 12 at night, and collides about twice as often as the full policy.
- **Coarse maps add little.** Removing them costs about 3 points by day and nothing measurable at night; the intervals overlap with the full policy's.
- **Anchored PPO does not help.** It scores slightly below the DAgger checkpoint it starts from on validation and on the study buildings, consistent with the earlier failed PPO attempts.
- **Validation is noisy from round to round.** Each run's best round is round 3, and later rounds can drop by 30 points (round 5 of the full run scores 44%), which is why the round is picked on validation and why the shipped checkpoint averages rounds.

These runs use the default recipe without weight averaging, so the full arm is not the shipped checkpoint; it lands within a point of it on the same buildings.

{{CURVES_FIGURE}}

## Outdoor search for a lost person

Before this work the drone searched outdoors with a boustrophedon lawnmower: parallel lanes over the whole area at a fixed spacing, then every building it had found. That is the right plan when nothing is known about where the person is. A lost-person search almost always knows something: where they were last seen, and how lost people tend to move. This section replaces the lanes with a search that flies where the person is most likely to be, and measures it against the lanes on worlds it was never tuned on.

### Method

**Probability of area (POA).** The search area is a 2 m grid (`src/mission/autonomy/sar`). Each cell holds the probability that the person is there. Each building holds one number for its whole interior, and a *rest of world* term holds what the drone cannot reach: ground beyond the geofence, and ground hidden under dense tree cores or inside no-fly zones.

**Lost-person prior.** The prior is a mixture of five reorientation strategies of the kind catalogued in the lost-person behavior literature (Syrotuck 1977; Hill 1998; Koester 2008): staying put, following a trail, wandering, climbing for a view, and heading for shelter. Each strategy is a log-normal distance ring around the last known point, shaped by the map: the trail strategy is drawn to trail corridors, the view strategy to higher ground, the shelter strategy to the buildings (60% of its mass goes inside them). The weights and ring sizes are set by hand to be plausible for a short, local loss; they are not taken from the published tables. Every prior also gives each building the mass of its floor area, so a person can always be inside one.

**Probability of detection (POD).** For each camera frame, the model predicts the chance that the detector reports a person in each cell inside the gimbal camera's footprint, from range, viewing angle, light, motion blur and the sensor (RGB by day, thermal at night). A precomputed sky-view map gives, for every cell and 32 bearings, the elevation angles at which tree crowns and roofs block the sightline, so the model does not count looks through foliage.

**Belief update.** After every frame each cell's mass is multiplied by one minus its POD: Bayesian search with negative information (Stone, *Theory of Optimal Search*, 1975). The cumulative probability of success (POS) is the mass removed. A building's mass is cleared by the probability that its interior search would have found the person.

**Planner.** A receding-horizon beam search over 6 m legs in 12 headings, five legs deep, scores each path by the discounted mass it would remove, with a pull toward the richest remaining block so the drone does not strand itself. It flies two legs and replans. A building is visited when its mass per second of travel and search beats what the open ground offers. The search ends when 99% of the reachable mass is gone or the budget runs out.

**Confirmation outdoors.** The gimbal camera sees a person on the ground from 12–14 m, beyond the 9 m the confirmation tracker used to count, so outdoor views never counted toward confirmation. Outdoor tracks now count views to the gimbal's real range, and the drone drops to 7 m or 9.5 m above the person for its confirming looks when the air is clear. This applies to every outdoor search mode, the lanes included.

### Benchmark

Each scene is a seeded world, a last known point 8–20 m from the command post, and a person placed by simulating one of the five strategies as an agent that walks the map (the planner never sees which). In the *uniform* condition the person is instead placed uniformly over open ground and building floors, so the behavior prior is wrong by construction. At night the drone flies a thermal camera. Six methods fly every scene:

| Method | What it is |
| --- | --- |
| Lane sweep (shipped) | The lawnmower the drone flew before, lanes at its default 20% footprint overlap, buildings as found |
| Lane sweep, sweep-width spacing | The same lanes spaced at the sweep width computed from the POD model ({{R_SAR_SWEEP_WIDTH}}), buildings last: the strongest no-information baseline |
| Probability search | The method above |
| One-leg greedy | The same belief, flying the best single leg each time |
| Distance ring only | The prior with no map features |
| Flat prior | A uniform prior: the planner and belief with no lost-person knowledge |

{{SAR_NOTE}}

### Results

{{SAR_TABLE}}

{{SAR_CURVES_FIGURE}}

Paired over the same scenes (bootstrap 95% intervals; negative times are faster than the baseline):

{{SAR_PAIRED_TABLE}}

**What the numbers say.** Against the shipped lanes the probability search is faster in every condition (by day, {{R_SAR_D_LANES}}). The honest baseline is the sweep-width lanes, and against it the difference is {{R_SAR_D_SW}} by day and {{R_SAR_NIGHT_D_SW}} at night: within noise. Two changes explain most of the gain over the shipped lanes, and both are classical search theory rather than the prior: spacing lanes at the sweep width instead of a 20% image overlap, and leaving buildings until the open ground is done. The ablations agree. With the distance ring alone the probability search averages {{R_SAR_RING_MEAN}}, and with a flat prior {{R_SAR_FLAT_MEAN}}, against {{R_SAR_POA_MEAN}} for the full prior. In these scenes most people stay within a sweep width or two of the last known point, so any search that starts there and spirals out finds them in the first minute; the map features would need larger areas or longer walks to matter, and the benchmark does not show that they do.

Where the prior is wrong it hurts. With the person placed uniformly the full prior is {{R_SAR_UNI_D_SW}} slower than the sweep-width lanes, most likely because it spends its first minutes where it believes the person is, near the last known point. The one place it helps is buildings: in the uniform condition it searches every building interior that holds the person ({{R_SAR_UNI_POA_BLD}} found against {{R_SAR_UNI_SW_BLD}} for the sweep-width lanes), because every building carries prior mass and is visited when that mass is worth the trip.

By how the person moved (daytime scenes; found / median time to find):

{{SAR_STRATEGY_TABLE}}

{{SAR_BELIEF_FIGURE}}

**Is the model calibrated?** {{SAR_POD_NOTE}} The belief's own POS is checked against outcomes with blind flights: each scene is flown once more with nobody in it, so the POS curve runs to the end of the budget instead of stopping at a find. By day the model predicts {{R_SAR_POS_90_PRED}} success by 90 s and {{R_SAR_POS_300_PRED}} by 300 s; {{R_SAR_POS_90_OBS}} and {{R_SAR_POS_300_OBS}} of searches had confirmed the person by then. With the person placed uniformly the prior is wrong and the model says so poorly: it predicts {{R_SAR_UNI_POS_900_PRED}} success by the end of the budget against {{R_SAR_UNI_POS_900_OBS}} found.

{{SAR_CALIBRATION_FIGURE}}

## Failure gallery

Top-down views of chosen coverage episodes: walls dark, shafts orange, furniture grey, flown voxels light blue, the track blue, the start green, collisions red. The episodes are picked by rule from the coverage study (`scripts/study/gallery.ts`).

{{GALLERY}}

## Negative results and limitations

- **Imitation is bounded by its teacher and adds little over a classical planner here.** The policy imitates an oracle that sees the floor plan, yet on fresh hard buildings it does not beat a planner that sees only lidar. The teacher's advantage is information, not skill, and the policy cannot inherit it.
- **The policy collides; the guard is what makes it safe.** A teacher that never collides gives no examples of recovering from near-contacts. The guard removes most collisions without retraining.
- **No climbing in the coverage environment.** The teacher, the policy and the planner all chase ground-floor frontiers on two-floor buildings. Shaft climbing exists only in the mission explorer.
- **Quadrotor dynamics break every pilot.** Lag, latency and overshoot make the kinematic stopping distance wrong. The planner degrades least because its filter still brakes; a model-aware filter, or training on the quad, is needed.
- **Night confirmation is near the detector's ceiling.** Under the flashlight alone the per-frame confidence barely clears the 0.8 threshold, so confirmation needs several clean bearings. A target boxed in by furniture can leave too few.
- **Entry is conservative by design.** When no opening is safe enough, the drone does not enter and the sweep stays incomplete. That costs success rate and never a false clear.
- **The lost-person prior is not what makes the outdoor search fast.** Sweep-width lanes, a distance ring and a flat prior all do about as well by day. The strategy weights and ring sizes are hand-set, not fitted to incident data, and the simulated walker that places the person was written alongside the prior, so even this agreement is optimistic.
- **The search belief is overconfident early and when the prior is wrong.** It treats a detection as a find, while a find here needs a three-bearing confirmation, and it trusts its prior: with the person placed uniformly it predicts {{R_SAR_UNI_POS_900_PRED}} success and {{R_SAR_UNI_POS_900_OBS}} of searches succeed.
- **One simulator.** Everything here is measured in this repository's simulator; nothing has flown on hardware.

## Seeds and protocol

| Use | Seeds |
| --- | --- |
| Training | Curriculum seeds drawn by the world pool, excluding every seed below |
| Checkpoint selection | 9001–9166 (coverage validation buildings) |
| Tuning the sensor planner and guard | Coverage buildings 9101–9108 |
| Tuning the mission explorer and re-views | Mission worlds 9101–9116 and the three showcase seeds |
| Coverage study (reported) | 20001+, 30001+, 40001+, 50001+ (30 each) |
| Mission bench (reported) | 9600–9629 |
| Tuning the outdoor search, its prior and POD model | 7100–7139 |
| Outdoor search bench (reported) | 7600–7699 (day), 7600–7659 (uniform person, night) |
| Fixed eval protocol ([BENCHMARKS.md](BENCHMARKS.md)) | 101, 202, 303, 404 and 4242, 5150, 6160 |

## Reproduce

```bash
npm ci
npm run study:coverage   # docs/study.json: every pilot, 30 fresh buildings per condition
npm run study:missions   # docs/mission-bench.json: three indoor explorers, 30 fresh worlds per scenario
npm run study:sar        # docs/sar-bench.json and docs/report/sar-belief.svg: six outdoor search methods
vite-node scripts/study/gallery.ts          # docs/report/*.svg
# Ablations: one train:indoor run per arm, then scripts/study/ablations.ts (see its header)
npm run docs             # re-render this page, BENCHMARKS.md and the README
```
