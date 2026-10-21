# Searching a building without its floor plan

A technical report on frontier-fly's indoor autonomy: how much of it works from the drone's own sensors, what the learned policy adds, and where it still fails. Every number here is rendered from a JSON file a script in this repository writes; the scripts are listed under [Reproduce](#reproduce).

## Summary

- **Nothing the mission flies indoors reads the floor plan any more.** Before this work, the mission's indoor planner, its viewpoint checks and its exit routes all read the building's true cell grid. They now run on a map the drone builds from its lidar, its up and down range finders, and its own estimate of where the camera has looked. The floor-plan planner is kept only as an *oracle* reference.
- **Missions did not get worse without the floor plan.** On {{R_M_FLIGHTS}} fresh mission worlds per explorer, the sensor-only explorer meets the expected outcome in {{R_M_SENSOR}} of flights, the oracle planner in {{R_M_ORACLE}}, and the policy with the sensor-only fallback in {{R_M_POLICY}}; the intervals overlap. {{R_M_COLL}} of the {{R_M_ALL}} flights touched anything. No flight declared a building clear while a target was inside it, and none confirmed a decoy.
- **A classical sensor-only planner is the stronger indoor coverage pilot.** On 30 fresh hard buildings it covers {{R_PLANNER_HARD}} with no collisions. The learned policy covers {{R_POLICY_HARD}} and averages {{R_POLICY_COLL}} collisions per episode; under a lidar guard it keeps its coverage ({{R_GUARD_HARD}}) at {{R_GUARD_COLL}} collisions per episode. The oracle teacher it imitates covers {{R_TEACHER_HARD}}.
- **What the policy learns from is the frontier waypoint and the local maps.** Removing the navigation block costs 14 points of coverage and removing the local maps 7, with twice the collisions; the coarse maps and an anchored PPO fine-tune change little. DAgger adds 11 points over behavior cloning.
- **Outdoors, a probability search finds a lost person much sooner than the lane sweep the drone used to fly**: {{R_SAR_POA_MEAN}} restricted mean time against {{R_SAR_LANES_MEAN}} over {{R_SAR_SEEDS}} fresh daytime scenes, a paired difference of {{R_SAR_D_LANES}}. Most of that gain is not the prior: lanes spaced at the computed sweep width with buildings left for last differ from it by only {{R_SAR_D_SW}}, and the lost-person map features add nothing measurable over a bare distance ring. When the person is placed with no regard to the prior, the behavior prior costs time ({{R_SAR_UNI_D_SW}} against the sweep-width lanes).
- **On a quadrotor, a braking-model safety filter removes most collisions.** Quadrotor dynamics with sensor noise cost every pilot coverage ({{R_PLANNER_QUAD}} for the planner, {{R_POLICY_QUAD}} for the policy, {{R_TEACHER_QUAD}} for the oracle teacher). A velocity filter built on the airframe's measured stopping distance cuts the planner's collisions from {{R_Q_PLANNER_COLL}} to {{R_Q_PLANNER_BRAKE_COLL}} per episode and the guarded policy's from {{R_Q_GUARD_COLL}} to {{R_Q_GUARD_BRAKE_COLL}}. Whole missions flown on the quadrotor meet the expected outcome in {{R_MQ_SENSOR}} (planner) and {{R_MQ_POLICY}} (policy) of flights.
- **The planner now climbs.** Given the up and down range finders and a share of the budget per floor, the sensor-only planner takes shafts in the coverage environment. It reaches another floor in {{R_CLIMB_REACHED}} fresh multi-floor buildings and covers {{R_CLIMB_FLOORS}} of them against {{R_PLANNER_FLOORS}} without climbing, with {{R_CLIMB_COLL}} collisions per episode. The policy and the teacher still stay on the ground floor.
- **Learned map prediction is accurate and does not help coverage here.** A small network predicts the unseen flyable space behind a frontier far better than counting unknown voxels (R² {{R_MP_R2}} against {{R_MP_BASE_R2}}). Ranking frontiers by it leaves coverage unchanged ({{R_PRED_HARD}} against {{R_PLANNER_HARD}} on hard buildings), and so does the true hidden gain ({{R_ORACLEGAIN_HARD}}). In this environment coverage is limited by flight time, not by which frontier comes next.
- **A team of drones is the lever coverage was waiting for.** On {{R_TEAM_SEEDS}} fresh 40 × 40 m buildings with four minutes of flight, one sensor-only planner searches {{R_TEAM_SOLO_LARGE}}, three search {{R_TEAM_THREE_LARGE}} and four {{R_TEAM_FOUR_LARGE}}, sharing maps over a weak radio that walls attenuate. On the 20 m hard buildings three drones reach 80% in a median {{R_TEAM_THREE_HARD_EIGHTY}}; one drone alone does not get there in two minutes. Sharing maps is what matters: three drones without a radio do little better than one. Splitting frontiers adds a few points more and keeps the drones apart. A dedicated relay keeps the operator's picture fresher on a weak radio but costs a fifth of the coverage, so the shipped team has none.

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

**Braking filter for the quadrotor.** The kinematic filter assumes the drone stops within a fixed horizon. A quadrotor stops by tilting back, which takes a reaction time (control latency plus the attitude loop) and then a bounded deceleration. The filter models the stopping distance as S(v) = v·t<sub>r</sub> + v²/(2a) and caps the approach speed along each lidar ray at the speed whose stopping distance fits the free range less a margin. It uses the measured velocity, so a drone that is already too fast for the gap is told to slow by the excess. t<sub>r</sub> = 0.21 s and a = 5 m/s² are the 97th percentile of stopping distances over randomized quadrotor airframes at the worst latency, measured on the tuning buildings; a unit test checks that the 1.2 kg mission airframe stops inside the same curve. A missing lidar reading counts as a return at the margin. The flight controller's body-rate limit was also lowered from 8 to 4 rad/s, close to PX4's default of 220°/s: at 8 rad/s, light randomized airframes flipped during hard stops.

**Taking shafts.** Wherever the up range finder reads at least 0.8 m past the ceiling, or the down range finder reads at least 0.8 m past the floor, the planner marks the voxel as part of a shaft. After 80% of an equal per-floor share of the step budget, or once no frontier is left, the drone does three things. It routes through its own map to the centre of the nearest marked group that leads to an unexplored floor. It climbs or descends there while holding xy. Then it explores the new floor. If a column turns out to be blocked, the drone forgets it and keeps sweeping. The share and the 0.8 m threshold were chosen on the tuning buildings.

**Map prediction.** For each frontier candidate the planner can subtract a predicted *hidden gain* from its path cost. The gain is the number of flyable voxels the drone has neither seen nor flown that lie within 4 m of travel. The input is the drone's own maps around the candidate at 0.5, 1 and 2 m resolution, 11×11 cells each, with known-flyable, flown, unknown and inside-the-footprint fractions. A 128×64 MLP maps that input to the gain. It is trained on {{R_MP_TRAIN}} candidates sampled while the planner explores buildings from a seed stream no evaluation draws (`trainingBuilding(stage, seed, 'mappred')`), labelled by a breadth-first search on the building's true flight map. On {{R_MP_VAL}} held-out candidates its mean absolute error is {{R_MP_MAE}} voxels, against {{R_MP_BASE_MAE}} for a least-squares fit of the unknown-voxel count. The same search on the true map is kept as an oracle gain source, the ceiling for any predictor. Both arms use one weight, 0.3 path-cost steps per voxel of gain, chosen on the tuning buildings.

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

### Quadrotor dynamics and the braking filter

The quadrotor condition flies the same pilots on rigid-body dynamics with randomized mass, thrust, drag and motor lag, sensor noise and 0–2 ticks of control latency. The braking pilots are the planner and the guarded policy with the braking filter in place of the kinematic one; they run only on the quadrotor, where the model applies.

- **The planner** with the kinematic filter covers {{R_Q_PLANNER}} at {{R_Q_PLANNER_COLL}} collisions per episode. With the braking filter it covers {{R_Q_PLANNER_BRAKE}} at {{R_Q_PLANNER_BRAKE_COLL}}.
- **The guarded policy** covers {{R_Q_GUARD}} at {{R_Q_GUARD_COLL}} collisions per episode with the kinematic guard, and {{R_Q_GUARD_BRAKE}} at {{R_Q_GUARD_BRAKE_COLL}} with the braking guard. Fewer crashes leave it more of the episode to explore.
- **The coverage gap to the kinematic airframe is physical.** On the tuning buildings the quadrotor flies about 13% less distance in the same episode, because it accelerates at about 4 m/s² rather than instantly within the lag. The remainder comes from latency and noise, which slow every approach.

On whole missions, `policyQuad` and `sensorQuad` in the bench above fly the policy and the planner on the 1.2 kg quadrotor with the braking filter as the indoor shield: {{R_MQ_COLL}} of {{R_MQ_ALL}} quadrotor flights touched anything.

### Climbing and map prediction

- **Climbing** on {{R_PLANNER_FLOORS_N}} fresh multi-floor buildings: {{R_CLIMB_FLOORS}} ({{R_CLIMB_FLOORS_CI}}) against {{R_PLANNER_FLOORS}} ({{R_PLANNER_FLOORS_CI}}) for the same planner without it. The gain is modest because the step budget is shared. The drone gives up the slow tail of the first floor for the fast start of the next, and coverage of a whole building stays budget-limited.
- **Map prediction** on hard buildings covers {{R_PRED_HARD}} ({{R_PRED_HARD_CI}}), and the oracle gain covers {{R_ORACLEGAIN_HARD}}. On multi-floor buildings the two cover {{R_PRED_FLOORS}} and {{R_ORACLEGAIN_FLOORS}}. Neither moves coverage outside the noise. The planner already prefers frontiers that border flown and blocked space, and in a 2400-step episode the drone is still finding new space when time runs out. Which frontier it takes next costs a few steps, not whole rooms. A learned gain is likely to matter where the goal is to see a target soon rather than to fly every voxel. That is not measured here.

A geometry fix in this round changes the multi-floor numbers slightly for pilots that do not climb. Before it, on buildings with three or more floors, a slab opened over every shaft cell of the floor above it, while the physics kept some of those holes closed. The range finders could see through ceilings the drone could not fly through. Slabs and physics now open only where a shaft joins the two floors, and the ground-floor slab no longer has holes over shafts that lead up.

### Speed and energy

Every pilot above cruises at 2 m/s. This study raises the speed cap to 3, 4 and 5 m/s on the same 30 held-out hard buildings, on the kinematic drone and on the quadrotor with domain randomization, and meters energy along each flight. A full action now asks for the episode's cap, so the shipped policy flies faster without retraining. The planner's speed is set to the cap. Brackets are bootstrap 95% intervals; collisions per episode are in parentheses in the table. The momentum weight was chosen on the tuning buildings (`scripts/study/speed-diag.ts`).

{{SPEED_FINDINGS}}

{{SPEED_TABLE}}

{{SPEED_FIGURE}}

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

## Drone teams

The single-drone results above say coverage is limited by flight time. The direct answer is more drones. Here two to four copies of the sensor-only planner search one building together (`src/team/`).

### Method

- **Radio.** Every drone and a base radio outside the entry talk over a 2.4 GHz link modelled with the multi-wall model: free-space loss plus a fixed loss for each wall the straight line crosses (drywall 3 dB, glass 2 dB, wood 5 dB, concrete 12 dB, metal 26 dB, a floor slab 15 dB; typical 2.4 GHz values from ITU-R P.2040 and indoor measurements), and 4 dB of log-normal shadowing per packet. The transmitter is deliberately weak (0 dBm, sensitivity −82 dBm) so walls matter inside a 40 m building; the weak setting is −10 dBm. The radio reads the true floor plan, as radio waves do; no drone does.
- **Map sharing.** Each radio keeps the team map as far as it knows it (seen-free and flown voxels) and an append-only log of what it learned. Twice a second it sends every other radio the part of its log that radio has not acknowledged, at most 1,500 voxels per packet; a lost packet is resent next round. What a drone hears it forwards, so maps travel over several hops, through teammates, to the base.
- **Frontier split.** Each drone runs its own planner on its own copy of the team map. A frontier costs more within 6 m of a goal a lower-numbered teammate announced, and more when a teammate it has heard from is closer to it, in the spirit of coordinated multi-robot exploration and RACER's pairwise split. Without the fixed order, strong goal claims let two drones push each other off their goals in turn. The two weights and the order were picked from 25 settings on the tuning buildings (`docs/team-tuning.json`).
- **Relay.** In the relay arms the last drone explores until its mean link margin to the base drops below 6 dB, then flies back along its own track to the deepest point with 3 dB to spare and hovers there.
- **Separation.** Drones are not physics bodies in the simulator, so they cannot hit each other. Each drone instead cuts the part of its velocity that closes on a teammate within 1.5 m, as its lidar filter does for walls, and the bench counts steps where two drone centres come within 0.35 m.
- **Buildings and metrics.** *Hard* is the coverage study's 20 m stage-3 building and two-minute budget. *Large* tiles four hard floor plans 2 × 2 with doorways between them (the stage-3 generator itself stops at eight rooms, so a 40 m request gives halls wider than the lidar's range) and allows four minutes. Drones launch one second apart from the same entry. Coverage is the union of voxels flown. *Base knows* is the share whose report reached the base radio, the picture an operator could act on; the *awareness gap* is coverage minus base-known, averaged over the run and at its worst.

### Results

{{TEAM_NOTE}}

{{TEAM_TABLE}}

{{TEAM_FIGURE}}

Three drones on large buildings search {{R_TEAM_THREE_LARGE_D}} more than one. Scaling is close to linear up to three drones: coverage per drone-minute falls only a little as drones are added, and overlap, the flown voxels summed over drones divided by their union, stays near 1.1. On hard buildings two drones already reach the 95% stop, so more drones show up as time to clear rather than coverage.

Ablations, three drones:

{{TEAM_ABLATION_TABLE}}

- **Sharing maps is most of the gain.** Without the radio each drone explores alone and covers what the others already did: {{R_TEAM_NONE_D}} against the full team on large buildings, and overlap near 2.7.
- **Splitting frontiers adds a few points and keeps drones apart.** Without claims the drones share maps but chase the same frontiers: {{R_TEAM_NOCLAIMS_D}}, and they spend many more steps within 0.35 m of each other.
- **The radio limits what the base knows, not how much gets searched.** A perfect radio changes coverage by {{R_TEAM_PERFECT_D}}. With the 0 dBm radio the base trails the truth by {{R_TEAM_GAP}} (mean / worst); with the −10 dBm radio by {{R_TEAM_WEAK_GAP}}.
- **Energy follows drone count, and no pack runs low.** The battery column integrates the power model from the speed study (`DEFAULT_BATTERY`, 74 Wh pack) over each drone's flight. A 240 s search never uses more than {{R_TEAM_PACK}} of any drone's usable pack, so endurance does not bind at this budget; time does. Searched points per team Wh fall slowly with team size on large buildings ({{R_TEAM_WH_SOLO}} for one drone, {{R_TEAM_WH_FOUR}} for four), because extra drones spend some of their flight on ground a teammate already covered. Without the radio, each searched point costs more than twice the energy.
- **A dedicated relay is not worth a drone here.** On the weak radio it changes the worst awareness gap by {{R_TEAM_WEAK_RELAY_GAPMAX_D}} ({{R_TEAM_WEAK_RELAY_GAP}} against {{R_TEAM_WEAK_GAP}}) and coverage by {{R_TEAM_WEAK_RELAY_D}}. Three explorers already chain the link back to the base most of the time. A relay should pay where the building is deeper than the team can chain, or where an operator needs a live picture more than a fast one.

The live page (`team.html`) runs this code in the browser: the truth beside what the base sees, with the radio links drawn.

## Failure gallery

Top-down views of chosen coverage episodes: walls dark, shafts orange, furniture grey, flown voxels light blue, the track blue, the start green, collisions red. The episodes are picked by rule from the coverage study (`scripts/study/gallery.ts`).

{{GALLERY}}

## Negative results and limitations

- **Imitation is bounded by its teacher and adds little over a classical planner here.** The policy imitates an oracle that sees the floor plan, yet on fresh hard buildings it does not beat a planner that sees only lidar. The teacher's advantage is information, not skill, and the policy cannot inherit it.
- **The policy collides; the guard is what makes it safe.** A teacher that never collides gives no examples of recovering from near-contacts. The guard removes most collisions without retraining.
- **Only the planner climbs.** The teacher and the policy still chase ground-floor frontiers on multi-floor buildings. The planner's floor share is a fixed rule, not a plan over the whole building.
- **The policy was never trained on the quadrotor.** The braking guard makes the policy safe on it, but its coverage there is well below the planner's. Training on the quadrotor, or a policy that outputs attitude rather than velocity, is not attempted.
- **The braking model is fitted, not derived.** Its two constants come from simulated stopping tests on the randomized airframes. A heavier airframe, a stronger wind or a slower controller needs them measured again.
- **Night confirmation is near the detector's ceiling.** Under the flashlight alone the per-frame confidence barely clears the 0.8 threshold, so confirmation needs several clean bearings. A target boxed in by furniture can leave too few.
- **Entry is conservative by design.** When no opening is safe enough, the drone does not enter and the sweep stays incomplete. That costs success rate and never a false clear.
- **The lost-person prior is not what makes the outdoor search fast.** Sweep-width lanes, a distance ring and a flat prior all do about as well by day. The strategy weights and ring sizes are hand-set, not fitted to incident data, and the simulated walker that places the person was written alongside the prior, so even this agreement is optimistic.
- **The search belief is overconfident early and when the prior is wrong.** It treats a detection as a find, while a find here needs a three-bearing confirmation, and it trusts its prior: with the person placed uniformly it predicts {{R_SAR_UNI_POS_900_PRED}} success and {{R_SAR_UNI_POS_900_OBS}} of searches succeed.
- **Teams fly a simple world.** Drones in a team cannot collide physically (they keep apart with a velocity filter, and the bench counts close passes: {{R_TEAM_CONTACTS}} steps within 0.35 m over every team episode, mostly around the shared entry). They share a radio channel with no contention, all fly the kinematic airframe by day. The bench reports battery energy, but no drone turns back for a low pack, since none comes close in 240 s.
- **The quadrotor can't use a faster cruise.** The planner steers with pure pursuit and a velocity command, which suits a drone that changes velocity almost instantly. A simple corner-speed tracker didn't help, and the evidence points to the airframe's acceleration (about 3 m/s² at the controller's tilt limit) rather than the planner. A full minimum-time trajectory planner, or a stiffer airframe, is untested.
- **One simulator.** Everything here is measured in this repository's simulator; nothing has flown on hardware.

## Seeds and protocol

| Use | Seeds |
| --- | --- |
| Training | Curriculum seeds drawn by the world pool, excluding every seed below |
| Checkpoint selection | 9001–9166 (coverage validation buildings) |
| Tuning the sensor planner and guard | Coverage buildings 9101–9108 |
| Tuning the mission explorer and re-views | Mission worlds 9101–9116 and the three showcase seeds |
| Coverage study (reported) | 20001+, 30001+, 40001+, 50001+ (30 each) |
| Speed study (reported) | 30001+ and 50001+, the coverage study's hard and quadrotor buildings |
| Mission bench (reported) | 9600–9629 |
| Tuning the outdoor search, its prior and POD model | 7100–7139 |
| Outdoor search bench (reported) | 7600–7699 (day), 7600–7659 (uniform person, night) |
| Tuning the team's frontier split | 66001–66016 (hard and large) |
| Drone-team bench (reported) | 70001–70030 (hard), 71001–71030 (large) |
| Fixed eval protocol ([BENCHMARKS.md](BENCHMARKS.md)) | 101, 202, 303, 404 and 4242, 5150, 6160 |

## Reproduce

```bash
npm ci
npm run study:coverage   # docs/study.json: every pilot, 30 fresh buildings per condition
npm run study:missions   # docs/mission-bench.json: each indoor explorer and airframe, 30 fresh worlds per scenario
npm run study:team       # docs/team-bench.json and docs/report/team-curves.svg: 1 to 4 drones and the team ablations
npm run study:sar        # docs/sar-bench.json and docs/report/sar-belief.svg: six outdoor search methods
npm run study:speed      # docs/speed-study.json: coverage and energy at speed caps of 2 to 5 m/s
vite-node scripts/study/gallery.ts          # docs/report/*.svg
# Ablations: one train:indoor run per arm, then scripts/study/ablations.ts (see its header)
npm run docs             # re-render this page, BENCHMARKS.md and the README
```
