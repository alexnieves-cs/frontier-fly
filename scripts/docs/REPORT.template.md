# Searching a building without its floor plan

A technical report on frontier-fly's indoor autonomy: how much of it works from the drone's own sensors, what the learned policy adds, and where it still fails. Every number here is rendered from a JSON file a script in this repository writes; the scripts are listed under [Reproduce](#reproduce).

## Summary

- **Nothing the mission flies indoors reads the floor plan any more.** Before this work, the mission's indoor planner, its viewpoint checks and its exit routes all read the building's true cell grid. They now run on a map the drone builds from its lidar, its up and down range finders, and its own estimate of where the camera has looked. The floor-plan planner is kept only as an *oracle* reference.
- **Missions did not get worse without the floor plan.** On {{R_M_FLIGHTS}} fresh mission worlds per explorer, the sensor-only explorer meets the expected outcome in {{R_M_SENSOR}} of flights, the oracle planner in {{R_M_ORACLE}}, and the policy with the sensor-only fallback in {{R_M_POLICY}}; the intervals overlap. {{R_M_COLL}} of the {{R_M_ALL}} flights touched anything. No flight declared a building clear while a target was inside it, and none confirmed a decoy.
- **After fine-tuning on its own flights, the learned policy is the stronger indoor coverage pilot, and the planner the safer one.** On 30 fresh hard buildings the sensor-only planner covers {{R_PLANNER_HARD}} with no collisions. The learned policy covers {{R_POLICY_HARD}} and averages {{R_POLICY_COLL}} collisions per episode; under a lidar guard it covers {{R_GUARD_HARD}} at {{R_GUARD_COLL}} collisions per episode. The oracle teacher it imitates covers {{R_TEACHER_HARD}}.
- **What the policy learns from is the frontier waypoint and the local maps.** Removing the navigation block costs 14 points of coverage and removing the local maps 7, with twice the collisions; the coarse maps change little. DAgger adds 11 points over behavior cloning, and PPO on the policy's own flights adds {{R_RL_DELTA}} more on {{R_RL_N}} held-out hard flights ({{R_RL_BASE_HARD}} to {{R_RL_HARD}}).
- **Outdoors, a probability search finds a lost person much sooner than the lane sweep the drone used to fly**: {{R_SAR_POA_MEAN}} restricted mean time against {{R_SAR_LANES_MEAN}} over {{R_SAR_SEEDS}} fresh daytime scenes, a paired difference of {{R_SAR_D_LANES}}. Most of that gain is not the prior: lanes spaced at the computed sweep width with buildings left for last differ from it by only {{R_SAR_D_SW}}, and the lost-person map features add nothing measurable over a bare distance ring. When the person is placed with no regard to the prior, the behavior prior costs time ({{R_SAR_UNI_D_SW}} against the sweep-width lanes).
- **On a quadrotor, a braking-model safety filter removes most collisions.** Quadrotor dynamics with sensor noise cost every pilot coverage ({{R_PLANNER_QUAD}} for the planner, {{R_POLICY_QUAD}} for the policy, {{R_TEACHER_QUAD}} for the oracle teacher). A velocity filter built on the airframe's measured stopping distance cuts the planner's collisions from {{R_Q_PLANNER_COLL}} to {{R_Q_PLANNER_BRAKE_COLL}} per episode and the guarded policy's from {{R_Q_GUARD_COLL}} to {{R_Q_GUARD_BRAKE_COLL}}. Whole missions flown on the quadrotor meet the expected outcome in {{R_MQ_SENSOR}} (planner) and {{R_MQ_POLICY}} (policy) of flights.
- **The planner now climbs.** Given the up and down range finders and a share of the budget per floor, the sensor-only planner takes shafts in the coverage environment. It reaches another floor in {{R_CLIMB_REACHED}} fresh multi-floor buildings and covers {{R_CLIMB_FLOORS}} of them against {{R_PLANNER_FLOORS}} without climbing, with {{R_CLIMB_COLL}} collisions per episode. The policy and the teacher still stay on the ground floor.
- **A rendered camera and a learned detector can fly the same missions.** The drone now renders what its camera sees and a small network, trained here on CC0 people and procedural objects, finds targets in the pixels. On {{R_V_PAIRS}} fresh paired flights it meets the expected outcome in {{R_V_VISION}} of them against {{R_V_ANALYTIC}} for the analytic detector, with no false alarms or false clears. The gap is all at night: under the flashlight alone it confirms the backpack in {{R_V_NIGHT_VISION}} of flights against {{R_V_NIGHT_ANALYTIC}}.
- **Learned map prediction is accurate and does not help coverage here.** A small network predicts the unseen flyable space behind a frontier far better than counting unknown voxels (R² {{R_MP_R2}} against {{R_MP_BASE_R2}}). Ranking frontiers by it leaves coverage unchanged ({{R_PRED_HARD}} against {{R_PLANNER_HARD}} on hard buildings), and so does the true hidden gain ({{R_ORACLEGAIN_HARD}}). In this environment coverage is limited by flight time, not by which frontier comes next.
- **A team of drones is the lever coverage was waiting for.** On {{R_TEAM_SEEDS}} fresh 40 × 40 m buildings with four minutes of flight, one sensor-only planner searches {{R_TEAM_SOLO_LARGE}}, three search {{R_TEAM_THREE_LARGE}} and four {{R_TEAM_FOUR_LARGE}}, sharing maps over a weak radio that walls attenuate. On the 20 m hard buildings three drones reach 80% in a median {{R_TEAM_THREE_HARD_EIGHTY}}; one drone alone does not get there in two minutes. Sharing maps is what matters: three drones without a radio do little better than one. Splitting frontiers adds a few points more and keeps the drones apart. A dedicated relay keeps the operator's picture fresher on a weak radio but costs a fifth of the coverage, so the shipped team has none.
- **The drone can estimate its own pose indoors.** An estimator that matches each lidar sweep against a map it builds itself, fused with the gyro and a downward optical-flow sensor, drifts {{R_LOC_DRIFT}} by day and {{R_LOC_DRIFT_NIGHT}} at night from its own map over about 200 m of flight in fresh hard buildings, and was 0.5 m off at some point in {{R_LOC_OVER}} flights. Dead reckoning on the same sensors drifts {{R_LOC_ODO}}. Displaced mid-flight, it noticed and re-acquired in {{R_LOC_KIDNAP}} flights. Flown closed loop, with every indoor map, path and command on the estimate instead of the simulator's pose, whole missions meet the expected outcome in {{R_ML_SENSOR}} of flights for the planner (vs {{R_ML_SENSOR_TRUTH}} on the true pose) and {{R_ML_POLICY}} for the policy (vs {{R_ML_POLICY_TRUTH}}), with the estimate {{R_ML_MAP}} from the drone's own map; daytime missions are unaffected and most of the gap is the night search.

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
| Pose estimator (`src/loc`) | Gyro, accelerometer, optical flow, barometer, down range finder and a 90-beam 2D lidar, all with noise; the entry pose with a 10 cm, 1.7° error | No. In the estimated-pose mission arms, every indoor map, path, shield check and command runs on its output; the fly-through in and out, outdoor flight and detection placement still use the true pose. |
| Confirm-clear (`clearStatus`) | Ground-truth cells the camera saw | Partly. It decides whether a building may be declared clear and whether a safety sweep flies a second pass, so it tells the drone one thing per building: whether its look was complete. It never routes the drone. |

## Methods

**Policy.** A 512×256 MLP maps the 1088-float observation to `[vx, vy, vz, yawRate]`. It is trained by behavior cloning on the oracle teacher, then DAgger: the policy flies, the teacher labels every state it reaches, and the round kept is the one that scores best on held-out validation buildings (seeds 9001–9166), never the evaluation seeds. The shipped weights then fine-tune with PPO on the policy's own flights, with a critic that sees privileged state and a small imitation term (see [Fine-tuning on its own flights](#fine-tuning-on-its-own-flights)); that checkpoint is picked on its own tuning block, 66001–66299.

**Sensor-only planner.** Dijkstra over voxels the drone has flown or seen free and not occupied, with a cost for passing near unknown or occupied space. The goal is the frontier voxel with the best distance minus a bonus for unknown neighbours, kept with hysteresis and banned after 60 steps without progress. It flies a pure-pursuit point on the path at constant speed and 1.2 m above the floor, between the 0.9 m and 1.4 m furniture heights so the lidar ring sees the taller pieces. Every command passes a lidar velocity filter: along each ray with a return, the approach speed is capped so the drone can stop before the surface.

**Lidar occupancy.** Lidar that passes through a small window hole paints the wall behind it free. Each voxel counts returns and pass-throughs, and is occupied when its returns exceed half its pass-throughs. That closes window holes and keeps doorways, whose jambs only graze a few rays, open.

**Braking filter for the quadrotor.** The kinematic filter assumes the drone stops within a fixed horizon. A quadrotor stops by tilting back, which takes a reaction time (control latency plus the attitude loop) and then a bounded deceleration. The filter models the stopping distance as S(v) = v·t<sub>r</sub> + v²/(2a) and caps the approach speed along each lidar ray at the speed whose stopping distance fits the free range less a margin. It uses the measured velocity, so a drone that is already too fast for the gap is told to slow by the excess. t<sub>r</sub> = 0.21 s and a = 5 m/s² are the 97th percentile of stopping distances over randomized quadrotor airframes at the worst latency, measured on the tuning buildings; a unit test checks that the 1.2 kg mission airframe stops inside the same curve. A missing lidar reading counts as a return at the margin. The flight controller's body-rate limit was also lowered from 8 to 4 rad/s, close to PX4's default of 220°/s: at 8 rad/s, light randomized airframes flipped during hard stops.

**Taking shafts.** Wherever the up range finder reads at least 0.8 m past the ceiling, or the down range finder reads at least 0.8 m past the floor, the planner marks the voxel as part of a shaft. After 80% of an equal per-floor share of the step budget, or once no frontier is left, the drone does three things. It routes through its own map to the centre of the nearest marked group that leads to an unexplored floor. It climbs or descends there while holding xy. Then it explores the new floor. If a column turns out to be blocked, the drone forgets it and keeps sweeping. The share and the 0.8 m threshold were chosen on the tuning buildings.

**Map prediction.** For each frontier candidate the planner can subtract a predicted *hidden gain* from its path cost. The gain is the number of flyable voxels the drone has neither seen nor flown that lie within 4 m of travel. The input is the drone's own maps around the candidate at 0.5, 1 and 2 m resolution, 11×11 cells each, with known-flyable, flown, unknown and inside-the-footprint fractions. A 128×64 MLP maps that input to the gain. It is trained on {{R_MP_TRAIN}} candidates sampled while the planner explores buildings from a seed stream no evaluation draws (`trainingBuilding(stage, seed, 'mappred')`), labelled by a breadth-first search on the building's true flight map. On {{R_MP_VAL}} held-out candidates its mean absolute error is {{R_MP_MAE}} voxels, against {{R_MP_BASE_MAE}} for a least-squares fit of the unknown-voxel count. The same search on the true map is kept as an oracle gain source, the ceiling for any predictor. Both arms use one weight, 0.3 path-cost steps per voxel of gain, chosen on the tuning buildings.

**Lidar guard for the policy.** The policy's action goes through the same velocity filter with a tighter margin, and its altitude is pulled to 1.15 m above the floor. The guard does not descend while a wall is within 0.6 m, because a sill under the lidar ring is invisible to it.

**Mission explorer.** Per floor: *sweep* (the planner), then *look* (fly to the viewpoint that shows the most voxels the camera has not looked at, per metre of travel, and turn a full circle), then take a shaft that the up or down range finder found to a floor not yet searched, and finally return to the entry floor. At night the look estimate models the flashlight cone around the tilted camera, so the floor right under the drone does not count as looked at. The policy can fly the sweep instead; on a stall or collision it hands over to the planner, which always does the look phase.

**Pose estimation** (`src/loc`). An extended Kalman filter over position, heading and gyro bias. Between sweeps it integrates the gyro and a body-velocity filter (the accelerometer predicts, the optical-flow sensor corrects; in the dark the flow sensor drops most samples). Each 10 Hz lidar sweep is matched by Gauss-Newton against the estimator's own occupancy map, point to line: every occupied cell keeps the mean of the returns that hit it, and its wall direction comes from its neighbours, so walls sit where the lidar saw them rather than at cell centres. The odometry prior enters the match as a Mahalanobis term. Three details carry most of the accuracy. A *Manhattan heading fix* reads the building's wall directions in each sweep: walls run along the axes the drone measured from outside, so the dominant wall direction measures heading modulo 90° and removes both the entry heading error and gyro drift. *Degenerate directions* (the axis of a bare corridor) get neither a correction nor information from the sweep (Zhang, Kaess and Singh, 2016), so slight tilts in the map's wall normals can't drag the pose along the wall. When sweeps keep contradicting the map (returns landing in space it knows is free), a *correlative search* over a widening window re-acquires. The estimator reports its own health: position sigma against the building and against its map, and tracking, degraded or lost.

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
- **The quad-aware tracker** (`SensorPlannerOptions.tracker`, src/sim/tracker.ts) shapes the planner's command into one the airframe can fly: it ramps the command at the airframe's acceleration, slows for corners and for the U-turn at the end of each sweep lane, and caps the yaw rate while facing the direction of travel. With the braking filter the planner then covers {{R_Q_BRAKE_TRACK}} ({{R_Q_BRAKE_TRACK_CI}}) at {{R_Q_BRAKE_TRACK_COLL}} collisions per episode, touching anything in {{R_Q_BRAKE_TRACK_EPS}} day episodes; without the filter it covers {{R_Q_TRACK}} at {{R_Q_TRACK_COLL}}. On the tuning buildings the yaw cap did most of the work. The env's command is body-frame velocity, so a drone yawing at full rate through a U-turn asks for a world velocity that keeps rotating under it, and with the quadrotor's lag its sweep lanes broke up into loops. The default controller and the trained checkpoints are unchanged; the tracker is opt-in.
- **Most of the coverage gap to the kinematic airframe is physical.** The planner covers by sweeping lanes one 0.5 m voxel apart, and the U-turn between two lanes has a 0.25 m radius, which the quadrotor (about 4 m/s² of acceleration at its 0.4 rad tilt limit) can only take at about 1 m/s. The kinematic drone takes it at cruise speed. On the tuning buildings the quadrotor also flies about 13% less distance in the same episode, and latency and noise slow every approach.

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

### Indoor localization

{{LOC_NOTE}}

{{LOC_HEADLINE}}

The shipped estimator's error against the building ({{R_LOC_ABS}} on average) is mostly the entry offset, which no amount of matching against its own map can remove; its drift against that map is about 5 cm. Without the lidar the same sensors drift {{R_LOC_ODO}} by day and at night, mostly from gyro bias ({{R_LOC_ODO_YAW}} of heading error on average). Matching sweeps without the IMU and flow drifts {{R_LOC_SCAN}} and occasionally loses the building outright. The 32-beam ring the policy reads is too sparse: {{R_LOC_SPARSE}} of drift, with {{R_LOC_SPARSE_OVER}} flights at some point 0.5 m off. The seen-free map the explorers plan on agrees {{R_LOC_IOU}} with the one painted from the true pose. The filter's own 95% ellipse held the true map-relative error {{R_LOC_HONEST}} of the time. The shipped estimator called itself lost while still accurate in {{R_LOC_FALSE_LOST}} of {{R_LOC_FLIGHTS}} flights.

**Kidnapped drone.** Halfway through each flight the estimate is moved 1 m and 15° without telling the filter, as a collision spin might. It re-acquired in {{R_LOC_KIDNAP}} flights, in {{R_LOC_KIDNAP_SEC}} on average:

{{LOC_KIDNAP}}

<details>
<summary>Every condition and estimator</summary>

{{LOC_TABLE}}

</details>

**Closed loop, in missions.** The `Loc` arms of the mission bench fly each explorer indoors on this estimate. The estimator starts at the doorway from the pose the fly-through ends at, plus a 10 cm and 1.7° error drawn per visit. From there the drone's lidar paints its maps where the estimate says it is, the planners and the shield read the estimate, and each velocity and heading command turns by the heading error on its way to the airframe, as a flight controller holding the estimated heading would fly it. When the estimate reports itself lost, the drone hovers so the matcher can re-acquire. If it is still lost after 20 s, the failsafe ([SAFETY_CASE.md](SAFETY_CASE.md)) lands it where it is. The failsafe also sends it out of the building slowly once the estimate's own 1-sigma error passes 0.35 m. Over {{R_ML_FLIGHTS}} flights, {{R_ML_INDOOR}} of which entered a building:

{{MISSION_LOC_TABLE}}

Daytime missions are unchanged: the cabin search succeeds in every world on either pose. The estimate costs {{R_ML_GAP}} points of expected outcome overall, and the Wilson intervals overlap. Most of it is the night search ({{R_ML_NIGHT}} on the estimate vs {{R_ML_NIGHT_TRUTH}} on the true pose), where the flashlight cone leaves little margin for a re-view placed 15–20 cm off; in an earlier check on 30 other night worlds (9117–9146), before the last two fixes below, the sensor planner found the backpack in 28 on either pose, so part of the gap may be these worlds. The rest is mostly compound sweeps that ended aborted. In those flights the estimate went lost and did not re-acquire within 20 s, so the failsafe landed the drone where it was. {{R_ML_COLL}} estimated-pose flights touched anything (vs {{R_ML_COLL_TRUTH}} for the same explorers on the true pose). The worst error in any flight was {{R_ML_WORST}}; flights ever 0.5 m off: {{R_ML_OVER}}. Flights that called the pose lost at some point: {{R_ML_LOST}}.

Closing the loop found faults that replaying logged flights could not, all fixed on the mission tuning worlds (9101–9116) or by construction:

- **Walls first seen edge-on read as contradictions.** A free ray grazing along a wall marks the cells beside it free; a later return from that wall, a few cm off, lands in one of those cells, and the estimator rejected whole sweeps as inconsistent with its map. A return now contradicts the map only where the cell and all its neighbours have been seen: free space next to unseen space is where new surfaces appear. On the first mission tried, matched sweeps went from 78% to all but one.
- **Shaft climbs relied on the voxel grid lining up with the building.** The explorer climbed or descended at the centre of the voxels where the range finder saw through to the next floor. On the true pose those 0.5 m voxels align with the 1 m building cells, so a voxel that saw through lies inside the shaft. Offset by the entry error they don't, and a drone 9 cm from the slab edge clipped it. Before a climb the drone now finds the hole's edges with the range finder itself, flying slowly along each building axis until the reading closes, and centres between them. This changes the true-pose arms too.
- **Detections were placed with the true pose.** The camera model places a contact from the true pose, while the drone planned its re-views in the estimate's frame, so night re-views aimed 15–25 cm off the target. Contacts are now re-placed with the estimate. This fix came after a first full bench run, which it changed only in the estimated-pose arms (sensor planner at night: 73% to 80%).
- **A gyro read once per tick misses the turn.** The quadrotor's body rate sampled at the end of each 50 ms tick differs from its mean over the tick while the heading controller is turning, and the estimator, trusting it, drifted 15° in heading on quadrotor flights. A real IMU is read far faster than the control loop and its rate is integrated over the tick; the localizer now feeds the tick's heading change, and the quadrotor's heading error dropped to under 1°.
- **A sweep through the slab founded the next floor's map.** Climbing a shaft, the drone's estimated floor changes as it passes the slab, and the first sweep on the new floor founds that floor's map. Taken at slab height, that sweep sees the edge of the hole, not the rooms above, and nothing later matched it: the estimate coasted on odometry until it called itself lost. Sweeps within 35 cm of an upper slab are now neither matched nor mapped. It showed up after the slab geometry changed (a slab is now open only where a shaft joins two floors). It appeared on a tuning world and in a third full bench run, where the sensor planner's compound sweep ended aborted in 8 of 30 worlds. The fix was found on the tuning world, and the bench above is the fourth run.

### Ablations

{{ABLATION_NOTE}}

{{ABLATION_TABLE}}

What the ablations show, on day coverage without the guard:

- **DAgger is worth about 11 points over behavior cloning** (61.5% vs 50.5%), and BC alone collides more. Imitating the teacher only on the teacher's own states leaves the policy lost once it drifts off them.
- **The navigation block carries the most information.** Without the frontier waypoint and its distance (observation 52–63) the policy drops to 47.2%, the largest loss of any arm, and its validation curve starts lowest.
- **Local maps matter for safety as much as for coverage.** Without them the policy loses 7 points by day and 12 at night, and collides about twice as often as the full policy.
- **Coarse maps add little.** Removing them costs about 3 points by day and nothing measurable at night; the intervals overlap with the full policy's.
- **Anchored PPO does not help.** It scores slightly below the DAgger checkpoint it starts from on validation and on the study buildings, consistent with the earlier failed PPO attempts. The recipe in the next section, which does help, differs in scale, noise and critic.
- **Validation is noisy from round to round.** Each run's best round is round 3, and later rounds can drop by 30 points (round 5 of the full run scores 44%), which is why the round is picked on validation and why the shipped checkpoint averages rounds.

These runs use the default recipe without weight averaging, so the full arm is not the shipped checkpoint; it lands within a point of it on the same buildings.

{{CURVES_FIGURE}}

### Fine-tuning on its own flights

The DAgger policy only ever learned to copy its teacher. Here it learns from its own flights with PPO, starting from the DAgger weights. Three changes made PPO work where the anchored fine-tune above did not:

- **Scale and a privileged critic.** Rollouts run on every core, 32 environments at about 11,000 steps a second, in the same episodes the policy is evaluated in (2400 steps, 16-collision limit, a quarter of them in the dark). A 400-update run is 3.3 million steps and takes about 45 minutes on four CPU cores. The critic is a separate network that also sees what the policy cannot: true coverage so far, the true distance to the nearest unflown voxel, the teacher's action, the collision budget used and the voxels left. It only shapes the advantages and never ships.
- **Little exploration noise.** The policy's action noise is cut to a standard deviation of 0.1 (pre-tanh). At 0.2 the sampled flights crash into the 16-collision limit, and PPO learns from wrecks.
- **A small imitation term on a replay, or none.** The teacher labels every state the policy reaches, and a BC term on a reservoir of those labels (weight 0.05) keeps the policy near the teacher where PPO has no signal. Imitating only the current batch, or imitating without PPO at all, wrecks the policy within 20 updates: on its own states the teacher's labels pull toward walls the student cannot see the way the teacher does.

{{RL_NOTE}}

{{RL_TABLE}}

The fine-tuned policy covers {{R_RL_HARD}} of the hard buildings against {{R_RL_BASE_HARD}} for the DAgger checkpoint it started from, a paired change of {{R_RL_DELTA}} points (95% interval {{R_RL_CI}}) over {{R_RL_N}} flights, and collides {{R_RL_COLL}} times per episode instead of {{R_RL_BASE_COLL}}. On the coverage study's hard buildings it covers {{R_RL_STUDY_DAY}} by day and {{R_RL_STUDY_NIGHT}} at night, more than the oracle teacher it imitated. The teacher is a greedy nearest-frontier rule and PPO optimizes coverage over the whole episode; this report does not measure where the extra coverage comes from.

The runs behind it, with hard coverage on the tuning block (66001–66299, the only seeds used to pick checkpoints):

{{RL_RUNS_TABLE}}

{{RL_CURVES_FIGURE}}

**Memory did not help.** The same recipe with a small recurrent memory (a 64-unit GRU feeding the second layer, starting from zero so it changes nothing at first) reaches 72.2% on the held-out hard flights after 260 updates, against 72.4% for the feed-forward run at the same point, with fewer collisions (2.0 against 2.9 per flight). It trains about three times slower, so the shipped policy has no memory. Both runs are in the table above.

Every bench in this report that flies the policy was re-run once with the new weights: the coverage study, the speed study, the mission bench, the real-floor-plan bench and the camera bench. The ablation runs above and the FrontierBench baseline in [BENCHMARKS.md](BENCHMARKS.md) keep the DAgger policy they were measured with.

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

## Search by sight

Every mission above detects with an *analytic* camera: a recall curve over distance, light and blur, and a dice roll, with no pixels (`src/perception/detector.ts`). This section replaces it with a camera and a learned detector and flies the same missions again.

### Method

**Camera** (`src/vision/scene.ts`). A software rasterizer draws the drone's view at 160×120 from the same world the missions fly: walls with door and window cut-outs, floors, furniture, terrain, trees, people and objects. Shading has the sun, room lights and the drone's flashlight cone; a sensor model sets exposure from the scene, adds read noise that grows with gain, and quantizes to 8 bits, so night frames are dark and grainy. The renderer also writes a depth image and an instance image. The instance image is used only for labels and for scoring; the drone never reads it. It runs in Node and in the browser at about 4 ms a frame.

**People** are the CC0 KayKit mannequin ([CREDITS.md](../CREDITS.md)) baked into ten poses (standing, sitting, lying, hiding and six walking frames) by `scripts/vision/bake-humanoid.mjs`. The pose, shirt colour and turn match the 3D view. Decoys look different from what they imitate: the mannequin is plain grey plastic, and the real backpack and toolbox have a flap, straps or a handle that their look-alikes lack. The same details are in the 3D view.

**Detector** (`src/vision/detector.ts`). A CenterNet-style convolutional network with about 180k parameters reads the 160×120 frame and outputs, on a 20×15 grid, a heatmap for each of nine classes (person and eight objects) and a box size and offset. Decoys are trained as background. It is trained in TF.js with focal loss on 32,000 frames (80 per world), rendered from worlds 66001–66400 with random surface colours, shirt colours and light levels on three worlds in four. The checkpoint is the epoch with the lowest loss on practice frames from worlds 66501–66560. Confidences are calibrated on the same practice frames with a two-parameter logistic fit.

**From box to contact** (`src/vision/source.ts`). Each detection above a calibrated confidence of 0.3 becomes a contact. Its position is the ray through the box centre at the range of the nearest surface the depth image sees inside the box, plus 15 cm to the body centre and 3% noise. The camera is assumed to be RGB-D. Everything after that is unchanged: the same tracker, the same three-bearing confirmation at the same threshold, and the same re-view flights. Which space counts as looked at is still estimated geometrically, as before.

### Detector on fresh frames

On {{R_V_FRAMES}} from fresh worlds 67001–67060 the detector scores a mean AP of {{R_V_MAP}} at IoU 0.5, {{R_V_PERSON_AP}} for people. Calibration brings the expected calibration error from {{R_V_ECE_RAW}} to {{R_V_ECE_CAL}}. It fires on empty background {{R_V_BG_FP}} times a frame and called a mannequin a person {{R_V_MANNEQUIN}} times it saw one; it called the decoy pack a backpack {{R_V_DECOY_PACK}} times. It takes {{R_V_MS}} a frame on the CPU with TF.js's native backend.

{{VISION_CLASS_TABLE}}

Recall at a calibrated confidence of 0.5, by distance (labelled boxes in brackets):

{{VISION_RECALL_TABLE}}

![Eight fresh camera frames with the detector's boxes, day above and night below](report/vision-frames.png)

### Missions with the camera

Each scenario is flown on 30 fresh worlds (67101–67130) with both indoor explorers, once with the analytic detector and once with the camera, {{R_V_FLIGHTS}} camera flights in all. Contact gating was set on practice worlds 66601–66616 and not changed after.

{{VISION_MISSION_TABLE}}

{{VISION_RESULT_NOTE}}

## Failure gallery

Top-down views of chosen coverage episodes: walls dark, shafts orange, furniture grey, flown voxels light blue, the track blue, the start green, collisions red. The episodes are picked by rule from the coverage study (`scripts/study/gallery.ts`).

{{GALLERY}}

## Negative results and limitations

- **Imitation alone is bounded by its teacher.** The DAgger policy imitates an oracle that sees the floor plan, yet on fresh hard buildings it did not beat a planner that sees only lidar. Only PPO on its own flights took it past both, and it still collides where the planner does not.
- **The policy collides; the guard is what makes it safe.** A teacher that never collides gives no examples of recovering from near-contacts. The guard removes most collisions without retraining.
- **Only the planner climbs.** The teacher and the policy still chase ground-floor frontiers on multi-floor buildings. The planner's floor share is a fixed rule, not a plan over the whole building.
- **The policy was never trained on the quadrotor.** The braking guard keeps its collisions down and it now covers about as much there as the braking planner, but it still collides several times per flight where the planner rarely does. Training on the quadrotor, or a policy that outputs attitude rather than velocity, is not attempted.
- **The braking model is fitted, not derived.** Its two constants come from simulated stopping tests on the randomized airframes. A heavier airframe, a stronger wind or a slower controller needs them measured again.
- **Night confirmation is near the detector's ceiling.** Under the flashlight alone the per-frame confidence barely clears the 0.8 threshold, so confirmation needs several clean bearings. A target boxed in by furniture can leave too few.
- **Entry is conservative by design.** When no opening is safe enough, the drone does not enter and the sweep stays incomplete. That costs success rate and never a false clear.
- **The lost-person prior is not what makes the outdoor search fast.** Sweep-width lanes, a distance ring and a flat prior all do about as well by day. The strategy weights and ring sizes are hand-set, not fitted to incident data, and the simulated walker that places the person was written alongside the prior, so even this agreement is optimistic.
- **The search belief is overconfident early and when the prior is wrong.** It treats a detection as a find, while a find here needs a three-bearing confirmation, and it trusts its prior: with the person placed uniformly it predicts {{R_SAR_UNI_POS_900_PRED}} success and {{R_SAR_UNI_POS_900_OBS}} of searches succeed.
- **Teams fly a simple world.** Drones in a team cannot collide physically (they keep apart with a velocity filter, and the bench counts close passes: {{R_TEAM_CONTACTS}} steps within 0.35 m over every team episode, mostly around the shared entry). They share a radio channel with no contention, all fly the kinematic airframe by day. The bench reports battery energy, but no drone turns back for a low pack, since none comes close in 240 s.
- **The quadrotor can't use a faster cruise.** The planner steers with pure pursuit and a velocity command, which suits a drone that changes velocity almost instantly. A simple corner-speed tracker didn't help, and the evidence points to the airframe's acceleration (about 3 m/s² at the controller's tilt limit) rather than the planner. A full minimum-time trajectory planner, or a stiffer airframe, is untested.
- **The camera is a renderer, not a photograph.** People are one low-poly mannequin in ten poses with a coloured shirt, objects are boxes with a few details, and the detector trains and tests on the same renderer. Its scores say nothing about real images. Range comes from a perfect depth image plus 3% noise, and which space counts as looked at is still estimated geometrically, not from the detector.
- **Phones are not found.** The detector's AP for phones is near zero; at 160×120 a phone is a few pixels. No mission asks for one.
- **Localization is closed loop only inside buildings.** In the estimated-pose arms the drone starts each visit from the doorway with a 10 cm, 1.7° pose error and flies the visit on its estimate, but the fly-through in and out still servoes on the true pose (standing in for tracking the opening it can see), outdoor flight uses the true pose (standing in for GNSS), and the perception source places detections with the true pose. Each would add error the bench does not count.
- **The coverage study's policy and planner never fly on the estimate.** Only the mission bench does; the coverage numbers above are on the true pose.
- **The localization sensors are idealized in places.** Each lidar sweep is instantaneous (no motion distortion), the flow sensor's errors are white plus a per-flight scale error, walls are perfectly flat and axis-aligned (which the Manhattan heading fix relies on), and nothing in the building moves. There is no loop closure: drift that enters the map stays in it.
- **One simulator.** Everything here is measured in this repository's simulator; nothing has flown on hardware.

## Seeds and protocol

| Use | Seeds |
| --- | --- |
| Training | Curriculum seeds drawn by the world pool, excluding every seed below |
| Checkpoint selection | 9001–9166 (coverage validation buildings) |
| Picking the PPO fine-tune | 66001–66299 (stage 3 and 0, day and night) |
| PPO fine-tune comparison (reported) | 68001–68299, with the protocol seeds and coverage study buildings |
| Tuning the sensor planner and guard | Coverage buildings 9101–9108 |
| Tuning the mission explorer and re-views | Mission worlds 9101–9116 and the three showcase seeds |
| Coverage study (reported) | 20001+, 30001+, 40001+, 50001+ (30 each) |
| Speed study (reported) | 30001+ and 50001+, the coverage study's hard and quadrotor buildings |
| Mission bench (reported) | 9600–9629 |
| Tuning the outdoor search, its prior and POD model | 7100–7139 |
| Outdoor search bench (reported) | 7600–7699 (day), 7600–7659 (uniform person, night) |
| Tuning the team's frontier split | 66001–66016 (hard and large) |
| Drone-team bench (reported) | 70001–70030 (hard), 71001–71030 (large) |
| Training and checking the camera detector | Frames from worlds 66001–66400 (training), 66501–66560 (checkpoint and calibration), 67001–67060 (reported) |
| Tuning camera contacts | Mission worlds 66601–66616 |
| Camera mission bench (reported) | 67101–67130 |
| Tuning the pose estimator | Coverage buildings 7001–7012 (stage 3), mission worlds 9101–9116 for the closed-loop fixes |
| Localization study (reported) | 60001+ (hard) and 62001+ (easy), 30 each |
| Fixed eval protocol ([BENCHMARKS.md](BENCHMARKS.md)) | 101, 202, 303, 404 and 4242, 5150, 6160 |

## Reproduce

```bash
npm ci
npm run study:coverage   # docs/study.json: every pilot, 30 fresh buildings per condition
npm run study:missions   # docs/mission-bench.json: each indoor explorer, airframe and pose source, 30 fresh worlds per scenario
npm run study:team       # docs/team-bench.json and docs/report/team-curves.svg: 1 to 4 drones and the team ablations
npm run study:sar        # docs/sar-bench.json and docs/report/sar-belief.svg: six outdoor search methods
npm run train:ppo -- --init public/checkpoints/pretrained.bin --out ppo.bin   # PPO fine-tune (see scripts/train-ppo.ts for the shipped recipe)
vite-node scripts/study/rl-finetune.ts -- dagger=old.bin ppo=ppo.bin   # docs/rl-finetune.json
npm run study:speed      # docs/speed-study.json: coverage and energy at speed caps of 2 to 5 m/s
npm install --no-save @tensorflow/tfjs-node@4.22.0   # native TF.js for the camera scripts
npm run study:vision     # docs/vision-bench.json: missions with the camera and the learned detector
# Camera detector: scripts/vision/make-dataset.ts, train-detector.ts and eval-detector.ts (see their headers)
npm run study:loc        # docs/localization.json: pose estimator arms over 120 fresh flights
vite-node scripts/study/gallery.ts          # docs/report/*.svg
# Ablations: one train:indoor run per arm, then scripts/study/ablations.ts (see its header)
npm run docs             # re-render this page, BENCHMARKS.md and the README
```
