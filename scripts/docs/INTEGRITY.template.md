# Drift and integrity: what a bad pose costs, and who knows when theirs has gone bad

_A writeup draft of two FrontierBench results. Every flight here is simulated in frontier-fly; nothing was flown on hardware. Every number is regenerated from bench output on each docs build: the drift tables from `bench/results/hard-drift*`, the pilot board from `bench/results/hard-integrity`, the monitor study from [`integrity-dev.json`](integrity-dev.json) and [`integrity.json`](integrity.json). Scoring code: [`src/bench/drift.ts`](../src/bench/drift.ts), [`src/bench/integrity.ts`](../src/bench/integrity.ts). Full boards: [LEADERBOARD.md](LEADERBOARD.md#frontierbench-hard)._

- **Drift only hurts with weak ranging.** With a 90-beam lidar to 8 m, flying on the pose estimate costs no pilot more than noise. With the 24-beam, 4 m ring a light drone carries, it costs the view-first planner {{INT_HARD_COST}}, mostly in getting home.
- **The best integrity score goes to a pilot that reports nothing.** {{INT_SILENT_BEST}}, rarely loses its pose. The view-first planner, the only pilot that reports a protection level, ranks {{INT_PV_RANK}} at {{INT_PV_SCORE}}: it sees the most of the building and loses its pose the most.
- **The estimator's own 1σ is a poor integrity signal.** On the 40 m building with the short ring it sits over the alert limit whether the pose is good or bad. When sensors fail mid-flight it stays near {{INT_FAIL_BAD}} while the true error reaches {{INT_FAIL_WORST}}, so no multiple of it flags the failure reliably.
- **Open problem:** a monitor that catches the mid-flight failure without giving up the healthy steps. The track is scored and open to entries.

## 1. What flying on an estimate costs

A drone indoors has no GPS. On the drift tracks nothing in the pilot's loop reads the true pose: it flies on the indoor estimator of [`src/loc`](../src/loc) (gyro, accelerometer, optical flow, range finder and lidar matched against the map it builds), while the score is still measured on the truth. Each pilot also flies the same seeds on the true pose, so the difference prices the estimate alone. The score is the share of the building that passed through the forward camera's view within 3 m; return tasks count only a flight that ends within 1.5 m of take-off. Test seeds: drift 99001+, hard drift 111001+, 30 per task, scored once.

**Drift track** (90-beam lidar to 8 m, four tasks, three of them return tasks):

{{INT_DRIFT}}

With a good ranging sensor the estimate costs nothing measurable. Every pilot's gap to its own true-pose flights is a tie except the planner + quad tracker's, which scores a little higher on its estimate than on the truth; we have not found why. This board shows that the pilots do not depend on ground truth. It does not show that drift is hard.

**Hard drift track** (the ranging a light drone carries: 24 beams to 4 m; every task returns home):

{{INT_DRIFT_HARD}}

Here the estimate hurts. The view-first planner scores {{INT_HARD_EST}} on its estimate against {{INT_HARD_TRUTH}} on the true pose, a cost of {{INT_HARD_COST}}. Most of it is coming home: {{INT_HARD_HOME}}. The cost grows with how well a pilot does on the truth: the pilots that score highest on the true pose lose the most to the estimate, and the view-first planner still leads on its estimate.

## 2. Does the pilot know when it is lost?

Every pilot on those boards flies on its estimate and says nothing about how good it is. The integrity track asks it to. After every action a pilot may report a **protection level**: a horizontal radius, in metres, that it claims holds its true position. The terms come from satellite navigation integrity monitoring:

- **Alert limit:** 1.5 m, the return tasks' home radius. A step is *available* when the claimed level is within it.
- **Hazardously misleading:** the pilot claims a level within the alert limit while its true error is over it, for longer than a 2 s **time to alert** in one run.
- **Score:** availability minus 10 × the hazardously misleading rate, over the whole step budget. Honest doubt costs a step; confident error costs ten. A crash leaves the rest of the budget unavailable, so crashing out of a bad pose does not pay.

A pilot that reports nothing claims a level of 0: it flies as if its pose were exact. Four tasks on the randomized quadrotor and its own estimate: the 40 m building with the short-range ring, dark rooms with the ring, rooms where 120° of lidar and the optical-flow sensor fail together at a random time between 20 and 60 s, and rooms with healthy sensors. Test seeds 161001+, 30 per task, scored once.

{{INT_PILOTS}}

The view-first planner is the only pilot that reports a level, and it ranks {{INT_PV_RANK}} at {{INT_PV_SCORE}}. Against the leader, {{INT_SILENT_BEST}}, which reports nothing, the paired gap is {{INT_SILENT_GAP}}. The score is about the pose as flown, so it rewards a pilot that is never lost as much as one that knows when it is. The leader's pose is bad on {{INT_SILENT_BAD}} of steps and its camera sees {{INT_SILENT_CAMERA}} of the building; the view-first planner's is bad on {{INT_PV_BAD}} of steps and it sees {{INT_PV_CAMERA}}, the most of any pilot. Its monitor beats claiming nothing on the same flights by {{INT_MONITOR_GAIN}}{{INT_MONITOR_GAIN_NOTE}}.

## 3. Picking the monitor, and what it misses

A protection level does not change how the drone flies, so every candidate monitor can be scored exactly on one set of flights ([`scripts/bench/integrity-study.ts`](../scripts/bench/integrity-study.ts)). All candidates read only the estimator's own health: its 1σ position uncertainty and its tracking state. The house monitor was picked on {{INT_DEV_SEEDS}} and is reported on {{INT_TEST_SEEDS}}, the same flights as the pilot board above.

{{INT_MONITORS}}

The pick is {{INT_HOUSE_LABEL}}: {{INT_HOUSE_DEV}} on dev, {{INT_HOUSE_TEST}} on test. Always claiming the pose is good scores {{INT_ALWAYS_TEST}} on test; never claiming it scores {{INT_NEVER_TEST}}. Wider margins and "∞ once the estimator says it is degraded or lost" catch more bad steps but give up more good ones, and lose on both splits.

The monitor is the weak part, and the reason is the estimator's own uncertainty. On test the pose was over the alert limit on {{INT_BAD_TEST}} of steps ({{INT_BAD_FLIGHTS_TEST}} flights, worst {{INT_WORST_TEST}}), and the house monitor flagged {{INT_HOUSE_CAUGHT_TEST}} of those steps, leaving {{INT_HOUSE_HMI_TEST}} of the budget hazardously misleading. By task:

{{INT_TASKS}}

Two failure modes show up, one per hard task. On the 40 m building with the short ring, the 1σ barely moves with the true error: its median is {{INT_LARGE_GOOD}} when the pose is good and {{INT_LARGE_BAD}} when it is bad, both over the alert limit. The monitor flags most bad steps there, but only by saying "not sure" for most of the flight: it is available {{INT_LARGE_AVAIL}} of the time. When lidar and optical flow fail mid-flight, the opposite happens. The 1σ stays at a median of {{INT_FAIL_BAD}} while the true error grows to {{INT_FAIL_WORST}}, and the estimator still says it is tracking on {{INT_FAIL_TRACKING}} of those steps. The house monitor flags {{INT_FAIL_CAUGHT1}} of the bad steps there, and even eight times the 1σ flags {{INT_FAIL_CAUGHT8}}.

A monitor that reads only the estimator's health cannot see this failure, because the estimator does not see it either. The obvious next entry is a monitor with an independent check, such as lidar returns that keep disagreeing with the drone's own map, the signal the view-first planner already uses to slow down on the degraded-sensor track. It is not used here: the house monitor was picked from the estimator-only candidates above, on dev, before the test flights were scored.

## Limits

- Simulation only. The estimator, its sensors and their faults are frontier-fly's models; the result says how these pilots behave on these models, not how a real estimator fails.
- One estimator. Every pilot shares [`src/loc`](../src/loc), so the integrity board compares what pilots do with the same health signal, not different estimators.
- The house monitor was picked from twelve simple candidates on 80 dev flights. Nothing on these boards was tuned on test.
- A crash leaves the rest of the budget unavailable, the same as an honest "not sure". On the sensor-failure task the other pilots crash in {{INT_OTHERS_FAIL_CRASHES}} flights and so are rarely confidently wrong, while the view-first planner flies on ({{INT_PV_FAIL_CRASHES}} crashes) with a pose it does not flag. Whether a crash should cost more than doubt is a scoring choice; at this setting, crashing never beats flagging.
- The hazard weight (10) and time to alert (2 s) are choices, not standards. They are constants in [`src/bench/integrity.ts`](../src/bench/integrity.ts); a different weight would change the scores, and the per-step columns are published so a reader can rescore.

## Reproduce

```
npm run bench:hard -- --track drift-hard --split test          # and again with --truth
npm run bench:hard -- --track integrity --split test
npm run bench:integrity -- --split dev                          # pick the monitor
npm run bench:integrity -- --split test --out docs/integrity.json
npm run docs                                                    # regenerate this page
```

Results commit: `{{INT_COMMIT}}`.
