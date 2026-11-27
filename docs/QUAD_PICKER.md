# Teaching the policy to pick the quadrotor's waypoints

Short answer: it works. A small learned network that only chooses *where the quadrotor goes next* covers 54.8% of hard, never-seen buildings, against 46.2% for the published quad pilot (`classicalBrakeTrack`) and 51.1% for the same planner with its knobs tuned on the same budget. It crashes in 21 of 664 flights, against 27 for the published pilot. The planner, lidar braking filter and trajectory tracker still do all the flying. This was the pre-registered test, and it passed on report-only seeds flown once after everything was frozen.

Since 2026-11-25 the picker, with a more cautious braking model, is the **default quad explorer** (`quadPilot()` in [`src/eval/pilots.ts`](../src/eval/pilots.ts), options in [`src/sim/quadPilot.ts`](../src/sim/quadPilot.ts)). On a fresh report-only block it covers 54.2% with 1 crash in 664 flights, against 45.9% and 30 crashes for the planner-only quad pilot. See [The default quad pilot](#the-default-quad-pilot) below. Missions, drone teams and FrontierBench still fly their own planner settings; the picker was not measured there.

## Why this split

Flying the quadrotor end to end with the learned policy failed: 11.9% coverage and 30 of 30 crashes ([QUAD_POLICY.md](QUAD_POLICY.md)); a later network trained on the quadrotor itself does fly it ([QUAD_STUDENT.md](QUAD_STUDENT.md)). The tilt-to-stop dynamics are what the policy got wrong, and the planner plus tracker already handle those. So this study leaves control alone and learns only the goal choice, the step where the planner uses a hand-written rule (lowest path cost, a small bonus for frontier next to flown or blocked space, hysteresis toward the current goal). That is the split learned next-best-view and frontier-selection work usually makes (NBVP, FUEL, and learned frontier rankers): learn the decision, keep a verified planner underneath.

## What the picker sees and does

Every replan, the planner groups its frontier into 8-connected clusters and hands the picker one candidate per cluster (the voxel the rule likes best), the 12 best by rule score plus the current goal. Each candidate has 18 features, all from the drone's own maps, pose and measured velocity, never the floor plan: path cost and length, the rule's own score, cluster size, unknown space nearby, how far the drone would have to turn and reverse to get there, an estimate of flight time under the tracker's speed profile, how much open unflown lane lies past it, time left, speed and a bias ([`src/sim/picker.ts`](../src/sim/picker.ts)). The picker returns one index; Dijkstra, the braking filter, the tracker and the watchdog fly to it.

The learned picker scores each candidate with `lin · x + w2 · tanh(W1 x + b1)` (32 hidden units) and takes the best. The linear part starts as the planner rule and the hidden layer's output starts at zero, so training begins from exactly the planner's behavior.

## Arms and budget

| Arm | What is searched | How |
| --- | --- | --- |
| Published pilot (`classicalBrakeTrack`) | nothing | the coverage study's quad pilot |
| Tuned planner | the rule's 3 knobs: edge bonus, momentum, replan interval | CMA-ES, 2 seeds |
| Linear picker | 18 feature weights + replan interval | CMA-ES, 2 seeds |
| Learned picker | MLP weights | decision-level PPO (semi-Markov discount per decision, reward = coverage gained, crash cost), 4 configs from the rule and 4 from the tuned planner |

All training ran on GitHub Actions (`train.yml`, option `train:picker`; runs 43114686245, 43115091349, 43145211371). Training buildings are quad episodes 66001-66299 with random day or dark-night lighting, randomized quadrotor dynamics and sensor noise, and the 0.5 m/s crash rule.

## Seeds, in the order they were used

1. **Tune** (66301-66332, day and dark night, 64 flights): every training run's own checkpoint choice.
2. **Selection** (66401-66464, both lightings, 128 flights): picking one checkpoint per arm among finished runs, so that the choice is not the maximum of many noisy tune scores.
3. **Report** (89001-89332, both lightings, 664 flights): flown once, with `--final`, after the three chosen checkpoints were frozen (SHA-256 in [`results/picker/final/frozen.sha256`](../results/picker/final/frozen.sha256)).
4. **Published** (50001-50030, both lightings, 60 flights): the coverage study's published set, as a second look.

Score for every choice = coverage − 0.1 × crash rate.

## What happened on tune

| Arm | Tune coverage | Crashes |
| --- | --- | --- |
| Published pilot | 47.4% | 3 of 64 |
| FUEL-style flight-time weighting of the rule | 44.7-47.0% | |
| Tuned planner (best of 2) | 53.6% | 2 of 64 |
| Linear picker (best of 2) | 51.9% | 0 of 64 |
| PPO from the plain rule (best of 4) | 51.3% | |
| PPO from the tuned planner (best of 4) | 56.4% | 0 of 64 |

PPO started from the plain rule plateaued below the tuned planner. Started from the tuned planner's knobs, every one of the four configs passed it by about 3 points, so the stop rule (nothing beats the tuned planner by a point on tune: write up a negative result) did not trigger.

## Selection block

| Candidate | Coverage | Crashes |
| --- | --- | --- |
| Published pilot | 46.8% | 2 of 128 |
| Tuned planner, CMA seed 1 (chosen) | 51.6% | 6 of 128 |
| Tuned planner, CMA seed 2 | 50.5% | 6 of 128 |
| Linear picker, CMA seed 2 (chosen) | 50.6% | 4 of 128 |
| Linear picker, CMA seed 1 | 50.3% | 5 of 128 |
| PPO a / b / d | 54.2% / 54.5% / 54.4% | 4 / 3 / 3 of 128 |
| PPO c, lr 3e-3, tune iteration 160 (chosen) | 54.6% | 2 of 128 |

All four PPO runs land within 0.4 points of each other, so the result does not hang on one lucky run. Every arm lost 2-3 points between tune and selection, about the same amount, which is the selection bias the extra block was there to remove.

## Report-only result (89001-89332, flown once)

| Pilot | Coverage | Day | Dark night | Crashes |
| --- | --- | --- | --- | --- |
| Published pilot | 46.2% | 46.0% | 46.4% | 27 of 664 |
| Tuned planner | 51.1% | 51.0% | 51.2% | 44 of 664 |
| Linear picker | 50.2% | 50.2% | 50.1% | 20 of 664 |
| **Learned picker** | **54.8%** | 54.6% | 55.0% | **21 of 664** |

Paired by building and lighting, 10,000 bootstrap resamples:

| Comparison | Coverage difference (95% CI) | Crash difference (95% CI) |
| --- | --- | --- |
| Learned picker − published pilot | **+8.6 points (+8.1 to +9.1)** | −6 (−17 to +5) |
| Learned picker − tuned planner | **+3.7 points (+3.1 to +4.4)** | −23 (−36 to −11) |
| Learned picker − linear picker | +4.6 points (+4.1 to +5.1) | +1 (−11 to +13) |
| Tuned planner − published pilot | +4.9 points (+4.3 to +5.5) | +17 (+5 to +29) |
| Linear picker − published pilot | +4.0 points (+3.5 to +4.4) | −7 (−19 to +4) |

Against the pre-registered test: the learned picker beats the published pilot with a confidence interval well above zero and does not crash more (21 against 27; that difference is within noise). It also beats the tuned planner by 3.7 points, so this counts as a learning win and not just better knob tuning. Tuning the planner's knobs alone buys coverage by replanning every 2 steps and crashes 63% more often than the published pilot; the learned picker gets more coverage with the published pilot's crash rate.

## Published set (50001-50030, both lightings, 60 flights)

| Pilot | Coverage | Crashes |
| --- | --- | --- |
| Published pilot | 46.8% | 0 of 60 |
| Tuned planner | 52.2% | 4 of 60 |
| Linear picker | 50.1% | 2 of 60 |
| Learned picker | 55.5% | 3 of 60 |

The coverage gain holds (+8.7 points, +6.6 to +10.4). On these 60 flights the learned picker crashed 3 times against the published pilot's 0, so its crash rate on this set is no better and plausibly a little worse; the 664-flight report block is the better estimate of the crash rate.

## The default quad pilot

Goal (set before starting): the picker should crash less than the published planner-only pilot on held-out practice flights, and keep its coverage lead on a fresh report block flown once. Stop after three attempts in a row with no improvement. Seeds were recorded in the ledger before use: practice 66001-66160 (training buildings) and 66701-66764, a held-out practice gate 66801-66960, and report-only 89401-89732.

**Why it crashed.** On the practice flights, every crash of both the picker and the planner came within about 10 steps of a goal switch at 1.5-3 m/s, with a wall about a meter away: the new path turns, and the braking filter's stopping model (0.21 s reaction, 5 m/s²) is too optimistic for some of the randomized airframes. So the first attempt changed only the braking model the planner plans with, not the picker.

**Attempt 1: braking settings, chosen on practice (448 flights).** The rule was the fewest crashes within a point of the picker's coverage.

| Setting | Picker coverage | Picker crashes |
| --- | --- | --- |
| As published (0.21 s, 5 m/s²) | 53.5% | 18 |
| Margin 0.35 m | 48.3% | 9 |
| Speed 1.6 m/s | 49.8% | 7 |
| Reaction 0.3 s | 52.2% | 8 |
| Reaction 0.35 s | 51.1% | 5 |
| **Reaction 0.25 s, decel 4 m/s² (chosen)** | **53.0%** | **5** |
| Planner-only pilot, as published | 45.4% | 19 |

**Held-out practice gate (66801-66960, 320 flights).** The goal was met on the first attempt, so no further attempts were needed.

| Pilot | Coverage | Crashes |
| --- | --- | --- |
| Planner-only pilot, as published | 45.7% | 22 |
| Planner-only pilot, new braking | 44.9% | 7 |
| Picker, as published | 54.0% | 17 |
| **Picker, new braking (default)** | **53.9%** | **4** |

**Report-only block (89401-89732, 664 flights, flown once after freezing; SHA-256 in [`results/picker/default-report/frozen.sha256`](../results/picker/default-report/frozen.sha256)).**

| Pilot | Coverage | Day | Dark night | Crashes |
| --- | --- | --- | --- | --- |
| Planner-only pilot, as published | 45.9% | 46.0% | 45.8% | 30 of 664 |
| Planner-only pilot, new braking | 45.3% | 45.3% | 45.3% | 5 of 664 |
| Picker, as published | 54.8% | 54.9% | 54.6% | 22 of 664 |
| **Default quad pilot (picker, new braking)** | **54.2%** | 54.3% | 54.2% | **1 of 664** |

Paired by building and lighting, 10,000 bootstrap resamples:

| Comparison | Coverage (95% CI) | Crashes (95% CI) |
| --- | --- | --- |
| Default − planner-only, as published | +8.3 points (+7.9 to +8.8) | −29 (−40 to −19) |
| Default − planner-only, new braking | +8.9 points (+8.6 to +9.2) | −4 (−9 to 0) |
| Default − picker, as published | −0.5 points (−1.0 to −0.1) | −21 (−31 to −12) |
| Planner-only: new braking − as published | −0.6 points (−1.0 to −0.1) | −25 (−36 to −14) |

Read honestly: most of the crash cut comes from the braking model, and the planner-only pilot gets the same cut from it (30 to 5). The coverage lead comes from the picker. With both on the same braking, the picker still crashes less (1 against 5), but that difference is at the edge of noise. The cost is half a point of coverage.

On the coverage study's published quad set (50001-50030, 60 flights) the default covers 55.6% with 1 crash, against 46.8% with 0 for the planner-only pilot.

## Limits

- Simulation only, same simulator and quadrotor model as every other number in this repository ([PHYSICS_VALIDATION.md](PHYSICS_VALIDATION.md)).
- Flown coverage only. The 3 m footprint metric was not measured for these arms.
- Stage 3 buildings with randomized dynamics; the picker was not tried on missions, teams, multi-floor or hazard scenarios, and is not wired into them. The coverage study's quad condition has the new pilot (`quadPicker`), but `docs/study.json` gets it at the next shared rerun.
- The comparison to the tuned planner is at equal search budget on the planner's 3 knobs; a richer hand-written rule might close some of the gap.

## Reproduce

```bash
npm run train:picker -- --mode eval --picker results/picker/frozen-ppo.json --set tune
npm run train:picker -- --mode eval --picker results/picker/frozen-ppo.json --set report --final   # the report block
npm run train:picker -- --mode ppo --init results/picker/sweep1-planner-seed2.json --lr 3e-3 --out-dir out  # retrain (hours; use train.yml, run 43145211371 job 2 has the exact flags)
```

The default quad pilot's numbers: `npm run train:picker -- --mode eval --picker results/picker/default.json --set gate` (and `--set report2 --final` for the report block; the planner-only control is `--picker rule`, with new braking `--picker results/picker/rule-safe.json`). Crash traces: `npx vite-node scripts/study/picker-crash-diag.ts -- results/picker/default.json`.

The frozen checkpoints are `results/picker/frozen-{planner,linear,ppo}.json`; every flight of the final runs is in `results/picker/final/`, and every number on this page is in [`quad-picker.json`](quad-picker.json).
