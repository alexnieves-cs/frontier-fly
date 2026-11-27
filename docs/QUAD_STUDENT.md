# A learned policy that flies the quadrotor

Short answer: it does now. [QUAD_POLICY.md](QUAD_POLICY.md) ended with the learned policy covering 11.9% of a hard building on the quadrotor and crashing in 30 of 30 flights. The quad student is a new network trained for the quadrotor from scratch. It flies the randomized quadrotor end to end, from the same sensor observation every other pilot gets, and the planner, picker and tracker are not running. On 664 report-only flights it covers 52.4% behind a braking shield that changes 1.4% of its commands, with 7 crashes. The hand-built planner-only quad pilot covers 51.4% of the same flights with 4 crashes. Its teacher, the default quad pilot (planner, learned goal picker, braking filter and tracker), covers 54.3% with 8 crashes. So the student beats the hand-built pilot by about a point and trails its teacher by about two. With no shield at all it covers 50.6% and crashes in 100 of 664 flights (15%).

## Held-out results

The checkpoint and the 1.15 gain were frozen on practice seeds before any of these flights, and each block was flown once ([`scripts/study/quad-student-study.ts`](../scripts/study/quad-student-study.ts), on GitHub Actions `study.yml`). Every block pairs the four pilots on the same buildings, day and dark night, with randomized quadrotor dynamics, sensor noise and the 0.5 m/s crash rule. The seed ranges and the gate rule are in the project's seed ledger.

| Block | Pilot | Coverage (95% CI) | Crashes | Shield changed |
| --- | --- | --- | --- | --- |
| Report 185001-185332 (664 flights) | **Quad student + shield** | **52.4%** (52.0-52.8) | 7 | 1.4% of commands |
| | Quad student alone | 50.6% (50.0-51.3) | 100 | — |
| | Planner-only quad pilot | 51.4% (50.9-51.9) | 4 | — |
| | Default quad pilot (teacher) | 54.3% (53.7-54.8) | 8 | — |
| Gate 181001-181160 (320 flights) | **Quad student + shield** | **51.6%** (51.0-52.2) | 9 | 1.4% |
| | Quad student alone | 49.7% (48.7-50.6) | 58 | — |
| | Planner-only quad pilot | 49.3% (48.4-50.2) | 6 | — |
| | Default quad pilot (teacher) | 52.3% (51.4-53.1) | 5 | — |
| Published quad seeds 50001-50030 (60 flights) | **Quad student + shield** | **53.0%** (51.1-54.8) | 1 | 1.2% |
| | Quad student alone | 52.5% (50.8-54.2) | 6 | — |
| | Planner-only quad pilot | 52.5% (50.8-54.1) | 0 | — |
| | Default quad pilot (teacher) | 55.6% (54.0-57.2) | 1 | — |

Paired differences, student behind the shield minus each control, per flight:

| Block | vs planner-only | vs default quad pilot | vs itself alone |
| --- | --- | --- | --- |
| Report (664) | **+1.0 pts** (+0.6 to +1.4) | -1.9 pts (-2.3 to -1.4) | +1.8 pts (+1.2 to +2.3) |
| Gate (320) | +2.3 pts (+1.6 to +3.1) | -0.6 pts (-1.3 to 0.0) | +2.0 pts (+1.1 to +2.8) |
| Published (60) | +0.5 pts (-1.6 to +2.7) | -2.6 pts (-4.6 to -0.7) | +0.5 pts (-2.0 to +3.0) |

The gate rule was written down before the gate was flown: pass if the student behind the shield covers more than the planner-only pilot, with no more than the planner-only pilot's crashes plus 3. It passed with 9 crashes against a limit of 9, which is closer than the coverage margin. On the day half of the published seeds, the same 30 day flights as the old policy's 11.9% and 30 crashes, the student covers 54.0% with 0 crashes. Day and night are within a point of each other in every block, as for the other lidar pilots. On the report block the student beats the planner-only pilot on 363 of 664 flights and its teacher on 234.

The gate's flights ran in study run 46007432964. Its merge step read the `--block` argument as a file name, so run 46025176762 merged the same shards again without flying anything. The raw flights are in [`quad-student-report.json`](quad-student-report.json), [`quad-student-gate.json`](quad-student-gate.json) and [`quad-student-published.json`](quad-student-published.json).

## What the student is

- **Inputs.** The 1088-value observation every pilot gets ([`src/sim/OBS_LAYOUT.md`](../src/sim/OBS_LAYOUT.md)), plus 16 more values ([`src/rl/quadStudent.ts`](../src/rl/quadStudent.ts)):
  - the last two commands it executed;
  - 8 braking-clearance features, one per 45° sector around the body. Each is the nearest horizontal lidar return in that sector, minus the distance needed to stop from the measured velocity toward it (0.25 s reaction, 4 m/s² deceleration), in units of 2 m and clipped to ±1. This is the quantity the planner's braking filter thresholds. The student only sees it; nothing acts on it for the student.
- **Network.** A tanh MLP with hidden layers 512 and 256, in the same checkpoint format as the shipped kinematic policy.
- **Output.** A command `[forward, left, climb, yaw rate]`. The horizontal part is multiplied by a fixed gain of 1.15 (see [the speed gap](#the-speed-gap)).
- **Shield.** The braking shield, when it is used, is the default quad pilot's braking filter (`guardedAction` with `QUAD_PICKER_BRAKING`). It only changes the horizontal command. Climb and yaw pass through untouched. The shield counts a step as an intervention when it moves the command by more than 0.02 in action units.

It has no memory beyond its last two commands and the observation's egocentric maps. It never sees the floor plan or its true pose.

## How it was trained

The recipe follows recent work on learned agile flight. Train against a privileged expert, then fine-tune with RL so the student stops copying the expert's mistakes and its own (Loquercio et al. 2021; Xing et al., CoRL 2024, "Bootstrapping Reinforcement Learning with Imitation for Vision-Based Agile Flight").

1. **DAgger from the default quad pilot** (`npm run train:quad`, [`scripts/train-quad.ts`](../scripts/train-quad.ts), jobs in [`src/train/quadDistillJob.ts`](../src/train/quadDistillJob.ts)). The student flies, and the expert labels every state it visits. Mixing starts at full expert control and decays to a beta of 0.1. The expert is told which command was executed (`planner.holdCommand`), so its acceleration ramp starts from what the airframe was actually asked to do. Small DART noise goes on the executed horizontal command and slow OU noise on the climb rate, so the student sees recoveries. Rows get extra weight in three cases: in the 30 steps before a crash, where student and expert disagree, and where the expert's braking filter cut its own command. The loss is action-space MSE, weighted 1, 1, 4, 2 on forward, left, climb and yaw rate. The teacher is a more cautious version of the default pilot: braking deceleration 3 m/s² instead of 4, reaction 0.35 s instead of 0.25. That leaves the student a margin for its own error. Training buildings are 170001-179999, with randomized quadrotor dynamics, sensor noise, day and dark night, and the 0.5 m/s crash rule.
2. **PPO from the DAgger student.** Rollouts use stochastic actions on the same training buildings. The reward is coverage gained, minus 150 for a crash. A behavior-cloning anchor on the expert's labels for the same states uses the same action-space loss at weight 0.5. Exploration uses a fixed log-std of -2.5. Any update whose KL spikes above 0.1 is undone.
3. **Shielded PPO with the gain.** The run continues behind the braking shield, with the 1.15 output gain. Every shield correction is penalized at 10 per unit of correction, and the anchor weight drops to 0.3. The shield keeps exploration from ending flights, and the penalty pushes the policy to brake on its own.

All long training ran on GitHub Actions (`train.yml`, option `train:quad`, runs 45929645090, 45946278245, 45951067872, 45976058569). The shipped checkpoint is `public/checkpoints/quad-student.bin`, sweep G arm 2 (`train/45976058569-2`). Every run picked its own checkpoint on practice flights 180001-180040, and the finalists were compared on practice flights 180101-180200.

## What mattered, in the order it was found

All rows are the student flying alone on practice seeds, day and dark night. "+ shield" is the same checkpoint behind the braking shield.

| Step | Coverage | Crashes | Notes |
| --- | --- | --- | --- |
| Starting point: the shipped kinematic policy on the quad (published) | 11.9% | 30 of 30 | [QUAD_POLICY.md](QUAD_POLICY.md) |
| DAgger, loss on pre-tanh means against atanh(labels) | about 1% | every flight | the loss ignored the small climb and yaw labels |
| DAgger, action-space loss weighted 1, 1, 4, 2 | about 30% | 21 of 32 | crashes head-on at 1-2 m/s, lidar 0.2 m |
| + braking-clearance features | 33-35% | 1-8 of 80 | the student starts braking |
| + PPO (80 practice flights) | 44% | 17 of 80 | more coverage, more crashes |
| + shielded PPO (200 practice flights) | 51.1% (+ shield) | 11 of 200 | alone 47.8%, 82 of 200 |
| Cautious teacher, PPO alone (200 flights) | 45.4% | 9 of 200 | + shield 45.9%, 3 of 200 |
| + output gain 1.15, shielded PPO with correction penalty 10 (200 flights, shipped) | 50.2% | 28 of 200 | + shield 52.2%, 4 of 200, 1.4% of commands changed |

Three findings did most of the work:

- **The imitation loss.** Squashed actions trained against atanh of the labels made a student that crashed every flight. The climb and yaw labels are small, so that loss could ignore them. Comparing squashed actions with the labels, with weights on climb and yaw, fixed it. The same bug sat in the PPO anchor and was fixed there too.
- **Braking features.** The observation already carries lidar and velocity, but a 512-unit MLP did not work out stopping distance from them. Handing it the clearance per sector, the same number the braking filter uses, took crashes from 21 of 32 to a few in 80.
- **The speed gap.** See the next section.

Things that did not help:

- Wider DART or climb noise. It made the teacher itself crash (16 crashes in 200 became 134).
- Wider braking margins on the expert, when used for robustness rather than as the teacher.
- Pure PPO without the anchor. It traded coverage for crashes.
- A much larger crash penalty with no shield. That run stayed below 40% for 120 updates.

## The speed gap

Every early student flew about 3 points below its teacher, even on flights where it never crashed. Only 18.5% of 200 paired flights went better than the teacher's. Squared-error imitation hedges: when the teacher might go left or right, or might be about to brake, the student averages the options, and the average is slower. Scaling the student's horizontal command after the fact shows this directly, on one checkpoint behind the shield on 200 practice flights:

| Gain | Coverage | Crashes | Commands the shield changed |
| --- | --- | --- | --- |
| 1.00 | 51.1% | 11 | 2.0% |
| 1.15 | 55.9% | 12 | 11.6% |
| 1.30 | 56.8% | 15 | 24.5% |

At 1.15 the shield was doing a lot of the braking. So the gain went into training (`--gain`). The policy flew PPO at the higher pace behind the shield, and every correction cost reward. Interventions fell from about 10% to 1.4% while coverage held above the planner. A run with a smaller correction penalty kept more coverage (55.7%, 7 crashes, 10.3% of commands changed). It was not chosen, because then the shield, not the network, would be doing the flying.

## How to run it

```sh
npm run train:quad -- --out-dir /tmp/quad --iters 16 --val 40 --noise 0.08 --vz-noise 0.1 --beta-min 0.1 --brake-boost 3 --expert-decel 3 --expert-reaction 0.35 --rl 80 --rl-episodes 48 --rl-lr 5e-5 --log-std -2.5 --crash-penalty -150 --bc 0.5:0.5
npx vite-node scripts/study/quad-student-eval.ts -- --ckpt public/checkpoints/quad-student.bin --gain 1.15 --base 180101 --n 100 --planner
npx vite-node scripts/study/quad-student-diag.ts public/checkpoints/quad-student.bin 180149d,180103n --shield
```

`QuadStudentPilot` in [`src/rl/quadStudent.ts`](../src/rl/quadStudent.ts) flies a checkpoint as a coverage-env pilot. Its `act(obs, filter)` takes a shield as the filter, so the command history records what was actually executed.

## Limits

- The student is not yet better than its teacher: about two points less coverage on the report block. With no shield, 15% of report flights crash, against 1.2% for the default pilot. The shield is part of the shipped system, and every result here says whether it was on.
- The shield is a braking filter on the horizontal command only. It cannot stop a climb into something the horizontal lidar does not see. Some of the remaining crashes are of that kind.
- It is quad-only and trained on stage-3 single-floor buildings. Missions, multi-floor flights, teams and FrontierBench still fly the planner.
