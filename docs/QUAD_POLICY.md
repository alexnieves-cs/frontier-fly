# Can the learned policy fly the quadrotor? A negative result

Short answer: not as well as the classical planner. Nine training runs (about 45 runner-hours, 2026-11-10) tried to make the learned policy fly the quadrotor airframe. The best of them, flown behind the same lidar braking filter the planner uses, cover about 41% of a hard building and crash in about one flight in ten. The sensor-only planner with its trajectory tracker covers 47.8% of the same buildings and never crashes. Flown alone, without the braking filter, every trained policy still crashes in more than 80% of flights. No checkpoint from this work is shipped.

Numbers here are from the tuning block only (quad episodes 66301-66332, 32 flights, randomized quadrotor dynamics and sensor noise, the 0.5 m/s crash rule). Because nothing was selected to ship, no report-only seeds were flown. The published quadrotor comparison is still the coverage study's `quad` condition in [REPORT.md](REPORT.md). The raw validation curves are in [`quad-policy.json`](quad-policy.json).

## Where it started

| Pilot, on the 32 tuning quad flights | Flown coverage | Crashes |
| --- | --- | --- |
| Planner with braking filter and trajectory tracker (`classicalBrakeTrack`) | 47.8% | 0 of 32 |
| Planner with braking filter only (`classicalBrake`) | 34.2% | 14 of 32 |
| Shipped policy behind the braking guard (`guardedBrake`) | 32.1% | 14 of 32 |
| Shipped policy alone | 9.2% | 32 of 32 |

The shipped policy learned on a kinematic airframe that stops in about 0.15 s. The quadrotor has to tilt back to stop: from 2 m/s it needs about 0.8 m. `scripts/study/quad-policy-diag.ts` wraps the shipped policy in simple action filters on 16 practice buildings (9101+) to see what it is missing:

| Wrapper around the shipped policy | Coverage | Crashes |
| --- | --- | --- |
| None | 9.6% | 16 of 16 |
| Yaw rate capped at 0.6 rad/s | 10.6% | 16 of 16 |
| Half speed | 23.1% | 9 of 16 |
| Braking guard | 35.7% | 5 of 16 |
| Braking guard and yaw cap 0.6 rad/s | 37.7% | 2 of 16 |

So the policy needs to brake while there is still room to stop, and to turn more slowly. Both are things its lidar and velocity inputs could tell it.

## What was tried

All runs start from the shipped checkpoint and use `npm run train:ppo` with the flags added for this work:

- `--quad-guard`: quadrotor episodes fly behind the braking guard, in training and validation, so the policy trains as the pilot that would be deployed.
- `--labels brake-self`: the imitation target is the policy's own action through the braking guard, with a yaw rate cap (it should learn to brake as the guard does).
- `--labels planner`: the planner that flies the quad (`classicalBrakeTrack`) runs beside the policy on the policy's own states and labels them.
- `--mem 64`: a GRU memory in front of the policy.
- `--quad 1` or `0.7`: the share of training flights on the quadrotor.

Mean of the last five validation points of each run (2,200 updates, or 1,000 with memory):

| Run | Labels | Quad share | Other | Quad behind guard | Crashes behind guard | Quad alone | Crashes alone | Kinematic hard |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| start | — | — | shipped checkpoint | 32.1% | 43.8% | 9.2% | 100% | 73.5% |
| 0 | planner | 1 | | 41.3% | 10.0% | 22.3% | 83.8% | 28.7% |
| 1 | planner | 1 | imitation weight annealed 1 to 0.1 | 41.5% | 11.3% | 12.2% | 93.8% | 15.0% |
| 2 | planner | 0.7 | | 39.0% | 21.3% | 18.2% | 83.8% | 46.9% |
| 3 | brake-self | 1 | | 38.7% | 25.0% | 19.2% | 91.3% | 25.6% |
| 4 | brake-self | 0.7 | | 37.5% | 26.9% | 12.8% | 97.5% | 64.2% |
| 5 | planner | 1 | memory | 37.4% | 15.0% | 19.3% | 90.6% | 18.2% |
| 6 | none (PPO only) | 1 | | 38.7% | 61.9% | 9.9% | 98.8% | 11.2% |
| 7 | planner | 1 | learning rate 1e-4 | 36.2% | 22.5% | 9.3% | 96.9% | 15.2% |
| 8 | brake-self | 1 | memory | 38.0% | 26.9% | 13.9% | 99.4% | 20.4% |

## What it shows

- **Nothing reaches the planner.** Every run levels off between 36% and 42% behind the guard from about update 500 on. The planner's 47.8% with no crashes is out of reach of all nine.
- **Imitating the planner is what cuts crashes.** It takes crashes behind the guard from 44% to about 10%, and it is the only thing that moves the policy alone off 100% (to 65-85% at best). Reinforcement learning alone does not learn to brake in 2,200 updates.
- **The policy does not internalize braking.** Alone, the best run still crashes in four flights of five. The braking filter does the stopping; the policy learned to fly in a way that lets it.
- **Memory does not help**, at half the updates for the same compute.
- **Quadrotor skill costs kinematic skill.** Runs trained only on the quad lose most of their kinematic coverage (73.5% to 11-29%). The mixed runs keep more of it and gain less on the quad. One network did not hold both.
- **The trained policy does beat the planner without its tracker** (about 41% and 10% crashes against 34.2% and 44%). The planner's lead on the quadrotor comes from its trajectory tracker: a speed profile from the path ahead, an acceleration limit and a yaw rate cap. That is a model of the airframe, and writing it down was cheaper than learning it.

## What this means for the project

On the airframe a real drone has, the classical sensor-only planner is the better pilot, and missions already fly it there. The learned policy's lead is on the kinematic airframe behind a hand-written guard. The project's strongest claims are the planner, the failure atlas and the way everything is evaluated, not a learned autopilot.

Not tried, and where a next attempt would start: a policy trained from scratch on the quadrotor rather than fine-tuned from a kinematic one; an action space of acceleration or path waypoints handed to the tracker, so the policy chooses where to go and the tracker how; and far more training (these runs are 35 million steps each).
