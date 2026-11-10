# Archived results

Raw results that the current docs no longer lead with but still cite, kept so every number in the
history sections of the docs can be traced back to the flights behind it. Current results stay in
`docs/` next to the pages that read them.

| Folder | What | Cited by |
| --- | --- | --- |
| `fault-bench/` | Fault bench runs 1 to 7 (`fault-bench-runN.json`) and the previous-code controls of runs 3 to 7 (`-runN-control.json`). Run 8, the current one, is `docs/fault-bench.json` with its control `docs/fault-bench-control.json`. | `docs/SAFETY_CASE.md` (runs table, written by `npm run docs`) |
| `checkpoints/` | The two policies that shipped before the current one: `dagger.bin` (BC + DAgger, shipped until 2026-10-21) and `ppo800.bin` (the first PPO fine-tune, shipped until the crash-rule retrain). The current one is `public/checkpoints/pretrained.bin`. | `docs/rl-finetune.json` (`scripts/study/rl-finetune.ts` flies all three), `docs/REPORT.md` |
| `coverage-study/` | `pre-rule-checkpoint.json`: the coverage study's hard, two-floor and quadrotor buildings flown by `checkpoints/ppo800.bin`, alone and under the guard, with the crash rule and the current code (study run 42765507842, 2026-11-10). | `docs/REPORT.md` (the 19 of 30 crashes and 63.0% of the checkpoint trained before the rule), the paper's coverage table |

Nothing here is regenerated. A new archived run goes in a folder named after its bench, and the
table above gets a row.
