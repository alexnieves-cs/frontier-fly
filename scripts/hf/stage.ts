// Stage the Hugging Face model repository for the shipped policy: the checkpoints, the results
// they are scored by, and a model card generated from the same JSON the README reads. Nothing
// is uploaded; .github/workflows/huggingface.yml uploads this folder when run by hand.
//
//   npm run hf:stage -- [--out dist-hf] [--repo <user>/frontier-fly-policy]
import { execSync } from 'node:child_process';
import { copyFileSync, mkdirSync, readFileSync, rmSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { parseCheckpoint, SHIPPED_CHECKPOINT_PATH, SHIPPED_POLICY_PATH } from '../../src/rl/checkpoint';

const argv = process.argv.slice(2);
const opt = (name: string, fallback: string) => {
  const i = argv.indexOf(`--${name}`);
  return i >= 0 && argv[i + 1] ? argv[i + 1]! : fallback;
};
const out = opt('out', 'dist-hf');
const repo = opt('repo', 'alexnieves-cs/frontier-fly-policy');
const GH = 'https://github.com/alexnieves-cs/frontier-fly';
const commit = execSync('git rev-parse --short HEAD').toString().trim();

interface Cell {
  condition: string;
  lighting: string;
  pilot: string;
  episodes: number;
  coverage: number;
  coverageCi: [number, number];
  crashes: number;
}
const headline = JSON.parse(readFileSync('docs/headline.json', 'utf8')) as { commit: string; crashSpeed: number; cells: Cell[] };
const cell = (condition: string, lighting: string, pilot: string) => {
  const c = headline.cells.find((x) => x.condition === condition && x.lighting === lighting && x.pilot === pilot);
  if (!c) throw new Error(`docs/headline.json has no ${condition}/${lighting}/${pilot} cell`);
  return c;
};
interface BenchResult {
  name: string;
  split: string;
  commit: string;
  score: number;
  safeShare: number;
  tasks: Record<string, { coverage: number }>;
}
const benchFiles = ['guarded', 'policy', 'planner', 'waypoint'];
const bench = Object.fromEntries(benchFiles.map((f) => [f, JSON.parse(readFileSync(`bench/results/${f}.json`, 'utf8')) as BenchResult]));

const ck = parseCheckpoint(readFileSync(SHIPPED_POLICY_PATH));
const params = ck.weights.reduce((a, w) => a + w.length, 0);
const pct = (x: number) => (x * 100).toFixed(1);
const ci = (c: Cell) => `${pct(c.coverageCi[0])}–${pct(c.coverageCi[1])}%`;

const rows = (['guarded', 'policy', 'teacher', 'classical'] as const).map((pilot) => {
  const label = { guarded: 'Shipped policy under its lidar guard', policy: 'Shipped policy alone', teacher: 'Privileged teacher (imitation source)', classical: 'Sensor-only frontier planner' }[pilot];
  const day = cell('hard', 'day', pilot);
  const night = cell('hard', 'night-dark', pilot);
  return `| ${label} | ${pct(day.coverage)}% (${ci(day)}) | ${day.crashes} of ${day.episodes} | ${pct(night.coverage)}% | ${night.crashes} of ${night.episodes} |`;
});
const benchTasks = Object.keys(bench.guarded!.tasks);
const benchRows = benchFiles.map((f) => {
  const r = bench[f]!;
  return `| ${r.name} | **${pct(r.score)}%** | ${benchTasks.map((t) => (r.tasks[t] ? `${pct(r.tasks[t].coverage)}%` : '–')).join(' | ')} | ${pct(r.safeShare)}% |`;
});
const g = cell('hard', 'day', 'guarded');
const gn = cell('hard', 'night-dark', 'guarded');
const p = cell('hard', 'day', 'policy');

const card = `---
license: mit
library_name: frontier-fly
pipeline_tag: reinforcement-learning
tags:
  - reinforcement-learning
  - imitation-learning
  - ppo
  - dagger
  - robotics
  - drone
  - autonomous-exploration
  - search-and-rescue
  - lidar
model-index:
  - name: frontier-fly-policy
    results:
      - task:
          type: reinforcement-learning
          name: Indoor exploration of an unseen building
        dataset:
          type: frontier-fly-hard-buildings
          name: frontier-fly coverage study, 30 fresh hard buildings
        metrics:
          - type: coverage
            name: Coverage under the lidar guard, day (%)
            value: ${pct(g.coverage)}
          - type: coverage
            name: Coverage under the lidar guard, lights off (%)
            value: ${pct(gn.coverage)}
          - type: coverage
            name: Coverage of the policy alone, day (%)
            value: ${pct(p.coverage)}
      - task:
          type: reinforcement-learning
          name: FrontierBench v2
        dataset:
          type: frontierbench
          name: FrontierBench v2, test split
        metrics:
          - type: coverage
            name: FrontierBench score, guarded (%)
            value: ${pct(bench.guarded!.score)}
---

# frontier-fly policy

The indoor exploration policy of [frontier-fly](${GH}), a browser simulator of a search-and-rescue drone
that sweeps woods, finds a building, enters through an opening and searches it room by room. This small
network flies the indoor part on lidar alone: it was learned by imitation (DAgger) from a teacher that
sees the true floor plan, then fine-tuned with PPO on its own flights.

<p align="center"><img src="https://raw.githubusercontent.com/alexnieves-cs/frontier-fly/main/docs/media/fpv.gif" alt="The policy exploring a building, seen from the drone" width="560"></p>

It is a simulator result. It has not flown a real aircraft, and it is published for research and teaching,
civilian and defensive use only.

## Results

Coverage of 30 fresh hard buildings (rooms, hallways and loops) that no model or setting was tuned on, with
95% bootstrap intervals. A flight ends in a crash when it hits a surface faster than ${headline.crashSpeed} m/s.
From [\`docs/headline.json\`](${GH}/blob/${headline.commit}/docs/headline.json) at ${headline.commit}.

| Pilot | Day | Day crashes | Lights off | Lights-off crashes |
| --- | --- | --- | --- | --- |
${rows.join('\n')}

Read the crash column. Alone, the network ends most hard flights by hitting something. The simulator
always flies it under a lidar guard that brakes before contact, with the planner taking over on a stall,
and that pair covers ${pct(g.coverage)}% with ${g.crashes} crash${g.crashes === 1 ? '' : 'es'} in ${g.episodes} flights.

**FrontierBench v2** (test split; [leaderboard](${GH}/blob/main/docs/LEADERBOARD.md)), mean coverage over
seven tasks and the share of flights that touched nothing. Scored at ${bench.guarded!.commit}; the scorer
is deterministic, and CI re-scores every entry from its code.

| Pilot | Score | ${benchTasks.join(' | ')} | Touched nothing |
| --- | --- | ${benchTasks.map(() => '---').join(' | ')} | --- |
${benchRows.join('\n')}

## Files

| File | What |
| --- | --- |
| \`policy.bin\` | The actor only (${(statSync(SHIPPED_POLICY_PATH).size / 1e6).toFixed(1)} MB): what the simulator, the CLI and the Python package fly |
| \`pretrained.bin\` | The full checkpoint with its value head and observation statistics, to fine-tune from |
| \`results/\` | The JSON the tables above are generated from |

Both are frontier-fly's packed checkpoint format \`FFW1\`: 4 magic bytes, a little-endian u32 header length, a
JSON header (\`obsDim\` ${ck.obsDim}, \`actDim\` ${ck.actDim}, \`hidden\` [${ck.hidden.join(', ')}], array shapes), padding to 4 bytes,
then float32 arrays. [\`src/rl/checkpoint.ts\`](${GH}/blob/${commit}/src/rl/checkpoint.ts) reads and writes it, and
[\`src/sim/OBS_LAYOUT.md\`](${GH}/blob/${commit}/src/sim/OBS_LAYOUT.md) documents the ${ck.obsDim} observation values
(lidar ring, local and coarse occupancy and frontier maps, velocity, heading and a waypoint to the nearest frontier). The actor is an MLP with ${ck.hidden.join(' and ')}
hidden units and ${params.toLocaleString('en-US')} parameters; it outputs ${ck.actDim} actions (forward, lateral and vertical
velocity, and yaw rate) at 20 Hz.

## Use it

The weights need the simulator to mean anything. Three ways to fly them:

\`\`\`bash
# Browser: nothing to install
open https://alexnieves-cs.github.io/frontier-fly/

# Headless, in a container
docker run --rm ghcr.io/alexnieves-cs/frontier-fly mission --scenario cabin-day
docker run --rm ghcr.io/alexnieves-cs/frontier-fly bench --pilot guarded --split test

# Python (Gymnasium), from a checkout; see python/README.md
npm ci && npm run build:py && pip install ./python
\`\`\`

## Training

Imitation first: DAgger on a privileged teacher across a curriculum of generated buildings. Then 800 PPO updates from that checkpoint on cloud CPUs, with a small
behaviour-cloning term to keep it near the teacher. The whole pipeline is in the repository
(\`npm run train:dagger\`, \`npm run train:ppo\`), and the [tech report](${GH}/blob/main/docs/REPORT.md) has the
ablations, what failed, and the seed blocks used for tuning and for scoring.

## Limits

- Simulator only: generated and real (Swiss Dwellings, CC BY 4.0) floor plans, simulated lidar and physics.
- Alone, the network crashes on ${p.crashes} of ${p.episodes} hard buildings; it is meant to fly under the guard.
- The [project guide](${GH}/blob/main/docs/GUIDE.md#honest-limits) lists the rest.

## Citation

\`\`\`bibtex
@software{nieves_frontier_fly,
  author = {Nieves, Alex},
  title = {frontier-fly: a browser search-and-rescue drone autopilot and the FrontierBench exploration benchmark},
  doi = {10.5281/zenodo.23717793},
  url = {${GH}},
  license = {MIT}
}
\`\`\`

Staged from ${GH}/tree/${commit} for \`${repo}\`.
`;

rmSync(out, { recursive: true, force: true });
mkdirSync(join(out, 'results', 'frontierbench'), { recursive: true });
writeFileSync(join(out, 'README.md'), card);
copyFileSync(SHIPPED_POLICY_PATH, join(out, 'policy.bin'));
copyFileSync(SHIPPED_CHECKPOINT_PATH, join(out, 'pretrained.bin'));
copyFileSync('LICENSE', join(out, 'LICENSE'));
copyFileSync('docs/headline.json', join(out, 'results', 'headline.json'));
for (const f of benchFiles) copyFileSync(`bench/results/${f}.json`, join(out, 'results', 'frontierbench', `${f}.json`));
// .gitattributes so the Hub stores the checkpoints with LFS.
writeFileSync(join(out, '.gitattributes'), '*.bin filter=lfs diff=lfs merge=lfs -text\n');
console.log(`staged ${out}/ for ${repo} from ${commit}: README.md, policy.bin, pretrained.bin, results/`);
