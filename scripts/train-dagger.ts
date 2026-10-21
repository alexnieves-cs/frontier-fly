/**
 * DAgger (+ optional anchored PPO fine-tune) for the indoor policy.
 * Usage: npm run train:dagger -- --out /tmp/dagger.json [--iters 4] [--bc-episodes 16]
 *        [--episodes 12] [--bc-epochs 6] [--retrain-epochs 3] [--capacity 60000]
 *        [--seed 11] [--ppo-updates 0] [--init ckpt.json] [--lr 1e-3] [--no-reweight]
 * Writes the DAgger checkpoint to --out and, with --ppo-updates N, the anchored
 * PPO result to <out>.ppo.json. Never writes public/checkpoints unless --out says so.
 */
import { readFileSync, writeFileSync } from 'node:fs';
import { parseCheckpoint, serializeCheckpoint, serializeCheckpointFor } from '../src/rl/checkpoint';
import { trainDagger } from '../src/rl/dagger';
import { ppoFinetune } from '../src/rl/finetune';

const args = process.argv.slice(2);
function arg(name: string): string | undefined {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
}
function num(name: string, fallback: number): number {
  const v = arg(name);
  return v == null ? fallback : Number(v);
}

async function main() {
  const out = arg('out') ?? '/tmp/frontier-fly-dagger.json';
  const init = arg('init');
  const started = Date.now();
  const pct = (v: number) => `${(v * 100).toFixed(1)}%`;
  const result = await trainDagger({
    seed: num('seed', 11),
    iterations: num('iters', 4),
    bcEpisodes: num('bc-episodes', 16),
    episodesPerIter: num('episodes', 12),
    bcEpochs: num('bc-epochs', 6),
    retrainEpochs: num('retrain-epochs', 3),
    capacity: num('capacity', 60_000),
    beta0: num('beta0', 0.5),
    betaDecay: num('beta-decay', 0.5),
    lr: num('lr', 1e-3),
    maxStall: num('max-stall', 300),
    weighting: args.includes('--no-reweight')
      ? { divergeBoost: 0, stallBoost: 0, collisionBoost: 0 }
      : undefined,
    init: init ? parseCheckpoint(readFileSync(init)) : undefined,
    onIter: (m) => {
      const v = m.validation;
      console.log(
        `dagger iter ${m.iter} beta ${m.beta.toFixed(2)} samples ${m.samples} data ${m.datasetSize} ` +
          `rolloutCov ${pct(m.rolloutCoverage)} diverge ${pct(m.divergentFrac)} stall ${pct(m.stalledFrac)} ` +
          `loss ${m.bcLoss.toFixed(4)}` +
          (v ? ` val ${pct(v.score)} (small ${pct(v.small)} large ${pct(v.large)})` : '') +
          ` ${m.seconds.toFixed(1)}s`,
      );
    },
  });
  console.log(`dagger best iter ${result.bestIter}`);
  writeFileSync(out, serializeCheckpointFor(out, result.checkpoint));
  writeFileSync(out.replace(/\.json$/, '') + '.last.json', serializeCheckpoint(result.lastCheckpoint));
  console.log(`wrote ${out} (best) and .last.json`);
  const ppoUpdates = num('ppo-updates', 0);
  if (ppoUpdates > 0) {
    const tuned = await ppoFinetune({
      checkpoint: result.checkpoint,
      dataset: result.dataset.view(),
      updates: ppoUpdates,
      seed: num('seed', 11),
      onUpdate: (m) => {
        console.log(
          `ppo ${m.update} pi ${m.policyLoss.toFixed(3)} v ${m.valueLoss.toFixed(3)} anchorKl ${m.anchorKl.toFixed(4)} ` +
            `bc ${m.bcLoss.toFixed(4)} klCoef ${m.klCoef.toFixed(2)} bcCoef ${m.bcCoef.toFixed(2)} cov ${pct(m.rolloutCoverage)}` +
            (m.validation ? ` val ${pct(m.validation.score)}` : ''),
        );
      },
    });
    console.log(`ppo start val ${pct(tuned.startValidation.score)} best ${pct(tuned.bestValidation.score)} at ${tuned.bestUpdate}`);
    const ppoOut = out.replace(/\.json$/, '') + '.ppo.json';
    writeFileSync(ppoOut, serializeCheckpointFor(ppoOut, tuned.checkpoint));
    console.log(`wrote ${ppoOut}`);
  }
  console.log(`done in ${((Date.now() - started) / 1000).toFixed(1)} s`);
}

void main();
