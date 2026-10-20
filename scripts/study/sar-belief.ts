// Draws docs/report/sar-belief.svg (src/eval/sarFigure.ts) without re-running the benchmark.
//
//   vite-node scripts/study/sar-belief.ts -- [--seed 7102]
//
// scripts/study/sar-study.ts draws it when it merges, so `npm run study:sar` keeps it current.
import { drawBelief } from '../../src/eval/sarFigure';

const i = process.argv.indexOf('--seed');
await drawBelief(i >= 0 ? Number(process.argv[i + 1]) : undefined);
