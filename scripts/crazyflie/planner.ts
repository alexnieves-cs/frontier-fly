/**
 * The explorer as a line-oriented process for scripts/crazyflie/fly.py.
 *
 *   npx vite-node scripts/crazyflie/planner.ts -- --arena arena.json
 *
 * Prints {"ready":true} once loaded. Then for each JSON line on stdin, one
 * reading in the arena frame:
 *   {"t":s,"x":m,"y":m,"z":m,"yaw":rad,"vx":m/s,"vy":m/s,
 *    "front":m|null,"back":..,"left":..,"right":..,"up":..}
 * (null = no return, a negative number = invalid reading), it prints one line:
 *   {"vx":m/s,"vy":m/s,"yawRate":rad/s,"z":m,"phase":"spin"|"explore"|"done","known":0..1}
 * The command is a request. fly.py's guard decides what reaches the drone.
 */
import { readFileSync } from 'node:fs';
import { createInterface } from 'node:readline';
import { validateArena, type Arena } from '../../src/crazyflie/arena';
import { CrazyflieExplorer, type CfReading } from '../../src/crazyflie/explorer';

const args = process.argv.slice(2);
const i = args.indexOf('--arena');
if (i < 0) {
  console.error('usage: planner.ts -- --arena arena.json');
  process.exit(2);
}
const arena = JSON.parse(readFileSync(args[i + 1]!, 'utf8')) as Arena;
const errs = validateArena(arena);
if (errs.length) {
  console.error(`arena: ${errs.join('; ')}`);
  process.exit(2);
}
const explorer = new CrazyflieExplorer(arena);

const range = (v: unknown): number => (v === null || v === undefined ? Infinity : typeof v === 'number' && v >= 0 ? v : NaN);

process.stdout.write(JSON.stringify({ ready: true }) + '\n');
const rl = createInterface({ input: process.stdin });
rl.on('line', (line) => {
  if (!line.trim()) return;
  let out: object;
  try {
    const m = JSON.parse(line) as Record<string, unknown>;
    const r: CfReading = {
      t: Number(m.t),
      x: Number(m.x),
      y: Number(m.y),
      z: Number(m.z),
      yaw: Number(m.yaw),
      vx: Number(m.vx ?? 0),
      vy: Number(m.vy ?? 0),
      front: range(m.front),
      back: range(m.back),
      left: range(m.left),
      right: range(m.right),
      up: range(m.up),
    };
    const c = explorer.step(r);
    out = { ...c, known: Math.round(explorer.knownFree() * 1000) / 1000 };
  } catch (e) {
    out = { error: String(e) };
  }
  process.stdout.write(JSON.stringify(out) + '\n');
});
rl.on('close', () => process.exit(0));
