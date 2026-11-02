/**
 * Check an arena file before a flight: shape, take-off clearance, and that
 * the floor is connected.
 *
 *   npx vite-node scripts/crazyflie/check-arena.ts -- my-room.json
 */
import { readFileSync } from 'node:fs';
import { reachableCells, validateArena, type Arena } from '../../src/crazyflie/arena';

const path = process.argv.slice(2).filter((a) => a !== '--')[0];
if (!path) {
  console.error('usage: check-arena.ts -- arena.json');
  process.exit(2);
}
const arena = JSON.parse(readFileSync(path, 'utf8')) as Arena;
const errs = validateArena(arena);
if (errs.length) {
  console.error(`${path}:\n  - ${errs.join('\n  - ')}`);
  process.exit(1);
}
const r = reachableCells(arena);
const area = (r.count * r.cell * r.cell).toFixed(1);
console.log(`${path}: ok. ${arena.w} x ${arena.d} m, ${arena.walls.length} inner walls, ${arena.boxes.length} boxes, ${area} m² of floor the drone can reach.`);
