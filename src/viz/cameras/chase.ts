import { CELL } from '../../shared/constants';
import type { BuildingSpec } from '../../shared/types';

/** Pull the chase boom in before it crosses a wall cell. Returns the fraction of the boom to keep. */
export function boomScale(
  spec: BuildingSpec,
  from: [number, number, number],
  to: [number, number, number],
): number {
  const steps = 16;
  let scale = 1;
  const floor = spec.floors[Math.max(0, Math.min(spec.floors.length - 1, Math.floor(from[2] / spec.floorHeight)))]!;
  for (let i = 1; i <= steps; i++) {
    const t = i / steps;
    const x = from[0] + (to[0] - from[0]) * t;
    const y = from[1] + (to[1] - from[1]) * t;
    const gx = Math.floor(x / spec.cellSize);
    const gz = Math.floor(y / spec.cellSize);
    if (gx < 0 || gz < 0 || gx >= floor.w || gz >= floor.d) {
      scale = Math.max(0.12, (i - 1) / steps);
      break;
    }
    if (floor.cells[gz * floor.w + gx] === CELL.WALL) {
      scale = Math.max(0.12, (i - 1) / steps);
      break;
    }
  }
  return scale;
}
