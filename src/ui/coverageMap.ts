import { CELL } from '../shared/constants';
import type { BuildingSpec, DroneState } from '../shared/types';

export function drawCoverage(
  canvas: HTMLCanvasElement,
  spec: BuildingSpec,
  visited: Uint8Array | null,
  state: DroneState | null,
  floor: number,
): void {
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  const grid = spec.floors[Math.max(0, Math.min(spec.floors.length - 1, floor))]!;
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#071018';
  ctx.fillRect(0, 0, w, h);
  const pad = 10;
  const cw = (w - pad * 2) / grid.w;
  const ch = (h - pad * 2) / grid.d;
  const vw = spec.floors[0]!.w * 2;
  const vd = spec.floors[0]!.d * 2;
  for (let z = 0; z < grid.d; z++) {
    for (let x = 0; x < grid.w; x++) {
      const cell = grid.cells[z * grid.w + x]!;
      let fill = '#121a24';
      if (cell === CELL.WALL) fill = '#243140';
      else if (cell === CELL.DOOR) fill = '#3d4e63';
      else if (cell === CELL.SHAFT) fill = '#c47b3a';
      else fill = '#182230';
      if (visited && cell !== CELL.WALL) {
        let seen = 0;
        const vx0 = x * 2;
        const vy0 = z * 2;
        for (let dy = 0; dy < 2; dy++) {
          for (let dx = 0; dx < 2; dx++) {
            const vi = floor * vw * vd + (vy0 + dy) * vw + (vx0 + dx);
            if (visited[vi]) seen += 1;
          }
        }
        if (seen > 0) fill = seen === 4 ? '#1ad6b0' : '#12786a';
        const frontier = seen > 0 && seen < 4;
        ctx.fillStyle = fill;
        ctx.fillRect(pad + x * cw, pad + z * ch, Math.ceil(cw), Math.ceil(ch));
        if (frontier) {
          ctx.fillStyle = '#ff8a3d';
          ctx.fillRect(pad + x * cw, pad + z * ch, 3, 3);
        }
        continue;
      }
      ctx.fillStyle = fill;
      ctx.fillRect(pad + x * cw, pad + z * ch, Math.ceil(cw), Math.ceil(ch));
    }
  }
  if (state) {
    const px = pad + (state.pos[0] / spec.cellSize) * cw;
    const py = pad + (state.pos[1] / spec.cellSize) * ch;
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(state.yaw);
    ctx.fillStyle = '#f4fbff';
    ctx.beginPath();
    ctx.moveTo(7, 0);
    ctx.lineTo(-5, 4);
    ctx.lineTo(-5, -4);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}
