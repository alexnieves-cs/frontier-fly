// Draw docs/media/real-plans.svg: three dev-split apartments, source plan beside the sim grid.
//
//   vite-node scripts/plans/figure.ts
import { readFileSync, writeFileSync } from 'node:fs';
import { parsePlanSet, PLAN_SET_PATH, planToBuilding } from '../../src/plans';
import { ringBox } from '../../src/plans/geometry';
import { CELL, SURFACE } from '../../src/shared/constants';

const IDS = ['sd-8e52f9f6f8', 'sd-133fc35ed6', 'sd-36b8bf2b15'];
const S = 15;
const GAP = 40;
const ROOM_COLORS = ['#cfe3f6', '#f6e3c4', '#d8efd2', '#f2d4e4', '#e2dcf5', '#f6f0c0', '#d3eeee', '#f3d6cf', '#e0e8c8', '#d6dcef', '#f0dcd0'];

const set = parsePlanSet(JSON.parse(readFileSync(PLAN_SET_PATH, 'utf8')));
const parts: string[] = [];
let y = 44;
let width = 0;
for (const id of IDS) {
  const plan = set.plans.find((p) => p.id === id);
  if (!plan) throw new Error(`no plan ${id}`);
  const conv = planToBuilding(plan)!;
  const floor = conv.spec.floors[0]!;
  const boxes = [...plan.walls, ...plan.areas.map((a) => a.poly)].map(ringBox);
  const pw = Math.max(...boxes.map((b) => b.x1));
  const ph = Math.max(...boxes.map((b) => b.y1));
  const rowH = Math.max(ph, floor.d) * S;
  const poly = (ring: number[], fill: string) =>
    `<polygon points="${ring.map((v, i) => (i % 2 === 0 ? 10 + v * S : y + rowH - v * S).toFixed(1)).join(' ')}" fill="${fill}"/>`;
  parts.push(`<text x="10" y="${y - 24}" class="t"><tspan font-weight="600">${id}</tspan> · Swiss Dwellings site ${plan.source.site}</text>`);
  parts.push(`<text x="10" y="${y - 8}" class="t">plan ${conv.planSize[0].toFixed(1)}×${conv.planSize[1].toFixed(1)} m · ${plan.areas.length} rooms · ${conv.planArea.toFixed(0)} m²</text>`);
  plan.areas.forEach((a, k) => parts.push(poly(a.poly, ROOM_COLORS[k % ROOM_COLORS.length]!)));
  for (const w of plan.walls) parts.push(poly(w, '#2b2f36'));
  for (const w of plan.windows) parts.push(poly(w, '#47b8f0'));
  for (const d of plan.doors) parts.push(poly(d, '#ef8a2c'));
  const gx0 = 10 + pw * S + GAP;
  for (let z = 0; z < floor.d; z++) {
    for (let x = 0; x < floor.w; x++) {
      const i = z * floor.w + x;
      const cell = floor.cells[i]!;
      let fill = '#2b2f36';
      if (cell === CELL.DOOR) fill = '#ef8a2c';
      else if (cell === CELL.WALL && floor.surfaces?.[i] === SURFACE.GLASS) fill = '#47b8f0';
      else if (cell !== CELL.WALL) {
        const r = floor.rooms.findIndex((room) => x >= room.x && x < room.x + room.w && z >= room.z && z < room.z + room.d);
        fill = r >= 0 ? ROOM_COLORS[conv.roomAreas[r]! % ROOM_COLORS.length]! : '#eef0f3';
      }
      parts.push(`<rect x="${gx0 + x * S}" y="${y + rowH - (z + 1) * S}" width="${S - 1}" height="${S - 1}" fill="${fill}"/>`);
    }
  }
  for (const box of conv.spec.furniture?.boxes ?? []) parts.push(`<circle cx="${(gx0 + box.cx * S).toFixed(1)}" cy="${(y + rowH - box.cy * S).toFixed(1)}" r="1.6" fill="#555"/>`);
  parts.push(`<circle cx="${gx0 + conv.spec.spawn[0] * S}" cy="${y + rowH - conv.spec.spawn[1] * S}" r="4" fill="#e0245e"/>`);
  parts.push(`<text x="${gx0}" y="${y - 8}" class="t">sim ${floor.w}×${floor.d} cells · ${floor.rooms.length} rooms · ${conv.walkableCells} walkable</text>`);
  width = Math.max(width, gx0 + floor.w * S + 10, 520);
  y += rowH + 64;
}
const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${y}" viewBox="0 0 ${width} ${y}" font-family="system-ui, sans-serif">
<style>.t{font-size:12px;fill:#333}</style>
<rect width="100%" height="100%" fill="#ffffff"/>
${parts.join('\n')}
<text x="10" y="${y - 40}" class="t">Left: source plan (rooms, walls, orange doors, blue windows).</text>
<text x="10" y="${y - 24}" class="t">Right: the 1 m sim grid. Red dot: spawn at the entrance. Grey dots: furniture.</text>
</svg>
`;
writeFileSync('docs/media/real-plans.svg', svg);
console.log(`wrote docs/media/real-plans.svg (${svg.length} bytes)`);
