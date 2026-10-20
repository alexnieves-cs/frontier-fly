// One development-seed search seen from above, for the report: docs/report/sar-belief.svg and its caption.
// Left, the prior the drone starts with, the shipped lane sweep's track and the probability search's track up
// to their finds. Right, the probability search's belief when it found the person.
import { writeFileSync } from 'node:fs';
import { sarScene, searchOptions, SAR_DEV_SEED_BASE, type SarScene } from './sar';
import type { RunRecord } from '../mission/autonomy/contracts';
import { createPerceptionSource } from '../mission/autonomy/perception';
import { createMissionRunner } from '../mission/autonomy/runner';
import { cellCenter } from '../mission/autonomy/sar/grid';
import type { SarSearch } from '../mission/autonomy/sar/search';

async function fly(scene: SarScene, method: 'lanes' | 'poa'): Promise<{ run: RunRecord; sar: SarSearch | null }> {
  const runner = await createMissionRunner(scene.mission, scene.world, createPerceptionSource(scene.world), {
    dt: 0.05,
    indoor: 'frontier',
    operator: () => 'approve',
    search: searchOptions(method, scene),
  });
  const limit = Math.ceil((scene.mission.budgets.timeSec + 5) / 0.05);
  // Stop at the find so the belief is the one the drone held when it confirmed the person.
  for (let i = 0; i < limit && !runner.done; i++) {
    runner.step();
    if (runner.snapshot().confirmed.some((c) => c.classId === 'person')) break;
  }
  const sar = (runner as unknown as { sar: SarSearch | null }).sar;
  return { run: runner.record(), sar };
}

export async function drawBelief(seed = SAR_DEV_SEED_BASE + 2, out = 'docs/report/sar-belief.svg'): Promise<void> {
  const scene = sarScene(seed, 'day');
  const poa = await fly(scene, 'poa');
  const lanes = await fly(scene, 'lanes');
  const sar = poa.sar!;
  const g = sar.grid;
  const panel = 300;
  const gap = 20;
  const W = panel * 2 + gap;
  const H = panel + 56;
  const scale = panel / Math.max(g.nx, g.ny) / g.cell;
  const X = (x: number, ox: number) => ox + (x - g.x0) * scale;
  const Y = (y: number) => 24 + panel - (y - g.y0) * scale;
  const parts: string[] = [`<rect width="${W}" height="${H}" fill="#fff"/>`];
  const heat = (mass: Float64Array, ox: number) => {
    let max = 0;
    for (const k of g.cells) max = Math.max(max, mass[k]! / g.free[k]!);
    for (const k of g.cells) {
      const [cx, cy] = cellCenter(g, k);
      const v = Math.sqrt(mass[k]! / g.free[k]! / max);
      const shade = Math.round(255 - v * 200);
      parts.push(
        `<rect x="${X(cx - g.cell / 2, ox).toFixed(1)}" y="${Y(cy + g.cell / 2).toFixed(1)}" width="${(g.cell * scale).toFixed(2)}" height="${(g.cell * scale).toFixed(2)}" fill="rgb(${shade},${shade + Math.round(v * 30)},255)"/>`,
      );
    }
    for (const b of scene.world.buildings) {
      const xs = b.footprint.map((p) => p.x);
      const ys = b.footprint.map((p) => p.y);
      parts.push(
        `<rect x="${X(Math.min(...xs), ox).toFixed(1)}" y="${Y(Math.max(...ys)).toFixed(1)}" width="${((Math.max(...xs) - Math.min(...xs)) * scale).toFixed(1)}" height="${((Math.max(...ys) - Math.min(...ys)) * scale).toFixed(1)}" fill="#6b5a48"/>`,
      );
    }
    const fence = scene.world.outdoor.geofence.polygon.map(([x, y]) => `${X(x, ox).toFixed(1)},${Y(y).toFixed(1)}`).join(' ');
    parts.push(`<polygon points="${fence}" fill="none" stroke="#333" stroke-width="1" stroke-dasharray="3 3"/>`);
  };
  const track = (run: RunRecord, ox: number, color: string, width: number) => {
    const hit = run.confirmed.find((c) => c.classId === 'person');
    const until = hit ? hit.tSec : Infinity;
    const pts = run.track.filter((s) => s.t <= until).map((s) => `${X(s.pos[0], ox).toFixed(1)},${Y(s.pos[1]).toFixed(1)}`);
    parts.push(`<polyline fill="none" stroke="${color}" stroke-width="${width}" stroke-linejoin="round" points="${pts.join(' ')}"/>`);
  };
  const marks = (ox: number) => {
    const [ix, iy] = scene.ipp;
    parts.push(`<circle cx="${X(ix, ox).toFixed(1)}" cy="${Y(iy).toFixed(1)}" r="5" fill="none" stroke="#d08a3a" stroke-width="2.5"/>`);
    const [px, py] = scene.truth.xy;
    parts.push(`<path d="M${(X(px, ox) - 5).toFixed(1)},${(Y(py) - 5).toFixed(1)} l10,10 m0,-10 l-10,10" stroke="#d93a3a" stroke-width="2.5"/>`);
    const [hx, hy] = scene.world.home;
    parts.push(`<rect x="${(X(hx, ox) - 4).toFixed(1)}" y="${(Y(hy) - 4).toFixed(1)}" width="8" height="8" fill="#2a9d55"/>`);
  };
  heat(sar.prior.cell, 0);
  track(lanes.run, 0, '#8a8f98', 1.5);
  track(poa.run, 0, '#1f6fd1', 2.5);
  marks(0);
  heat(sar.belief.mass, panel + gap);
  track(poa.run, panel + gap, '#1f6fd1', 2.5);
  marks(panel + gap);
  const label = (x: number, text: string) =>
    parts.push(`<text x="${x}" y="15" font-size="12" text-anchor="middle" fill="#333" font-family="sans-serif">${text}</text>`);
  const tPoa = poa.run.confirmed.find((c) => c.classId === 'person')?.tSec;
  const tLanes = lanes.run.confirmed.find((c) => c.classId === 'person')?.tSec;
  label(panel / 2, 'Prior and tracks to the find');
  label(panel + gap + panel / 2, `Belief at the find (${tPoa ? Math.round(tPoa) : '—'} s)`);
  const legend = [
    ['#1f6fd1', 'probability search'],
    ['#8a8f98', 'lane sweep (shipped)'],
  ] as const;
  legend.forEach(([c, t], i) => {
    parts.push(`<rect x="${10 + i * 170}" y="${H - 18}" width="14" height="3" fill="${c}"/>`);
    parts.push(`<text x="${30 + i * 170}" y="${H - 13}" font-size="11" fill="#333" font-family="sans-serif">${t}</text>`);
  });
  parts.push(`<circle cx="${360}" cy="${H - 17}" r="5" fill="none" stroke="#d08a3a" stroke-width="2.5"/><text x="370" y="${H - 13}" font-size="11" fill="#333" font-family="sans-serif">last known point</text>`);
  parts.push(`<path d="M470,${H - 22} l10,10 m0,-10 l-10,10" stroke="#d93a3a" stroke-width="2.5"/><text x="486" y="${H - 13}" font-size="11" fill="#333" font-family="sans-serif">person</text>`);
  parts.push(`<rect x="540" y="${H - 21}" width="8" height="8" fill="#2a9d55"/><text x="552" y="${H - 13}" font-size="11" fill="#333" font-family="sans-serif">home</text>`);
  writeFileSync(out, `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}">${parts.join('')}</svg>\n`);
  const caption =
    `Development world ${seed}, by day. Shading is probability per square metre of open ground, darker is likelier; buildings brown, the geofence dashed. ` +
    `The person (${scene.truth.strategy.replace('_', ' ')}, ${scene.truth.dIpp.toFixed(0)} m from the last known point) was confirmed after ${tPoa ? Math.round(tPoa) : '—'} s by the probability search and after ${tLanes ? Math.round(tLanes) : '—'} s by the shipped lane sweep. ` +
    'On the right, the ground the camera has looked at is pale: the frames have searched its probability away.';
  writeFileSync(out.replace(/\.svg$/, '.json'), `${JSON.stringify({ seed, caption }, null, 2)}\n`);
  console.log(`wrote ${out}`);
}
