// Build public/plans/swiss-dwellings.json from the Swiss Dwellings dataset (CC BY 4.0).
//
//   curl -L -o sd.zip https://zenodo.org/api/records/7788422/files/swiss-dwellings-v3.0.0.zip/content
//   unzip sd.zip 'swiss-dwellings-v3.0.0/geometries.csv'
//   vite-node scripts/plans/import-swiss-dwellings.ts -- swiss-dwellings-v3.0.0/geometries.csv [--dev 20 --eval 60]
//
// Deterministic: apartments are ordered by a hash of their id, at most one per building, and
// buildings are split dev/eval by a hash of the building id (one in four is dev). A plan is kept
// only when it converts (src/plans/convert.ts) into a building whose rooms are all reachable, that
// keeps all but at most one of its rooms, and that fits the stage-3 budget. Every rejection is
// counted in the output's `selection` block.
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname } from 'node:path';
import { planSeed, planToBuilding } from '../../src/plans/convert';
import { axisAlignedShare, dominantRotation, ringArea, ringBox, rotateRing } from '../../src/plans/geometry';
import type { PlanRecord, PlanSet } from '../../src/plans/types';
import { DATASET, eachRow, INDOOR, type Row } from './swiss-common';

/** Stage-3 buildings are 20×20 cells; real apartments may be a little larger after the wall warp. */
const MAX_GRID = 28;
const MIN_AXIS_SHARE = 0.97;
const MIN_ROOMS = 3;

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 ? process.argv[i + 1]! : fallback;
}

interface Meta {
  site: number;
  building: number;
  floors: Set<number>;
  residential: boolean;
  indoor: number;
  stairs: boolean;
}

const round = (ring: number[]) => ring.map((v) => Math.round(v * 100) / 100);

export function toRecord(id: string, meta: Meta, floor: number, rows: Row[], split: 'dev' | 'eval'): PlanRecord {
  const areas = rows.filter((r) => r.type === 'area' && INDOOR.has(r.subtype));
  const walls = rows.filter((r) => r.type === 'separator' && (r.subtype === 'WALL' || r.subtype === 'COLUMN'));
  const doors = rows.filter((r) => r.type === 'opening' && (r.subtype === 'DOOR' || r.subtype === 'ENTRANCE_DOOR'));
  const windows = rows.filter((r) => r.type === 'opening' && r.subtype === 'WINDOW');
  const rotation = dominantRotation([...walls, ...areas].map((r) => r.ring!));
  const rot = (rs: Row[]) => rs.map((r) => rotateRing(r.ring!, rotation));
  const rotated = { areas: rot(areas), walls: rot(walls), doors: rot(doors), windows: rot(windows) };
  let x0 = Infinity;
  let y0 = Infinity;
  for (const ring of [...rotated.areas, ...rotated.walls]) {
    const b = ringBox(ring);
    x0 = Math.min(x0, b.x0);
    y0 = Math.min(y0, b.y0);
  }
  const shift = (ring: number[]) => round(ring.map((v, i) => (i % 2 === 0 ? v - x0 : v - y0)));
  return {
    id,
    split,
    source: { dataset: `swiss-dwellings-${DATASET.version}`, site: meta.site, building: meta.building, floor, apartment: rows[0]!.apartment },
    rotation: Math.round(rotation * 1e5) / 1e5,
    areas: rotated.areas.map((ring, k) => ({ type: areas[k]!.subtype, poly: shift(ring) })),
    walls: rotated.walls.map(shift),
    doors: rotated.doors.map(shift),
    windows: rotated.windows.map(shift),
  };
}

async function main(): Promise<void> {
  const path = process.argv.slice(2).find((a) => !a.startsWith('--') && a.endsWith('.csv'));
  if (!path) throw new Error('usage: import-swiss-dwellings.ts -- geometries.csv');
  const want = { dev: Number(arg('dev', '20')), eval: Number(arg('eval', '60')) };
  const out = arg('out', 'public/plans/swiss-dwellings.json');

  const metas = new Map<string, Meta>();
  await eachRow(path, false, (row) => {
    let m = metas.get(row.apartment);
    if (!m) {
      m = { site: row.site, building: row.building, floors: new Set(), residential: true, indoor: 0, stairs: false };
      metas.set(row.apartment, m);
    }
    m.floors.add(row.floor);
    if (row.usage !== 'RESIDENTIAL') m.residential = false;
    if (row.type === 'area' && INDOOR.has(row.subtype)) m.indoor += 1;
    if ((row.type === 'area' && row.subtype === 'STAIRCASE') || (row.type === 'feature' && row.subtype === 'STAIRS')) m.stairs = true;
  });

  const counts: Record<string, number> = { apartments: metas.size };
  const bump = (k: string) => (counts[k] = (counts[k] ?? 0) + 1);
  const candidates: string[] = [];
  for (const [id, m] of metas) {
    if (!m.residential) bump('notResidential');
    else if (m.floors.size !== 1) bump('multiFloor');
    else if (m.stairs) bump('stairs');
    else if (m.indoor < MIN_ROOMS) bump('fewRooms');
    else candidates.push(id);
  }
  counts.candidates = candidates.length;
  const hashed = candidates.map((id) => ({ id, h: planSeed(id) })).sort((a, b) => a.h - b.h || (a.id < b.id ? -1 : 1));
  // At most one apartment per building, so the set spans many architects and sites.
  const byBuilding = new Set<number>();
  const picked: string[] = [];
  for (const { id } of hashed) {
    const m = metas.get(id)!;
    if (byBuilding.has(m.building)) continue;
    byBuilding.add(m.building);
    picked.push(id);
  }
  counts.buildings = picked.length;
  // Generous prefix so rejections don't run us dry; geometry is collected only for these.
  const pool = new Set(picked.slice(0, (want.dev + want.eval) * 6));
  const rows = new Map<string, Row[]>();
  await eachRow(path, true, (row) => {
    if (!pool.has(row.apartment)) return;
    let list = rows.get(row.apartment);
    if (!list) rows.set(row.apartment, (list = []));
    list.push(row);
  });

  const plans: PlanRecord[] = [];
  const have = { dev: 0, eval: 0 };
  for (const apt of picked) {
    if (!pool.has(apt)) break;
    if (have.dev >= want.dev && have.eval >= want.eval) break;
    const m = metas.get(apt)!;
    const split = planSeed(`building-${m.building}`) % 4 === 0 ? 'dev' : 'eval';
    if (have[split] >= want[split]) continue;
    const list = rows.get(apt) ?? [];
    bump('examined');
    const rings = list.filter((r) => r.type === 'separator' && r.subtype === 'WALL').map((r) => r.ring!);
    if (rings.length === 0) {
      bump('rejectNoWalls');
      continue;
    }
    const record = toRecord(`sd-${apt.slice(0, 10)}`, m, [...m.floors][0]!, list, split);
    if (axisAlignedShare(record.walls) < MIN_AXIS_SHARE) {
      bump('rejectNotAxisAligned');
      continue;
    }
    const conv = planToBuilding(record);
    if (!conv) {
      bump('rejectUnreachable');
      continue;
    }
    if (conv.gridSize[0] > MAX_GRID || conv.gridSize[1] > MAX_GRID) {
      bump('rejectTooLarge');
      continue;
    }
    if (conv.spec.floors[0]!.rooms.length < MIN_ROOMS) {
      bump('rejectFewRoomsAfterConvert');
      continue;
    }
    if (conv.spec.floors[0]!.rooms.length < record.areas.length - 1) {
      bump('rejectLostRooms');
      continue;
    }
    plans.push(record);
    have[split] += 1;
  }
  counts.kept = plans.length;
  const area = plans.reduce((s, p) => s + p.areas.reduce((a, r) => a + ringArea(r.poly), 0), 0);
  console.log(JSON.stringify({ counts, have, meanIndoorArea: area / Math.max(1, plans.length) }, null, 2));
  const set: PlanSet & { selection: unknown } = {
    dataset: DATASET,
    selection: { rule: 'see scripts/plans/import-swiss-dwellings.ts', maxGrid: MAX_GRID, minAxisShare: MIN_AXIS_SHARE, minRooms: MIN_ROOMS, counts },
    plans,
  };
  mkdirSync(dirname(out), { recursive: true });
  writeFileSync(out, `${JSON.stringify(set)}\n`);
}

await main();
