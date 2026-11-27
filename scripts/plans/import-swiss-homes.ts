// Build public/plans/swiss-homes.json: real multi-storey homes (maisonettes) from the Swiss
// Dwellings dataset (CC BY 4.0), each storey joined to the next by its real staircase.
//
//   curl -L -o sd.zip https://zenodo.org/api/records/7788422/files/swiss-dwellings-v3.0.0.zip/content
//   unzip sd.zip 'swiss-dwellings-v3.0.0/geometries.csv'
//   vite-node scripts/plans/import-swiss-homes.ts -- swiss-dwellings-v3.0.0/geometries.csv [--dev 20 --eval 60]
//
// Deterministic, like import-swiss-dwellings.ts: homes are ordered by a hash of their id, at most one
// per building, and buildings are split dev/eval by a hash of the building id (one in four is dev).
// A home is kept when it is residential, has two or three storeys at consecutive elevations, has a
// staircase on every storey, its storeys share a plan frame (the staircases overlap), and it converts
// (src/plans/convert.ts floorsToBuilding) into a building whose every storey is reachable through a
// stair shaft, that keeps all but at most one room per storey, and that fits the grid budget. Every
// rejection is counted in the output's `selection` block.
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname } from 'node:path';
import { planSeed } from '../../src/plans/convert';
import { homeToBuilding } from '../../src/plans/homes';
import { axisAlignedShare, dominantRotation, ringArea, ringBox, rotateRing } from '../../src/plans/geometry';
import type { HomeRecord, HomeSet } from '../../src/plans/types';
import { DATASET, eachRow, INDOOR, type Row } from './swiss-common';

export const HOMES_DATASET: HomeSet['dataset'] = {
  ...DATASET,
  changes:
    'A subset of two- and three-storey homes (maisonettes). Rooms, walls, doors, windows and staircases only (features other than stairs, balconies and simulation results dropped). All storeys rotated together so walls run along the axes, translated to the origin, rounded to 1 cm.',
};

const MAX_GRID = 28;
const MIN_AXIS_SHARE = 0.97;
const MIN_ROOMS = 3;
const MAX_STOREYS = 3;
/** Consecutive storeys are 2.2-4.5 m apart; a larger gap skips a storey that is not part of the home. */
const STOREY_GAP: [number, number] = [2.2, 4.5];

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 ? process.argv[i + 1]! : fallback;
}

interface Meta {
  site: number;
  building: number;
  floors: Map<number, number>;
  residential: boolean;
  stairFloors: Set<number>;
}

const isStair = (r: Row) => (r.type === 'feature' && r.subtype === 'STAIRS') || (r.type === 'area' && r.subtype === 'STAIRCASE');
const round = (ring: number[]) => ring.map((v) => Math.round(v * 100) / 100);

export function toHome(id: string, meta: Meta, floors: number[], rows: Row[], split: 'dev' | 'eval'): HomeRecord {
  const pick = (f: number) => rows.filter((r) => r.floor === f);
  const parts = floors.map((f) => {
    const rs = pick(f);
    return {
      areas: rs.filter((r) => r.type === 'area' && INDOOR.has(r.subtype)),
      walls: rs.filter((r) => r.type === 'separator' && (r.subtype === 'WALL' || r.subtype === 'COLUMN')),
      doors: rs.filter((r) => r.type === 'opening' && (r.subtype === 'DOOR' || r.subtype === 'ENTRANCE_DOOR')),
      windows: rs.filter((r) => r.type === 'opening' && r.subtype === 'WINDOW'),
      stairs: rs.filter(isStair),
    };
  });
  const rotation = dominantRotation(parts.flatMap((p) => [...p.walls, ...p.areas].map((r) => r.ring!)));
  const rot = (rs: Row[]) => rs.map((r) => rotateRing(r.ring!, rotation));
  const rotated = parts.map((p) => ({ areas: rot(p.areas), walls: rot(p.walls), doors: rot(p.doors), windows: rot(p.windows), stairs: rot(p.stairs) }));
  let x0 = Infinity;
  let y0 = Infinity;
  for (const p of rotated) {
    for (const ring of [...p.areas, ...p.walls]) {
      const b = ringBox(ring);
      x0 = Math.min(x0, b.x0);
      y0 = Math.min(y0, b.y0);
    }
  }
  const shift = (ring: number[]) => round(ring.map((v, i) => (i % 2 === 0 ? v - x0 : v - y0)));
  return {
    id,
    split,
    source: {
      dataset: `swiss-dwellings-${DATASET.version}`,
      site: meta.site,
      building: meta.building,
      floors,
      elevations: floors.map((f) => Math.round(meta.floors.get(f)! * 100) / 100),
      apartment: rows[0]!.apartment,
    },
    rotation: Math.round(rotation * 1e5) / 1e5,
    storeys: rotated.map((p, k) => ({
      areas: p.areas.map((ring, j) => ({ type: parts[k]!.areas[j]!.subtype, poly: shift(ring) })),
      walls: p.walls.map(shift),
      doors: p.doors.map(shift),
      windows: p.windows.map(shift),
      stairs: p.stairs.map(shift),
    })),
  };
}

async function main(): Promise<void> {
  const path = process.argv.slice(2).find((a) => !a.startsWith('--') && a.endsWith('.csv'));
  if (!path) throw new Error('usage: import-swiss-homes.ts -- geometries.csv');
  const want = { dev: Number(arg('dev', '20')), eval: Number(arg('eval', '60')) };
  const out = arg('out', 'public/plans/swiss-homes.json');

  const metas = new Map<string, Meta>();
  await eachRow(path, false, (row) => {
    let m = metas.get(row.apartment);
    if (!m) {
      m = { site: row.site, building: row.building, floors: new Map(), residential: true, stairFloors: new Set() };
      metas.set(row.apartment, m);
    }
    if (!m.floors.has(row.floor)) m.floors.set(row.floor, row.elevation);
    if (row.usage !== 'RESIDENTIAL') m.residential = false;
    if (isStair(row)) m.stairFloors.add(row.floor);
  });

  const counts: Record<string, number> = { apartments: metas.size };
  const bump = (k: string) => (counts[k] = (counts[k] ?? 0) + 1);
  const candidates: { id: string; floors: number[] }[] = [];
  for (const [id, m] of metas) {
    const floors = [...m.floors.keys()].sort((a, b) => m.floors.get(a)! - m.floors.get(b)! || a - b);
    const gaps = floors.slice(1).map((f, k) => m.floors.get(f)! - m.floors.get(floors[k]!)!);
    if (!m.residential) bump('notResidential');
    else if (floors.length < 2) bump('singleStorey');
    else if (floors.length > MAX_STOREYS) bump('tooManyStoreys');
    else if (floors.some((f) => !m.stairFloors.has(f))) bump('storeyWithoutStairs');
    else if (gaps.some((g) => g < STOREY_GAP[0] || g > STOREY_GAP[1])) bump('storeysNotConsecutive');
    else candidates.push({ id, floors });
  }
  counts.candidates = candidates.length;
  const hashed = candidates.map((c) => ({ ...c, h: planSeed(c.id) })).sort((a, b) => a.h - b.h || (a.id < b.id ? -1 : 1));
  const byBuilding = new Set<number>();
  const picked: { id: string; floors: number[] }[] = [];
  for (const c of hashed) {
    const m = metas.get(c.id)!;
    if (byBuilding.has(m.building)) continue;
    byBuilding.add(m.building);
    picked.push(c);
  }
  counts.buildings = picked.length;
  const pool = new Set(picked.map((c) => c.id));
  const rows = new Map<string, Row[]>();
  await eachRow(path, true, (row) => {
    if (!pool.has(row.apartment)) return;
    let list = rows.get(row.apartment);
    if (!list) rows.set(row.apartment, (list = []));
    list.push(row);
  });

  const homes: HomeRecord[] = [];
  const have = { dev: 0, eval: 0 };
  for (const { id: apt, floors } of picked) {
    if (have.dev >= want.dev && have.eval >= want.eval) break;
    const m = metas.get(apt)!;
    const split = planSeed(`building-${m.building}`) % 4 === 0 ? 'dev' : 'eval';
    if (have[split] >= want[split]) continue;
    const list = rows.get(apt) ?? [];
    bump('examined');
    const home = toHome(`sh-${apt.slice(0, 10)}`, m, floors, list, split);
    if (home.storeys.some((s) => s.walls.length === 0 || s.areas.length === 0)) {
      bump('rejectEmptyStorey');
      continue;
    }
    if (axisAlignedShare(home.storeys.flatMap((s) => s.walls)) < MIN_AXIS_SHARE) {
      bump('rejectNotAxisAligned');
      continue;
    }
    const conv = homeToBuilding(home);
    if (!conv) {
      bump('rejectUnreachable');
      continue;
    }
    if (conv.gridSize[0] > MAX_GRID || conv.gridSize[1] > MAX_GRID) {
      bump('rejectTooLarge');
      continue;
    }
    const rooms = conv.spec.floors.map((f) => f.rooms.length);
    if (rooms.reduce((a, b) => a + b, 0) < MIN_ROOMS) {
      bump('rejectFewRoomsAfterConvert');
      continue;
    }
    if (home.storeys.some((s, k) => rooms[k]! < s.areas.length - 1)) {
      bump('rejectLostRooms');
      continue;
    }
    homes.push(home);
    have[split] += 1;
  }
  counts.kept = homes.length;
  const area = homes.reduce((s, h) => s + h.storeys.reduce((t, st) => t + st.areas.reduce((a, r) => a + ringArea(r.poly), 0), 0), 0);
  console.log(JSON.stringify({ counts, have, meanIndoorArea: area / Math.max(1, homes.length) }, null, 2));
  const set: HomeSet & { selection: unknown } = {
    dataset: HOMES_DATASET,
    selection: { rule: 'see scripts/plans/import-swiss-homes.ts', maxGrid: MAX_GRID, minAxisShare: MIN_AXIS_SHARE, minRooms: MIN_ROOMS, maxStoreys: MAX_STOREYS, storeyGap: STOREY_GAP, counts },
    homes,
  };
  mkdirSync(dirname(out), { recursive: true });
  writeFileSync(out, `${JSON.stringify(set)}\n`);
}

await main();
