// Shared Swiss Dwellings 3.0.0 (CC BY 4.0) reading for the plan importers.
import { createReadStream } from 'node:fs';
import { createInterface } from 'node:readline';
import type { PlanSet } from '../../src/plans/types';

export const DATASET: PlanSet['dataset'] = {
  name: 'Swiss Dwellings',
  version: '3.0.0',
  license: 'CC BY 4.0',
  url: 'https://zenodo.org/records/7788422',
  doi: '10.5281/zenodo.7788422',
  attribution:
    'Standfest, M., Franzen, M., Schröder, Y., Gonzalez Medina, L., Villanueva Hernandez, Y., Buck, J. H., Tan, Y.-L., Niedzwiecka, M., Colmegna, R. (2023). Swiss Dwellings: A large dataset of apartment models. Archilyse AG. Zenodo.',
  changes:
    'A subset of single-floor apartments. Rooms, walls, doors and windows only (features, balconies, stairs and simulation results dropped). Rotated so walls run along the axes, translated to the origin, rounded to 1 cm.',
};

/** Indoor room types kept. Everything else (balconies, shafts, stairs, voids) is left out. */
export const INDOOR = new Set([
  'ROOM',
  'BEDROOM',
  'LIVING_ROOM',
  'LIVING_DINING',
  'DINING',
  'KITCHEN',
  'KITCHEN_DINING',
  'BATHROOM',
  'CORRIDOR',
  'STOREROOM',
]);

export interface Row {
  apartment: string;
  site: number;
  building: number;
  floor: number;
  usage: string;
  type: string;
  subtype: string;
  ring: number[] | null;
  /** Storey elevation, m (the column after the geometry). */
  elevation: number;
}

export function parseRow(line: string, withGeometry: boolean): Row | null {
  const q0 = line.indexOf('"');
  const q1 = line.lastIndexOf('"');
  if (q0 < 0 || q1 <= q0) return null;
  const head = line.slice(0, q0).split(',');
  // apartment_id,site_id,building_id,plan_id,floor_id,unit_id,area_id,unit_usage,entity_type,entity_subtype,geometry,...
  if (head.length < 11) return null;
  let ring: number[] | null = null;
  if (withGeometry) {
    const wkt = line.slice(q0 + 1, q1);
    if (!wkt.startsWith('POLYGON ((')) return null;
    const outer = wkt.slice('POLYGON (('.length, wkt.indexOf(')'));
    const pts = outer.split(',').map((p) => p.trim().split(/\s+/).map(Number));
    ring = [];
    for (let i = 0; i < pts.length - 1; i++) ring.push(pts[i]![0]!, pts[i]![1]!);
  }
  return {
    apartment: head[0]!,
    site: Number(head[1]),
    building: Number(head[2]),
    floor: Number(head[4]),
    usage: head[7]!,
    type: head[8]!,
    subtype: head[9]!,
    ring,
    elevation: Number(line.slice(q1 + 1).split(',')[1]),
  };
}

export async function eachRow(path: string, withGeometry: boolean, fn: (row: Row) => void): Promise<void> {
  const rl = createInterface({ input: createReadStream(path), crlfDelay: Infinity });
  let first = true;
  for await (const line of rl) {
    if (first) {
      first = false;
      continue;
    }
    const row = parseRow(line, withGeometry);
    if (row && row.apartment) fn(row);
  }
}

