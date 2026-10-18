import { OBJECT_IDS } from '../../targets/catalog';
import { placeIndoor, placeOutdoor } from '../../targets/placement';
import type { OutdoorArea, PlacedTarget, TargetScene } from '../../targets/types';
import { RNG } from '../../shared/rng';
import { generateOutdoor } from '../../world/outdoor/generate';
import { MAX_BUILDINGS } from '../../world/outdoor/constants';
import { sampleHeight } from '../../world/outdoor/terrain';
import type { BuildingShell, OutdoorWorld } from '../../world/outdoor/types';
import { pointInRing, ringsOverlap, signedArea } from '../polygon';
import { suggestedTargets } from '../sample';
import type { Mission, NoFlyZone, Point } from '../types';
import type { WorldFootprint, WorldProvider, WorldSnapshot } from '../world';
import type { MissionBuilding, MissionForWorldOptions, MissionWorld, MissionWorldOptions, TargetWant } from './contracts';

/** Lowest altitude above ground the mission allows. Clears undergrowth and the drone's own downwash. */
export const MISSION_MIN_AGL_M = 3;
/** Highest altitude above ground. Canopy tops reach about 8 m, so this leaves room for an aerial sweep. */
export const MISSION_MAX_AGL_M = 30;
/** Foliage is porous. Occlusion and target placement use a box this fraction of the canopy radius. */
export const CANOPY_CORE = 0.55;

const SITE_NAMES = ['Pine hollow', 'Cedar ridge', 'Birch flats', 'Hemlock draw', 'Aspen bench', 'Fir creek'] as const;
const SMALL_LABELS = ['Cabin', 'Shed', 'Bunkhouse', 'Pump house', 'Tool shed'] as const;
const LARGE_LABELS = ['Barn', 'Workshop', 'Boathouse'] as const;
const TALL_LABELS = ['Lodge', 'Ranger station', 'Farmhouse'] as const;

/** An axis-aligned ground rectangle, world frame. */
export interface GroundBox {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

export function siteNameFor(seed: number): string {
  return SITE_NAMES[(seed >>> 0) % SITE_NAMES.length]!;
}

function shellSize(shell: BuildingShell): { w: number; d: number } {
  const floor = shell.spec.floors[0]!;
  return { w: floor.w * shell.spec.cellSize, d: floor.d * shell.spec.cellSize };
}

/** World-frame outer footprint, counter-clockwise from the min corner. */
export function shellFootprint(shell: BuildingShell): Point[] {
  const { w, d } = shellSize(shell);
  const [ox, oy] = shell.origin;
  return [
    { x: ox, y: oy },
    { x: ox + w, y: oy },
    { x: ox + w, y: oy + d },
    { x: ox, y: oy + d },
  ];
}

/** Seeded labels by size and floor count. Labels are unique within a world. */
export function buildingLabels(outdoor: OutdoorWorld, seed: number): string[] {
  const rng = new RNG(seed >>> 0, 'building-labels');
  const used = new Set<string>();
  return outdoor.buildings.map((shell) => {
    const { w, d } = shellSize(shell);
    const pool: readonly string[] = shell.spec.floors.length > 1 ? TALL_LABELS : w * d >= 121 ? LARGE_LABELS : SMALL_LABELS;
    const start = rng.int(pool.length);
    for (let i = 0; i < pool.length; i++) {
      const label = pool[(start + i) % pool.length]!;
      if (!used.has(label)) {
        used.add(label);
        return label;
      }
    }
    const label = `${pool[start]!} ${shell.id + 1}`;
    used.add(label);
    return label;
  });
}

function roomTotal(shell: BuildingShell): number {
  return shell.spec.floors.reduce((sum, floor) => sum + floor.rooms.length, 0);
}

function lightsFor(shell: BuildingShell, mode: MissionWorldOptions['interiorLights'], seed: number): Uint8Array {
  const lights = new Uint8Array(roomTotal(shell));
  if (mode === 'off') return lights;
  if (mode === 'seeded') {
    const rng = new RNG(seed >>> 0, `lights-${shell.id}`);
    for (let i = 0; i < lights.length; i++) lights[i] = rng.bool(0.5) ? 1 : 0;
    return lights;
  }
  return lights.fill(1);
}

/** What one scene must hold. Persons without a state take any state. */
interface Demand {
  people: Array<PlacedTarget['state'] | null>;
  objects: string[];
  decoys: number;
}

function demandOf(wants: readonly TargetWant[], decoys: number): Demand {
  return {
    people: wants.filter((want) => want.classId === 'person').map((want) => want.state ?? null),
    objects: wants.filter((want) => want.classId !== 'person').map((want) => want.classId),
    decoys,
  };
}

function poolSize(demand: Demand): { people: number; objects: number } {
  const byState = new Map<string, number>();
  let any = 0;
  for (const state of demand.people) {
    if (state === null) any += 1;
    else byState.set(state, (byState.get(state) ?? 0) + 1);
  }
  const perState = Math.max(0, ...byState.values()) + any;
  const byObject = new Map<string, number>();
  for (const id of demand.objects) byObject.set(id, (byObject.get(id) ?? 0) + 1);
  const perObject = Math.max(0, ...byObject.values());
  // Placement cycles person states (5) and object ids (8), so these sizes hold every request.
  return { people: perState * 5, objects: perObject * OBJECT_IDS.length };
}

/** Pick the demanded targets from a placed pool. Null when the pool lacks one. */
function selectFrom(pool: readonly PlacedTarget[], demand: Demand, rng: RNG): PlacedTarget[] | null {
  const taken = new Set<string>();
  const out: PlacedTarget[] = [];
  const take = (match: (target: PlacedTarget) => boolean, random: boolean): boolean => {
    const options = pool.filter((target) => !taken.has(target.id) && match(target));
    if (options.length === 0) return false;
    const pick = random ? options[rng.int(options.length)]! : options[0]!;
    taken.add(pick.id);
    out.push(pick);
    return true;
  };
  // Exact states first so an open request cannot steal the only lying person.
  for (const state of demand.people) {
    if (state !== null && !take((target) => target.classId === 'person' && target.state === state, false)) return null;
  }
  for (const state of demand.people) {
    if (state === null && !take((target) => target.classId === 'person', true)) return null;
  }
  for (const id of demand.objects) {
    if (!take((target) => target.classId === id, false)) return null;
  }
  for (let i = 0; i < demand.decoys; i++) {
    if (!take((target) => target.category === 'decoy', false)) return null;
  }
  return out;
}

function prefixed(targets: PlacedTarget[], prefix: string): PlacedTarget[] {
  return targets.map((target) => ({ ...target, id: `${prefix}/${target.id}` }));
}

function plantIndoor(shell: BuildingShell, prefix: string, wants: readonly TargetWant[], decoys: number, seed: number): TargetScene {
  const demand = demandOf(wants, decoys);
  const sceneSeed = (seed + 31 * (shell.id + 1)) >>> 0;
  if (demand.people.length === 0 && demand.objects.length === 0 && decoys === 0) {
    return { seed: sceneSeed, space: 'indoor', targets: [] };
  }
  const size = poolSize(demand);
  for (let attempt = 0; attempt < 16; attempt++) {
    const attemptSeed = (sceneSeed + attempt * 7919) >>> 0;
    const pool = placeIndoor(shell.spec, { seed: attemptSeed, people: size.people, objects: size.objects, decoys: Math.ceil(decoys / 4) * 4 });
    const picked = selectFrom(pool.targets, demand, new RNG(attemptSeed, 'select-indoor'));
    if (picked) return { seed: attemptSeed, space: 'indoor', targets: prefixed(picked, prefix) };
  }
  throw new Error(`Could not place the requested targets in ${prefix} for seed ${seed}.`);
}

/** Trunks, bushes, and canopy cores, as ground boxes. Targets are never planted inside these. */
export function vegetationBoxes(outdoor: OutdoorWorld): GroundBox[] {
  return outdoor.trees.map((tree) => {
    const half = tree.kind === 'undergrowth' ? tree.canopyRadius : Math.max(tree.trunkRadius + 0.35, tree.canopyRadius * CANOPY_CORE);
    return { minX: tree.x - half, minY: tree.y - half, maxX: tree.x + half, maxY: tree.y + half };
  });
}

function ringBox(points: readonly [number, number][], pad: number): GroundBox {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const [x, y] of points) {
    minX = Math.min(minX, x);
    minY = Math.min(minY, y);
    maxX = Math.max(maxX, x);
    maxY = Math.max(maxY, y);
  }
  return { minX: minX - pad, minY: minY - pad, maxX: maxX + pad, maxY: maxY + pad };
}

/** Ground boxes no target sits in: buildings with a margin, vegetation, and no-fly zones. */
export function plantingObstacles(outdoor: OutdoorWorld): GroundBox[] {
  const buildings = outdoor.buildings.map((shell) => {
    const { w, d } = shellSize(shell);
    return { minX: shell.origin[0] - 1.5, minY: shell.origin[1] - 1.5, maxX: shell.origin[0] + w + 1.5, maxY: shell.origin[1] + d + 1.5 };
  });
  const noFly = outdoor.noFly.map((zone) => ringBox(zone.polygon, 1));
  return [...buildings, ...vegetationBoxes(outdoor), ...noFly];
}

/** Largest square around the spawn whose edges stay inside the geofence. */
export function outdoorSearchBox(outdoor: OutdoorWorld): GroundBox {
  const ring = outdoor.geofence.polygon.map(([x, y]) => ({ x, y }));
  const [cx, cy] = outdoor.spawn;
  const fits = (half: number): boolean => {
    for (let s = 0; s <= 16; s++) {
      const t = -half + (2 * half * s) / 16;
      for (const p of [
        { x: cx + t, y: cy - half },
        { x: cx + t, y: cy + half },
        { x: cx - half, y: cy + t },
        { x: cx + half, y: cy + t },
      ]) {
        if (!pointInRing(p, ring)) return false;
      }
    }
    return true;
  };
  let lo = 2;
  let hi = outdoor.size / 2;
  for (let i = 0; i < 24; i++) {
    const mid = (lo + hi) / 2;
    if (fits(mid)) lo = mid;
    else hi = mid;
  }
  const half = lo - 0.5;
  return { minX: cx - half, minY: cy - half, maxX: cx + half, maxY: cy + half };
}

function minTerrain(outdoor: OutdoorWorld, box: GroundBox): number {
  const { resolution: n, size, heights } = outdoor.terrain;
  const cell = size / (n - 1);
  let min = Infinity;
  for (let j = 0; j < n; j++) {
    const y = j * cell;
    if (y < box.minY - cell || y > box.maxY + cell) continue;
    for (let i = 0; i < n; i++) {
      const x = i * cell;
      if (x < box.minX - cell || x > box.maxX + cell) continue;
      min = Math.min(min, heights[j * n + i]!);
    }
  }
  return Number.isFinite(min) ? min : sampleHeight(outdoor.terrain, (box.minX + box.maxX) / 2, (box.minY + box.maxY) / 2);
}

/** The rectangle placeOutdoor uses. groundZ is the lowest terrain in it, so draped targets never sit below it. */
export function outdoorArea(outdoor: OutdoorWorld): OutdoorArea {
  const box = outdoorSearchBox(outdoor);
  return {
    id: 'woods',
    ...box,
    groundZ: minTerrain(outdoor, box),
    obstacles: plantingObstacles(outdoor),
  };
}

function drape(outdoor: OutdoorWorld, target: PlacedTarget): PlacedTarget {
  const lift = target.classId === 'coat' ? 0.45 : 0;
  const at = (x: number, y: number): number => sampleHeight(outdoor.terrain, x, y) + lift;
  return {
    ...target,
    feet: [target.feet[0], target.feet[1], at(target.feet[0], target.feet[1])],
    path: target.path?.map(([x, y]) => [x, y, at(x, y)] as [number, number, number]),
  };
}

function plantOutdoor(outdoor: OutdoorWorld, wants: readonly TargetWant[], seed: number): TargetScene {
  const area = outdoorArea(outdoor);
  const sceneSeed = (seed + 17) >>> 0;
  const demand = demandOf(wants, 0);
  if (wants.length === 0) return { seed: sceneSeed, space: 'outdoor', outdoor: area, targets: [] };
  const size = poolSize(demand);
  for (let attempt = 0; attempt < 16; attempt++) {
    const attemptSeed = (sceneSeed + attempt * 7919) >>> 0;
    const pool = placeOutdoor(area, { seed: attemptSeed, people: size.people, objects: size.objects, decoys: 0 });
    const picked = selectFrom(pool.targets, demand, new RNG(attemptSeed, 'select-outdoor'));
    if (picked) {
      return {
        seed: attemptSeed,
        space: 'outdoor',
        outdoor: area,
        targets: prefixed(picked, 'outdoor').map((target) => drape(outdoor, target)),
      };
    }
  }
  throw new Error(`Could not place the requested outdoor targets for seed ${seed}.`);
}

/** Seeded default: one person in a seeded building and one object in the woods. */
function defaultWants(seed: number, buildings: number): TargetWant[] {
  return [
    { classId: 'person', where: 'indoor', building: (seed >>> 0) % Math.max(1, buildings) },
    { classId: OBJECT_IDS[(seed >>> 0) % OBJECT_IDS.length]!, where: 'outdoor' },
  ];
}

/**
 * Seeded woods-to-building world: the outdoor generator's forest and shells,
 * labeled buildings, the requested targets, and lighting.
 */
export function buildMissionWorld(options: MissionWorldOptions): MissionWorld {
  const seed = options.seed >>> 0;
  const wanted = options.targets ?? [];
  const highest = wanted.reduce((max, want) => (want.where === 'indoor' ? Math.max(max, want.building ?? 0) : max), -1);
  if (highest >= MAX_BUILDINGS) throw new RangeError(`Target building ${highest} is beyond the ${MAX_BUILDINGS}-building limit.`);
  if (options.buildings != null && highest >= options.buildings) {
    throw new RangeError(`Target building ${highest} does not exist in a ${options.buildings}-building world.`);
  }
  let outdoor = generateOutdoor({ seed, buildings: options.buildings });
  // An omitted count comes from the seed; grow it when a target names a later building.
  if (outdoor.buildings.length <= highest) outdoor = generateOutdoor({ seed, buildings: highest + 1 });
  const wants = options.targets ?? defaultWants(seed, outdoor.buildings.length);
  const labels = buildingLabels(outdoor, seed);
  const decoys = Math.max(0, Math.floor(options.decoys ?? 0));
  const count = outdoor.buildings.length;
  const buildings: MissionBuilding[] = outdoor.buildings.map((shell, index) => {
    const id = `bldg-${shell.id}`;
    const mine = wants.filter((want) => want.where === 'indoor' && (want.building ?? 0) === index);
    const myDecoys = Math.floor(decoys / count) + (index < decoys % count ? 1 : 0);
    return {
      id,
      label: labels[index]!,
      shell,
      footprint: shellFootprint(shell),
      heightM: shell.spec.floors.length * shell.spec.floorHeight,
      scene: plantIndoor(shell, id, mine, myDecoys, seed),
      interiorLightsOn: lightsFor(shell, options.interiorLights ?? 'on', seed),
    };
  });
  return {
    seed,
    outdoor,
    buildings,
    outdoorScene: plantOutdoor(outdoor, wants.filter((want) => want.where === 'outdoor'), seed),
    timeOfDay: options.timeOfDay ?? 12,
    flashlightOn: options.flashlightOn ?? false,
    sensorMode: options.sensorMode ?? 'rgb',
    home: [outdoor.spawn[0], outdoor.spawn[1], outdoor.spawn[2]],
    homeYaw: outdoor.spawnYaw,
  };
}

// ------------------------------------------------------------ no-fly

function ringPoints(polygon: readonly [number, number][]): Point[] {
  return polygon.map(([x, y]) => ({ x, y }));
}

function ringCentroid(points: readonly Point[]): Point {
  let x = 0;
  let y = 0;
  for (const point of points) {
    x += point.x;
    y += point.y;
  }
  return { x: x / points.length, y: y / points.length };
}

/** Sutherland-Hodgman. `clip` must be convex; the outdoor geofence is a near-circle. */
export function clipToConvex(subject: readonly Point[], clip: readonly Point[]): Point[] {
  const ccw = signedArea(clip) > 0;
  const inside = (p: Point, a: Point, b: Point): boolean => {
    const cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    return ccw ? cross >= 0 : cross <= 0;
  };
  const cut = (p: Point, q: Point, a: Point, b: Point): Point => {
    const a1 = b.y - a.y;
    const b1 = a.x - b.x;
    const c1 = a1 * a.x + b1 * a.y;
    const a2 = q.y - p.y;
    const b2 = p.x - q.x;
    const c2 = a2 * p.x + b2 * p.y;
    const det = a1 * b2 - a2 * b1;
    if (Math.abs(det) < 1e-12) return p;
    return { x: (b2 * c1 - b1 * c2) / det, y: (a1 * c2 - a2 * c1) / det };
  };
  let output = [...subject];
  for (let i = 0; i < clip.length && output.length > 0; i++) {
    const a = clip[i]!;
    const b = clip[(i + 1) % clip.length]!;
    const input = output;
    output = [];
    for (let j = 0; j < input.length; j++) {
      const p = input[j]!;
      const q = input[(j + 1) % input.length]!;
      const pIn = inside(p, a, b);
      const qIn = inside(q, a, b);
      if (pIn) {
        output.push(p);
        if (!qIn) output.push(cut(p, q, a, b));
      } else if (qIn) {
        output.push(cut(p, q, a, b));
      }
    }
  }
  const clean: Point[] = [];
  for (const point of output) {
    const rounded = { x: Math.round(point.x * 1000) / 1000, y: Math.round(point.y * 1000) / 1000 };
    if (clean.some((other) => Math.hypot(other.x - rounded.x, other.y - rounded.y) < 0.06)) continue;
    clean.push(rounded);
  }
  return clean;
}

function boxRing(points: readonly Point[]): Point[] {
  const xs = points.map((point) => point.x);
  const ys = points.map((point) => point.y);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  return [
    { x: minX, y: minY },
    { x: maxX, y: minY },
    { x: maxX, y: maxY },
    { x: minX, y: maxY },
  ];
}

const ZONE_REASONS: Record<string, string> = {
  ravine: 'Steep ravine. Low flight there is unsafe.',
  ceiling: 'Restricted airspace overhead. The 2D plan keeps the whole column out.',
};

/**
 * Site no-fly zones as mission zones. Every zone is kept: each one is clipped
 * to the geofence (pulled in 0.3 m so validation sees it strictly inside),
 * and zones that overlap merge into their bounding box, which only grows the
 * area the drone avoids. The `ceiling` zone only restricts flight above its
 * minZ, but the mission schema is 2D, so it becomes a full no-fly zone.
 */
export function missionNoFly(outdoor: OutdoorWorld): NoFlyZone[] {
  const fence = ringPoints(outdoor.geofence.polygon);
  const center = ringCentroid(fence);
  const inset = fence.map((point) => {
    const r = Math.hypot(point.x - center.x, point.y - center.y) || 1;
    const k = Math.max(0, (r - 0.3) / r);
    return { x: center.x + (point.x - center.x) * k, y: center.y + (point.y - center.y) * k };
  });
  let zones = outdoor.noFly.map((zone) => ({
    ids: [zone.id],
    names: [zone.name],
    points: clipToConvex(ringPoints(zone.polygon), inset),
  }));
  for (let pass = 0; pass < 8; pass++) {
    let merged = false;
    outer: for (let i = 0; i < zones.length; i++) {
      for (let j = i + 1; j < zones.length; j++) {
        const a = zones[i]!;
        const b = zones[j]!;
        if (a.points.length < 3 || b.points.length < 3 || !ringsOverlap(a.points, b.points)) continue;
        const union = clipToConvex(boxRing([...a.points, ...b.points]), inset);
        zones = zones.filter((_, k) => k !== i && k !== j);
        zones.push({ ids: [...a.ids, ...b.ids], names: [...a.names, ...b.names], points: union });
        merged = true;
        break outer;
      }
    }
    if (!merged) break;
  }
  // A zone that lies wholly outside the fence clips to nothing; the fence already keeps the drone out of it.
  return zones
    .filter((zone) => zone.points.length >= 3 && Math.abs(signedArea(zone.points)) >= 0.5)
    .map((zone) => ({
      id: `nf-${zone.ids.join('-')}`,
      name: zone.names.map((name) => name[0]!.toUpperCase() + name.slice(1)).join(' and '),
      polygon: { points: zone.points },
      reason: zone.names.map((name) => ZONE_REASONS[name] ?? 'No-fly zone from the site plan.').join(' '),
    }));
}

/** A valid mission over this world: its geofence, no-fly zones, a forest-safe altitude band, and home. */
export function missionForWorld(world: MissionWorld, options: MissionForWorldOptions): Mission {
  return {
    schemaVersion: 1,
    id: `msn-${world.seed}`,
    name: options.name ?? `Woods search, ${siteNameFor(world.seed)}`,
    seed: world.seed,
    notes: 'Civilian search and rescue practice. Everything is simulated.',
    geofence: { points: ringPoints(world.outdoor.geofence.polygon).map((p) => ({ x: Math.round(p.x * 1000) / 1000, y: Math.round(p.y * 1000) / 1000 })) },
    noFlyZones: missionNoFly(world.outdoor),
    altitude: { minM: MISSION_MIN_AGL_M, maxM: MISSION_MAX_AGL_M },
    missionType: options.missionType,
    targets: options.targets ?? suggestedTargets(options.missionType),
    requiredConfidence: options.requiredConfidence ?? 0.8,
    entryPolicy: options.entryPolicy ?? 'auto',
    budgets: options.budgets ?? { batteryPct: 80, timeSec: 900 },
    startPose: { x: world.home[0], y: world.home[1], z: MISSION_MIN_AGL_M, yawRad: world.homeYaw },
  };
}

// ----------------------------------------------------------- provider

function hull(points: Point[]): Point[] {
  const sorted = [...points].sort((a, b) => a.x - b.x || a.y - b.y);
  if (sorted.length < 3) return sorted;
  const cross = (o: Point, a: Point, b: Point): number => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  const lower: Point[] = [];
  for (const p of sorted) {
    while (lower.length >= 2 && cross(lower[lower.length - 2]!, lower[lower.length - 1]!, p) <= 0) lower.pop();
    lower.push(p);
  }
  const upper: Point[] = [];
  for (let i = sorted.length - 1; i >= 0; i--) {
    const p = sorted[i]!;
    while (upper.length >= 2 && cross(upper[upper.length - 2]!, upper[upper.length - 1]!, p) <= 0) upper.pop();
    upper.push(p);
  }
  return [...lower.slice(0, -1), ...upper.slice(0, -1)];
}

const CANOPY_BIN_M = 16;

/** Canopy as at most (size / 16)^2 hulls, one per 16 m bin of trees. */
export function canopyFootprints(outdoor: OutdoorWorld): WorldFootprint[] {
  const bins = new Map<string, { points: Point[]; top: number; n: number }>();
  for (const tree of outdoor.trees) {
    if (tree.kind === 'undergrowth') continue;
    const key = `${Math.floor(tree.x / CANOPY_BIN_M)}:${Math.floor(tree.y / CANOPY_BIN_M)}`;
    const bin = bins.get(key) ?? { points: [], top: 0, n: 0 };
    for (let k = 0; k < 8; k++) {
      const a = (k / 8) * Math.PI * 2;
      bin.points.push({ x: tree.x + Math.cos(a) * tree.canopyRadius, y: tree.y + Math.sin(a) * tree.canopyRadius });
    }
    bin.top = Math.max(bin.top, tree.canopyZ + tree.canopyRadius);
    bin.n += 1;
    bins.set(key, bin);
  }
  return [...bins.entries()]
    .filter(([, bin]) => bin.n >= 2)
    .sort(([a], [b]) => (a < b ? -1 : a > b ? 1 : 0))
    .map(([key, bin]) => ({
      id: `canopy-${key.replace(':', '-')}`,
      kind: 'canopy' as const,
      label: 'Tree canopy',
      polygon: hull(bin.points).map((p) => ({ x: Math.round(p.x * 100) / 100, y: Math.round(p.y * 100) / 100 })),
      heightM: Math.round(bin.top * 10) / 10,
    }));
}

/** Footprints of the generated world: labeled buildings, cleared trails, and canopy clusters. */
export function worldFootprints(outdoor: OutdoorWorld, seed: number): WorldFootprint[] {
  const labels = buildingLabels(outdoor, seed);
  const buildings: WorldFootprint[] = outdoor.buildings.map((shell, index) => ({
    id: `bldg-${shell.id}`,
    kind: 'building',
    label: labels[index]!,
    polygon: shellFootprint(shell),
    heightM: shell.spec.floors.length * shell.spec.floorHeight,
  }));
  const trails: WorldFootprint[] = outdoor.corridors.map((corridor, index) => {
    const dx = corridor.bx - corridor.ax;
    const dy = corridor.by - corridor.ay;
    const len = Math.hypot(dx, dy) || 1;
    const nx = (-dy / len) * corridor.halfWidth * 0.5;
    const ny = (dx / len) * corridor.halfWidth * 0.5;
    const round = (v: number): number => Math.round(v * 100) / 100;
    return {
      id: `trail-${index}`,
      kind: 'road',
      label: `Trail to the ${labels[index] ?? 'building'}`,
      polygon: [
        { x: round(corridor.ax + nx), y: round(corridor.ay + ny) },
        { x: round(corridor.ax - nx), y: round(corridor.ay - ny) },
        { x: round(corridor.bx - nx), y: round(corridor.by - ny) },
        { x: round(corridor.bx + nx), y: round(corridor.by + ny) },
      ],
      heightM: null,
    };
  });
  return [...canopyFootprints(outdoor), ...trails, ...buildings];
}

/** Map context from the real outdoor generator. Replaces the mock site on the mission page. */
export const outdoorWorldProvider: WorldProvider = {
  id: 'outdoor-world',
  load(seed: number): WorldSnapshot {
    const outdoor = generateOutdoor({ seed: seed >>> 0 });
    return {
      provider: 'outdoor-world',
      seed: seed >>> 0,
      siteName: siteNameFor(seed),
      footprints: worldFootprints(outdoor, seed >>> 0),
    };
  },
};
