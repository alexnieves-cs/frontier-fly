/**
 * A real apartment floor plan in vector form, as stored in public/plans/*.json.
 *
 * Coordinates are meters in a plan frame that the importer has rotated so the
 * walls run along the axes, with the origin at the plan's lower-left corner.
 * Polygons are flat rings [x0, y0, x1, y1, ...] without the closing vertex.
 * +x maps to sim x and +y to sim y (the renderer's z).
 */
export interface PlanRecord {
  id: string;
  /** dev plans may inform converter choices; eval plans are only measured. */
  split: 'dev' | 'eval';
  source: PlanSource;
  /** Rotation (radians, counter-clockwise) the importer applied to the source coordinates. */
  rotation: number;
  /** Indoor rooms only. Balconies, loggias, stairs and shafts are left out. */
  areas: PlanArea[];
  walls: number[][];
  doors: number[][];
  windows: number[][];
}

export interface PlanArea {
  /** Source room type, e.g. LIVING_ROOM, BEDROOM, KITCHEN, BATHROOM, CORRIDOR. */
  type: string;
  poly: number[];
}

export interface PlanSource {
  dataset: string;
  site: number;
  building: number;
  floor: number;
  apartment: string;
}

export interface PlanSet {
  /** Dataset name, version, license and attribution, copied into every derived file. */
  dataset: { name: string; version: string; license: string; url: string; doi: string; attribution: string; changes: string };
  plans: PlanRecord[];
}

/**
 * A real home of two or more storeys (a maisonette), as stored in public/plans/swiss-homes.json.
 * Every storey shares one plan frame (same rotation and origin), storey 0 lowest.
 */
export interface HomeRecord {
  id: string;
  split: 'dev' | 'eval';
  source: Omit<PlanSource, 'floor'> & { floors: number[]; elevations: number[] };
  rotation: number;
  storeys: HomeStorey[];
}

export interface HomeStorey {
  areas: PlanArea[];
  walls: number[][];
  doors: number[][];
  windows: number[][];
  /** Staircase polygons (Swiss Dwellings STAIRS features and STAIRCASE areas). */
  stairs: number[][];
}

export interface HomeSet {
  dataset: PlanSet['dataset'];
  homes: HomeRecord[];
}
