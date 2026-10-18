import { CELL_SIZE, FLOOR_HEIGHT } from '../../shared/constants';
import { buildingAabbs } from '../../gen/parts';
import { doorPose } from './clearance';
import { terrainMesh } from './terrain';
import type { BuildingShell, Opening, OutdoorWorld, Solid } from './types';

const IDENTITY = { qx: 0, qy: 0, qz: 0, qw: 1 };

export function planColliders(world: OutdoorWorld): Solid[] {
  const solids: Solid[] = [];
  const mesh = terrainMesh(world.terrain);
  solids.push({ shape: 'trimesh', tag: 'terrain', vertices: mesh.vertices, indices: mesh.indices });
  for (const shell of world.buildings) addBuilding(solids, shell);
  for (const tree of world.trees) {
    if (tree.trunkHeight > 0.2 && tree.trunkRadius > 0) {
      solids.push({
        shape: 'cylinder',
        tag: 'trunk',
        cx: tree.x,
        cy: tree.y,
        cz: tree.z + tree.trunkHeight / 2,
        radius: tree.trunkRadius,
        halfHeight: tree.trunkHeight / 2,
      });
    }
    solids.push({
      shape: 'ball',
      tag: tree.kind === 'undergrowth' ? 'undergrowth' : 'canopy',
      cx: tree.x,
      cy: tree.y,
      cz: tree.z + tree.canopyZ,
      radius: tree.canopyRadius,
    });
  }
  return solids;
}

function addBuilding(solids: Solid[], shell: BuildingShell): void {
  const spec = shell.spec;
  const top = spec.floors.length * spec.floorHeight;
  for (const box of buildingAabbs(spec)) {
    if (box.kind === 'slab' && box.cz > top - 0.2) continue;
    solids.push({
      shape: 'cuboid',
      tag: box.kind === 'slab' ? 'floor' : 'wall',
      cx: box.cx + shell.origin[0],
      cy: box.cy + shell.origin[1],
      cz: box.cz + shell.origin[2],
      hx: box.hx,
      hy: box.hy,
      hz: box.hz,
      ...IDENTITY,
    });
  }
  for (const opening of shell.openings) {
    if (opening.kind === 'roof_hatch') continue;
    addJambs(solids, shell, opening);
    const pose = doorPose(opening);
    if (!pose) continue;
    solids.push({
      shape: 'cuboid',
      tag: 'door',
      cx: pose.center[0],
      cy: pose.center[1],
      cz: pose.center[2],
      hx: pose.hx,
      hy: pose.hy,
      hz: pose.hz,
      qx: pose.quat.x,
      qy: pose.quat.y,
      qz: pose.quat.z,
      qw: pose.quat.w,
    });
  }
  addRoof(solids, shell);
}

function addJambs(solids: Solid[], shell: BuildingShell, opening: Opening): void {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const cell of opening.apertureCells) {
    minX = Math.min(minX, shell.origin[0] + cell.x * CELL_SIZE);
    maxX = Math.max(maxX, shell.origin[0] + (cell.x + 1) * CELL_SIZE);
    minY = Math.min(minY, shell.origin[1] + cell.z * CELL_SIZE);
    maxY = Math.max(maxY, shell.origin[1] + (cell.z + 1) * CELL_SIZE);
  }
  const z0 = shell.origin[2] + opening.floor * FLOOR_HEIGHT;
  const z1 = z0 + FLOOR_HEIGHT;
  const holeZ0 = opening.center[2] - opening.height / 2;
  const holeZ1 = opening.center[2] + opening.height / 2;
  const throughY = Math.abs(opening.normal[1]) > Math.abs(opening.normal[0]);
  push(solids, 'jamb', minX, minY, z0, maxX, maxY, holeZ0);
  push(solids, 'jamb', minX, minY, holeZ1, maxX, maxY, z1);
  if (throughY) {
    const holeX0 = opening.center[0] - opening.width / 2;
    const holeX1 = opening.center[0] + opening.width / 2;
    push(solids, 'jamb', minX, minY, holeZ0, holeX0, maxY, holeZ1);
    push(solids, 'jamb', holeX1, minY, holeZ0, maxX, maxY, holeZ1);
  } else {
    const holeY0 = opening.center[1] - opening.width / 2;
    const holeY1 = opening.center[1] + opening.width / 2;
    push(solids, 'jamb', minX, minY, holeZ0, maxX, holeY0, holeZ1);
    push(solids, 'jamb', minX, holeY1, holeZ0, maxX, maxY, holeZ1);
  }
}

function addRoof(solids: Solid[], shell: BuildingShell): void {
  const floor = shell.spec.floors[0]!;
  const x0 = shell.origin[0];
  const y0 = shell.origin[1];
  const x1 = x0 + floor.w * shell.spec.cellSize;
  const y1 = y0 + floor.d * shell.spec.cellSize;
  const z0 = shell.origin[2] + shell.spec.floors.length * shell.spec.floorHeight;
  const z1 = z0 + 0.08;
  const hatch = shell.openings.find((opening) => opening.kind === 'roof_hatch');
  if (!hatch) {
    push(solids, 'roof', x0, y0, z0, x1, y1, z1);
    return;
  }
  const hx0 = hatch.center[0] - hatch.width / 2;
  const hx1 = hatch.center[0] + hatch.width / 2;
  const hy0 = hatch.center[1] - hatch.height / 2;
  const hy1 = hatch.center[1] + hatch.height / 2;
  push(solids, 'roof', x0, y0, z0, hx0, y1, z1);
  push(solids, 'roof', hx1, y0, z0, x1, y1, z1);
  push(solids, 'roof', hx0, y0, z0, hx1, hy0, z1);
  push(solids, 'roof', hx0, hy1, z0, hx1, y1, z1);
}

function push(
  solids: Solid[],
  tag: string,
  minX: number,
  minY: number,
  minZ: number,
  maxX: number,
  maxY: number,
  maxZ: number,
): void {
  const hx = (maxX - minX) / 2;
  const hy = (maxY - minY) / 2;
  const hz = (maxZ - minZ) / 2;
  if (hx < 0.02 || hy < 0.02 || hz < 0.02) return;
  solids.push({
    shape: 'cuboid',
    tag,
    cx: (minX + maxX) / 2,
    cy: (minY + maxY) / 2,
    cz: (minZ + maxZ) / 2,
    hx,
    hy,
    hz,
    ...IDENTITY,
  });
}
