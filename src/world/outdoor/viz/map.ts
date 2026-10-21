import type { OutdoorWorld } from '../types';

export interface MapDrone {
  x: number;
  y: number;
  yaw: number;
}

const mapCache = new WeakMap<OutdoorWorld, { width: number; height: number; layer: HTMLCanvasElement }>();

/** Top-down overlay: terrain, geofence, no-fly zones, trees, shells, openings, drone. */
export function drawOutdoorMap(
  ctx: CanvasRenderingContext2D,
  world: OutdoorWorld,
  drone: MapDrone,
  cssWidth: number,
  cssHeight: number,
): void {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.floor(cssWidth * dpr));
  const height = Math.max(1, Math.floor(cssHeight * dpr));
  if (ctx.canvas.width !== width || ctx.canvas.height !== height) {
    ctx.canvas.width = width;
    ctx.canvas.height = height;
  }
  const cached = mapCache.get(world);
  const layer = cached && cached.width === width && cached.height === height ? cached.layer : paintMapLayer(world, width, height, cssWidth, cssHeight, dpr);
  if (!cached || cached.layer !== layer) mapCache.set(world, { width, height, layer });
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, width, height);
  ctx.drawImage(layer, 0, 0);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  const { toX, toY } = mapProject(world, cssWidth, cssHeight);
  ctx.save();
  ctx.translate(toX(drone.x), toY(drone.y));
  ctx.rotate(-drone.yaw);
  ctx.fillStyle = '#f4fff8';
  ctx.beginPath();
  ctx.moveTo(7, 0);
  ctx.lineTo(-5, 4);
  ctx.lineTo(-5, -4);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function paintMapLayer(
  world: OutdoorWorld,
  width: number,
  height: number,
  cssWidth: number,
  cssHeight: number,
  dpr: number,
): HTMLCanvasElement {
  const layer = document.createElement('canvas');
  layer.width = width;
  layer.height = height;
  const ctx = layer.getContext('2d');
  if (!ctx) return layer;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#102018';
  ctx.fillRect(0, 0, cssWidth, cssHeight);
  const { toX, toY, scale } = mapProject(world, cssWidth, cssHeight);
  const n = world.terrain.resolution;
  const step = 2;
  const cell = world.size / (n - 1);
  for (let j = 0; j < n; j += step) {
    for (let i = 0; i < n; i += step) {
      const h = world.terrain.heights[j * n + i]!;
      const t = Math.max(0, Math.min(1, (h - 1.2) / 7));
      const g = Math.round(70 + t * 40);
      const r = Math.round(28 + t * 50);
      ctx.fillStyle = `rgb(${r},${g},48)`;
      const x = toX(i * cell);
      const y = toY(j * cell);
      ctx.fillRect(x, y - cell * step * scale, Math.ceil(cell * step * scale) + 1, Math.ceil(cell * step * scale) + 1);
    }
  }

  for (const zone of world.noFly) {
    ctx.beginPath();
    zone.polygon.forEach((vertex, index) => {
      const x = toX(vertex[0]);
      const y = toY(vertex[1]);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.fillStyle = zone.minZ > 8 ? 'rgba(120, 170, 255, 0.28)' : 'rgba(255, 80, 90, 0.38)';
    ctx.fill();
  }

  ctx.beginPath();
  world.geofence.polygon.forEach((vertex, index) => {
    const x = toX(vertex[0]);
    const y = toY(vertex[1]);
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.closePath();
  ctx.strokeStyle = '#7dfff0';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  ctx.fillStyle = 'rgba(18, 48, 28, 0.9)';
  for (const tree of world.trees) {
    if (tree.kind === 'undergrowth') continue;
    ctx.beginPath();
    ctx.arc(toX(tree.x), toY(tree.y), Math.max(1.2, tree.canopyRadius * scale * 0.55), 0, Math.PI * 2);
    ctx.fill();
  }

  for (const shell of world.buildings) {
    const floor = shell.spec.floors[0]!;
    const x = toX(shell.origin[0]);
    const y = toY(shell.origin[1] + floor.d * shell.spec.cellSize);
    const w = floor.w * shell.spec.cellSize * scale;
    const h = floor.d * shell.spec.cellSize * scale;
    ctx.fillStyle = 'rgba(214, 206, 196, 0.92)';
    ctx.fillRect(x, y, w, h);
    ctx.strokeStyle = '#1b140f';
    ctx.strokeRect(x, y, w, h);
    for (const opening of shell.openings) {
      ctx.strokeStyle = opening.passable ? '#3dff9a' : '#ffb15a';
      ctx.lineWidth = opening.passable ? 2 : 1;
      ctx.beginPath();
      const ox = toX(opening.center[0]);
      const oy = toY(opening.center[1]);
      ctx.moveTo(ox, oy);
      ctx.lineTo(ox + opening.normal[0] * 7, oy - opening.normal[1] * 7);
      ctx.stroke();
    }
  }
  return layer;
}

export function mapProject(world: OutdoorWorld, cssWidth: number, cssHeight: number): {
  scale: number;
  toX: (x: number) => number;
  toY: (y: number) => number;
} {
  const pad = 10;
  const scale = (cssWidth - pad * 2) / world.size;
  return {
    scale,
    toX: (x) => pad + x * scale,
    toY: (y) => cssHeight - pad - y * scale,
  };
}
