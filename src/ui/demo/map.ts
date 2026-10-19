import type { EntryCandidate, MissionWorld, RunnerSnapshot, TruthTarget, V3 } from '../../mission/autonomy/contracts';
import { sampleHeight } from '../../world/outdoor/terrain';
import { CLASS_COLORS, classLabel, openingSafetyHint, safetyColor } from './format';

export interface MapOverlay {
  snapshot: RunnerSnapshot | null;
  /** Shown after the run: where the targets really were. */
  truth: TruthTarget[] | null;
  /** Entries the runner already decided, by building and opening id. */
  decided: Map<string, number>;
  /** Milliseconds, drives the pulse on a pending entry. */
  now: number;
}

interface Projection {
  scale: number;
  x: (wx: number) => number;
  y: (wy: number) => number;
}

const MAP_FONT = '600 11px ui-sans-serif, system-ui, "Segoe UI", sans-serif';

export function isNight(world: MissionWorld): boolean {
  return world.timeOfDay < 6.5 || world.timeOfDay > 19.5;
}

/** Top-down mission map: static site layer, then plan, track, contacts, drone. */
export class DemoMap {
  private layer: HTMLCanvasElement | null = null;
  private layerKey = '';
  private proj: Projection = { scale: 1, x: (v) => v, y: (v) => v };

  constructor(
    private readonly canvas: HTMLCanvasElement,
    private world: MissionWorld,
  ) {}

  setWorld(world: MissionWorld): void {
    this.world = world;
    this.layerKey = '';
  }

  draw(overlay: MapOverlay): void {
    const cssW = Math.max(1, this.canvas.clientWidth);
    const cssH = Math.max(1, this.canvas.clientHeight);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.floor(cssW * dpr);
    const h = Math.floor(cssH * dpr);
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
    const ctx = this.canvas.getContext('2d');
    if (!ctx) return;
    const key = `${this.world.seed}:${this.world.buildings.length}:${w}x${h}`;
    if (key !== this.layerKey || !this.layer) {
      this.proj = project(this.world, cssW, cssH);
      this.layer = paintSite(this.world, this.proj, w, h, dpr);
      this.layerKey = key;
      this.canvas.dataset.scale = this.proj.scale.toFixed(3);
    }
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, w, h);
    ctx.drawImage(this.layer, 0, 0);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.paintEntries(ctx, overlay);
    const snap = overlay.snapshot;
    if (snap) {
      this.paintPlan(ctx, snap);
      this.paintTrack(ctx, snap.track);
      this.paintContacts(ctx, snap);
    }
    if (overlay.truth) this.paintTruth(ctx, overlay.truth);
    if (snap) this.paintDrone(ctx, snap);
  }

  private paintEntries(ctx: CanvasRenderingContext2D, overlay: MapOverlay): void {
    const pending = overlay.snapshot?.pendingEntry ?? null;
    for (const building of this.world.buildings) {
      for (const opening of building.shell.openings) {
        const decided = overlay.decided.get(`${building.id}:${opening.id}`);
        const safety = decided ?? openingSafetyHint(opening.clearWidth, opening.clearHeight, opening.passable);
        const cx = this.proj.x(opening.center[0]);
        const cy = this.proj.y(opening.center[1]);
        const tx = opening.tangent[0];
        const ty = -opening.tangent[1];
        const half = Math.max(3, (opening.width * this.proj.scale) / 2);
        ctx.lineCap = 'round';
        if (!opening.passable) {
          ctx.strokeStyle = 'rgba(120, 126, 132, 0.9)';
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.moveTo(cx - tx * half, cy - ty * half);
          ctx.lineTo(cx + tx * half, cy + ty * half);
          ctx.stroke();
          continue;
        }
        const color = safetyColor(safety);
        ctx.strokeStyle = color;
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.moveTo(cx - tx * half, cy - ty * half);
        ctx.lineTo(cx + tx * half, cy + ty * half);
        ctx.stroke();
        const mx = cx + opening.normal[0] * 9;
        const my = cy - opening.normal[1] * 9;
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(mx, my, 3.2, 0, Math.PI * 2);
        ctx.fill();
        if (pending && isPending(pending, building.id, opening.id)) {
          const pulse = 8 + 4 * Math.sin(overlay.now / 160);
          ctx.strokeStyle = '#ffd54a';
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.arc(mx, my, pulse, 0, Math.PI * 2);
          ctx.stroke();
        }
      }
    }
  }

  private paintPlan(ctx: CanvasRenderingContext2D, snap: RunnerSnapshot): void {
    if (snap.plan.length === 0) return;
    ctx.save();
    ctx.setLineDash([5, 5]);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.75)';
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    ctx.moveTo(this.proj.x(snap.pose.pos[0]), this.proj.y(snap.pose.pos[1]));
    for (const point of snap.plan) ctx.lineTo(this.proj.x(point[0]), this.proj.y(point[1]));
    ctx.stroke();
    ctx.restore();
  }

  private paintTrack(ctx: CanvasRenderingContext2D, track: V3[]): void {
    if (track.length < 2) return;
    ctx.save();
    ctx.lineJoin = 'round';
    ctx.strokeStyle = 'rgba(16, 24, 32, 0.55)';
    ctx.lineWidth = 5;
    this.tracePath(ctx, track);
    ctx.stroke();
    ctx.strokeStyle = '#5ab8ff';
    ctx.lineWidth = 2.4;
    this.tracePath(ctx, track);
    ctx.stroke();
    ctx.restore();
  }

  private tracePath(ctx: CanvasRenderingContext2D, track: V3[]): void {
    ctx.beginPath();
    track.forEach((point, index) => {
      const x = this.proj.x(point[0]);
      const y = this.proj.y(point[1]);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
  }

  private paintContacts(ctx: CanvasRenderingContext2D, snap: RunnerSnapshot): void {
    for (const contact of snap.contacts) {
      const age = Math.max(0, snap.tSec - contact.tSec);
      const alpha = Math.max(0.15, 1 - age / 30);
      ctx.globalAlpha = alpha;
      ctx.fillStyle = CLASS_COLORS[contact.classId] ?? '#ffffff';
      ctx.beginPath();
      ctx.arc(this.proj.x(contact.estimate[0]), this.proj.y(contact.estimate[1]), 2.6, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = 1;
    ctx.font = MAP_FONT;
    for (const found of snap.confirmed) {
      const x = this.proj.x(found.estimate[0]);
      const y = this.proj.y(found.estimate[1]);
      const color = CLASS_COLORS[found.classId] ?? '#ffffff';
      ctx.strokeStyle = '#0b1210';
      ctx.lineWidth = 5;
      ctx.beginPath();
      ctx.arc(x, y, 9, 0, Math.PI * 2);
      ctx.stroke();
      ctx.strokeStyle = color;
      ctx.lineWidth = 2.5;
      ctx.stroke();
      label(ctx, `${classLabel(found.classId)} ${(found.confidence * 100).toFixed(0)}%`, x + 12, y - 10, color);
    }
  }

  private paintTruth(ctx: CanvasRenderingContext2D, truth: TruthTarget[]): void {
    ctx.font = MAP_FONT;
    for (const target of truth) {
      const x = this.proj.x(target.world[0]);
      const y = this.proj.y(target.world[1]);
      ctx.strokeStyle = target.category === 'decoy' ? 'rgba(220, 220, 220, 0.85)' : '#ffffff';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x, y - 6);
      ctx.lineTo(x + 6, y);
      ctx.lineTo(x, y + 6);
      ctx.lineTo(x - 6, y);
      ctx.closePath();
      ctx.stroke();
      label(ctx, `${target.category === 'decoy' ? 'decoy ' : ''}${classLabel(target.classId)}`, x + 8, y + 14, 'rgba(255,255,255,0.9)');
    }
  }

  private paintDrone(ctx: CanvasRenderingContext2D, snap: RunnerSnapshot): void {
    const x = this.proj.x(snap.pose.pos[0]);
    const y = this.proj.y(snap.pose.pos[1]);
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-snap.pose.yaw);
    if (this.world.flashlightOn && isNight(this.world)) {
      const reach = 8 * this.proj.scale;
      const cone = ctx.createRadialGradient(0, 0, 2, 0, 0, reach);
      cone.addColorStop(0, 'rgba(255, 240, 190, 0.55)');
      cone.addColorStop(1, 'rgba(255, 240, 190, 0)');
      ctx.fillStyle = cone;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.arc(0, 0, reach, -0.35, 0.35);
      ctx.closePath();
      ctx.fill();
    }
    ctx.fillStyle = '#0b1210';
    ctx.beginPath();
    ctx.arc(0, 0, 10, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = snap.inside ? '#ffb15a' : '#f4fff8';
    ctx.beginPath();
    ctx.moveTo(9, 0);
    ctx.lineTo(-6, 5.5);
    ctx.lineTo(-3, 0);
    ctx.lineTo(-6, -5.5);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}

function isPending(entry: EntryCandidate, buildingId: string, openingId: number): boolean {
  return entry.buildingId === buildingId && entry.openingId === openingId;
}

function label(ctx: CanvasRenderingContext2D, text: string, x: number, y: number, color: string): void {
  const width = ctx.measureText(text).width;
  ctx.fillStyle = 'rgba(8, 14, 12, 0.78)';
  ctx.fillRect(x - 3, y - 11, width + 6, 15);
  ctx.fillStyle = color;
  ctx.fillText(text, x, y);
}

function project(world: MissionWorld, cssW: number, cssH: number): Projection {
  const fence = world.outdoor.geofence.polygon;
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const [x, y] of fence) {
    minX = Math.min(minX, x);
    minY = Math.min(minY, y);
    maxX = Math.max(maxX, x);
    maxY = Math.max(maxY, y);
  }
  const pad = 3;
  minX = Math.max(0, minX - pad);
  minY = Math.max(0, minY - pad);
  maxX = Math.min(world.outdoor.size, maxX + pad);
  maxY = Math.min(world.outdoor.size, maxY + pad);
  const scale = Math.min(cssW / (maxX - minX), cssH / (maxY - minY));
  const ox = (cssW - (maxX - minX) * scale) / 2;
  const oy = (cssH - (maxY - minY) * scale) / 2;
  return {
    scale,
    x: (wx) => ox + (wx - minX) * scale,
    y: (wy) => cssH - oy - (wy - minY) * scale,
  };
}

function paintSite(world: MissionWorld, proj: Projection, w: number, h: number, dpr: number): HTMLCanvasElement {
  const layer = document.createElement('canvas');
  layer.width = w;
  layer.height = h;
  const ctx = layer.getContext('2d');
  if (!ctx) return layer;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const night = isNight(world);
  const { outdoor } = world;
  ctx.fillStyle = night ? '#0a1210' : '#1d3324';
  ctx.fillRect(0, 0, w / dpr, h / dpr);

  // Terrain: height tint plus a hillshade from the north-west.
  const n = outdoor.terrain.resolution;
  const cell = outdoor.size / (n - 1);
  const px = Math.ceil(cell * proj.scale) + 1;
  for (let j = 0; j < n - 1; j++) {
    for (let i = 0; i < n - 1; i++) {
      const x = (i + 0.5) * cell;
      const y = (j + 0.5) * cell;
      const hgt = sampleHeight(outdoor.terrain, x, y);
      const gx = sampleHeight(outdoor.terrain, x + cell, y) - sampleHeight(outdoor.terrain, x - cell, y);
      const gy = sampleHeight(outdoor.terrain, x, y + cell) - sampleHeight(outdoor.terrain, x, y - cell);
      const shade = Math.max(-0.35, Math.min(0.35, (-gx + gy) * 0.22));
      const t = Math.max(0, Math.min(1, (hgt - 1) / 8));
      let r = 52 + t * 58;
      let g = 86 + t * 46;
      let b = 50 + t * 22;
      r *= 1 + shade;
      g *= 1 + shade;
      b *= 1 + shade;
      if (night) {
        r *= 0.42;
        g *= 0.48;
        b *= 0.6;
      }
      ctx.fillStyle = `rgb(${r | 0},${g | 0},${b | 0})`;
      ctx.fillRect(proj.x(i * cell), proj.y((j + 1) * cell), px, px);
    }
  }

  // Trees: canopy discs, darker rim so dense stands read as forest.
  for (const tree of outdoor.trees) {
    const r = Math.max(1, tree.canopyRadius * proj.scale * (tree.kind === 'undergrowth' ? 0.5 : 0.8));
    const x = proj.x(tree.x);
    const y = proj.y(tree.y);
    const base = tree.kind === 'pine' ? [22, 60, 36] : tree.kind === 'broadleaf' ? [44, 82, 40] : [58, 92, 48];
    const k = night ? 0.45 : 0.85 + tree.tint * 0.3;
    ctx.fillStyle = `rgba(${(base[0]! * k) | 0},${(base[1]! * k) | 0},${(base[2]! * k) | 0},0.92)`;
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fill();
    if (tree.kind !== 'undergrowth') {
      ctx.strokeStyle = 'rgba(6, 18, 10, 0.5)';
      ctx.lineWidth = 0.8;
      ctx.stroke();
    }
  }

  // No-fly zones, hatched.
  for (const zone of outdoor.noFly) {
    ctx.save();
    tracePolygon(ctx, proj, zone.polygon);
    ctx.fillStyle = 'rgba(232, 72, 72, 0.22)';
    ctx.fill();
    ctx.clip();
    ctx.strokeStyle = 'rgba(255, 110, 110, 0.55)';
    ctx.lineWidth = 1;
    for (let k = -w; k < w; k += 9) {
      ctx.beginPath();
      ctx.moveTo(k, 0);
      ctx.lineTo(k + h, h);
      ctx.stroke();
    }
    ctx.restore();
    tracePolygon(ctx, proj, zone.polygon);
    ctx.strokeStyle = 'rgba(255, 120, 120, 0.9)';
    ctx.lineWidth = 1.4;
    ctx.stroke();
    const c = centroid(zone.polygon);
    ctx.font = MAP_FONT;
    label(ctx, 'NO-FLY', proj.x(c[0]) - 20, proj.y(c[1]) + 4, '#ff9a9a');
  }

  // Geofence.
  ctx.save();
  ctx.setLineDash([8, 5]);
  tracePolygon(ctx, proj, outdoor.geofence.polygon);
  ctx.strokeStyle = '#6ff5e6';
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.restore();

  // Buildings: footprint, ground-floor rooms, label.
  ctx.font = MAP_FONT;
  for (const building of world.buildings) {
    const { shell } = building;
    const floor = shell.spec.floors[0]!;
    const cs = shell.spec.cellSize;
    const x0 = proj.x(shell.origin[0]);
    const y0 = proj.y(shell.origin[1] + floor.d * cs);
    const bw = floor.w * cs * proj.scale;
    const bh = floor.d * cs * proj.scale;
    ctx.fillStyle = 'rgba(0, 0, 0, 0.35)';
    ctx.fillRect(x0 + 3, y0 + 3, bw, bh);
    ctx.fillStyle = night ? '#5b5650' : '#d9d0c3';
    ctx.fillRect(x0, y0, bw, bh);
    ctx.strokeStyle = night ? 'rgba(20,16,12,0.7)' : 'rgba(80, 66, 52, 0.55)';
    ctx.lineWidth = 1;
    for (const room of floor.rooms) {
      ctx.strokeRect(proj.x(shell.origin[0] + room.x * cs), proj.y(shell.origin[1] + (room.z + room.d) * cs), room.w * cs * proj.scale, room.d * cs * proj.scale);
    }
    ctx.strokeStyle = '#1b140f';
    ctx.lineWidth = 2;
    ctx.strokeRect(x0, y0, bw, bh);
    label(ctx, `${building.label}${shell.spec.floors.length > 1 ? ` · ${shell.spec.floors.length} floors` : ''}`, x0 + 4, y0 - 6, '#f6efe4');
  }

  // Home pad.
  const hx = proj.x(world.home[0]);
  const hy = proj.y(world.home[1]);
  ctx.fillStyle = 'rgba(10, 16, 14, 0.8)';
  ctx.beginPath();
  ctx.arc(hx, hy, 11, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#ffd54a';
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.fillStyle = '#ffd54a';
  ctx.font = '700 12px ui-sans-serif, system-ui, sans-serif';
  ctx.fillText('H', hx - 4.2, hy + 4.3);
  return layer;
}

function tracePolygon(ctx: CanvasRenderingContext2D, proj: Projection, polygon: [number, number][]): void {
  ctx.beginPath();
  polygon.forEach(([x, y], index) => {
    if (index === 0) ctx.moveTo(proj.x(x), proj.y(y));
    else ctx.lineTo(proj.x(x), proj.y(y));
  });
  ctx.closePath();
}

function centroid(polygon: [number, number][]): [number, number] {
  let x = 0;
  let y = 0;
  for (const point of polygon) {
    x += point[0];
    y += point[1];
  }
  return [x / polygon.length, y / polygon.length];
}
