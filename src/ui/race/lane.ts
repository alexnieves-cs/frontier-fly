import { CELL, VOXEL } from '../../shared/constants';
import type { BuildingSpec } from '../../shared/types';
import type { LaneFrame } from './protocol';

/**
 * Top-down view of one lane: the floor plan at voxel resolution, what the
 * drone's lidar has seen, where it has flown (the coverage metric), its trail,
 * and the drone itself. Drawn on a 2D canvas, so it costs almost nothing.
 */

type RGB = [number, number, number];

// Walls read as the plan's dark lines, unexplored floor as a faint fill, and lidar-seen floor lighter.
const FOG: RGB = [20, 30, 38];
const WALL: RGB = [5, 8, 11];
const DOOR: RGB = [26, 38, 48];
const SHAFT: RGB = [120, 82, 40];
const SEEN: RGB = [48, 66, 80];
const FURNITURE = 'rgba(150, 170, 186, 0.55)';

export function hexToRgb(hex: string): RGB {
  const n = parseInt(hex.slice(1), 16);
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
}

function mix(a: RGB, b: RGB, t: number): RGB {
  return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
}

interface TrailPoint {
  x: number;
  y: number;
  floor: number;
}

export class LaneView {
  private readonly ctx: CanvasRenderingContext2D;
  private readonly voxels: HTMLCanvasElement;
  private readonly vctx: CanvasRenderingContext2D;
  private image: ImageData | null = null;
  private spec: BuildingSpec | null = null;
  private vw = 0;
  private vd = 0;
  private trail: TrailPoint[] = [];
  private flashes: Array<{ x: number; y: number; floor: number; t: number }> = [];
  private lastCollisions = 0;
  private readonly visitedRgb: RGB;
  private readonly color: string;

  constructor(
    readonly canvas: HTMLCanvasElement,
    color: string,
  ) {
    this.ctx = canvas.getContext('2d')!;
    this.voxels = document.createElement('canvas');
    this.vctx = this.voxels.getContext('2d')!;
    this.color = color;
    this.visitedRgb = mix(SEEN, hexToRgb(color), 0.62);
  }

  reset(spec: BuildingSpec): void {
    this.spec = spec;
    this.vw = spec.floors[0]!.w * 2;
    this.vd = spec.floors[0]!.d * 2;
    this.voxels.width = this.vw;
    this.voxels.height = this.vd;
    this.image = this.vctx.createImageData(this.vw, this.vd);
    this.trail = [];
    this.flashes = [];
    this.lastCollisions = 0;
  }

  /** Record the lane's pose for the trail and spot new collisions. Call once per worker frame. */
  update(frame: LaneFrame, now: number): void {
    const last = this.trail[this.trail.length - 1];
    const p = { x: frame.pos[0], y: frame.pos[1], floor: frame.floor };
    if (!last || Math.hypot(last.x - p.x, last.y - p.y) > 0.08 || last.floor !== p.floor) this.trail.push(p);
    if (frame.collisions > this.lastCollisions) this.flashes.push({ ...p, t: now });
    this.lastCollisions = frame.collisions;
  }

  draw(frame: LaneFrame | null, visited: Uint8Array | null, seen: Uint8Array | null, now: number): void {
    const spec = this.spec;
    const image = this.image;
    if (!spec || !image) return;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    const cw = Math.max(1, Math.round(this.canvas.clientWidth * dpr));
    const ch = Math.max(1, Math.round(this.canvas.clientHeight * dpr));
    if (this.canvas.width !== cw || this.canvas.height !== ch) {
      this.canvas.width = cw;
      this.canvas.height = ch;
    }
    const floorIndex = Math.max(0, Math.min(spec.floors.length - 1, frame?.floor ?? 0));
    const floor = spec.floors[floorIndex]!;
    const { vw, vd } = this;
    const base = floorIndex * vw * vd;
    const px = image.data;
    for (let vy = 0; vy < vd; vy++) {
      const gz = vy >> 1;
      for (let vx = 0; vx < vw; vx++) {
        const cell = floor.cells[gz * floor.w + (vx >> 1)]!;
        const i = base + vy * vw + vx;
        let c: RGB;
        if (cell === CELL.WALL) c = WALL;
        else if (visited?.[i]) c = this.visitedRgb;
        else if (seen?.[i]) c = SEEN;
        else if (cell === CELL.SHAFT) c = SHAFT;
        else if (cell === CELL.DOOR) c = DOOR;
        else c = FOG;
        const o = (vy * vw + vx) * 4;
        px[o] = c[0];
        px[o + 1] = c[1];
        px[o + 2] = c[2];
        px[o + 3] = 255;
      }
    }
    this.vctx.putImageData(image, 0, 0);

    const ctx = this.ctx;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = '#05080b';
    ctx.fillRect(0, 0, cw, ch);
    const worldW = vw * VOXEL;
    const worldD = vd * VOXEL;
    const pad = 6 * dpr;
    const scale = Math.min((cw - pad * 2) / worldW, (ch - pad * 2) / worldD);
    const ox = (cw - worldW * scale) / 2;
    const oy = (ch - worldD * scale) / 2;
    // World y points up the screen, so the drone's left is on its left here and in the inspector.
    ctx.setTransform(scale, 0, 0, -scale, ox, oy + worldD * scale);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(this.voxels, 0, 0, worldW, worldD);

    // Furniture on this floor, as the rotated boxes the physics uses.
    ctx.fillStyle = FURNITURE;
    for (const box of spec.furniture?.boxes ?? []) {
      if (box.floor !== floorIndex) continue;
      ctx.save();
      ctx.translate(box.cx, box.cy);
      ctx.rotate(box.yaw);
      ctx.fillRect(-box.hx, -box.hy, box.hx * 2, box.hy * 2);
      ctx.restore();
    }

    // Trail on this floor.
    ctx.strokeStyle = this.color;
    ctx.globalAlpha = 0.85;
    ctx.lineWidth = 1.6 / scale * dpr;
    ctx.lineJoin = 'round';
    ctx.beginPath();
    let pen = false;
    for (const p of this.trail) {
      if (p.floor !== floorIndex) {
        pen = false;
        continue;
      }
      if (pen) ctx.lineTo(p.x, p.y);
      else ctx.moveTo(p.x, p.y);
      pen = true;
    }
    ctx.stroke();
    ctx.globalAlpha = 1;

    // Collision flashes fade over a second.
    this.flashes = this.flashes.filter((f) => now - f.t < 1200);
    for (const f of this.flashes) {
      if (f.floor !== floorIndex) continue;
      const k = (now - f.t) / 1200;
      ctx.strokeStyle = `rgba(255, 92, 92, ${1 - k})`;
      ctx.lineWidth = (2.2 / scale) * dpr;
      ctx.beginPath();
      ctx.arc(f.x, f.y, 0.3 + k * 0.9, 0, Math.PI * 2);
      ctx.stroke();
    }

    if (frame) {
      ctx.save();
      ctx.translate(frame.pos[0], frame.pos[1]);
      ctx.rotate(frame.yaw);
      ctx.shadowColor = this.color;
      ctx.shadowBlur = 10 * dpr;
      ctx.fillStyle = '#f4fbff';
      ctx.beginPath();
      ctx.moveTo(0.5, 0);
      ctx.lineTo(-0.32, 0.28);
      ctx.lineTo(-0.18, 0);
      ctx.lineTo(-0.32, -0.28);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }
    ctx.setTransform(1, 0, 0, 1, 0, 0);
  }
}
