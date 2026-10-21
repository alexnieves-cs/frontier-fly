import { CELL, SURFACE, VOXEL } from '../../shared/constants';
import type { BuildingSpec } from '../../shared/types';
import type { RadioModel } from '../../team/radio';
import type { TeamView } from '../../team/run';

/** One colour per drone, shared by the map, the legend and the chart. */
export const DRONE_COLORS = ['#56e0ff', '#ffd166', '#ff7ab6', '#3dff9a'];
const BASE_COLOR = '#eef4f7';

function hexToRgb(hex: string): [number, number, number] {
  const n = parseInt(hex.slice(1), 16);
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
}

/**
 * Top-down map of the team: walls, the voxels flown (coloured by the drone
 * that flew them first), radio links, goals and the base. `known` limits the
 * flown voxels drawn to a mask, for the operator's view.
 */
export class TeamMap {
  private scale = 1;
  private ox = 0;
  private oy = 0;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    private readonly spec: BuildingSpec,
  ) {}

  private fit(): CanvasRenderingContext2D {
    const c = this.canvas;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    const w = Math.max(1, Math.round(c.clientWidth * dpr));
    const h = Math.max(1, Math.round(c.clientHeight * dpr));
    if (c.width !== w || c.height !== h) {
      c.width = w;
      c.height = h;
    }
    const f = this.spec.floors[0]!;
    // The base stands up to 3 m outside; leave room for it on every side.
    const pad = 4.5;
    const spanW = f.w * this.spec.cellSize + pad * 2;
    const spanD = f.d * this.spec.cellSize + pad * 2;
    this.scale = Math.min(w / spanW, h / spanD);
    this.ox = (w - f.w * this.spec.cellSize * this.scale) / 2;
    this.oy = (h - f.d * this.spec.cellSize * this.scale) / 2;
    return c.getContext('2d')!;
  }

  private px(x: number): number {
    return this.ox + x * this.scale;
  }

  private py(y: number): number {
    return this.oy + y * this.scale;
  }

  draw(view: TeamView, firstBy: Uint8Array, known: Uint8Array | null, radio: RadioModel | null): void {
    const ctx = this.fit();
    const { width: w, height: h } = this.canvas;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = '#05080b';
    ctx.fillRect(0, 0, w, h);
    const spec = this.spec;
    const f = spec.floors[0]!;
    const cs = spec.cellSize;
    const s = this.scale;

    // Floor, then flown voxels, then walls on top.
    ctx.fillStyle = '#16252f';
    ctx.fillRect(this.px(0), this.py(0), f.w * cs * s, f.d * cs * s);
    const vw = f.w * 2;
    const vd = f.d * 2;
    const rgb = DRONE_COLORS.map(hexToRgb);
    const vs = VOXEL * s;
    for (let vy = 0; vy < vd; vy++) {
      for (let vx = 0; vx < vw; vx++) {
        const k = vy * vw + vx;
        const who = firstBy[k]!;
        if (who === 0) continue;
        if (known && known[k] !== 1) continue;
        const [r, g, b] = rgb[(who - 1) % rgb.length]!;
        ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.5)`;
        ctx.fillRect(this.px(vx * VOXEL), this.py(vy * VOXEL), vs + 0.5, vs + 0.5);
      }
    }
    for (let z = 0; z < f.d; z++) {
      for (let x = 0; x < f.w; x++) {
        const i = z * f.w + x;
        const c = f.cells[i]!;
        if (c !== CELL.WALL && c !== CELL.SHAFT) continue;
        const glass = f.surfaces?.[i] === SURFACE.GLASS;
        ctx.fillStyle = glass ? '#4f8ea6' : c === CELL.SHAFT ? '#5b6670' : '#05080b';
        ctx.fillRect(this.px(x * cs), this.py(z * cs), Math.ceil(cs * s), Math.ceil(cs * s));
      }
    }

    // Radio links: drone to drone and drone to base, brighter when packets get through.
    if (radio) {
      const nodes = [...view.pos.map((p, i) => (view.launched[i] ? p : null)), view.base];
      ctx.lineWidth = Math.max(1, s * 0.08);
      for (let i = 0; i < nodes.length; i++) {
        for (let j = i + 1; j < nodes.length; j++) {
          const a = nodes[i];
          const b = nodes[j];
          if (!a || !b) continue;
          const p = Math.min(radio.deliveryProbability(a, b), radio.deliveryProbability(b, a));
          if (p < 0.05) continue;
          ctx.strokeStyle = `rgba(61, 255, 154, ${0.15 + 0.6 * p})`;
          ctx.setLineDash(p < 0.5 ? [s * 0.4, s * 0.4] : []);
          ctx.beginPath();
          ctx.moveTo(this.px(a[0]), this.py(a[1]));
          ctx.lineTo(this.px(b[0]), this.py(b[1]));
          ctx.stroke();
        }
      }
      ctx.setLineDash([]);
    }

    // Base radio.
    const [bx, by] = view.base;
    ctx.fillStyle = BASE_COLOR;
    ctx.beginPath();
    ctx.moveTo(this.px(bx), this.py(by) - s * 0.9);
    ctx.lineTo(this.px(bx) + s * 0.8, this.py(by) + s * 0.6);
    ctx.lineTo(this.px(bx) - s * 0.8, this.py(by) + s * 0.6);
    ctx.closePath();
    ctx.fill();
    ctx.font = `600 ${Math.max(10, s * 0.75)}px Inter, system-ui, sans-serif`;
    ctx.textAlign = 'center';
    ctx.fillText('BASE', this.px(bx), this.py(by) + s * 1.8);

    // Goals and drones.
    for (let i = 0; i < view.pos.length; i++) {
      if (!view.launched[i]) continue;
      const color = DRONE_COLORS[i % DRONE_COLORS.length]!;
      const g = view.goal[i];
      const [x, y] = view.pos[i]!;
      if (g && !view.done[i]) {
        ctx.strokeStyle = color;
        ctx.lineWidth = Math.max(1, s * 0.07);
        ctx.setLineDash([s * 0.25, s * 0.25]);
        ctx.beginPath();
        ctx.moveTo(this.px(x), this.py(y));
        ctx.lineTo(this.px(g[0]), this.py(g[1]));
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath();
        ctx.arc(this.px(g[0]), this.py(g[1]), s * 0.35, 0, Math.PI * 2);
        ctx.stroke();
      }
      const relay = view.role[i] === 'relay';
      ctx.fillStyle = color;
      ctx.strokeStyle = '#05080b';
      ctx.lineWidth = Math.max(1, s * 0.08);
      ctx.beginPath();
      if (relay) ctx.rect(this.px(x) - s * 0.45, this.py(y) - s * 0.45, s * 0.9, s * 0.9);
      else ctx.arc(this.px(x), this.py(y), s * 0.45, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      const yaw = view.yaw[i]!;
      ctx.strokeStyle = color;
      ctx.beginPath();
      ctx.moveTo(this.px(x), this.py(y));
      ctx.lineTo(this.px(x + Math.cos(yaw) * 0.9), this.py(y + Math.sin(yaw) * 0.9));
      ctx.stroke();
    }
  }
}
