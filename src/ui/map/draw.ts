/**
 * Canvas primitives the 2D site maps share: the demo and film map (ui/demo/map.ts), which draws the
 * runner on a MissionWorld, and the mission planner's editable map (ui/mission/map.ts). Both work in
 * CSS pixels with east to the right and north up; each keeps its own look, these keep the geometry
 * in one place.
 */

/** World meters to canvas CSS pixels. */
export interface MapProjection {
  /** Pixels per meter. */
  readonly scale: number;
  x(wx: number): number;
  y(wy: number): number;
}

/** World-space bounds. */
export interface MapBounds {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

/** Fit bounds into a cssW x cssH canvas, centred, keeping the aspect ratio. */
export function fitProjection(b: MapBounds, cssW: number, cssH: number): MapProjection {
  const scale = Math.min(cssW / (b.maxX - b.minX), cssH / (b.maxY - b.minY));
  const ox = (cssW - (b.maxX - b.minX) * scale) / 2;
  const oy = (cssH - (b.maxY - b.minY) * scale) / 2;
  return {
    scale,
    x: (wx) => ox + (wx - b.minX) * scale,
    y: (wy) => cssH - oy - (wy - b.minY) * scale,
  };
}

/**
 * A pannable view: `view()` gives the centre (meters), meters per pixel and the canvas size, read on
 * every call, so the projection follows pan and zoom without being rebuilt.
 */
export function viewProjection(view: () => { cx: number; cy: number; mpp: number; width: number; height: number }): MapProjection {
  return {
    get scale() {
      return 1 / view().mpp;
    },
    x: (wx) => {
      const v = view();
      return (wx - v.cx) / v.mpp + v.width / 2;
    },
    y: (wy) => {
      const v = view();
      return (v.cy - wy) / v.mpp + v.height / 2;
    },
  };
}

/** A world point as either an [x, y, ...] tuple or an {x, y} record. */
export type MapPoint = readonly number[] | { readonly x: number; readonly y: number };

function px(p: MapPoint): number {
  return Array.isArray(p) ? (p as readonly number[])[0]! : (p as { x: number }).x;
}

function py(p: MapPoint): number {
  return Array.isArray(p) ? (p as readonly number[])[1]! : (p as { y: number }).y;
}

/** Begin a path through the points (a track, a plan, a polygon ring); `close` closes the ring. */
export function tracePath(ctx: CanvasRenderingContext2D, proj: MapProjection, points: readonly MapPoint[], close = false): void {
  ctx.beginPath();
  points.forEach((p, i) => {
    if (i === 0) ctx.moveTo(proj.x(px(p)), proj.y(py(p)));
    else ctx.lineTo(proj.x(px(p)), proj.y(py(p)));
  });
  if (close) ctx.closePath();
}

/** Arrowhead outline of the drone glyph, pixels: nose ahead, tail corners behind, and the notch between them. */
export interface GlyphShape {
  nose: number;
  tail: number;
  half: number;
  notch: number;
}

/** The demo map's drone and the planner's: same arrow, the planner's a size up. */
export const DEMO_GLYPH: GlyphShape = { nose: 9, tail: 6, half: 5.5, notch: 3 };
export const PLANNER_GLYPH: GlyphShape = { nose: 11, tail: 7, half: 5, notch: 4 };

/** Fill the drone arrow at canvas (x, y), pointing along world heading yaw (rad, counter-clockwise from east). */
export function fillDroneGlyph(ctx: CanvasRenderingContext2D, x: number, y: number, yaw: number, shape: GlyphShape, fill: string): void {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-yaw);
  ctx.beginPath();
  ctx.moveTo(shape.nose, 0);
  ctx.lineTo(-shape.tail, shape.half);
  ctx.lineTo(-shape.notch, 0);
  ctx.lineTo(-shape.tail, -shape.half);
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.fill();
  ctx.restore();
}
