import type { CoverageGrid } from '../../mission/session';
import { isSelfIntersecting, snapToGrid } from '../../mission/polygon';
import type { Point } from '../../mission/types';
import type { WorldFootprint } from '../../mission/world';

export type MapTool = 'geofence' | 'nofly' | 'start' | 'edit' | 'view';

export interface MapSelection {
  kind: 'geofence' | 'nofly' | 'start';
  zoneId?: string;
  vertex?: number;
}

export interface MapLayers {
  footprints: WorldFootprint[];
  geofence: Point[];
  noFly: { id: string; points: Point[]; selected: boolean }[];
  draft: Point[] | null;
  draftKind: 'geofence' | 'nofly' | null;
  start: { x: number; y: number; yawRad: number } | null;
  plan: Point[];
  track: Point[];
  drone: { x: number; y: number; yawRad: number } | null;
  coverage: CoverageGrid | null;
  detections: { x: number; y: number; kind: string; label: string }[];
  selection: MapSelection | null;
}

export interface MapHandlers {
  onPoint: (point: Point) => void;
  onClose: () => void;
  onPlaceStart: (point: Point) => void;
  onMoveVertex: (selection: MapSelection, point: Point) => void;
  onInsertVertex: (selection: MapSelection, point: Point) => void;
  onSelect: (selection: MapSelection | null) => void;
  onHover: (point: Point | null) => void;
  onReady: () => void;
}

const INK = '#1c1e1a';
const MUTED = '#5e635c';
const ACCENT = '#1e4a38';
const DANGER = '#8c2f2a';
const TRACK = '#1d4e89';
const PAPER = '#e4e1d8';
const FONT = '"Segoe UI", Helvetica, Arial, sans-serif';

interface DragVertex {
  selection: MapSelection;
}

export interface MissionMap {
  setLayers(layers: MapLayers): void;
  setTool(tool: MapTool): void;
  setSnap(enabled: boolean, gridM: number): void;
  fit(points: Point[]): void;
  redraw(): void;
}

export function createMissionMap(canvas: HTMLCanvasElement, handlers: MapHandlers): MissionMap {
  const ctx = canvas.getContext('2d');
  if (!ctx) throw new Error('The mission map could not get a canvas context.');
  const context = ctx;

  let layers: MapLayers = {
    footprints: [],
    geofence: [],
    noFly: [],
    draft: null,
    draftKind: null,
    start: null,
    plan: [],
    track: [],
    drone: null,
    coverage: null,
    detections: [],
    selection: null,
  };
  let tool: MapTool = 'edit';
  let snapEnabled = true;
  let gridM = 1;
  let cx = 46;
  let cy = 30;
  let mpp = 0.25;
  let width = 1;
  let height = 1;
  let hover: Point | null = null;
  let drag: DragVertex | null = null;
  let panning: { x: number; y: number } | null = null;
  let pendingInsert: MapSelection | null = null;
  let downScreen: { x: number; y: number } | null = null;
  let pendingFit: Point[] | null = null;
  let ready = false;

  function worldToScreen(point: Point): { x: number; y: number } {
    return { x: (point.x - cx) / mpp + width / 2, y: (cy - point.y) / mpp + height / 2 };
  }

  function screenToWorld(sx: number, sy: number): Point {
    return { x: cx + (sx - width / 2) * mpp, y: cy - (sy - height / 2) * mpp };
  }

  function localPoint(event: { clientX: number; clientY: number }): { x: number; y: number } {
    const rect = canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  function vertexList(): { point: Point; selection: MapSelection }[] {
    const list: { point: Point; selection: MapSelection }[] = [];
    layers.geofence.forEach((point, index) => list.push({ point, selection: { kind: 'geofence', vertex: index } }));
    for (const zone of layers.noFly) {
      zone.points.forEach((point, index) => list.push({ point, selection: { kind: 'nofly', zoneId: zone.id, vertex: index } }));
    }
    if (layers.draft) {
      layers.draft.forEach((point, index) => list.push({ point, selection: { kind: layers.draftKind === 'nofly' ? 'nofly' : 'geofence', vertex: index } }));
    }
    return list;
  }

  function sameSelection(a: MapSelection, b: MapSelection): boolean {
  return a.kind === b.kind && a.zoneId === b.zoneId && a.vertex === b.vertex;
}

  function snapWorld(world: Point, screen: { x: number; y: number }, ignore?: MapSelection): Point {
    let best: Point | null = null;
    let bestD = 8;
    for (const vertex of vertexList()) {
      if (ignore && sameSelection(vertex.selection, ignore)) continue;
      const at = worldToScreen(vertex.point);
      const distance = Math.hypot(at.x - screen.x, at.y - screen.y);
      if (distance < bestD) {
        bestD = distance;
        best = vertex.point;
      }
    }
    if (best) return { x: best.x, y: best.y };
    if (snapEnabled) return snapToGrid(world, gridM);
    return { x: Math.round(world.x * 1000) / 1000, y: Math.round(world.y * 1000) / 1000 };
  }

  function nearFirstDraft(screen: { x: number; y: number }): boolean {
    if (!layers.draft || layers.draft.length < 3) return false;
    const at = worldToScreen(layers.draft[0]!);
    return Math.hypot(at.x - screen.x, at.y - screen.y) <= 12;
  }

  function hitVertex(screen: { x: number; y: number }): MapSelection | null {
    if (layers.start) {
      const at = worldToScreen(layers.start);
      if (Math.hypot(at.x - screen.x, at.y - screen.y) <= 10) return { kind: 'start' };
    }
    let best: MapSelection | null = null;
    let bestD = 8;
    const consider = (point: Point, selection: MapSelection): void => {
      const at = worldToScreen(point);
      const distance = Math.hypot(at.x - screen.x, at.y - screen.y);
      if (distance <= bestD) {
        bestD = distance;
        best = selection;
      }
    };
    layers.geofence.forEach((point, index) => consider(point, { kind: 'geofence', vertex: index }));
    for (const zone of layers.noFly) {
      zone.points.forEach((point, index) => consider(point, { kind: 'nofly', zoneId: zone.id, vertex: index }));
    }
    return best;
  }

  function hitEdge(screen: { x: number; y: number }): MapSelection | null {
    const rings: { points: Point[]; selection: (index: number) => MapSelection }[] = [
      { points: layers.geofence, selection: (index) => ({ kind: 'geofence', vertex: index }) },
    ];
    for (const zone of layers.noFly) {
      rings.push({
        points: zone.points,
        selection: (index) => ({ kind: 'nofly', zoneId: zone.id, vertex: index }),
      });
    }
    for (const ring of rings) {
      const { points } = ring;
      if (points.length < 2) continue;
      for (let i = 0; i < points.length; i++) {
        const a = worldToScreen(points[i]!);
        const b = worldToScreen(points[(i + 1) % points.length]!);
        if (distanceToSegment(screen, a, b) <= 6) return ring.selection(i + 1);
      }
    }
    return null;
  }

  function resize(): void {
    const rect = canvas.getBoundingClientRect();
    width = Math.max(1, rect.width);
    height = Math.max(1, rect.height);
    const dpr = window.devicePixelRatio || 1;
    const nextW = Math.round(width * dpr);
    const nextH = Math.round(height * dpr);
    if (canvas.width !== nextW || canvas.height !== nextH) {
      canvas.width = nextW;
      canvas.height = nextH;
    }
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function publishView(): void {
    canvas.dataset.cx = cx.toFixed(4);
    canvas.dataset.cy = cy.toFixed(4);
    canvas.dataset.mpp = String(mpp);
    canvas.dataset.width = String(width);
    canvas.dataset.height = String(height);
  }

  function markReady(): void {
    if (ready || width < 2 || height < 2) return;
    ready = true;
    handlers.onReady();
  }

  function fit(points: Point[]): void {
    pendingFit = points;
    resize();
    if (width < 2 || height < 2) return;
    const source = points.length > 0 ? points : [
      { x: 0, y: 0 },
      { x: 90, y: 60 },
    ];
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const point of source) {
      minX = Math.min(minX, point.x);
      minY = Math.min(minY, point.y);
      maxX = Math.max(maxX, point.x);
      maxY = Math.max(maxY, point.y);
    }
    const pad = 28;
    const spanX = Math.max(10, maxX - minX);
    const spanY = Math.max(10, maxY - minY);
    mpp = Math.min(8, Math.max(0.05, Math.max(spanX / Math.max(1, width - pad * 2), spanY / Math.max(1, height - pad * 2))));
    cx = (minX + maxX) / 2;
    cy = (minY + maxY) / 2;
    pendingFit = null;
    redraw();
    markReady();
  }

  function trace(points: Point[], close: boolean): void {
    if (points.length === 0) return;
    context.beginPath();
    const first = worldToScreen(points[0]!);
    context.moveTo(first.x, first.y);
    for (let i = 1; i < points.length; i++) {
      const point = worldToScreen(points[i]!);
      context.lineTo(point.x, point.y);
    }
    if (close && points.length > 2) context.closePath();
  }

  function hatchRing(points: Point[]): void {
    if (points.length < 3) return;
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const point of points) {
      const at = worldToScreen(point);
      minX = Math.min(minX, at.x);
      minY = Math.min(minY, at.y);
      maxX = Math.max(maxX, at.x);
      maxY = Math.max(maxY, at.y);
    }
    context.save();
    trace(points, true);
    context.clip();
    context.strokeStyle = 'rgba(140, 47, 42, 0.45)';
    context.lineWidth = 1;
    context.beginPath();
    const span = maxY - minY + (maxX - minX);
    for (let x = minX - span; x < maxX; x += 8) {
      context.moveTo(x, minY);
      context.lineTo(x + span, maxY);
    }
    context.stroke();
    context.restore();
  }

  function fillRing(points: Point[], fill: string): void {
    if (points.length < 3) return;
    trace(points, true);
    context.fillStyle = fill;
    context.fill();
  }

  function strokeRing(points: Point[], stroke: string, widthPx: number, dash: number[] = []): void {
    if (points.length < 2) return;
    trace(points, points.length > 2);
    context.strokeStyle = stroke;
    context.lineWidth = widthPx;
    context.setLineDash(dash);
    context.stroke();
    context.setLineDash([]);
  }

  function drawGrid(): void {
    const origin = screenToWorld(0, 0);
    const far = screenToWorld(width, height);
    const minX = Math.min(origin.x, far.x);
    const maxX = Math.max(origin.x, far.x);
    const minY = Math.min(origin.y, far.y);
    const maxY = Math.max(origin.y, far.y);
    const step = gridM > 0 ? gridM : 1;
    if ((maxX - minX) / step > 180) return;
    context.lineWidth = 1;
    const startX = Math.floor(minX / step) * step;
    const startY = Math.floor(minY / step) * step;
    for (let x = startX; x <= maxX; x += step) {
      const major = Math.abs(x / 10 - Math.round(x / 10)) < 1e-6;
      context.strokeStyle = major ? '#b7b1a4' : '#d4cfc3';
      const sx = worldToScreen({ x, y: 0 }).x;
      context.beginPath();
      context.moveTo(sx, 0);
      context.lineTo(sx, height);
      context.stroke();
    }
    for (let y = startY; y <= maxY; y += step) {
      const major = Math.abs(y / 10 - Math.round(y / 10)) < 1e-6;
      context.strokeStyle = major ? '#b7b1a4' : '#d4cfc3';
      const sy = worldToScreen({ x: 0, y }).y;
      context.beginPath();
      context.moveTo(0, sy);
      context.lineTo(width, sy);
      context.stroke();
    }
  }

  function drawFootprints(): void {
    for (const footprint of layers.footprints) {
      const fill = footprint.kind === 'building' ? '#cfc8ba' : footprint.kind === 'water' ? '#d5ddd8' : footprint.kind === 'canopy' ? '#d7ddd2' : '#d8d3c7';
      fillRing(footprint.polygon, fill);
      strokeRing(footprint.polygon, footprint.kind === 'building' ? '#6f6a60' : '#8a8478', 1);
      const box = footprint.polygon.reduce(
        (acc, point) => ({
          minX: Math.min(acc.minX, point.x),
          maxX: Math.max(acc.maxX, point.x),
          minY: Math.min(acc.minY, point.y),
          maxY: Math.max(acc.maxY, point.y),
        }),
        { minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity },
      );
      // Canopy is named once in the legend; labels are for the places a mission refers to.
      if (footprint.kind === 'canopy' || (box.maxX - box.minX) / mpp < 36) continue;
      const at = worldToScreen({ x: (box.minX + box.maxX) / 2, y: (box.minY + box.maxY) / 2 });
      context.fillStyle = MUTED;
      context.font = `12px ${FONT}`;
      context.textAlign = 'center';
      context.textBaseline = 'middle';
      context.fillText(footprint.label, at.x, at.y);
    }
  }

  function drawCoverage(): void {
    const grid = layers.coverage;
    if (!grid) return;
    context.fillStyle = 'rgba(30, 74, 56, 0.22)';
    for (let row = 0; row < grid.rows; row++) {
      for (let col = 0; col < grid.cols; col++) {
        if (grid.visited[row * grid.cols + col] !== 1) continue;
        const x = grid.originX + col * grid.cellM;
        const y = grid.originY + (row + 1) * grid.cellM;
        const a = worldToScreen({ x, y });
        const size = grid.cellM / mpp;
        context.fillRect(a.x, a.y, size, size);
      }
    }
  }

  function drawVertices(points: Point[], selectedIndex: number | undefined): void {
    points.forEach((point, index) => {
      const at = worldToScreen(point);
      context.fillStyle = selectedIndex === index ? ACCENT : '#fffcf7';
      context.strokeStyle = INK;
      context.lineWidth = 1;
      context.fillRect(at.x - 3.5, at.y - 3.5, 7, 7);
      context.strokeRect(at.x - 3.5, at.y - 3.5, 7, 7);
    });
  }

  function drawMarker(point: { x: number; y: number; yawRad: number }, fill: string): void {
    const at = worldToScreen(point);
    context.save();
    context.translate(at.x, at.y);
    context.rotate(-point.yawRad);
    context.beginPath();
    context.moveTo(11, 0);
    context.lineTo(-7, 5);
    context.lineTo(-4, 0);
    context.lineTo(-7, -5);
    context.closePath();
    context.fillStyle = fill;
    context.fill();
    context.restore();
  }

  function drawScale(): void {
    const meters = [1, 2, 5, 10, 20, 50, 100].find((candidate) => {
      const px = candidate / mpp;
      return px >= 48 && px <= 140;
    }) ?? 10;
    const px = meters / mpp;
    const x = 16;
    const y = height - 18;
    context.strokeStyle = INK;
    context.fillStyle = INK;
    context.lineWidth = 1.5;
    context.beginPath();
    context.moveTo(x, y);
    context.lineTo(x + px, y);
    context.moveTo(x, y - 4);
    context.lineTo(x, y + 4);
    context.moveTo(x + px, y - 4);
    context.lineTo(x + px, y + 4);
    context.stroke();
    context.font = `12px ${FONT}`;
    context.textAlign = 'left';
    context.textBaseline = 'alphabetic';
    context.fillText(`${meters} m`, x, y - 6);
    const nx = width - 28;
    const ny = 22;
    context.beginPath();
    context.moveTo(nx, ny + 16);
    context.lineTo(nx, ny);
    context.moveTo(nx - 3, ny + 4);
    context.lineTo(nx, ny);
    context.lineTo(nx + 3, ny + 4);
    context.stroke();
    context.textAlign = 'center';
    context.textBaseline = 'bottom';
    context.fillText('N', nx, ny - 2);
  }

  function redraw(): void {
    resize();
    publishView();
    context.clearRect(0, 0, width, height);
    context.textAlign = 'left';
    context.textBaseline = 'alphabetic';
    context.fillStyle = PAPER;
    context.fillRect(0, 0, width, height);
    drawGrid();
    drawFootprints();
    drawCoverage();
    fillRing(layers.geofence, 'rgba(30, 74, 56, 0.08)');
    strokeRing(layers.geofence, ACCENT, 2);
    for (const zone of layers.noFly) {
      fillRing(zone.points, 'rgba(140, 47, 42, 0.08)');
      hatchRing(zone.points);
      strokeRing(zone.points, DANGER, zone.selected ? 2.5 : 1.5);
    }
    if (layers.plan.length > 1) strokeRing(layers.plan, 'rgba(29, 78, 137, 0.45)', 1, [3, 4]);
    if (layers.track.length > 1) {
      context.beginPath();
      const first = worldToScreen(layers.track[0]!);
      context.moveTo(first.x, first.y);
      for (let i = 1; i < layers.track.length; i++) {
        const point = worldToScreen(layers.track[i]!);
        context.lineTo(point.x, point.y);
      }
      context.strokeStyle = TRACK;
      context.lineWidth = 2;
      context.stroke();
    }
    if (layers.draft && layers.draft.length > 0) {
      const preview = hover && (tool === 'geofence' || tool === 'nofly') ? [...layers.draft, hover] : layers.draft;
      const bad = preview.length >= 4 && isSelfIntersecting(preview);
      strokeRing(preview, bad ? DANGER : tool === 'nofly' ? DANGER : ACCENT, 1.5, [5, 4]);
    }
    for (const detection of layers.detections) {
      const at = worldToScreen(detection);
      context.fillStyle = detection.kind === 'hazard' ? DANGER : detection.kind === 'person' ? ACCENT : TRACK;
      context.fillRect(at.x - 4, at.y - 4, 8, 8);
      context.strokeStyle = '#fffcf7';
      context.lineWidth = 1;
      context.strokeRect(at.x - 4, at.y - 4, 8, 8);
    }
    if (layers.start) {
      const at = worldToScreen(layers.start);
      context.beginPath();
      context.arc(at.x, at.y, 6, 0, Math.PI * 2);
      context.fillStyle = '#fffcf7';
      context.fill();
      context.strokeStyle = ACCENT;
      context.lineWidth = 1.5;
      context.stroke();
      context.beginPath();
      context.moveTo(at.x, at.y);
      context.lineTo(at.x + Math.cos(layers.start.yawRad) * 16, at.y - Math.sin(layers.start.yawRad) * 16);
      context.stroke();
    }
    if (layers.drone) drawMarker(layers.drone, TRACK);
    if (tool === 'edit') {
      drawVertices(layers.geofence, layers.selection?.kind === 'geofence' ? layers.selection.vertex : undefined);
      for (const zone of layers.noFly) {
        const selected = layers.selection?.kind === 'nofly' && layers.selection.zoneId === zone.id ? layers.selection.vertex : undefined;
        drawVertices(zone.points, selected);
      }
    } else if (layers.draft) {
      drawVertices(layers.draft, undefined);
    }
    if (hover && (tool === 'geofence' || tool === 'nofly' || tool === 'start')) {
      const at = worldToScreen(hover);
      context.strokeStyle = MUTED;
      context.lineWidth = 1;
      context.beginPath();
      context.moveTo(at.x - 6, at.y);
      context.lineTo(at.x + 6, at.y);
      context.moveTo(at.x, at.y - 6);
      context.lineTo(at.x, at.y + 6);
      context.stroke();
    }
    drawScale();
  }

  canvas.addEventListener('contextmenu', (event) => event.preventDefault());
  canvas.addEventListener('wheel', (event) => {
    event.preventDefault();
    const screen = localPoint(event);
    const world = screenToWorld(screen.x, screen.y);
    mpp = Math.min(8, Math.max(0.05, mpp * (event.deltaY > 0 ? 1.12 : 1 / 1.12)));
    cx = world.x - (screen.x - width / 2) * mpp;
    cy = world.y + (screen.y - height / 2) * mpp;
    redraw();
  }, { passive: false });

  canvas.addEventListener('pointerdown', (event) => {
    const screen = localPoint(event);
    if (event.button === 1 || event.button === 2 || (tool === 'view' && event.button === 0)) {
      panning = screen;
      canvas.setPointerCapture(event.pointerId);
      return;
    }
    if (event.button !== 0 || tool === 'view') return;
    downScreen = screen;
    const world = screenToWorld(screen.x, screen.y);
    if (tool === 'geofence' || tool === 'nofly') {
      if (nearFirstDraft(screen)) handlers.onClose();
      else handlers.onPoint(snapWorld(world, screen));
      return;
    }
    if (tool === 'start') {
      handlers.onPlaceStart(snapWorld(world, screen));
      return;
    }
    const vertex = hitVertex(screen);
    if (vertex?.kind === 'start') {
      drag = { selection: vertex };
      handlers.onSelect(vertex);
      canvas.setPointerCapture(event.pointerId);
      return;
    }
    if (vertex && vertex.vertex != null) {
      drag = { selection: vertex };
      handlers.onSelect(vertex);
      canvas.setPointerCapture(event.pointerId);
      return;
    }
    const edge = hitEdge(screen);
    pendingInsert = edge;
    if (!edge) handlers.onSelect(null);
    canvas.setPointerCapture(event.pointerId);
  });

  canvas.addEventListener('pointermove', (event) => {
    const screen = localPoint(event);
    if (panning && (tool === 'view' || event.buttons === 2 || event.buttons === 4 || event.buttons === 1)) {
      cx -= (screen.x - panning.x) * mpp;
      cy += (screen.y - panning.y) * mpp;
      panning = screen;
      redraw();
      return;
    }
    const world = screenToWorld(screen.x, screen.y);
    if (drag) {
      const snapped = snapWorld(world, screen, drag.selection);
      handlers.onMoveVertex(drag.selection, snapped);
      hover = snapped;
      handlers.onHover(snapped);
      return;
    }
    hover = tool === 'edit' ? null : snapWorld(world, screen);
    handlers.onHover(hover);
    redraw();
  });

  canvas.addEventListener('pointerup', (event) => {
    const screen = localPoint(event);
    if (pendingInsert && downScreen && Math.hypot(screen.x - downScreen.x, screen.y - downScreen.y) < 4) {
      handlers.onInsertVertex(pendingInsert, snapWorld(screenToWorld(screen.x, screen.y), screen));
    }
    drag = null;
    panning = null;
    pendingInsert = null;
    downScreen = null;
  });

  canvas.addEventListener('pointerleave', () => {
    if (drag || panning) return;
    hover = null;
    handlers.onHover(null);
    redraw();
  });

  const observer = new ResizeObserver(() => {
    if (pendingFit) fit(pendingFit);
    else redraw();
    markReady();
  });
  observer.observe(canvas);

  return {
    setLayers(next) {
      layers = next;
      redraw();
    },
    setTool(next) {
      tool = next;
      canvas.dataset.tool = next;
      canvas.style.cursor = next === 'geofence' || next === 'nofly' || next === 'start' ? 'crosshair' : next === 'view' ? 'grab' : 'default';
      redraw();
    },
    setSnap(enabled, grid) {
      snapEnabled = enabled;
      gridM = grid;
      redraw();
    },
    fit,
    redraw,
  };
}

function distanceToSegment(p: { x: number; y: number }, a: { x: number; y: number }, b: { x: number; y: number }): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len2 = dx * dx + dy * dy;
  if (len2 < 1e-6) return Math.hypot(p.x - a.x, p.y - a.y);
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
  return Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}
