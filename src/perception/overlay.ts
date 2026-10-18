import type { Detection } from './detector';
import type { ClearReport } from './seenEmpty';
import { clearLabel } from './seenEmpty';

const COLOR: Record<string, string> = {
  person: '#49f0a0',
  backpack: '#8ec5ff',
  phone: '#d7c4ff',
  toolbox: '#ffb15a',
  medkit: '#ff7a8a',
  bottle: '#7ee0ff',
  radio: '#c6f08a',
  helmet: '#f0e2a0',
  blanket: '#e7c2ff',
  mannequin: '#d5dbe3',
  coat: '#e6c39a',
  decoy_pack: '#9bb7c9',
  decoy_case: '#c4a27a',
};

export function drawDetections(ctx: CanvasRenderingContext2D, width: number, height: number, detections: Detection[]): void {
  ctx.save();
  ctx.lineWidth = 2;
  ctx.font = '12px ui-monospace, "IBM Plex Mono", monospace';
  for (const det of detections) {
    const x = det.bbox.x * width;
    const y = det.bbox.y * height;
    const w = det.bbox.w * width;
    const h = det.bbox.h * height;
    const color = det.kind === 'fp' ? '#ffb020' : COLOR[det.classId] ?? '#ffffff';
    ctx.strokeStyle = color;
    ctx.strokeRect(x, y, w, h);
    const label = `${det.classId} ${det.confidence.toFixed(2)}`;
    const tw = ctx.measureText(label).width;
    ctx.fillStyle = 'rgba(6, 16, 24, 0.72)';
    ctx.fillRect(x, Math.max(0, y - 16), tw + 8, 16);
    ctx.fillStyle = color;
    ctx.fillText(label, x + 4, Math.max(12, y - 4));
  }
  ctx.restore();
}

export function detectionRows(detections: Detection[]): string[] {
  return detections.map((det) => {
    const tag = det.kind === 'fp' ? 'fp' : det.kind === 'confused' ? 'confused' : 'hit';
    return `${det.classId}  ${det.confidence.toFixed(2)}  ${det.distance.toFixed(1)}m  ${tag}`;
  });
}

export function detectionSummary(detections: Detection[], report: ClearReport | null): string {
  const lines = detectionRows(detections);
  if (report) lines.unshift(clearLabel(report));
  return lines.join('\n');
}
