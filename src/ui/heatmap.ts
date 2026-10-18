export interface HeatmapData {
  w: number;
  d: number;
  values: Float32Array;
  arrows: Float32Array;
}

export function drawHeatmap(canvas: HTMLCanvasElement, data: HeatmapData | null): void {
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#071018';
  ctx.fillRect(0, 0, w, h);
  if (!data) {
    ctx.fillStyle = '#7f90a8';
    ctx.font = '12px sans-serif';
    ctx.fillText('Policy map appears after the first update', 16, h / 2);
    return;
  }
  let lo = Infinity;
  let hi = -Infinity;
  for (const v of data.values) {
    if (!Number.isFinite(v)) continue;
    lo = Math.min(lo, v);
    hi = Math.max(hi, v);
  }
  if (!Number.isFinite(lo)) lo = 0;
  if (!Number.isFinite(hi) || hi === lo) hi = lo + 1;
  const cw = w / data.w;
  const ch = h / data.d;
  for (let z = 0; z < data.d; z++) {
    for (let x = 0; x < data.w; x++) {
      const v = data.values[z * data.w + x]!;
      if (!Number.isFinite(v)) {
        ctx.fillStyle = '#1c2836';
      } else {
        const t = (v - lo) / (hi - lo);
        const r = Math.round(20 + 40 * (1 - t));
        const g = Math.round(40 + 180 * t);
        const b = Math.round(80 + 40 * (1 - t));
        ctx.fillStyle = `rgb(${r},${g},${b})`;
      }
      ctx.fillRect(x * cw, z * ch, Math.ceil(cw), Math.ceil(ch));
      if (!Number.isFinite(v)) continue;
      const ax = data.arrows[(z * data.w + x) * 2] ?? 0;
      const ay = data.arrows[(z * data.w + x) * 2 + 1] ?? 0;
      if (ax * ax + ay * ay < 0.05) continue;
      const cx = (x + 0.5) * cw;
      const cy = (z + 0.5) * ch;
      ctx.strokeStyle = 'rgba(255,255,255,0.8)';
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + ax * cw * 0.4, cy + ay * ch * 0.4);
      ctx.stroke();
    }
  }
}
