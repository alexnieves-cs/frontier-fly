/** Coverage over sim time for every lane, on one small canvas. */
export class CoverageChart {
  private series: Array<Array<[number, number]>> = [];
  private colors: string[] = [];
  private tMax = 120;

  constructor(private readonly canvas: HTMLCanvasElement) {}

  reset(colors: string[], tMax: number): void {
    this.colors = colors;
    this.series = colors.map(() => [[0, 0]]);
    this.tMax = tMax;
  }

  push(lane: number, t: number, coverage: number): void {
    const s = this.series[lane];
    if (!s) return;
    const last = s[s.length - 1]!;
    if (t > last[0]) s.push([t, coverage]);
  }

  draw(): void {
    const c = this.canvas;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    const w = Math.max(1, Math.round(c.clientWidth * dpr));
    const h = Math.max(1, Math.round(c.clientHeight * dpr));
    if (c.width !== w || c.height !== h) {
      c.width = w;
      c.height = h;
    }
    const ctx = c.getContext('2d')!;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const left = 34 * dpr;
    const right = 8 * dpr;
    const top = 8 * dpr;
    const bottom = 20 * dpr;
    const pw = w - left - right;
    const ph = h - top - bottom;
    const yMax = 1;
    ctx.font = `${10 * dpr}px ui-monospace, Menlo, monospace`;
    ctx.fillStyle = 'rgba(157, 176, 188, 0.85)';
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.07)';
    ctx.lineWidth = dpr;
    for (let p = 0; p <= 1.0001; p += 0.25) {
      const y = top + ph - (p / yMax) * ph;
      ctx.beginPath();
      ctx.moveTo(left, y);
      ctx.lineTo(left + pw, y);
      ctx.stroke();
      ctx.fillText(`${Math.round(p * 100)}%`, 2 * dpr, y + 3 * dpr);
    }
    const stepT = this.tMax > 60 ? 30 : 10;
    for (let t = 0; t <= this.tMax + 1e-6; t += stepT) {
      const x = left + (t / this.tMax) * pw;
      ctx.fillText(`${t}s`, Math.min(x - 6 * dpr, w - 26 * dpr), h - 5 * dpr);
    }
    this.series.forEach((s, i) => {
      ctx.strokeStyle = this.colors[i]!;
      ctx.lineWidth = 2 * dpr;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      s.forEach(([t, v], k) => {
        const x = left + (Math.min(t, this.tMax) / this.tMax) * pw;
        const y = top + ph - (Math.min(v, yMax) / yMax) * ph;
        if (k) ctx.lineTo(x, y);
        else ctx.moveTo(x, y);
      });
      ctx.stroke();
    });
  }
}
