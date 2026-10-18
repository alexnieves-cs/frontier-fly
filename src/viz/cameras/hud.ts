export interface HudReadout {
  alt: number;
  speed: number;
  yaw: number;
  battery: number;
  coverage: number;
  mode: string;
  collision: boolean;
  lidar: ArrayLike<number> | null;
  fps: number;
}

export function drawHud(ctx: CanvasRenderingContext2D, width: number, height: number, hud: HudReadout): void {
  ctx.clearRect(0, 0, width, height);
  ctx.strokeStyle = 'rgba(120, 255, 190, 0.85)';
  ctx.fillStyle = 'rgba(190, 255, 220, 0.92)';
  ctx.lineWidth = 1.25;
  ctx.font = '12px "IBM Plex Mono", ui-monospace, monospace';
  const cx = width / 2;
  const cy = height / 2;
  ctx.beginPath();
  ctx.arc(cx, cy, 18, 0, Math.PI * 2);
  ctx.moveTo(cx - 28, cy);
  ctx.lineTo(cx - 8, cy);
  ctx.moveTo(cx + 8, cy);
  ctx.lineTo(cx + 28, cy);
  ctx.moveTo(cx, cy - 28);
  ctx.lineTo(cx, cy - 8);
  ctx.moveTo(cx, cy + 8);
  ctx.lineTo(cx, cy + 22);
  ctx.stroke();

  ctx.fillText(`ALT ${hud.alt.toFixed(2)} m`, 12, 22);
  ctx.fillText(`SPD ${hud.speed.toFixed(2)} m/s`, width - 130, 22);
  ctx.fillText(hud.mode, 12, height - 16);
  ctx.fillText(`${Math.round(hud.coverage * 100)}%  ${hud.fps.toFixed(0)} fps`, width - 120, height - 16);

  const heading = ((hud.yaw * 180) / Math.PI + 360) % 360;
  ctx.fillText(`HDG ${heading.toFixed(0).padStart(3, '0')}`, cx - 36, 22);

  const batX = width - 78;
  ctx.strokeRect(batX, 32, 54, 10);
  ctx.fillRect(batX + 2, 34, 50 * Math.max(0, Math.min(1, hud.battery)), 6);

  if (hud.lidar) {
    const r = 28;
    const ox = 46;
    const oy = height - 52;
    ctx.beginPath();
    ctx.arc(ox, oy, r, 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    for (let i = 0; i < 32; i++) {
      const ang = -Math.PI / 2 + (i / 32) * Math.PI * 2;
      const range = hud.lidar[i] ?? 1;
      const rr = 4 + range * (r - 6);
      const x = ox + Math.cos(ang) * rr;
      const y = oy + Math.sin(ang) * rr;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.closePath();
    ctx.globalAlpha = 0.35;
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.stroke();
  }

  if (hud.collision) {
    ctx.fillStyle = 'rgba(255, 70, 70, 0.28)';
    ctx.fillRect(0, 0, width, height);
  }
}
