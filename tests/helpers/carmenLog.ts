// Synthetic CARMEN log for the real-log tests (tests/unit/carmen.test.ts).

/** Range from (x, y) along heading a to the walls of the box [0, w] x [0, h]. */
export function boxRange(x: number, y: number, a: number, w: number, h: number): number {
  const c = Math.cos(a);
  const s = Math.sin(a);
  let r = Infinity;
  if (c > 1e-9) r = Math.min(r, (w - x) / c);
  if (c < -1e-9) r = Math.min(r, -x / c);
  if (s > 1e-9) r = Math.min(r, (h - y) / s);
  if (s < -1e-9) r = Math.min(r, -y / s);
  return r;
}

/** A CARMEN log of a robot driving a loop in a 10 x 6 m room, with exact ranges and odometry. */
export function syntheticLog(): string {
  const lines = ['# message_name [message contents] ipc_timestamp ipc_hostname logger_timestamp', 'ODOM 0 0 0 0 0 0 100.0 nohost 0'];
  const pts: [number, number][] = [
    [2, 1.5],
    [8, 1.5],
    [8, 4.5],
    [2, 4.5],
    [2, 1.5],
  ];
  let t = 0;
  for (let leg = 0; leg < pts.length - 1; leg++) {
    const [ax, ay] = pts[leg]!;
    const [bx, by] = pts[leg + 1]!;
    const th = Math.atan2(by - ay, bx - ax);
    const steps = Math.round(Math.hypot(bx - ax, by - ay) / 0.1);
    for (let k = 0; k < steps; k++) {
      const x = ax + ((bx - ax) * k) / steps;
      const y = ay + ((by - ay) * k) / steps;
      const ranges = Array.from({ length: 180 }, (_, b) => boxRange(x, y, th + (-90 + b) * (Math.PI / 180), 10, 6).toFixed(3));
      lines.push(`FLASER 180 ${ranges.join(' ')} ${x} ${y} ${th} ${x} ${y} ${th} ${(100 + t).toFixed(6)} nohost ${t.toFixed(6)}`);
      t += 0.1;
    }
  }
  return lines.join('\n');
}
