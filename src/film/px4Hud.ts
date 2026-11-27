/**
 * The flight-controller panel the film draws over a PX4-flown mission: PX4's mode and arming,
 * where EKF2 gets its position (GNSS, or visual odometry once GNSS is denied indoors), how far
 * EKF2's estimate sits from the truth, the attitude, and the four motor outputs. Everything shown
 * is what PX4 reported on that tick (scripts/px4/mission.ts --film).
 */
import type { FlightFrame, FlightLog, Px4Sample } from './flight';
import type { Px4FlightFacts } from './px4Film';

const HISTORY_SEC = 60;

function el<K extends keyof HTMLElementTagNameMap>(tag: K, cls: string, parent?: HTMLElement, text?: string): HTMLElementTagNameMap[K] {
  const node = document.createElement(tag);
  node.className = cls;
  if (text != null) node.textContent = text;
  parent?.appendChild(node);
  return node;
}

const MODE_LABELS: Record<string, string> = {
  offboard: 'OFFBOARD',
  'auto.loiter': 'HOLD',
  'auto.land': 'LAND',
  'auto.rtl': 'RETURN',
  'auto.takeoff': 'TAKEOFF',
  manual: 'MANUAL',
  posctl: 'POSITION',
  altctl: 'ALTITUDE',
};

/** Horizontal distance from EKF2's estimate to the truth, m. */
export function ekfError(frame: FlightFrame): number | null {
  const s = frame.px4;
  if (!s) return null;
  return Math.hypot(s.ekf[0] - frame.pos[0], s.ekf[1] - frame.pos[1]);
}

export class Px4Hud {
  readonly root: HTMLElement;
  private readonly mode: HTMLElement;
  private readonly armed: HTMLElement;
  private readonly nav: HTMLElement;
  private readonly navText: HTMLElement;
  private readonly err: HTMLElement;
  private readonly errNote: HTMLElement;
  private readonly spark: HTMLCanvasElement;
  private readonly horizon: HTMLCanvasElement;
  private readonly roll: HTMLElement;
  private readonly pitch: HTMLElement;
  private readonly bars: HTMLElement[] = [];
  private readonly track: HTMLElement;
  /** Per frame: EKF2 horizontal error (m), or -1 with no PX4 sample. */
  private readonly errors: Float32Array;

  constructor(
    parent: HTMLElement,
    private readonly log: FlightLog,
    private readonly facts: Px4FlightFacts,
  ) {
    this.errors = new Float32Array(log.frames.length);
    log.frames.forEach((f, i) => (this.errors[i] = ekfError(f) ?? -1));

    this.root = el('div', 'px4-hud', parent);
    const head = el('div', 'px4-head', this.root);
    el('div', 'px4-mark', head, 'PX4');
    const headText = el('div', '', head);
    el('div', 'px4-kicker', headText, 'FLIGHT CONTROLLER IN THE LOOP');
    el('div', 'px4-sub', headText, `${facts.autopilot} · lockstep ${facts.lockstepHz} Hz · setpoints ${facts.offboardHz} Hz`);

    const chips = el('div', 'px4-chips', this.root);
    this.mode = el('div', 'px4-chip px4-mode', chips, 'OFFBOARD');
    this.armed = el('div', 'px4-chip px4-armed', chips, 'ARMED');
    this.nav = el('div', 'px4-nav', this.root);
    el('span', 'px4-nav-dot', this.nav);
    this.navText = el('span', '', this.nav, 'GNSS');

    const ekf = el('div', 'px4-block', this.root);
    el('div', 'px4-label', ekf, 'EKF2 POSITION ESTIMATE VS TRUTH');
    const errRow = el('div', 'px4-err-row', ekf);
    this.err = el('div', 'px4-err', errRow, '0.00 m');
    this.errNote = el('div', 'px4-err-note', errRow, '');
    this.spark = el('canvas', 'px4-spark', ekf);
    this.spark.width = 640;
    this.spark.height = 110;

    const lower = el('div', 'px4-lower', this.root);
    const att = el('div', 'px4-att', lower);
    el('div', 'px4-label', att, 'ATTITUDE');
    this.horizon = el('canvas', 'px4-horizon', att);
    this.horizon.width = 240;
    this.horizon.height = 240;
    const angles = el('div', 'px4-angles', att);
    this.roll = el('span', '', angles, 'R 0.0°');
    this.pitch = el('span', '', angles, 'P 0.0°');

    const motors = el('div', 'px4-motors', lower);
    el('div', 'px4-label', motors, 'MOTORS');
    const barRow = el('div', 'px4-bars', motors);
    for (let i = 0; i < 4; i++) {
      const col = el('div', 'px4-bar-col', barRow);
      const bar = el('div', 'px4-bar', col);
      this.bars.push(el('div', 'px4-bar-fill', bar));
      el('div', 'px4-bar-label', col, String(i + 1));
    }
    this.track = el('div', 'px4-track', motors, '');
  }

  update(frame: FlightFrame, index: number, t: number): void {
    const s = frame.px4;
    this.root.classList.toggle('px4-off', !s);
    if (!s) return;
    this.mode.textContent = MODE_LABELS[s.mode] ?? s.mode.toUpperCase();
    this.armed.textContent = s.armed ? 'ARMED' : 'DISARMED';
    this.armed.classList.toggle('px4-disarmed', !s.armed);
    this.nav.classList.toggle('px4-denied', s.denied);
    this.navText.textContent = s.denied ? (s.vision ? 'GNSS DENIED · EKF2 ON VISION' : 'GNSS DENIED · DEAD RECKONING') : 'GNSS FIX · EKF2 ON GPS';
    const e = ekfError(frame) ?? 0;
    this.err.textContent = `${e.toFixed(2)} m`;
    this.errNote.textContent = `flight RMS ${this.facts.ekfHorizErrRmsM?.toFixed(2) ?? '—'} m · worst ${this.facts.ekfHorizErrMaxM.toFixed(2)} m`;
    this.drawSpark(index, t);
    this.drawHorizon(s);
    this.roll.textContent = `ROLL ${((s.roll * 180) / Math.PI).toFixed(1)}°`;
    // FLU pitch is positive nose down; pilots read nose up as positive.
    this.pitch.textContent = `PITCH ${((-s.pitch * 180) / Math.PI).toFixed(1)}°`;
    s.motors.forEach((m, i) => {
      this.bars[i]!.style.height = `${Math.max(2, Math.min(100, m * 100))}%`;
      this.bars[i]!.classList.toggle('px4-sat', m > 0.97);
    });
    const sp = Math.hypot(s.sp[0], s.sp[1]);
    const v = Math.hypot(frame.vel[0], frame.vel[1]);
    this.track.textContent = `ASKED ${sp.toFixed(1)} m/s · FLYING ${v.toFixed(1)} m/s`;
  }

  /** EKF2 error over the last minute, the GNSS-denied stretches shaded. */
  private drawSpark(index: number, t: number): void {
    const g = this.spark.getContext('2d')!;
    const w = this.spark.width;
    const h = this.spark.height;
    g.clearRect(0, 0, w, h);
    const frames = this.log.frames;
    const dt = this.log.dt;
    const span = Math.round(HISTORY_SEC / dt);
    const i0 = Math.max(0, index - span);
    const top = 1.5;
    const x = (i: number) => w - ((index - i) / span) * w;
    const y = (m: number) => h - 6 - (Math.min(top, m) / top) * (h - 14);
    // Denied stretches.
    g.fillStyle = 'rgba(255, 209, 102, 0.13)';
    let start = -1;
    for (let i = i0; i <= index + 1; i++) {
      const denied = i <= index && frames[i]?.px4?.denied === true;
      if (denied && start < 0) start = i;
      if (!denied && start >= 0) {
        g.fillRect(x(start), 0, x(i) - x(start), h);
        start = -1;
      }
    }
    // Gridline at 0.5 m and 1 m.
    g.strokeStyle = 'rgba(255,255,255,0.1)';
    g.lineWidth = 1;
    g.font = '600 18px "JetBrains Mono", ui-monospace, monospace';
    g.fillStyle = 'rgba(255,255,255,0.4)';
    for (const m of [0.5, 1]) {
      g.beginPath();
      g.moveTo(0, y(m));
      g.lineTo(w, y(m));
      g.stroke();
      g.fillText(`${m} m`, 6, y(m) - 4);
    }
    g.strokeStyle = '#56e0ff';
    g.lineWidth = 3;
    g.beginPath();
    let pen = false;
    const stride = Math.max(1, Math.floor(span / w));
    for (let i = i0; i <= index; i += stride) {
      const e = this.errors[i]!;
      if (e < 0) {
        pen = false;
        continue;
      }
      if (pen) g.lineTo(x(i), y(e));
      else g.moveTo(x(i), y(e));
      pen = true;
    }
    g.stroke();
    void t;
  }

  private drawHorizon(s: Px4Sample): void {
    const g = this.horizon.getContext('2d')!;
    const w = this.horizon.width;
    const r = w / 2 - 6;
    g.clearRect(0, 0, w, w);
    g.save();
    g.translate(w / 2, w / 2);
    g.beginPath();
    g.arc(0, 0, r, 0, Math.PI * 2);
    g.clip();
    // Right side down turns the horizon counter-clockwise on the display; nose down (FLU pitch
    // positive) lifts it, showing more ground.
    g.rotate(-s.roll);
    const pitchPx = ((-s.pitch * 180) / Math.PI) * (r / 30);
    g.fillStyle = '#1b4f73';
    g.fillRect(-w, -w * 2 + pitchPx, w * 2, w * 2);
    g.fillStyle = '#5a3d22';
    g.fillRect(-w, pitchPx, w * 2, w * 2);
    g.strokeStyle = 'rgba(255,255,255,0.85)';
    g.lineWidth = 3;
    g.beginPath();
    g.moveTo(-w, pitchPx);
    g.lineTo(w, pitchPx);
    g.stroke();
    g.lineWidth = 2;
    g.strokeStyle = 'rgba(255,255,255,0.5)';
    for (const deg of [-20, -10, 10, 20]) {
      const yy = pitchPx - deg * (r / 30);
      const half = deg % 20 === 0 ? 34 : 20;
      g.beginPath();
      g.moveTo(-half, yy);
      g.lineTo(half, yy);
      g.stroke();
    }
    g.restore();
    // Fixed aircraft symbol and bezel.
    g.save();
    g.translate(w / 2, w / 2);
    g.strokeStyle = '#ffd166';
    g.lineWidth = 5;
    g.beginPath();
    g.moveTo(-58, 0);
    g.lineTo(-20, 0);
    g.lineTo(-10, 10);
    g.moveTo(58, 0);
    g.lineTo(20, 0);
    g.lineTo(10, 10);
    g.stroke();
    g.fillStyle = '#ffd166';
    g.beginPath();
    g.arc(0, 0, 4, 0, Math.PI * 2);
    g.fill();
    g.strokeStyle = 'rgba(255,255,255,0.25)';
    g.lineWidth = 3;
    g.beginPath();
    g.arc(0, 0, r, 0, Math.PI * 2);
    g.stroke();
    g.restore();
  }
}
