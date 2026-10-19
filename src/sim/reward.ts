import { COVERAGE_SUCCESS, REWARD, SUCCESS_BONUS } from '../shared/constants';

export interface RewardInput {
  newVoxels: number;
  newRoom: boolean;
  newFloor: boolean;
  impact: boolean;
  inContact: boolean;
  action: ArrayLike<number>;
  prevAction: ArrayLike<number>;
  coverage: number;
}

export function rawReward(input: RewardInput): number {
  let reward = -REWARD.time;
  reward += input.newVoxels * REWARD.newVoxel;
  if (input.newRoom) reward += REWARD.newRoom;
  if (input.newFloor) reward += REWARD.newFloor;
  if (input.impact) reward += REWARD.collision;
  else if (input.inContact) reward += REWARD.contact;
  let smooth = 0;
  let effort = 0;
  for (let i = 0; i < 4; i++) {
    const a = input.action[i] ?? 0;
    const p = input.prevAction[i] ?? 0;
    const d = a - p;
    smooth += d * d;
    effort += a * a;
  }
  reward -= REWARD.smooth * smooth;
  reward -= REWARD.effort * effort;
  if (input.coverage >= COVERAGE_SUCCESS) reward += SUCCESS_BONUS;
  return reward;
}

export interface ProgressPenalty {
  /** Grows with steps since the last new voxel beyond REWARD.stuckGrace. */
  stuck: number;
  /** Net displacement over REWARD.stillWindow steps below REWARD.stillDist while not covering. */
  still: number;
  /** Sign flips of the lateral / yaw commands this step. */
  flip: number;
  /** stuck + still + flip, as a positive number to subtract from the reward. */
  total: number;
}

/**
 * Tracks recent progress so the env can penalize stalling and dithering.
 * All penalties are returned as positive magnitudes; the caller subtracts them.
 * Lateral (index 1) and yaw (index 3) commands are checked for sign flips.
 */
export class ProgressTracker {
  private readonly window: number;
  private readonly xs: Float64Array;
  private readonly ys: Float64Array;
  private head = 0;
  private filled = 0;
  private sinceNew = 0;
  private prevLat = 0;
  private prevYaw = 0;

  constructor(window: number = REWARD.stillWindow) {
    this.window = Math.max(1, Math.floor(window));
    this.xs = new Float64Array(this.window);
    this.ys = new Float64Array(this.window);
  }

  reset(x: number, y: number): void {
    this.head = 0;
    this.filled = 0;
    this.sinceNew = 0;
    this.prevLat = 0;
    this.prevYaw = 0;
    this.push(x, y);
  }

  get stepsSinceNewVoxel(): number {
    return this.sinceNew;
  }

  /** Call once per env step, after the move, with the new xy position. */
  update(x: number, y: number, newVoxels: number, action: ArrayLike<number>): ProgressPenalty {
    if (newVoxels > 0) this.sinceNew = 0;
    else this.sinceNew += 1;
    let stuck = 0;
    if (this.sinceNew > REWARD.stuckGrace) {
      const frac = Math.min(1, (this.sinceNew - REWARD.stuckGrace) / REWARD.stuckRamp);
      stuck = REWARD.stuck * frac;
    }
    let still = 0;
    if (this.filled >= this.window && newVoxels === 0) {
      // Oldest entry is the position `window` steps ago.
      const ox = this.xs[this.head]!;
      const oy = this.ys[this.head]!;
      if (Math.hypot(x - ox, y - oy) < REWARD.stillDist) still = REWARD.still;
    }
    this.push(x, y);
    const lat = action[1] ?? 0;
    const yaw = action[3] ?? 0;
    let flips = 0;
    const dead = REWARD.flipDeadband;
    if (Math.abs(lat) > dead && Math.abs(this.prevLat) > dead && Math.sign(lat) !== Math.sign(this.prevLat)) flips += 1;
    if (Math.abs(yaw) > dead && Math.abs(this.prevYaw) > dead && Math.sign(yaw) !== Math.sign(this.prevYaw)) flips += 1;
    // Remember the last command that cleared the deadband so slow dithering through zero still counts.
    if (Math.abs(lat) > dead) this.prevLat = lat;
    if (Math.abs(yaw) > dead) this.prevYaw = yaw;
    const flip = flips * REWARD.flip;
    return { stuck, still, flip, total: stuck + still + flip };
  }

  private push(x: number, y: number): void {
    this.xs[this.head] = x;
    this.ys[this.head] = y;
    this.head = (this.head + 1) % this.window;
    if (this.filled < this.window) this.filled += 1;
  }
}
