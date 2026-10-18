/** Fixed-capacity rollout storage. Observations and actions are copied in. */

export interface PpoBatch {
  obs: Float32Array;
  actions: Float32Array;
  logp: Float32Array;
  advantages: Float32Array;
  returns: Float32Array;
  n: number;
}

export interface RolloutArrays {
  obs: Float32Array;
  actions: Float32Array;
  logp: Float32Array;
  values: Float32Array;
  rewards: Float32Array;
  dones: Uint8Array;
  truncated: Uint8Array;
  n: number;
}

export class RolloutBuffer {
  readonly capacity: number;
  readonly obsDim: number;
  readonly actDim: number;
  private readonly obs: Float32Array;
  private readonly actions: Float32Array;
  private readonly logp: Float32Array;
  private readonly values: Float32Array;
  private readonly rewards: Float32Array;
  private readonly dones: Uint8Array;
  private readonly truncated: Uint8Array;
  private count = 0;

  constructor(capacity: number, obsDim: number, actDim: number) {
    if (capacity <= 0 || obsDim <= 0 || actDim <= 0) {
      throw new Error('RolloutBuffer sizes must be positive');
    }
    this.capacity = capacity;
    this.obsDim = obsDim;
    this.actDim = actDim;
    this.obs = new Float32Array(capacity * obsDim);
    this.actions = new Float32Array(capacity * actDim);
    this.logp = new Float32Array(capacity);
    this.values = new Float32Array(capacity);
    this.rewards = new Float32Array(capacity);
    this.dones = new Uint8Array(capacity);
    this.truncated = new Uint8Array(capacity);
  }

  get size(): number {
    return this.count;
  }

  reset(): void {
    this.count = 0;
  }

  add(
    obs: Float32Array,
    action: Float32Array,
    logp: number,
    value: number,
    reward: number,
    done: boolean,
    truncated: boolean,
  ): void {
    if (this.count >= this.capacity) throw new Error('RolloutBuffer full');
    const i = this.count;
    this.obs.set(obs.subarray(0, this.obsDim), i * this.obsDim);
    this.actions.set(action.subarray(0, this.actDim), i * this.actDim);
    this.logp[i] = logp;
    this.values[i] = value;
    this.rewards[i] = reward;
    this.dones[i] = done ? 1 : 0;
    this.truncated[i] = truncated ? 1 : 0;
    this.count += 1;
  }

  /** Copies of the filled prefix. */
  toArrays(): RolloutArrays {
    const n = this.count;
    return {
      obs: this.obs.slice(0, n * this.obsDim),
      actions: this.actions.slice(0, n * this.actDim),
      logp: this.logp.slice(0, n),
      values: this.values.slice(0, n),
      rewards: this.rewards.slice(0, n),
      dones: this.dones.slice(0, n),
      truncated: this.truncated.slice(0, n),
      n,
    };
  }
}
