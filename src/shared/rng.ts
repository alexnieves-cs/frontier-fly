/** sfc32 seeded PRNG. `fork(label)` is deterministic and does not consume the parent stream. */

function xmur3(str: string): () => number {
  let h = 1779033703 ^ str.length;
  for (let i = 0; i < str.length; i++) {
    h = Math.imul(h ^ str.charCodeAt(i), 3432918353);
    h = (h << 13) | (h >>> 19);
  }
  return () => {
    h = Math.imul(h ^ (h >>> 16), 2246822507);
    h = Math.imul(h ^ (h >>> 13), 3266489909);
    h ^= h >>> 16;
    return h >>> 0;
  };
}

function sfc32(a: number, b: number, c: number, d: number): () => number {
  return () => {
    a >>>= 0;
    b >>>= 0;
    c >>>= 0;
    d >>>= 0;
    const t = (a + b) | 0;
    a = b ^ (b >>> 9);
    b = (c + (c << 3)) | 0;
    c = ((c << 21) | (c >>> 11)) | 0;
    d = (d + 1) | 0;
    const t2 = (t + d) | 0;
    c = (c + t2) | 0;
    return (t2 >>> 0) / 4294967296;
  };
}

export class RNG {
  readonly seed: number;
  readonly label: string;
  private nextFn: () => number;
  private forks = 0;

  constructor(seed: number, label = '') {
    this.seed = seed >>> 0;
    this.label = label;
    const mix = xmur3(`${this.seed}::${label}`);
    this.nextFn = sfc32(mix(), mix(), mix(), mix());
  }

  /** Uniform in [0, 1). */
  next(): number {
    return this.nextFn();
  }

  /** Independent child stream. Call order of `next` does not change the child. */
  fork(label: string): RNG {
    const child = new RNG(this.seed ^ Math.imul(this.forks + 1, 0x9e3779b1), `${this.label}/${label}`);
    this.forks += 1;
    return child;
  }

  int(maxExclusive: number): number {
    if (maxExclusive <= 0) return 0;
    return Math.floor(this.next() * maxExclusive);
  }

  range(min: number, max: number): number {
    return min + (max - min) * this.next();
  }

  bool(p = 0.5): boolean {
    return this.next() < p;
  }

  /** Standard normal via Box-Muller. */
  gaussian(): number {
    const u1 = Math.max(this.next(), 1e-12);
    const u2 = this.next();
    return Math.sqrt(-2 * Math.log(u1)) * Math.cos(Math.PI * 2 * u2);
  }

  pick<T>(items: readonly T[]): T {
    return items[this.int(items.length)]!;
  }
}

export function hashSeed(text: string): number {
  const mix = xmur3(text);
  return mix();
}
