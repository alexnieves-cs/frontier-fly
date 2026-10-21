import type { RNG } from '../shared/rng';
import type { Point3, RadioModel } from './radio';

/** Map entry kinds carried over the radio. */
const SEEN = 0;
const VISITED = 1;
/** Bytes per map entry on the wire: a 3-byte voxel index and a kind bit, packed. */
export const ENTRY_BYTES = 3;
/** Bytes per message header: sender, pose, current goal, sequence number. */
export const HEADER_BYTES = 24;

/** What a node last heard directly from another node. */
export interface PeerState {
  pos: [number, number, number];
  /** The peer's current frontier goal, world xy, or null. */
  goal: [number, number] | null;
  /** Team step the message was sent. */
  at: number;
}

/**
 * One radio on the team: a drone or the base. It holds the team map as far
 * as this node knows it (seen-free and flown voxels, same layout as SeenMap),
 * and an append-only log of every voxel it learned, in order. A peer's cursor
 * is how much of the log has reached that peer, so a lost packet is resent
 * next round and nothing is sent twice once it arrives.
 */
export class MeshNode {
  readonly seen: Uint8Array;
  readonly visited: Uint8Array;
  /** Entries: voxel * 2 + kind. */
  private log = new Int32Array(1024);
  private logLen = 0;
  /** Per peer id: log entries already delivered to that peer. */
  readonly cursor: number[];
  /** Per peer id: what this node last heard directly from it (null before the first packet). */
  readonly heard: (PeerState | null)[];
  pos: [number, number, number] = [0, 0, 0];
  goal: [number, number] | null = null;

  constructor(
    readonly id: number,
    size: number,
    peers: number,
  ) {
    this.seen = new Uint8Array(size);
    this.visited = new Uint8Array(size);
    this.cursor = new Array<number>(peers).fill(0);
    this.heard = new Array<PeerState | null>(peers).fill(null);
  }

  get known(): number {
    return this.logLen;
  }

  private append(entry: number): void {
    if (this.logLen === this.log.length) {
      const next = new Int32Array(this.log.length * 2);
      next.set(this.log);
      this.log = next;
    }
    this.log[this.logLen++] = entry;
  }

  /** Merge this drone's own maps (its env's seen and visited grids). Returns voxels new to this node. */
  absorbOwn(seen: Uint8Array, visited: Uint8Array): number {
    let added = 0;
    const s = this.seen;
    const v = this.visited;
    for (let k = 0; k < s.length; k++) {
      if (seen[k] === 1 && s[k] === 0) {
        s[k] = 1;
        this.append(k * 2 + SEEN);
        added += 1;
      }
      if (visited[k] === 1 && v[k] === 0) {
        v[k] = 1;
        this.append(k * 2 + VISITED);
        added += 1;
      }
    }
    return added;
  }

  /** Apply received entries; new ones join this node's log so they are forwarded on (multi-hop). */
  receive(entries: Int32Array): number {
    let added = 0;
    for (let i = 0; i < entries.length; i++) {
      const e = entries[i]!;
      const k = e >> 1;
      const grid = (e & 1) === VISITED ? this.visited : this.seen;
      if (grid[k] === 0) {
        grid[k] = 1;
        this.append(e);
        added += 1;
      }
    }
    return added;
  }

  /** The next payload for a peer: unsent log entries, at most `max`. */
  pending(peer: number, max: number): Int32Array {
    const from = this.cursor[peer]!;
    return this.log.subarray(from, Math.min(this.logLen, from + max));
  }
}

export interface MeshStats {
  sent: number;
  delivered: number;
  bytes: number;
}

/**
 * One exchange round: every node sends one packet to every other node. Each
 * packet is the sender's pose and goal plus the map entries that peer has not
 * yet received from it, capped at `maxEntries`. Delivery is drawn per packet
 * from the radio model; on delivery the sender's cursor for that peer moves on
 * (a link-layer acknowledgement).
 */
export function exchange(nodes: MeshNode[], radio: RadioModel, rng: RNG, step: number, maxEntries: number, stats: MeshStats, perfect = false): void {
  // Draw every link first, then apply: what a node receives this round is forwarded next round, not instantly.
  const deliveries: { from: MeshNode; to: MeshNode; entries: Int32Array }[] = [];
  for (const a of nodes) {
    for (const b of nodes) {
      if (a === b) continue;
      const entries = a.pending(b.id, maxEntries);
      stats.sent += 1;
      stats.bytes += HEADER_BYTES + entries.length * ENTRY_BYTES;
      const ok = perfect || radio.deliver(a.pos as Point3, b.pos as Point3, rng);
      if (!ok) continue;
      stats.delivered += 1;
      deliveries.push({ from: a, to: b, entries: entries.slice() });
      a.cursor[b.id] = a.cursor[b.id]! + entries.length;
    }
  }
  for (const { from, to, entries } of deliveries) {
    to.receive(entries);
    to.heard[from.id] = { pos: [...from.pos], goal: from.goal ? [from.goal[0], from.goal[1]] : null, at: step };
  }
}
