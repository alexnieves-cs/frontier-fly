import { OBS, OBS_DIM } from '../../shared/constants';

/**
 * Occlusion attribution for the policy: blank one input block at a time,
 * re-run the network, and measure how far the command moves. A block whose
 * blanking barely changes the command is not what the policy is acting on
 * right now. Blank values are what the observation reads when that sense has
 * nothing to report (src/sim/OBS_LAYOUT.md), not zeros everywhere.
 */

export interface InputBlock {
  id: string;
  label: string;
  /** [start, end) slots and the blank value written there. */
  spans: Array<[number, number, number]>;
}

export const INPUT_BLOCKS: InputBlock[] = [
  { id: 'lidar', label: 'Lidar', spans: [[OBS.lidar, OBS.vel, 1]] },
  { id: 'motion', label: 'Motion', spans: [[OBS.vel, OBS.sinYaw, 0], [OBS.roll, OBS.floor, 0]] },
  {
    id: 'nav',
    label: 'Frontier cue',
    spans: [
      [OBS.wpDir, OBS.pathLen, 0],
      [OBS.pathLen, OBS.hasFrontier, 1],
      [OBS.hasFrontier, OBS.clusterDist, 0],
      [OBS.clusterDist, OBS.clusterMass, 1],
      [OBS.clusterMass, OBS.frontierDist, 0],
      [OBS.frontierDist, OBS.localVisited, 1],
    ],
  },
  { id: 'local', label: 'Local map 8 m', spans: [[OBS.localVisited, OBS.coarseVisited, 0]] },
  { id: 'coarse', label: 'Coarse map 32 m', spans: [[OBS.coarseVisited, OBS.end, 0]] },
];

/** One batch: the observation itself, then one copy per block with that block blanked. */
export function occlusionBatch(obs: Float32Array, out = new Float32Array((INPUT_BLOCKS.length + 1) * OBS_DIM)): Float32Array {
  out.set(obs.subarray(0, OBS_DIM), 0);
  INPUT_BLOCKS.forEach((block, b) => {
    const base = (b + 1) * OBS_DIM;
    out.set(obs.subarray(0, OBS_DIM), base);
    for (const [start, end, value] of block.spans) out.fill(value, base + start, base + end);
  });
  return out;
}

/** Command shift per block: Euclidean distance between the blanked and the real action (each in [-1, 1]^4). */
export function influenceFromActions(actions: Float32Array, actDim = 4): Float32Array {
  const out = new Float32Array(INPUT_BLOCKS.length);
  for (let b = 0; b < INPUT_BLOCKS.length; b++) {
    let sum = 0;
    for (let j = 0; j < actDim; j++) {
      const d = actions[(b + 1) * actDim + j]! - actions[j]!;
      sum += d * d;
    }
    out[b] = Math.sqrt(sum);
  }
  return out;
}
