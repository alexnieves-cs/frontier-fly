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
