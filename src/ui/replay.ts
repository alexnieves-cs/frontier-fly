import type { DroneState } from '../shared/types';
import { cloneDroneState } from '../sim/drone';

export interface StoredEpisode {
  label: 'latest' | 'best' | 'worst';
  reward: number;
  states: DroneState[];
}

/** Ring of latest / best / worst showcase episodes. Playback is just indexing stored states. */
export class ReplayBuffer {
  latest: StoredEpisode | null = null;
  best: StoredEpisode | null = null;
  worst: StoredEpisode | null = null;
  private current: DroneState[] = [];
  private reward = 0;

  note(state: DroneState, reward: number, step: number): void {
    if (step <= 1 && this.current.length > 5) this.close();
    this.current.push(cloneDroneState(state));
    this.reward = reward;
    if (this.current.length > 2000) this.current.shift();
  }

  close(): void {
    if (this.current.length < 2) {
      this.current = [];
      return;
    }
    const ep: StoredEpisode = { label: 'latest', reward: this.reward, states: this.current };
    this.latest = ep;
    if (!this.best || ep.reward > this.best.reward) this.best = { ...ep, label: 'best', states: ep.states };
    if (!this.worst || ep.reward < this.worst.reward) this.worst = { ...ep, label: 'worst', states: ep.states.slice() };
    this.current = [];
  }

  pick(which: 'latest' | 'best' | 'worst'): StoredEpisode | null {
    return this[which];
  }
}

export function replayFrame(ep: StoredEpisode, index: number, speed: number, clock: number): DroneState {
  const i = Math.max(0, Math.min(ep.states.length - 1, Math.floor(index + clock * speed)));
  return ep.states[i]!;
}
