import { describe, expect, it } from 'vitest';
import { CELL } from '../../src/shared/constants';
import { generateBuilding } from '../../src/gen/building';
import { makeDroneState } from '../../src/sim/drone';
import { boomScale } from '../../src/viz/cameras/chase';
import { readLaunchParams } from '../../src/ui/controls';
import { ReplayBuffer, replayFrame } from '../../src/ui/replay';

describe('launch params', () => {
  it('reads a shareable seed, camera, and shot preset', () => {
    const day = readLaunchParams('?seed=7&shot=day');
    expect(day.seed).toBe(7);
    expect(day.camera).toBe('chase');
    expect(day.tod).toBe(13);
    expect(day.flashlight).toBe(false);
    const night = readLaunchParams('?shot=fpv&seed=3');
    expect(night.camera).toBe('fpv');
    expect(night.flashlight).toBe(true);
    expect(night.tod).toBe(22);
    expect(night.pilot).toBe('heuristic');
  });
});

describe('replay', () => {
  it('plays a stored episode deterministically', () => {
    const buffer = new ReplayBuffer();
    for (let i = 0; i < 6; i++) {
      const state = makeDroneState([i, 0, 1], i * 0.1);
      buffer.note(state, i, i + 1);
    }
    buffer.close();
    const ep = buffer.pick('latest');
    expect(ep).not.toBeNull();
    const a = replayFrame(ep!, 2, 1, 0);
    const b = replayFrame(ep!, 2, 1, 0);
    expect(a.pos).toEqual(b.pos);
    expect(a.pos[0]).toBe(2);
    expect(buffer.pick('best')?.reward).toBe(5);
  });
});

describe('chase boom', () => {
  it('shortens the boom before a wall cell', () => {
    const spec = generateBuilding({ seed: 2, floors: 1, w: 12, d: 12, stage: 0 });
    const floor = spec.floors[0]!;
    let wall: [number, number] | null = null;
    for (let z = 0; z < floor.d && !wall; z++) {
      for (let x = 0; x < floor.w; x++) {
        if (floor.cells[z * floor.w + x] === CELL.WALL) {
          wall = [x + 0.5, z + 0.5];
          break;
        }
      }
    }
    expect(wall).not.toBeNull();
    const scale = boomScale(spec, [wall![0] - 0.2, wall![1], 1], [wall![0] + 1.5, wall![1], 1]);
    expect(scale).toBeLessThan(1);
    expect(scale).toBeGreaterThan(0);
  });
});
