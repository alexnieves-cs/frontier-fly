import { beforeAll, describe, expect, it } from 'vitest';
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { applyHazards } from '../../src/gen/hazards';
import { SensorExplorer } from '../../src/mission/autonomy/planner/indoor/explorers';
import { indoorGrid } from '../../src/mission/autonomy/planner/indoor/grid';
import { IndoorSensing } from '../../src/mission/autonomy/planner/indoor/sensing';
import { defaultLighting } from '../../src/sim/lighting';
import { initPhysics, PhysicsWorld } from '../../src/sim/world';

beforeAll(async () => {
  await initPhysics();
});

describe('look phase in smoke', () => {
  // Practice seed 81910: the map promised a view that smoke hides, and the explorer used to turn
  // at the same spot 137 times (609 s) before giving the floor up.
  it('does not turn at the same spot again after a turn that showed nothing', () => {
    const spec = applyHazards(evalEpisodeSetup(3, 81910, 'day').spec, { smokeRooms: 0.6 });
    expect(spec.hazards?.smoke).toBeTruthy();
    const grid = indoorGrid({ shell: { spec, origin: [0, 0, 0] } } as never);
    const rooms = spec.floors.reduce((n, f) => n + f.rooms.length, 0);
    const sensing = new IndoorSensing(grid, new PhysicsWorld(spec), defaultLighting(rooms), 81910);
    const explorer = new SensorExplorer(grid, sensing, 1, 'sweep');
    const pos: [number, number, number] = [spec.spawn[0], spec.spawn[1], 1.2];
    let vel: [number, number, number] = [0, 0, 0];
    let yaw = 0;
    const dt = 0.05;
    const turns = new Map<string, number>();
    let turning = false;
    let k = 0;
    for (; k < 24000 && !explorer.exhausted; k++) {
      const cmd = explorer.act(pos, vel, yaw, k * dt);
      const turn = explorer.phaseName === 'look' && cmd.vel[0] === 0 && cmd.vel[1] === 0 && Math.abs(cmd.yawRate - Math.PI / 2) < 1e-9;
      if (turn && !turning) {
        const key = `${pos[0].toFixed(1)},${pos[1].toFixed(1)}`;
        turns.set(key, (turns.get(key) ?? 0) + 1);
      }
      turning = turn;
      vel = [cmd.vel[0], cmd.vel[1], cmd.vel[2]];
      pos[0] += vel[0] * dt;
      pos[1] += vel[1] * dt;
      pos[2] += vel[2] * dt;
      yaw += cmd.yawRate * dt;
    }
    expect(explorer.exhausted).toBe(true);
    expect(Math.max(0, ...turns.values())).toBeLessThanOrEqual(1);
    expect(k * dt).toBeLessThan(540);
  });
});
