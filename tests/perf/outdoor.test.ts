import { describe, expect, it } from 'vitest';
import { generateOutdoor } from '../../src/world/outdoor/generate';
import { initOutdoorPhysics, OutdoorPhysics } from '../../src/world/outdoor/physics';

describe('outdoor perf', () => {
  it('generates a world quickly and steps lidar in real time', async () => {
    const samples: number[] = [];
    for (let i = 0; i < 5; i++) {
      const t0 = performance.now();
      generateOutdoor({ seed: 20 + i });
      samples.push(performance.now() - t0);
    }
    samples.sort((a, b) => a - b);
    const median = samples[2]!;
    console.info(`outdoor gen median ${median.toFixed(1)} ms`);
    expect(median).toBeLessThan(250);

    await initOutdoorPhysics();
    const world = generateOutdoor({ seed: 4 });
    const physics = new OutdoorPhysics(world);
    try {
      let pos: [number, number, number] = [...world.spawn];
      const delta: [number, number, number] = [0.4, 0.05, 0];
      for (let i = 0; i < 8; i++) pos = physics.move(pos, delta).pos;
      const frames = 90;
      const t0 = performance.now();
      for (let i = 0; i < frames; i++) {
        physics.lidar(pos, 0.2);
        pos = physics.move(pos, delta).pos;
      }
      const ms = (performance.now() - t0) / frames;
      const fps = 1000 / ms;
      console.info(`outdoor headless frame ${ms.toFixed(2)} ms (${fps.toFixed(0)} fps), colliders ${world.colliders.length}, trees ${world.trees.length}`);
      expect(fps).toBeGreaterThan(30);
    } finally {
      physics.free();
    }
  });
});