import { describe, expect, it } from 'vitest';
import { CommandShaper } from '../../src/sim/tracker';

describe('CommandShaper.hold', () => {
  it('ramps from a held command even before the first shape call', () => {
    const s = new CommandShaper();
    s.hold(2, 0);
    // Without priming on hold, the first shape call would restart from the measured velocity (0, 0).
    const [vx, vy] = s.shape(0, 0, 4, 0.05, [0, 0]);
    expect(vx).toBeCloseTo(1.8);
    expect(vy).toBeCloseTo(0);
  });
});
