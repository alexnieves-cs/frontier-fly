import { describe, expect, it } from 'vitest';
import { QUAD_BRAKING, safeSpeed } from '../../src/sim/lidarSafety';
import { BRAKING_SEPARATION, separateBraking } from '../../src/team/run';

const still = [0, 0, 0];

describe('separateBraking', () => {
  it('leaves a command alone when the teammate is far or on another floor', () => {
    expect(separateBraking(1, 0, [0, 0, 1], [1, 0, 0], [{ pos: [6, 0, 1], vel: still }], QUAD_BRAKING)).toEqual([1, 0]);
    expect(separateBraking(1, 0, [0, 0, 1], [1, 0, 0], [{ pos: [1, 0, 4], vel: still }], QUAD_BRAKING)).toEqual([1, 0]);
  });

  it('caps the approach to half the stopping budget before the keep-out', () => {
    const d = 2;
    const [vx, vy] = separateBraking(3, 0, [0, 0, 1], still, [{ pos: [d, 0, 1], vel: still }], QUAD_BRAKING);
    expect(vx).toBeCloseTo(safeSpeed(d - BRAKING_SEPARATION, QUAD_BRAKING) / 2, 6);
    expect(vy).toBeCloseTo(0, 9);
  });

  it('keeps motion across the line between the drones', () => {
    const [vx, vy] = separateBraking(3, 1.2, [0, 0, 1], still, [{ pos: [2, 0, 1], vel: still }], QUAD_BRAKING);
    expect(vx).toBeLessThan(3);
    expect(vy).toBeCloseTo(1.2, 9);
  });

  it('cuts the budget further when the pair is already closing faster than it can stop', () => {
    const d = 1.5;
    const cap = safeSpeed(d - BRAKING_SEPARATION, QUAD_BRAKING);
    const slow = separateBraking(3, 0, [0, 0, 1], still, [{ pos: [d, 0, 1], vel: still }], QUAD_BRAKING)[0];
    const fast = separateBraking(3, 0, [0, 0, 1], [cap + 1, 0, 0], [{ pos: [d, 0, 1], vel: still }], QUAD_BRAKING)[0];
    expect(fast).toBeLessThan(slow);
  });

  it('backs away inside the keep-out', () => {
    const [vx] = separateBraking(1, 0, [0, 0, 1], still, [{ pos: [BRAKING_SEPARATION - 0.1, 0, 1], vel: still }], QUAD_BRAKING);
    expect(vx).toBeLessThan(0);
  });

  it('two drones flying at each other both slow, and never close faster than they can stop', () => {
    const a = separateBraking(3, 0, [0, 0, 1], still, [{ pos: [2, 0, 1], vel: still }], QUAD_BRAKING)[0];
    const b = separateBraking(-3, 0, [2, 0, 1], still, [{ pos: [0, 0, 1], vel: still }], QUAD_BRAKING)[0];
    expect(a - b).toBeLessThanOrEqual(safeSpeed(2 - BRAKING_SEPARATION, QUAD_BRAKING) + 1e-9);
  });
});
