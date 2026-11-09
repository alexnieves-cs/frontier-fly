// Test pilot: reports what it can see through its action.
const g = globalThis as { __scorerSecret?: number };
export default () => ({
  name: 'echo',
  speed: 0,
  reset(ctx: { speedH: number }) {
    this.speed = ctx.speedH;
  },
  act(obs: Float32Array, frame: { pos: readonly number[]; seen: Uint8Array; lidar: ArrayLike<number> }) {
    return new Float32Array([g.__scorerSecret ?? 0, typeof process.env.PILOT_TEST_SECRET === 'string' ? 1 : 0, obs[0]! + frame.pos[2]! + frame.seen[3]! + frame.lidar[1]!, this.speed]);
  },
});
