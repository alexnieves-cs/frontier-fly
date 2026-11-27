// Test pilot: echoes the overwatch sighting and the pose health it is handed, and reports a protection level.
export default () => {
  let level = 0;
  return {
    name: 'integrity',
    reset() {},
    act(_obs: Float32Array, frame: { person?: { range: number } | null; health?: { sigmaM: number; state: string } }) {
      level = frame.health ? 4 * frame.health.sigmaM : 0;
      const person = frame.person === undefined ? -1 : frame.person === null ? 0 : frame.person.range;
      return new Float32Array([person, frame.health?.state === 'degraded' ? 1 : 0, frame.health?.sigmaM ?? -1, 0]);
    },
    protectionLevel() {
      return level;
    },
  };
};
