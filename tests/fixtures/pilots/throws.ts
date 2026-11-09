export default () => ({
  name: 'throws',
  reset() {},
  act(): Float32Array {
    throw new Error('pilot blew up');
  },
});
