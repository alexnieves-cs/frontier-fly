export default () => ({
  name: 'hangs',
  reset() {},
  act(): Float32Array {
    for (;;) {
      /* never returns */
    }
  },
});
