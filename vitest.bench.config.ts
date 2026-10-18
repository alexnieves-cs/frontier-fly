import { defineConfig } from 'vitest/config';

// `npm run bench` only. The default `npm test` config does not include bench/.
export default defineConfig({
  test: {
    environment: 'node',
    include: ['bench/**/*.test.ts'],
    testTimeout: 900_000,
    fileParallelism: false,
  },
});
