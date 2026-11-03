import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    environment: 'node',
    include: ['tests/unit/**/*.test.ts', 'tests/perf/**/*.test.ts'],
    testTimeout: 180_000,
    hookTimeout: 180_000,
    fileParallelism: false,
    // Used by `npm run test:coverage` only; plain `npm test` does not instrument.
    coverage: {
      provider: 'v8',
      include: ['src/**/*.ts'],
      reporter: ['text-summary', 'json-summary'],
      reportsDirectory: 'coverage',
      // A floor about one point under the 2026-11-02 numbers (lines 61.1, statements 60.4,
      // functions 56.8, branches 57.9) so coverage can only rise. Raise it when it does.
      thresholds: { lines: 60, statements: 59, functions: 55, branches: 56 },
    },
  },
});
