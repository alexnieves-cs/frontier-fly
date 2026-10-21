import eslint from '@eslint/js';
import tseslint from 'typescript-eslint';
import globals from 'globals';

export default tseslint.config(
  eslint.configs.recommended,
  ...tseslint.configs.recommended,
  {
    ignores: [
      'dist/**',
      'node_modules/**',
      'test-results/**',
      'playwright-report/**',
      'public/checkpoints/**',
      'public/coi-serviceworker.js',
      '.claude/**',
    ],
  },
  {
    // Node scripts. Browser globals are allowed for code passed to page.evaluate.
    files: ['scripts/media/**/*.mjs', 'scripts/docs/**/*.mjs', 'scripts/px4/**/*.mjs', 'scripts/check-pages.mjs'],
    languageOptions: { globals: { ...globals.node, ...globals.browser } },
  },
  {
    files: [
      'src/gen/**/*.ts',
      'src/plans/**/*.ts',
      'src/sim/**/*.ts',
      'src/rl/**/*.ts',
      'src/workers/env.worker.ts',
      'src/world/outdoor/**/*.ts',
    ],
    ignores: ['src/world/outdoor/viz/**'],
    rules: {
      'no-restricted-imports': [
        'error',
        {
          paths: [
            { name: 'three', message: 'Rendering is isolated to src/viz. Sim, gen, and rl must stay headless.' },
            { name: 'babylonjs', message: 'Rendering is isolated to src/viz.' },
          ],
          patterns: ['three/*', 'babylonjs/*'],
        },
      ],
      'no-restricted-globals': ['error', 'window', 'document', 'localStorage', 'navigator'],
    },
  },
);
