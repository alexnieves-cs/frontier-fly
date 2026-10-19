import eslint from '@eslint/js';
import tseslint from 'typescript-eslint';

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
      '.claude/**',
    ],
  },
  {
    files: [
      'src/gen/**/*.ts',
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
