import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { defineConfig } from 'vite';
import { COOP_COEP_HEADERS } from './coopHeaders';

const root = fileURLToPath(new URL('.', import.meta.url));

export default defineConfig({
  base: './',
  server: {
    host: '127.0.0.1',
    port: 3000,
    headers: COOP_COEP_HEADERS,
  },
  preview: {
    host: '127.0.0.1',
    port: 4173,
    headers: COOP_COEP_HEADERS,
  },
  worker: {
    format: 'es',
  },
  optimizeDeps: {
    exclude: ['@dimforge/rapier3d-compat'],
  },
  build: {
    target: 'es2022',
    sourcemap: false,
    chunkSizeWarningLimit: 2500,
    rollupOptions: {
      input: {
        main: resolve(root, 'index.html'),
        mission: resolve(root, 'mission.html'),
        outdoor: resolve(root, 'outdoor.html'),
      },
    },
  },
});
