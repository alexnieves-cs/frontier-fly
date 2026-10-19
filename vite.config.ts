import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { defineConfig, type Plugin } from 'vite';
import { COOP_COEP_HEADERS } from './coopHeaders';

const root = fileURLToPath(new URL('.', import.meta.url));

/**
 * GitHub Pages cannot send COOP/COEP headers, so the built pages load
 * public/coi-serviceworker.js first. It re-serves the site with those headers
 * and SharedArrayBuffer becomes available. The dev and preview servers already
 * send the headers, and the worker does nothing when the page is isolated.
 */
function crossOriginIsolation(): Plugin {
  return {
    name: 'coi-serviceworker',
    apply: 'build',
    transformIndexHtml: () => [{ tag: 'script', attrs: { src: './coi-serviceworker.js' }, injectTo: 'head-prepend' }],
  };
}

export default defineConfig({
  base: './',
  plugins: [crossOriginIsolation()],
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
        sim: resolve(root, 'sim.html'),
        mission: resolve(root, 'mission.html'),
        outdoor: resolve(root, 'outdoor.html'),
        demo: resolve(root, 'demo.html'),
        film: resolve(root, 'film.html'),
      },
    },
  },
});
