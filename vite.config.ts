import { existsSync, readdirSync, readFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
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

/**
 * The site bundles the runtime dependencies (three, rapier, tfjs, uplot) and what they depend on. Their
 * licenses ask for the notice to travel with the copy, so the build writes every one of them, with its
 * license text, to THIRD_PARTY_LICENSES.txt at the site root.
 */
function thirdPartyLicenses(): Plugin {
  return {
    name: 'third-party-licenses',
    apply: 'build',
    generateBundle() {
      type Entry = { name: string; version: string; license: string; author: string; text: string | null };
      const seen = new Map<string, Entry>();
      const visit = (name: string) => {
        if (seen.has(name) || name.startsWith('@types/')) return;
        const dir = join(root, 'node_modules', name);
        if (!existsSync(join(dir, 'package.json'))) return;
        const pkg = JSON.parse(readFileSync(join(dir, 'package.json'), 'utf8')) as {
          version: string;
          license?: string;
          author?: string | { name?: string };
          dependencies?: Record<string, string>;
        };
        const file = readdirSync(dir).find((f) => /^(licen[sc]e|copying)(\.|$)/i.test(f));
        const author = typeof pkg.author === 'string' ? pkg.author : (pkg.author?.name ?? '');
        seen.set(name, { name, version: pkg.version, license: pkg.license ?? 'unknown', author, text: file ? readFileSync(join(dir, file), 'utf8').trim() : null });
        for (const dep of Object.keys(pkg.dependencies ?? {})) visit(dep);
      };
      const own = JSON.parse(readFileSync(join(root, 'package.json'), 'utf8')) as { dependencies: Record<string, string> };
      for (const dep of Object.keys(own.dependencies)) visit(dep);
      const entries = [...seen.values()].sort((a, b) => a.name.localeCompare(b.name));
      // Some packages (the tfjs ones among them) ship no license file. Point those at a package with the same license whose full text is here.
      const fullText = new Map<string, string>();
      for (const e of entries) if (e.text && !fullText.has(e.license)) fullText.set(e.license, e.name);
      const body = entries.map((e) => {
        const ref = fullText.get(e.license);
        const text = e.text ?? `${e.license} license${e.author ? `, copyright ${e.author}` : ''}.${ref ? ` The full ${e.license} text is under ${ref} in this file.` : ''}`;
        return `${e.name} ${e.version} (${e.license})\n\n${text}`;
      });
      const head = 'Third-party software bundled into the frontier-fly site. Assets and data are credited in CREDITS.md.';
      const sep = `\n\n${'-'.repeat(72)}\n\n`;
      this.emitFile({ type: 'asset', fileName: 'THIRD_PARTY_LICENSES.txt', source: `${head}\n\n${'='.repeat(72)}\n\n${body.join(sep)}\n` });
    },
  };
}

export default defineConfig({
  base: './',
  plugins: [crossOriginIsolation(), thirdPartyLicenses()],
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
        race: resolve(root, 'race.html'),
        team: resolve(root, 'team.html'),
        px4: resolve(root, 'px4.html'),
        atlas: resolve(root, 'atlas.html'),
      },
    },
  },
});
