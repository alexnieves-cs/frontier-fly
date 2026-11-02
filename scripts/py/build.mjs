// Build the Node half of the Python package: bundle scripts/py/server.ts (and the whole
// headless sim it imports) into one ES module, and copy the two data files it reads.
//
//   npm run build:py   ->  python/frontierfly/_server/server.mjs
//                          python/frontierfly/_server/assets/{checkpoints/policy.bin, plans/swiss-dwellings.json}
//                          python/frontierfly/_server/LICENSE.* (Rapier's Apache-2.0 and this repo's MIT)
import { build } from 'esbuild';
import { copyFileSync, mkdirSync, rmSync, statSync } from 'node:fs';
import { dirname, join } from 'node:path';

const out = 'python/frontierfly/_server';
rmSync(out, { recursive: true, force: true });
mkdirSync(out, { recursive: true });
await build({
  entryPoints: ['scripts/py/server.ts'],
  outfile: join(out, 'server.mjs'),
  bundle: true,
  platform: 'node',
  format: 'esm',
  target: 'node22',
  // Some dependencies are CommonJS and call require(); give the ESM bundle one.
  banner: { js: "import { createRequire as __ffCreateRequire } from 'node:module'; const require = __ffCreateRequire(import.meta.url);" },
  legalComments: 'eof',
  logLevel: 'warning',
});
for (const file of ['checkpoints/policy.bin', 'plans/swiss-dwellings.json']) {
  const dest = join(out, 'assets', file);
  mkdirSync(dirname(dest), { recursive: true });
  copyFileSync(join('public', file), dest);
}
// The bundle carries Rapier (Apache-2.0, the only npm package in it) and the frontier-fly code (MIT).
copyFileSync('node_modules/@dimforge/rapier3d-compat/LICENSE', join(out, 'LICENSE.rapier3d'));
copyFileSync('LICENSE', join(out, 'LICENSE.frontier-fly'));
const mb = (p) => (statSync(p).size / 1e6).toFixed(1);
console.log(`python/frontierfly/_server: server.mjs ${mb(join(out, 'server.mjs'))} MB, policy.bin ${mb(join(out, 'assets/checkpoints/policy.bin'))} MB`);
