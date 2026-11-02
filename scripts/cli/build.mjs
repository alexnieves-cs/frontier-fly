// Bundle the headless CLI (scripts/cli/bin.ts) and the sim it imports into one ES module,
// with the two data files it reads beside it. The Docker image ships this folder.
//
//   npm run build:cli   ->  dist-cli/frontier-fly.mjs
//                           dist-cli/assets/{checkpoints/policy.bin, plans/swiss-dwellings.json}
//                           dist-cli/LICENSE.* (this repo's MIT, and the Apache-2.0 of Rapier and TensorFlow.js if bundled)
import { build } from 'esbuild';
import { copyFileSync, mkdirSync, readFileSync, rmSync, statSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';

const out = 'dist-cli';
const version = JSON.parse(readFileSync('package.json', 'utf8')).version;
rmSync(out, { recursive: true, force: true });
mkdirSync(out, { recursive: true });
const result = await build({
  entryPoints: ['scripts/cli/bin.ts'],
  outfile: join(out, 'frontier-fly.mjs'),
  bundle: true,
  platform: 'node',
  format: 'esm',
  target: 'node22',
  define: { __FF_VERSION__: JSON.stringify(version) },
  // Some dependencies are CommonJS and call require(); give the ESM bundle one.
  banner: { js: "#!/usr/bin/env node\nimport { createRequire as __ffCreateRequire } from 'node:module'; const require = __ffCreateRequire(import.meta.url);" },
  legalComments: 'eof',
  metafile: true,
  logLevel: 'warning',
});
for (const file of ['checkpoints/policy.bin', 'plans/swiss-dwellings.json']) {
  const dest = join(out, 'assets', file);
  mkdirSync(dirname(dest), { recursive: true });
  copyFileSync(join('public', file), dest);
}
// License files for every npm package in the bundle, plus this repo's.
const pkgs = new Set();
for (const input of Object.keys(result.metafile.inputs)) {
  const m = input.match(/node_modules\/((?:@[^/]+\/)?[^/]+)\//);
  if (m) pkgs.add(m[1]);
}
copyFileSync('LICENSE', join(out, 'LICENSE.frontier-fly'));
const bundled = [];
for (const pkg of [...pkgs].sort()) {
  const meta = JSON.parse(readFileSync(join('node_modules', pkg, 'package.json'), 'utf8'));
  bundled.push(`${pkg}@${meta.version} (${meta.license})`);
  for (const name of ['LICENSE', 'LICENSE.md', 'LICENSE.txt']) {
    try {
      copyFileSync(join('node_modules', pkg, name), join(out, `LICENSE.${pkg.replace(/^@/, '').replace('/', '__')}`));
      break;
    } catch {
      /* try the next spelling */
    }
  }
}
writeFileSync(join(out, 'BUNDLED.txt'), `frontier-fly ${version} CLI bundle. npm packages inside:\n${bundled.map((b) => `  ${b}`).join('\n')}\n`);
const mb = (p) => (statSync(p).size / 1e6).toFixed(1);
console.log(`dist-cli: frontier-fly.mjs ${mb(join(out, 'frontier-fly.mjs'))} MB; bundles ${bundled.join(', ') || 'no npm packages'}`);
