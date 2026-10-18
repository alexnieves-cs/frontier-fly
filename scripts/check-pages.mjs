#!/usr/bin/env node
// Loads every built page the way GitHub Pages serves it and fails if one does not become ready.
//
//   node scripts/check-pages.mjs                 serve dist/ under /frontier-fly/ with no COOP/COEP headers
//   node scripts/check-pages.mjs https://host/x/ check a deployed site
//
// GitHub Pages cannot set response headers, so a page only gets SharedArrayBuffer through
// public/coi-serviceworker.js. The local mode reproduces that by serving plain files.
import { createReadStream, existsSync, readdirSync, statSync } from 'node:fs';
import { createServer } from 'node:http';
import { extname, join, normalize } from 'node:path';
import { chromium } from '@playwright/test';

const DIST = 'dist';
const SUBPATH = '/frontier-fly/';
const READY_TIMEOUT = Number(process.env.READY_TIMEOUT ?? 60_000);
const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript',
  '.css': 'text/css',
  '.json': 'application/json',
  '.wasm': 'application/wasm',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.glb': 'model/gltf-binary',
  '.hdr': 'application/octet-stream',
};

function serveDist() {
  const server = createServer((req, res) => {
    const url = new URL(req.url ?? '/', 'http://x');
    if (!url.pathname.startsWith(SUBPATH)) {
      res.writeHead(404).end();
      return;
    }
    let rel = normalize(decodeURIComponent(url.pathname.slice(SUBPATH.length)));
    if (rel === '.' || rel === '') rel = 'index.html';
    const file = join(DIST, rel);
    if (!file.startsWith(DIST) || !existsSync(file) || !statSync(file).isFile()) {
      res.writeHead(404).end();
      return;
    }
    res.writeHead(200, { 'Content-Type': TYPES[extname(file)] ?? 'application/octet-stream' });
    createReadStream(file).pipe(res);
  });
  return new Promise((resolve) => {
    server.listen(0, '127.0.0.1', () => {
      const { port } = /** @type {import('node:net').AddressInfo} */ (server.address());
      resolve({ server, base: `http://127.0.0.1:${port}${SUBPATH}` });
    });
  });
}

/** Pages to check: every HTML file in dist/, or the same names on a remote site. */
function pageNames() {
  if (!existsSync(DIST)) throw new Error('dist/ is missing. Run npm run build first.');
  return readdirSync(DIST)
    .filter((name) => name.endsWith('.html'))
    .sort((a, b) => (a === 'index.html' ? -1 : b === 'index.html' ? 1 : a.localeCompare(b)));
}

async function checkPage(browser, url) {
  const context = await browser.newContext({ viewport: { width: 1280, height: 800 } });
  const page = await context.newPage();
  const errors = [];
  page.on('pageerror', (error) => errors.push(String(error)));
  const t0 = Date.now();
  const response = await page.goto(url, { waitUntil: 'load' });
  const status = response?.status() ?? 0;
  let ready = false;
  try {
    // The service worker reloads the page once on the first visit, so wait on the DOM, not navigation.
    await page.waitForSelector('#app[data-ready="1"]', { timeout: READY_TIMEOUT });
    ready = true;
  } catch {
    ready = false;
  }
  const isolated = await page.evaluate(() => globalThis.crossOriginIsolated === true).catch(() => false);
  const ms = Date.now() - t0;
  await context.close();
  return { url, status, ready, isolated, ms, errors };
}

async function main() {
  const remote = process.argv[2];
  let base = remote ? (remote.endsWith('/') ? remote : `${remote}/`) : '';
  let server;
  if (!remote) ({ server, base } = await serveDist());
  const names = remote ? (process.env.PAGES?.split(',') ?? pageNames()) : pageNames();
  const browser = await chromium.launch({
    executablePath: process.env.CHROMIUM_PATH || undefined,
    args: ['--enable-webgl', '--ignore-gpu-blocklist', '--use-gl=angle'],
  });
  const rows = [];
  for (const name of names) {
    const url = name === 'index.html' ? base : `${base}${name}`;
    rows.push({ name, ...(await checkPage(browser, url)) });
  }
  await browser.close();
  server?.close();

  let failed = false;
  for (const row of rows) {
    const ok = row.status === 200 && row.ready && row.isolated && row.errors.length === 0;
    failed ||= !ok;
    console.log(
      `${ok ? 'ok  ' : 'FAIL'} ${row.name.padEnd(14)} status=${row.status} ready=${row.ready} isolated=${row.isolated} ${row.ms} ms`,
    );
    for (const error of row.errors) console.log(`       page error: ${error}`);
  }
  if (failed) process.exit(1);
}

await main();
