#!/usr/bin/env node
// Renders docs/REPORT.md as a PDF: pandoc turns it into one self-contained HTML page (figures
// inlined), and headless Chromium prints that page. No LaTeX needed, so it runs on any runner
// that has pandoc and the Playwright Chromium.
//
//   npm run report:pdf                      writes dist/frontier-fly-report.pdf
//   npm run report:pdf -- out/report.pdf    somewhere else
import { execFileSync } from 'node:child_process';
import { mkdirSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { dirname, join } from 'node:path';
import { chromium } from '@playwright/test';

const REPO = 'https://github.com/alexnieves-cs/frontier-fly';
const out = process.argv[2] ?? 'dist/frontier-fly-report.pdf';
const pkg = JSON.parse(readFileSync('package.json', 'utf8'));
const commit = process.env.GITHUB_SHA?.slice(0, 7) ?? execFileSync('git', ['rev-parse', '--short', 'HEAD'], { encoding: 'utf8' }).trim();

// Links between docs and to source files point at GitHub, since the PDF travels without the repo.
// Collapsed <details> blocks are opened so their tables print.
const md = readFileSync('docs/REPORT.md', 'utf8')
  .replace(/^<!--.*?-->\n+/s, '')
  .replace(/^# .*\n/, '')
  .replace(/\]\(\.\.\/([^)#]+)/g, `](${REPO}/blob/main/$1`)
  .replace(/\]\((?!https?:|#|report\/)([^)#]+)/g, `](${REPO}/blob/main/docs/$1`)
  .replaceAll('<details>', '<details open>');

const css = `
  @page { size: A4; margin: 18mm 16mm 20mm; }
  body { font: 10pt/1.45 "Source Serif 4", Georgia, serif; color: #1b1f24; max-width: none; margin: 0; }
  h1, h2, h3, h4 { font-family: "Inter", "Helvetica Neue", Arial, sans-serif; line-height: 1.2; break-after: avoid; }
  h1.title { font-size: 20pt; margin: 0 0 4pt; }
  p.subtitle { font-size: 11pt; margin: 0 0 2pt; }
  p.author, p.date { margin: 0; color: #57606a; }
  h2 { font-size: 13.5pt; margin-top: 18pt; border-bottom: 0.5pt solid #d0d7de; padding-bottom: 2pt; }
  h3 { font-size: 11pt; margin-top: 12pt; }
  table { border-collapse: collapse; font-size: 8pt; margin: 6pt 0; width: 100%; break-inside: avoid; }
  th, td { border: 0.5pt solid #d0d7de; padding: 2pt 4pt; text-align: left; vertical-align: top; }
  th { background: #f6f8fa; }
  img { max-width: 100%; display: block; margin: 6pt auto; break-inside: avoid; }
  figure { margin: 8pt 0; break-inside: avoid; }
  figcaption { font-size: 8.5pt; color: #57606a; text-align: center; }
  code { font-size: 8.5pt; }
  pre { font-size: 8pt; background: #f6f8fa; padding: 6pt; white-space: pre-wrap; }
  a { color: #0550ae; text-decoration: none; }
  summary { font-weight: 600; list-style: none; }
  nav#TOC { font-size: 9pt; break-after: page; }
  nav#TOC ul { list-style: none; padding-left: 12pt; }
  nav#TOC > ul { padding-left: 0; }
`;

const work = mkdtempSync(join(tmpdir(), 'report-'));
try {
  const input = join(work, 'report.md');
  const html = join(work, 'report.html');
  writeFileSync(input, md);
  writeFileSync(join(work, 'style.css'), css);
  execFileSync('pandoc', [
    input,
    '--from', 'gfm+attributes',
    '--to', 'html5',
    '--standalone',
    '--embed-resources',
    '--resource-path', `docs${process.platform === 'win32' ? ';' : ':'}.`,
    '--css', join(work, 'style.css'),
    '--toc', '--toc-depth', '2',
    '--metadata', 'title=Searching a building without its floor plan',
    '--metadata', `subtitle=frontier-fly technical report, version ${pkg.version}`,
    '--metadata', 'author=Alex Nieves',
    '--metadata', `date=${new Date().toISOString().slice(0, 10)}, commit ${commit}, ${REPO}`,
    '--output', html,
  ], { stdio: 'inherit' });
  const browser = await chromium.launch({ executablePath: process.env.CHROMIUM_PATH || undefined });
  try {
    const page = await browser.newPage();
    await page.goto(`file://${html}`);
    mkdirSync(dirname(out), { recursive: true });
    await page.pdf({
      path: out,
      format: 'A4',
      printBackground: true,
      displayHeaderFooter: true,
      headerTemplate: '<span></span>',
      footerTemplate: '<div style="font-size:7pt;width:100%;text-align:center;color:#57606a">frontier-fly technical report &middot; <span class="pageNumber"></span> / <span class="totalPages"></span></div>',
      margin: { top: '18mm', bottom: '20mm', left: '16mm', right: '16mm' },
    });
  } finally {
    await browser.close();
  }
  console.log(`wrote ${out}`);
} finally {
  rmSync(work, { recursive: true, force: true });
}
