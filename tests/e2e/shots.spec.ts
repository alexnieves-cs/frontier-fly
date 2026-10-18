import { createHash } from 'node:crypto';
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { expect, test } from '@playwright/test';

const out = 'docs/media';
const frames = 'test-results/frames';

async function openShot(page: import('@playwright/test').Page, query: string): Promise<void> {
  const response = await page.goto(`/${query}`);
  expect(response?.headers()['cross-origin-opener-policy']).toBe('same-origin');
  expect(response?.headers()['cross-origin-embedder-policy']).toBe('require-corp');
  await expect(page.locator('#app[data-ready="1"]')).toBeVisible({ timeout: 40_000 });
  await page.waitForTimeout(600);
}

test.beforeAll(() => {
  mkdirSync(out, { recursive: true });
  mkdirSync(frames, { recursive: true });
});

async function viewLuma(page: import('@playwright/test').Page): Promise<number> {
  return page.evaluate(() => {
    const canvas = document.querySelector('#view') as HTMLCanvasElement | null;
    const gl = canvas?.getContext('webgl2');
    if (!canvas || !gl) return -1;
    const w = Math.min(96, canvas.width);
    const h = Math.min(96, canvas.height);
    const buf = new Uint8Array(w * h * 4);
    gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, buf);
    let sum = 0;
    let n = 0;
    for (let i = 0; i < buf.length; i += 16) {
      sum += (buf[i] ?? 0) + (buf[i + 1] ?? 0) + (buf[i + 2] ?? 0);
      n += 1;
    }
    return sum / Math.max(1, n);
  });
}

test('day chase view is non-blank', async ({ page }) => {
  await openShot(page, '?shot=day&seed=11&quality=high');
  const luma = await viewLuma(page);
  expect(luma).toBeGreaterThan(8);
  await page.screenshot({ path: `${out}/day.png` });
  const fps = await page.locator('#app').getAttribute('data-fps');
  const backend = await page.locator('#app').getAttribute('data-backend');
  writeFileSync(`${out}/capture.json`, JSON.stringify({ luma, fps, backend }, null, 2));
  expect(readFileSync(`${out}/day.png`).length).toBeGreaterThan(20_000);
});

test('night flashlight view differs from day', async ({ page }) => {
  await openShot(page, '?shot=night&seed=11&quality=high');
  await page.screenshot({ path: `${out}/night.png` });
  const day = createHash('sha256').update(readFileSync(`${out}/day.png`)).digest('hex');
  const night = createHash('sha256').update(readFileSync(`${out}/night.png`)).digest('hex');
  expect(night).not.toBe(day);
  expect(readFileSync(`${out}/night.png`).length).toBeGreaterThan(20_000);
});

test('fpv at night shows the hud', async ({ page }) => {
  await openShot(page, '?shot=fpv&seed=11&quality=high');
  await expect(page.locator('#hud')).toBeVisible();
  await page.screenshot({ path: `${out}/fpv.png` });
  expect(readFileSync(`${out}/fpv.png`).length).toBeGreaterThan(20_000);
  for (let i = 0; i < 12; i++) {
    await page.screenshot({ path: `${frames}/fpv-${String(i).padStart(2, '0')}.png` });
    await page.waitForTimeout(140);
  }
});
