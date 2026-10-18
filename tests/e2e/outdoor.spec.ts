import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { expect, test } from '@playwright/test';

const out = 'docs/media';

test.beforeAll(() => {
  mkdirSync(out, { recursive: true });
});

test('outdoor chase, fpv, and map are non-blank', async ({ page }) => {
  const response = await page.goto('/outdoor.html?seed=4&shot=chase');
  expect(response?.headers()['cross-origin-opener-policy']).toBe('same-origin');
  await expect(page.locator('#app[data-ready="1"]')).toBeVisible({ timeout: 40_000 });
  await page.waitForTimeout(700);
  const chaseLuma = await viewLuma(page);
  expect(chaseLuma).toBeGreaterThan(8);
  await page.screenshot({ path: `${out}/outdoor-chase.png` });
  expect(readFileSync(`${out}/outdoor-chase.png`).length).toBeGreaterThan(20_000);

  await page.locator('#cam-fpv').click();
  await page.waitForTimeout(250);
  await page.screenshot({ path: `${out}/outdoor-fpv.png` });
  expect(readFileSync(`${out}/outdoor-fpv.png`).length).toBeGreaterThan(20_000);

  await page.goto('/outdoor.html?seed=4&shot=map');
  await expect(page.locator('#app[data-ready="1"]')).toBeVisible({ timeout: 40_000 });
  await page.waitForTimeout(400);
  await page.screenshot({ path: `${out}/outdoor-map.png` });
  expect(readFileSync(`${out}/outdoor-map.png`).length).toBeGreaterThan(20_000);

  const fps = await page.locator('#app').getAttribute('data-fps');
  writeFileSync(
    `${out}/outdoor-capture.json`,
    JSON.stringify({ chaseLuma, fps, buildings: await page.locator('#app').getAttribute('data-buildings') }, null, 2),
  );
  expect(Number(fps ?? '0')).toBeGreaterThan(8);
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
