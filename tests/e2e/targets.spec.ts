import { mkdirSync, readFileSync } from 'node:fs';
import { expect, test } from '@playwright/test';
import { out } from './mediaOut';


test.beforeAll(() => {
  mkdirSync(out, { recursive: true });
});

async function openShot(page: import('@playwright/test').Page, query: string): Promise<void> {
  const response = await page.goto(`/sim.html${query}`);
  expect(response?.ok()).toBeTruthy();
  await expect(page.locator('#app[data-ready="1"]')).toBeVisible({ timeout: 40_000 });
  await page.waitForTimeout(700);
}

test('fpv lists detections over the target lineup', async ({ page }) => {
  await openShot(page, '?shot=targets&seed=11&quality=low');
  const count = Number(await page.locator('#app').getAttribute('data-detections'));
  expect(count).toBeGreaterThan(0);
  await expect(page.locator('#detect-list li').first()).toBeVisible();
  await page.screenshot({ path: `${out}/targets-fpv.png` });
  expect(readFileSync(`${out}/targets-fpv.png`).length).toBeGreaterThan(20_000);
});

test('room view shows people, objects, and decoys', async ({ page }) => {
  await openShot(page, '?shot=targets-room&seed=11&quality=low');
  await page.screenshot({ path: `${out}/targets-room.png` });
  expect(readFileSync(`${out}/targets-room.png`).length).toBeGreaterThan(20_000);
});

test('night flashlight still returns detections', async ({ page }) => {
  await openShot(page, '?shot=targets-night&seed=11&quality=low');
  const count = Number(await page.locator('#app').getAttribute('data-detections'));
  expect(count).toBeGreaterThan(0);
  await page.screenshot({ path: `${out}/targets-night.png` });
  expect(readFileSync(`${out}/targets-night.png`).length).toBeGreaterThan(20_000);
});
