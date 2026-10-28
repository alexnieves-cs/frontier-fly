import { mkdirSync, readFileSync } from 'node:fs';
import { expect, test, type Page } from '@playwright/test';
import { out } from './mediaOut';


test.use({ viewport: { width: 1440, height: 900 } });

test.beforeAll(() => {
  mkdirSync(out, { recursive: true });
});

async function openMission(page: Page, query: string): Promise<void> {
  const response = await page.goto(`/mission.html${query}`);
  expect(response?.headers()['cross-origin-opener-policy']).toBe('same-origin');
  await expect(page.locator('#app[data-ready="1"]')).toBeVisible();
}

async function clickWorld(page: Page, x: number, y: number): Promise<void> {
  const canvas = page.locator('[data-testid=mission-map]');
  const meta = await canvas.evaluate((el) => {
    const node = el as HTMLCanvasElement;
    return {
      cx: Number(node.dataset.cx),
      cy: Number(node.dataset.cy),
      mpp: Number(node.dataset.mpp),
      w: node.clientWidth,
      h: node.clientHeight,
    };
  });
  const sx = (x - meta.cx) / meta.mpp + meta.w / 2;
  const sy = (meta.cy - y) / meta.mpp + meta.h / 2;
  await canvas.click({ position: { x: sx, y: sy } });
}

test('draw a geofence, save the link, and reload it', async ({ page }) => {
  await openMission(page, '?blank=1&seed=11');
  await page.locator('[data-testid=tool-geofence]').click();
  await clickWorld(page, 10, 10);
  await clickWorld(page, 70, 10);
  await clickWorld(page, 70, 50);
  await clickWorld(page, 10, 50);
  await expect(page.locator('#app')).toHaveAttribute('data-draft', '4');
  await clickWorld(page, 10, 10);
  await expect(page.locator('#app')).toHaveAttribute('data-corners', '4');
  await page.locator('[data-testid=tool-start]').click();
  await clickWorld(page, 20, 40);
  await page.locator('[data-testid=mission-name]').fill('Lot sweep');
  await expect(page.locator('[data-testid=mission-save]')).toBeEnabled();
  await page.locator('[data-testid=mission-save]').click();
  await expect(page.locator('[data-testid=mission-link]')).toHaveValue(/#m=/);
  const link = await page.locator('[data-testid=mission-link]').inputValue();
  await page.goto(link);
  await expect(page.locator('#app[data-ready="1"]')).toBeVisible();
  await expect(page.locator('[data-testid=mission-name]')).toHaveValue('Lot sweep');
  await expect(page.locator('#app')).toHaveAttribute('data-corners', '4');
  await expect(page.locator('[data-testid=mission-summary]')).toContainText('Start E 20.0 m, N 40.0 m');
  await expect(page.locator('[data-testid=mission-issues]')).toContainText('No errors');
});

test('default page loads the generated woods and flies the autonomy runner', async ({ page }) => {
  test.setTimeout(360_000);
  await openMission(page, '?seed=11');
  await expect(page.locator('#app')).toHaveAttribute('data-site', 'outdoor-world');
  await expect(page.locator('[data-testid=site-list]')).toContainText('Tree canopy');
  await expect(page.locator('[data-testid=site-list]')).toContainText('Shed');
  await expect(page.locator('[data-testid=mission-issues]')).toContainText('No errors');
  await page.locator('[data-testid=mode-run]').click();
  await expect(page.locator('#app')).toHaveAttribute('data-flight', 'ready', { timeout: 60_000 });
  await page.locator('[data-testid=speed]').selectOption('64');
  await page.locator('[data-testid=play]').click();
  // The sample asks before entering; keep the drone outside so the flight ends sooner.
  await expect(page.locator('[data-testid=approval-prompt]')).toBeVisible({ timeout: 120_000 });
  await expect(page.locator('[data-testid=flight-log]')).toContainText(/opening/i);
  const deny = page.locator('[data-testid=approval-deny]');
  const report = page.locator('[data-testid=mission-report]');
  for (let i = 0; i < 6 && !(await report.isVisible()); i++) {
    await Promise.race([deny.waitFor({ state: 'visible', timeout: 180_000 }), report.waitFor({ state: 'visible', timeout: 180_000 })]);
    if (await deny.isVisible()) await deny.click();
  }
  await expect(report).toBeVisible({ timeout: 180_000 });
  await expect(page.locator('#app')).toHaveAttribute('data-outcome', /.+/);
  await expect(report).toContainText('False clears');
});

test('editor, live approval, and report screenshots', async ({ page }) => {
  test.setTimeout(420_000);
  await openMission(page, '?seed=11');
  await expect(page.locator('[data-testid=mission-summary]')).toContainText('Geofence');
  await page.screenshot({ path: `${out}/mission-editor.png` });
  expect(readFileSync(`${out}/mission-editor.png`).length).toBeGreaterThan(20_000);

  await page.locator('[data-testid=mode-run]').click();
  await expect(page.locator('#app')).toHaveAttribute('data-flight', 'ready', { timeout: 60_000 });
  await page.locator('[data-testid=speed]').selectOption('64');
  await page.locator('[data-testid=play]').click();
  await expect(page.locator('[data-testid=approval-prompt]')).toBeVisible({ timeout: 120_000 });
  await expect(page.locator('[data-testid=approval-prompt]')).toContainText('safety score');
  await page.screenshot({ path: `${out}/mission-live.png` });
  expect(readFileSync(`${out}/mission-live.png`).length).toBeGreaterThan(20_000);

  // Allow every entry: on this seed the drone finds the simulated person indoors.
  const allow = page.locator('[data-testid=approval-allow]');
  const report = page.locator('[data-testid=mission-report]');
  for (let i = 0; i < 6 && !(await report.isVisible()); i++) {
    await Promise.race([allow.waitFor({ state: 'visible', timeout: 240_000 }), report.waitFor({ state: 'visible', timeout: 240_000 })]);
    if (await allow.isVisible()) await allow.click();
  }
  await expect(report).toBeVisible({ timeout: 240_000 });
  await expect(report).toContainText('Collisions');
  await report.scrollIntoViewIfNeeded();
  await page.screenshot({ path: `${out}/mission-report.png` });
  expect(readFileSync(`${out}/mission-report.png`).length).toBeGreaterThan(20_000);
});
