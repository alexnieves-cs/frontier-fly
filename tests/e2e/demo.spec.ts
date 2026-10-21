import { mkdirSync, readFileSync } from 'node:fs';
import { expect, test, type Page } from '@playwright/test';

const out = 'docs/media';

const CASES = [
  { id: 'cabin-day', outcome: 'found' },
  { id: 'compound-clear', outcome: 'clear' },
  { id: 'backpack-night', outcome: 'found' },
] as const;

test.use({ viewport: { width: 1440, height: 900 } });

test.beforeAll(() => {
  mkdirSync(out, { recursive: true });
});

/** Run to the end, approving every entry prompt along the way. */
async function runToDone(page: Page, timeoutMs: number): Promise<number> {
  const app = page.locator('#app');
  const prompt = page.locator('[data-testid=operator-prompt]');
  const deadline = Date.now() + timeoutMs;
  let approvals = 0;
  while (Date.now() < deadline) {
    if ((await app.getAttribute('data-done')) === '1') return approvals;
    if (await prompt.isVisible()) {
      await page.locator('[data-testid=operator-approve]').click();
      approvals += 1;
      continue;
    }
    await page.waitForTimeout(250);
  }
  throw new Error(`run did not finish in ${timeoutMs} ms (phase ${await app.getAttribute('data-phase')})`);
}

for (const scenario of CASES) {
  test(`demo ${scenario.id} runs to a report`, async ({ page }) => {
    test.setTimeout(300_000);
    const errors: string[] = [];
    page.on('pageerror', (error) => errors.push(error.message));
    const external: string[] = [];
    page.on('request', (request) => {
      const url = new URL(request.url());
      if (url.hostname !== '127.0.0.1' && url.protocol !== 'data:' && url.protocol !== 'blob:') external.push(request.url());
    });

    const response = await page.goto(`/demo.html?scenario=${scenario.id}&speed=max&autorun=1`);
    expect(response?.headers()['cross-origin-opener-policy']).toBe('same-origin');
    expect(response?.headers()['cross-origin-embedder-policy']).toBe('require-corp');
    const app = page.locator('#app');
    await expect(page.locator('#app[data-ready="1"]')).toBeVisible({ timeout: 60_000 });
    await expect(app).toHaveAttribute('data-scenario', scenario.id);
    await expect(page.locator(`[data-testid=scenario-${scenario.id}]`)).toHaveAttribute('aria-pressed', 'true');

    const approvals = await runToDone(page, 240_000);
    // Capture before the outcome checks so a failing run still leaves a picture.
    await page.screenshot({ path: `${out}/demo-${scenario.id}.png` });
    expect(readFileSync(`${out}/demo-${scenario.id}.png`).length).toBeGreaterThan(20_000);
    if (scenario.id === 'compound-clear') expect(approvals).toBeGreaterThan(0);

    await expect(app).toHaveAttribute('data-outcome', scenario.outcome);
    await expect(app).toHaveAttribute('data-phase', 'done');
    const report = page.locator('[data-testid=demo-report]');
    await expect(report).toBeVisible();
    for (const id of ['time-to-find', 'coverage', 'false-clears', 'false-alarms', 'collisions', 'battery']) {
      await expect(page.locator(`[data-testid=kpi-${id}]`)).toBeVisible();
    }
    await expect(page.locator('[data-testid=kpi-false-clears] strong')).toHaveText('0');
    await expect(page.locator('[data-testid=kpi-false-alarms] strong')).toHaveText('0');
    if (scenario.outcome === 'found') await expect(page.locator('[data-testid=kpi-time-to-find] strong')).not.toHaveText('—');
    const rows = page.locator('[data-testid=report-rows]');
    for (const label of ['Time to find', 'coverage', 'False clears', 'False alarms', 'Collisions', 'Battery used']) {
      await expect(rows).toContainText(label);
    }
    await expect(page.locator('[data-testid=phase-stepper] li[data-step=report]')).toHaveAttribute('data-state', 'done');

    const state = await page.evaluate(() => window.__demo ?? null);
    expect(state?.scenario).toBe(scenario.id);
    expect(state?.report.outcome).toBe(scenario.outcome);
    expect(state?.run.landedHome).toBe(true);
    if (scenario.id === 'compound-clear') expect(state?.run.entries.some((e) => e.decision === 'approved')).toBe(true);

    expect(errors).toEqual([]);
    expect(external).toEqual([]);
  });
}

test('demo camera mode renders the drone camera and runs the learned detector', async ({ page }) => {
  test.setTimeout(240_000);
  const errors: string[] = [];
  page.on('pageerror', (error) => errors.push(error.message));
  await page.goto('/demo.html?scenario=cabin-day&speed=16&detector=camera&autorun=1&operator=approve');
  const app = page.locator('#app');
  await expect(app).toHaveAttribute('data-detector', 'camera');
  await expect(page.locator('[data-testid=camera-inset]')).toBeVisible();
  // Frames arrive once the drone is flying; the inset caption names the detector once it has loaded.
  await expect(page.locator('[data-testid=camera-caption]')).toHaveText(/learned detector/, { timeout: 60_000 });
  await expect.poll(async () => Number((await app.getAttribute('data-camera-frames')) ?? 0), { timeout: 120_000 }).toBeGreaterThan(20);
  if (process.env.VISION_SHOT) await page.locator('[data-testid=camera-inset]').screenshot({ path: process.env.VISION_SHOT });
  expect(errors).toEqual([]);
});
