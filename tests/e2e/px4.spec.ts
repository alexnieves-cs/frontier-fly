import { existsSync, readFileSync } from 'node:fs';
import Ajv2020 from 'ajv/dist/2020.js';
import { expect, test } from '@playwright/test';

const planSchema: object = JSON.parse(readFileSync('src/mission/export/qgc-plan.schema.json', 'utf8'));
const validatePlan = new Ajv2020({ strict: true, strictRequired: false, allErrors: true }).compile(planSchema);

test('the PX4 page loads the woods, explains the setup without a bridge, and downloads the plan PX4 flies', async ({ page }) => {
  // A port nothing listens on: the page must say so instead of hanging.
  await page.goto('/px4.html?seed=4242&ws=ws://127.0.0.1:9');
  await expect(page.locator('#app[data-ready="1"]')).toBeVisible();
  await expect(page.locator('#app')).toHaveAttribute('data-link', /error|closed/u);
  await expect(page.locator('#help')).toBeVisible();
  await expect(page.locator('#plan-summary')).toContainText('waypoints');

  const [file] = await Promise.all([page.waitForEvent('download'), page.locator('[data-testid=px4-download]').click()]);
  expect(file.suggestedFilename()).toMatch(/-seed4242-px4\.plan$/u);
  const data: unknown = JSON.parse(readFileSync(await file.path(), 'utf8'));
  expect(validatePlan(data), JSON.stringify(validatePlan.errors)).toBe(true);
  const plan = data as { mission: { plannedHomePosition: number[]; firmwareType: number } };
  expect(plan.mission.firmwareType).toBe(12);
  // Pinned to PX4 SITL's default home, where the page's GNSS puts the pad.
  expect(plan.mission.plannedHomePosition.slice(0, 2)).toEqual([47.397742, 8.545594]);
});

/**
 * The live run: PX4 SITL (airframe 10016) flies the page through the bridge
 * while scripts/px4/flight_check.py uploads and starts the mission. Run by
 * scripts/px4/run-browser-check.sh, which starts PX4, the bridge and the
 * check; skipped otherwise.
 */
test('PX4 SITL flies the browser sim through the bridge', async ({ page }) => {
  test.skip(!process.env.PX4_LIVE, 'needs PX4 SITL and the bridge (scripts/px4/run-browser-check.sh)');
  test.setTimeout(900_000);
  page.on('console', (msg) => {
    if (msg.type() === 'error') console.log(`[page] ${msg.text()}`);
  });
  const ws = process.env.PX4_BRIDGE_URL ?? 'ws://127.0.0.1:4561';
  await page.goto(`/px4.html?seed=${process.env.PX4_SEED ?? '4242'}&ws=${encodeURIComponent(ws)}`);
  await expect(page.locator('#app[data-ready="1"]')).toBeVisible();
  await expect(page.locator('#app')).toHaveAttribute('data-link', 'px4', { timeout: 240_000 });
  await expect(page.locator('#help')).toBeHidden();
  // The flight check arms PX4 once its estimator converges on the page's sensors.
  await expect(page.locator('#app')).toHaveAttribute('data-armed', '1', { timeout: 300_000 });
  await expect
    .poll(async () => Number(await page.locator('#app').getAttribute('data-max-agl')), { timeout: 120_000 })
    .toBeGreaterThan(10);
  await expect
    .poll(async () => Number(await page.locator('#app').getAttribute('data-flown-m')), { timeout: 120_000 })
    .toBeGreaterThan(40);
  await page.screenshot({ path: 'test-results/px4/browser-flight-midair.png' });
  // ...and the mission ends with PX4 landing and disarming.
  await expect(page.locator('#app')).toHaveAttribute('data-armed', '0', { timeout: 600_000 });
  const flown = Number(await page.locator('#app').getAttribute('data-flown-m'));
  console.log(`page flew ${flown} m, max ${await page.locator('#app').getAttribute('data-max-agl')} m AGL`);
  expect(flown).toBeGreaterThan(100);
  await page.screenshot({ path: 'test-results/px4/browser-flight-landed.png' });
  // Closing the page cuts PX4's sensors, so stay until the flight check has
  // seen the disarm and stopped (run-browser-check.sh creates this file).
  const done = process.env.PX4_CHECK_DONE;
  if (done) await expect.poll(() => existsSync(done), { timeout: 120_000 }).toBe(true);
});
