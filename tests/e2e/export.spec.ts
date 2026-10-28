import { readFileSync } from 'node:fs';
import Ajv2020 from 'ajv/dist/2020.js';
import { expect, test, type Page } from '@playwright/test';

const planSchema: object = JSON.parse(readFileSync('src/mission/export/qgc-plan.schema.json', 'utf8'));
const validatePlan = new Ajv2020({ strict: true, strictRequired: false, allErrors: true }).compile(planSchema);

async function openMission(page: Page, query: string): Promise<void> {
  await page.goto(`/mission.html${query}`);
  await expect(page.locator('#app[data-ready="1"]')).toBeVisible();
}

async function openExport(page: Page): Promise<void> {
  await page.locator('[data-testid=export-panel] summary').click();
  await expect(page.locator('[data-testid=export-download]')).toBeEnabled();
}

async function download(page: Page): Promise<{ name: string; text: string; path: string }> {
  const [file] = await Promise.all([page.waitForEvent('download'), page.locator('[data-testid=export-download]').click()]);
  const path = await file.path();
  return { name: file.suggestedFilename(), text: readFileSync(path, 'utf8'), path };
}

test('export the woods mission as a QGroundControl plan that passes the schema, then import it back', async ({ page }) => {
  await openMission(page, '?seed=11');
  await openExport(page);
  await expect(page.locator('[data-testid=export-summary]')).toContainText("autonomy stack's terrain-following sweep");
  const plan = await download(page);
  expect(plan.name).toMatch(/-seed11\.plan$/);
  const data: unknown = JSON.parse(plan.text);
  expect(validatePlan(data), JSON.stringify(validatePlan.errors)).toBe(true);
  const json = data as { mission: { items: { command: number }[]; firmwareType: number }; geoFence: { polygons: unknown[] } };
  expect(json.mission.firmwareType).toBe(12);
  expect(json.mission.items[0]!.command).toBe(22);
  expect(json.mission.items[json.mission.items.length - 1]!.command).toBe(20);
  expect(json.geoFence.polygons.length).toBeGreaterThan(0);

  // Round trip: the exported fence comes back without errors.
  await page.locator('[data-testid=export-import]').setInputFiles(plan.path);
  await expect(page.locator('[data-testid=export-import-status]')).toHaveAttribute('data-state', 'ok');
  await expect(page.locator('[data-testid=mission-issues]')).toContainText('No errors');
});

test('ArduPilot waypoint file and MAVLink items', async ({ page }) => {
  await openMission(page, '?seed=11');
  await openExport(page);
  await page.locator('[data-testid=export-firmware]').selectOption('ardupilot');
  await page.locator('[data-testid=export-origin]').selectOption('ardupilot-sitl');
  await page.locator('[data-testid=export-finish]').selectOption('land');

  await page.locator('[data-testid=export-format]').selectOption('wpl');
  const wpl = await download(page);
  expect(wpl.name).toMatch(/\.waypoints$/);
  const lines = wpl.text.trim().split('\n');
  expect(lines[0]).toBe('QGC WPL 110');
  expect(lines.slice(1).every((line) => line.split('\t').length === 12)).toBe(true);
  expect(lines[1]!.split('\t')[8]).toBe('-35.3632610');
  expect(lines[lines.length - 1]!.split('\t')[3]).toBe('21');

  await page.locator('[data-testid=export-format]').selectOption('mavlink');
  const mav = await download(page);
  const items = JSON.parse(mav.text) as { homeAtSeq0: boolean; mission: { seq: number }[]; fence: unknown[] };
  expect(items.homeAtSeq0).toBe(true);
  expect(items.mission.map((item) => item.seq)).toEqual(items.mission.map((_, i) => i));
  expect(items.fence.length).toBeGreaterThan(3);
});

test('a bad plan file is refused with a reason', async ({ page }) => {
  await openMission(page, '?seed=11');
  await openExport(page);
  await page.locator('[data-testid=export-import]').setInputFiles({
    name: 'broken.plan',
    mimeType: 'application/json',
    buffer: Buffer.from('{"fileType":"Mission"}'),
  });
  await expect(page.locator('[data-testid=export-import-status]')).toHaveAttribute('data-state', 'error');
  await expect(page.locator('[data-testid=export-import-status]')).toContainText('groundStation');
});
