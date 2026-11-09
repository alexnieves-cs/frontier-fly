import { readFileSync } from 'node:fs';
import { expect, test } from '@playwright/test';

test('Download after training in the tab writes the trained network, not the one that was loaded', async ({ page }) => {
  test.setTimeout(240_000);
  await page.goto('/sim.html?quality=low&train=1');
  await expect(page.locator('#app[data-ready="1"]')).toBeVisible({ timeout: 60_000 });
  // Two PPO updates have come back from the learner.
  await expect.poll(async () => Number(await page.locator('#ep-num').textContent()), { timeout: 180_000 }).toBeGreaterThanOrEqual(2);
  const download = page.waitForEvent('download');
  await page.locator('[data-act="save"]').dispatchEvent('click');
  const saved = JSON.parse(readFileSync((await (await download).path())!, 'utf8')) as { actorOnly?: boolean; weights: number[][]; obsCount: number };
  // The shipped policy the page loads is actor-only; a trained network carries its value head and
  // the observation statistics of this session's rollouts.
  expect(saved.actorOnly).toBeFalsy();
  expect(saved.weights.length).toBeGreaterThan(6);
  expect(saved.obsCount).toBeGreaterThan(100);
});
