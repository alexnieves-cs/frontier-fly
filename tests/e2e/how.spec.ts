import { expect, test } from '@playwright/test';

/** How-it-works page (how.html): the explorer flies a generated building and every chapter renders. */
test('the explainer explores a building and shows every chapter', async ({ page }) => {
  const errors: string[] = [];
  page.on('pageerror', (e) => errors.push(e.message));
  await page.goto('/how.html');
  await expect(page.locator('#app[data-ready="1"]')).toBeAttached();
  for (const id of ['world', 'sensing', 'network', 'training', 'mission', 'failure', 'measure']) {
    await expect(page.locator(`#${id}`)).toBeAttached();
  }
  await page.locator('#world').scrollIntoViewIfNeeded();
  await expect.poll(() => page.evaluate(() => window.__how?.coverage ?? 0), { timeout: 30_000 }).toBeGreaterThan(0.05);
  await page.getByRole('button', { name: 'New seed' }).click();
  await expect(page.locator('[data-readout]')).not.toContainText('Seed 4242');
  expect(errors).toEqual([]);
});
