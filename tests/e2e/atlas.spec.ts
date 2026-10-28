import { expect, test } from '@playwright/test';

/** Failure atlas (atlas.html): the rates render, and a failing flight's replay link rebuilds that case on the Missions page. */
test('the atlas shows both rates and replays a failing flight', async ({ page }) => {
  await page.goto('/atlas.html');
  await expect(page.locator('#app[data-ready="1"]')).toBeAttached();
  await expect(page.getByTestId('rate-falseClear')).toContainText('per flight');
  await expect(page.getByTestId('rate-collision')).toContainText('per flight');
  const replay = page.locator('.atlas-replay').first();
  await expect(replay).toHaveAttribute('href', /demo\.html\?atlas=/);
  await page.locator('[data-dim="fault"]').click();
  await expect(page.getByTestId('atlas-bars').locator('.atlas-bar').first()).toContainText(/%/);
  await expect(page.locator('[data-dim="fault"]')).toHaveAttribute('aria-pressed', 'true');
  const href = await replay.getAttribute('href');
  await page.goto(`/${href!.replace(/^\.\//, '').replace('autorun=1', 'autorun=0')}`);
  await expect(page.locator('.demo[data-ready="1"]')).toBeAttached({ timeout: 60_000 });
  await expect(page.getByText(/^Atlas case \d+$/)).toBeVisible();
});
