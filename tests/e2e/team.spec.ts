import { expect, test } from '@playwright/test';

/** Drone team (team.html): a team flies live in the tab, the operator view lags the truth, and the controls restart it. */

test.describe('drone team', () => {
  test.use({ viewport: { width: 1440, height: 900 } });

  test('flies a team to the end and keeps the base view within the truth', async ({ page }) => {
    test.setTimeout(180_000);
    const errors: string[] = [];
    const external: string[] = [];
    page.on('pageerror', (error) => errors.push(error.message));
    page.on('request', (request) => {
      const url = new URL(request.url());
      if (url.hostname !== '127.0.0.1' && url.protocol !== 'data:' && url.protocol !== 'blob:') external.push(request.url());
    });
    await page.goto('/team.html?condition=hard&drones=3&speed=max');
    await expect(page.locator('#app[data-ready="1"]')).toBeAttached({ timeout: 90_000 });
    await expect(page.locator('#team-legend li')).toHaveCount(3);
    await expect.poll(() => page.evaluate(() => window.__team?.status), { timeout: 120_000 }).toBe('finished');
    const s = await page.evaluate(() => window.__team!);
    expect(s.coverage).toBeGreaterThan(0.5);
    expect(s.baseKnown).toBeLessThanOrEqual(s.coverage + 1e-9);
    expect(s.baseKnown).toBeGreaterThan(0.3);

    // Switch to one drone on a large building: the run restarts with one lane in the legend.
    await page.locator('[data-drones="1"]').click();
    await expect(page.locator('#team-legend li')).toHaveCount(1);
    await expect(page.locator('#team-relay')).toBeDisabled();
    await page.locator('#team-condition').selectOption('large');
    await expect.poll(() => page.evaluate(() => window.__team?.setup?.condition)).toBe('large');
    await expect.poll(() => page.evaluate(() => window.__team?.step ?? 0)).toBeGreaterThan(50);
    expect(page.url()).toContain('drones=1');
    expect(errors).toEqual([]);
    expect(external).toEqual([]);
  });

  test('fits a phone screen', async ({ page }) => {
    await page.setViewportSize({ width: 390, height: 844 });
    await page.goto('/team.html?condition=hard&drones=2&speed=4');
    await expect(page.locator('#app[data-ready="1"]')).toBeAttached({ timeout: 90_000 });
    const overflow = await page.evaluate(() => document.documentElement.scrollWidth - window.innerWidth);
    expect(overflow).toBeLessThanOrEqual(1);
  });
});
