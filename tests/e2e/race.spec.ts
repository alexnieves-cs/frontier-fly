import { expect, test, type Page } from '@playwright/test';

/** Live pilot race (race.html): every lane runs in the tab and lands on the published study numbers. */

function watch(page: Page): { errors: string[]; external: string[] } {
  const errors: string[] = [];
  const external: string[] = [];
  page.on('pageerror', (error) => errors.push(error.message));
  page.on('request', (request) => {
    const url = new URL(request.url());
    if (url.hostname !== '127.0.0.1' && url.protocol !== 'data:' && url.protocol !== 'blob:') external.push(request.url());
  });
  return { errors, external };
}

async function raceStatus(page: Page): Promise<string | undefined> {
  return page.evaluate(() => window.__race?.status);
}

test.describe('pilot race', () => {
  test.use({ viewport: { width: 1440, height: 900 } });

  test('races four pilots to the study result and inspects any lane', async ({ page }) => {
    test.setTimeout(240_000);
    const { errors, external } = watch(page);
    const response = await page.goto('/race.html?condition=hard&seed=30001&speed=max');
    expect(response?.headers()['cross-origin-opener-policy']).toBe('same-origin');
    expect(response?.headers()['cross-origin-embedder-policy']).toBe('require-corp');
    await expect(page.locator('#app[data-ready="1"]')).toBeAttached({ timeout: 90_000 });
    await expect(page.locator('.lane')).toHaveCount(4);
    expect(await page.evaluate(() => window.__race?.policyLoaded)).toBe(true);

    // Look through the oracle's sensors while it flies.
    await page.locator('.lane[data-lane="3"]').click();
    await expect(page.locator('[data-ins-lane="3"]')).toHaveAttribute('aria-selected', 'true');
    await expect(page.locator('.ins-who')).toContainText('Oracle', { timeout: 10_000 });
    await expect(page.locator('.ins-ghost-note')).toBeVisible();
    await page.locator('[data-ins-lane="0"]').click();
    await expect(page.locator('.ins-who')).toContainText('Learned policy', { timeout: 10_000 });

    await expect.poll(() => raceStatus(page), { timeout: 180_000, intervals: [500] }).toBe('finished');
    const state = await page.evaluate(() => window.__race!);
    expect(state.step).toBe(2400);
    expect(state.lanes.map((l) => l.pilot)).toEqual(['policy', 'classical', 'waypoint', 'teacher']);
    for (const lane of state.lanes) {
      expect(lane.study, lane.pilot).not.toBeNull();
      expect(lane.match, `${lane.pilot} ${JSON.stringify(lane)}`).toBe(true);
    }
    await expect(page.locator('#race-result')).toBeVisible();
    await expect(page.locator('.repro.ok')).toContainText('Reproduces the study');

    // The policy's attribution bars are live numbers, not placeholders.
    const shares = await page.locator('.ins-infl-value').allTextContents();
    expect(shares.some((s) => /^\d+%$/.test(s) && s !== '0%')).toBe(true);

    // The lane canvases drew something besides the background.
    const painted = await page.evaluate(() => {
      const canvas = document.querySelector<HTMLCanvasElement>('.lane-canvas')!;
      const data = canvas.getContext('2d')!.getImageData(0, 0, canvas.width, canvas.height).data;
      const colors = new Set<number>();
      for (let i = 0; i < data.length; i += 4 * 97) colors.add((data[i]! << 16) | (data[i + 1]! << 8) | data[i + 2]!);
      return colors.size;
    });
    expect(painted).toBeGreaterThan(5);

    // Next building moves to the next study seed and keeps it in the link.
    await page.locator('#race-result [data-act="next"]').click();
    await expect(page).toHaveURL(/seed=30002/);
    await expect.poll(() => raceStatus(page), { timeout: 60_000 }).toBe('racing');

    expect(errors).toEqual([]);
    expect(external).toEqual([]);
  });

  test('fits a phone screen', async ({ browser }) => {
    test.setTimeout(120_000);
    const context = await browser.newContext({ viewport: { width: 390, height: 844 }, isMobile: true, hasTouch: true, deviceScaleFactor: 2 });
    const page = await context.newPage();
    const { errors } = watch(page);
    await page.goto('/race.html?condition=easy&speed=max');
    await expect(page.locator('#app[data-ready="1"]')).toBeAttached({ timeout: 90_000 });
    await expect(page.locator('.lane')).toHaveCount(4);
    const layout = await page.evaluate(() => {
      const lanes = [...document.querySelectorAll<HTMLElement>('.lane')].map((el) => el.getBoundingClientRect());
      return {
        overflow: document.documentElement.scrollWidth - document.documentElement.clientWidth,
        firstRow: lanes.filter((r) => Math.abs(r.top - lanes[0]!.top) < 2).length,
        laneWidth: lanes[0]!.width,
      };
    });
    expect(layout.overflow).toBeLessThanOrEqual(0);
    expect(layout.firstRow).toBe(2);
    expect(layout.laneWidth).toBeGreaterThan(140);
    await expect.poll(() => raceStatus(page), { timeout: 90_000, intervals: [500] }).toBe('finished');
    await expect(page.locator('#race-result')).toBeVisible();
    expect(errors).toEqual([]);
    await context.close();
  });
});
