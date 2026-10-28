import { expect, test, type Page } from '@playwright/test';

/** The guided recruiter tour: Home results, the mission film, the pilot race, then the closing card. */

function watch(page: Page): { errors: string[]; requests: string[] } {
  const errors: string[] = [];
  const requests: string[] = [];
  page.on('pageerror', (error) => errors.push(error.message));
  page.on('request', (request) => requests.push(request.url()));
  return { errors, requests };
}

const card = (page: Page) => page.locator('.ff-tour');
const title = (page: Page) => page.locator('.ff-tour-title');
const tourParam = (page: Page) => new URL(page.url()).searchParams.get('tour');

/** The card shows, and takes focus, as soon as the page's markup is up: it never waits for the 3D view. */
async function expectCardSoon(page: Page, heading: string | RegExp): Promise<void> {
  await expect(title(page)).toHaveText(heading, { timeout: 5_000 });
  await expect(title(page)).toBeFocused();
}

/** Tab from the focused card title to Next, then press `key` on it. */
async function pressNext(page: Page, key: 'Enter' | 'Space'): Promise<void> {
  await page.keyboard.press('Tab');
  await expect(page.locator('[data-tour-next]')).toBeFocused();
  await page.keyboard.press(key);
}

test.describe('guided tour', () => {
  test('walks Home, the film and the race to the closing card by keyboard', async ({ page }) => {
    test.setTimeout(300_000);
    const { errors, requests } = watch(page);
    await page.goto('/index.html');
    await expect(page.locator('#app[data-ready="1"]')).toBeVisible();
    await page.locator('#tour-start').evaluate((b: HTMLButtonElement) => b.click());

    await expectCardSoon(page, 'It all runs in this tab');
    expect(tourParam(page)).toBe('home-results');
    await expect(page.locator('.ff-tour-count')).toHaveText('Step 1 of 3');
    await expect(page.locator('[data-tour="results"]')).toHaveClass(/ff-tour-target/);

    await pressNext(page, 'Enter');
    await page.waitForURL(/film\.html\?tour=film-stage/);
    await expectCardSoon(page, 'One mission, woods to building');
    // A reload (or the isolation service worker's first-visit reload) lands on the same step.
    await page.reload();
    await expectCardSoon(page, 'One mission, woods to building');
    await expect(card(page)).toHaveAttribute('data-tour-state', 'ready', { timeout: 150_000 });
    await expect(page.locator('.ff-tour-status')).toContainText('Live in your browser');
    const fps = await page.evaluate(async () => {
      let frames = 0;
      const start = performance.now();
      await new Promise<void>((done) => {
        const tick = () => (++frames, performance.now() - start < 2000 ? requestAnimationFrame(tick) : done());
        requestAnimationFrame(tick);
      });
      return frames / ((performance.now() - start) / 1000);
    });
    expect(fps).toBeGreaterThan(8);

    // Space on the focused Next button belongs to the button, not the film's play/pause shortcut.
    await title(page).focus();
    await pressNext(page, 'Space');
    await page.waitForURL(/race\.html\?.*tour=race-lanes/);
    expect(requests.some((u) => u.endsWith('/race.html'))).toBe(true);
    await expectCardSoon(page, 'Four pilots, one building');
    await expect(card(page)).toHaveAttribute('data-tour-state', 'ready', { timeout: 90_000 });
    expect(tourParam(page)).toBe('race-lanes');

    await pressNext(page, 'Enter');
    await expectCardSoon(page, 'What it does, and where it fails');
    expect(tourParam(page)).toBe('end');
    await expect(page.locator('.ff-tour-claim')).toHaveCount(5);
    await expect(page.locator('.ff-tour-claim.tone-limit')).toHaveCount(3);
    await expect(page.locator('.ff-tour a.ff-tour-primary')).toHaveAttribute('href', 'https://github.com/alexnieves-cs/frontier-fly?ref=tour');
    await expect(page.locator('.ff-tour a.ff-tour-secondary')).toHaveAttribute('href', 'https://alexnieves.me/?from=frontier-fly-tour');

    await page.keyboard.press('Escape');
    await expect(card(page)).toHaveCount(0);
    expect(tourParam(page)).toBeNull();
    expect(requests.some((u) => u.includes('pretrained.bin'))).toBe(false);
    expect(errors).toEqual([]);
  });

  test('skips to the results from any step, and a shared step link shows nothing', async ({ page }) => {
    const { errors } = watch(page);
    // No "started from Home" marker: the step is stripped and no card appears.
    await page.goto('/race.html?tour=race-lanes&condition=easy');
    await expect(page.locator('#app[data-ready="1"]')).toBeAttached({ timeout: 90_000 });
    await expect.poll(() => tourParam(page)).toBeNull();
    await expect(card(page)).toHaveCount(0);
    expect(new URL(page.url()).searchParams.get('condition')).toBe('easy');

    // Started from Home, "Skip to results" opens the closing card in place.
    await page.goto('/index.html');
    await expect(page.locator('#app[data-ready="1"]')).toBeVisible();
    await page.locator('#tour-start').evaluate((b: HTMLButtonElement) => b.click());
    await expectCardSoon(page, 'It all runs in this tab');
    await page.locator('[data-tour-skip]').evaluate((b: HTMLButtonElement) => b.click());
    await expectCardSoon(page, 'What it does, and where it fails');
    expect(tourParam(page)).toBe('end');
    await page.locator('[data-tour-exit]').evaluate((b: HTMLButtonElement) => b.click());
    await expect(card(page)).toHaveCount(0);
    expect(errors).toEqual([]);
  });

  test('a link to the closing card opens it on a phone without loading the 3D hero', async ({ browser }) => {
    const context = await browser.newContext({ viewport: { width: 390, height: 844 }, isMobile: true, hasTouch: true, deviceScaleFactor: 2 });
    const page = await context.newPage();
    const { errors, requests } = watch(page);
    await page.goto('/index.html?tour=end');
    await expectCardSoon(page, 'What it does, and where it fails');
    await page.waitForTimeout(1500);
    await expect(page.locator('#hero-film canvas')).toHaveCount(0);
    expect(requests.some((u) => u.includes('policy.bin'))).toBe(false);
    const box = await card(page).boundingBox();
    expect(box).not.toBeNull();
    expect(box!.x).toBeGreaterThanOrEqual(0);
    expect(box!.x + box!.width).toBeLessThanOrEqual(390.5);
    expect(box!.y + box!.height).toBeLessThanOrEqual(844.5);
    expect(await page.evaluate(() => document.documentElement.scrollWidth - document.documentElement.clientWidth)).toBeLessThanOrEqual(0);
    // On a phone the sheet folds to one bar when asked, and opens again.
    await page.locator('[data-tour-collapse]').tap();
    await expect(page.locator('.ff-tour-inner')).toBeHidden();
    await page.locator('[data-tour-collapse]').tap();
    await expect(page.locator('.ff-tour-inner')).toBeVisible();
    expect(errors).toEqual([]);
    await context.close();
  });

  test('the phone tour docks above the film controls', async ({ browser }) => {
    test.setTimeout(240_000);
    const context = await browser.newContext({ viewport: { width: 390, height: 844 }, isMobile: true, hasTouch: true, deviceScaleFactor: 2 });
    const page = await context.newPage();
    await page.addInitScript(() => sessionStorage.setItem('ff-tour', '{"v":1}'));
    await page.goto('/film.html?tour=film-stage');
    await expectCardSoon(page, 'One mission, woods to building');
    await expect(card(page)).toHaveAttribute('data-tour-state', 'ready', { timeout: 150_000 });
    const [sheet, controls] = await Promise.all([card(page).boundingBox(), page.locator('.film-controls').boundingBox()]);
    expect(sheet!.y + sheet!.height).toBeLessThanOrEqual(controls!.y + 1);
    expect(sheet!.height).toBeLessThanOrEqual(844 * 0.4 + 1);
    await context.close();
  });
});
