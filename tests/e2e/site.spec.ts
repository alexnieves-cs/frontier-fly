import { expect, test } from '@playwright/test';

test('the homepage scrolls to its results and footer once the hero film has loaded its styles', async ({ page }) => {
  await page.goto('/index.html');
  await expect(page.locator('#app[data-ready="1"]')).toBeVisible();
  // The hero imports the film page's stylesheet, which used to lock html and body to the window.
  await expect(page.locator('#hero-film canvas')).toBeAttached({ timeout: 120_000 });
  const overflow = await page.evaluate(() => [getComputedStyle(document.documentElement).overflowY, getComputedStyle(document.body).overflowY]);
  expect(overflow).not.toContain('hidden');
  expect(await page.evaluate(() => document.documentElement.scrollHeight)).toBeGreaterThan(900);
  await page.locator('#results-title').scrollIntoViewIfNeeded();
  await expect(page.locator('#results-title')).toBeInViewport();
  await page.locator('.foot').scrollIntoViewIfNeeded();
  expect(await page.evaluate(() => window.scrollY)).toBeGreaterThan(0);
});

test('on a phone the page links fold into a menu that opens, closes, and marks the current page', async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto('/race.html');
  const toggle = page.locator('.site-menu-toggle');
  await expect(toggle).toBeVisible();
  await expect(page.locator('.site-nav a[href="./team.html"]')).toBeHidden();
  await toggle.click();
  await expect(toggle).toHaveAttribute('aria-expanded', 'true');
  await expect(page.locator('.site-nav a[href="./team.html"]')).toBeVisible();
  await expect(page.locator('.site-nav a[aria-current="page"]')).toHaveText('Race');
  await page.keyboard.press('Escape');
  await expect(toggle).toHaveAttribute('aria-expanded', 'false');
  // No page scrolls sideways at phone width.
  expect(await page.evaluate(() => document.documentElement.scrollWidth)).toBeLessThanOrEqual(390);
});
