import { expect, test } from '@playwright/test';

test('sim flies a real apartment from ?plan=', async ({ page }) => {
  const errors: string[] = [];
  page.on('pageerror', (error) => errors.push(error.message));
  await page.goto('/sim.html?plan=0&quality=low&pilot=policy');
  await expect(page.locator('#app[data-ready="1"]')).toBeVisible({ timeout: 40_000 });
  await expect(page.locator('#app')).toHaveAttribute('data-plan', /^sd-/);
  await page.screenshot({ path: 'test-results/plan-0.png' });
  expect(errors).toEqual([]);
});
