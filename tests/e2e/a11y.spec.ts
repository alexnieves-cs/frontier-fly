import AxeBuilder from '@axe-core/playwright';
import { expect, test, type Page } from '@playwright/test';

// Every page in the site nav (src/ui/siteNav.ts). Add new pages here too.
const PAGES = ['index', 'how', 'film', 'demo', 'race', 'team', 'atlas', 'sim', 'outdoor', 'mission', 'px4'] as const;

/** Canvases that are neither hidden from assistive tech nor an image with a name. */
async function unnamedCanvases(page: Page): Promise<string[]> {
  return page.evaluate(() =>
    [...document.querySelectorAll('canvas')]
      .filter((c) => c.isConnected && !c.closest('[aria-hidden="true"]'))
      .filter((c) => !(c.getAttribute('role') === 'img' && (c.getAttribute('aria-label') ?? '').trim().length > 0))
      .map((c) => c.id || c.className || c.dataset.testid || c.parentElement?.className || 'canvas'),
  );
}

for (const name of PAGES) {
  test(`${name}.html passes axe (WCAG 2.1 A/AA), names its canvases, and shows keyboard focus`, async ({ page }) => {
    await page.goto(`/${name}.html`);
    await page.waitForLoadState('load');
    // Let the page build its live parts (3D view, overlays, panels) before scanning them.
    await page.waitForTimeout(5000);

    const { violations } = await new AxeBuilder({ page }).withTags(['wcag2a', 'wcag2aa', 'wcag21a', 'wcag21aa']).analyze();
    const report = violations.map((v) => `${v.id} (${v.impact}): ${v.nodes.map((n) => n.target.join(' ')).join(', ')}`);
    expect(report, 'axe violations').toEqual([]);

    expect(await unnamedCanvases(page), 'canvases need role="img" and an aria-label, or aria-hidden="true"').toEqual([]);

    // The first Tab lands on a visible control in the page header with a visible focus ring.
    await page.locator('body').focus();
    await page.keyboard.press('Tab');
    const focus = await page.evaluate(() => {
      const el = document.activeElement as HTMLElement | null;
      if (!el || el === document.body) return null;
      const s = getComputedStyle(el);
      const ring = (s.outlineStyle !== 'none' && parseFloat(s.outlineWidth) > 0) || s.boxShadow !== 'none';
      const r = el.getBoundingClientRect();
      return { tag: el.tagName, ring, visible: r.width > 0 && r.height > 0 };
    });
    expect(focus, 'Tab moves focus off the body').not.toBeNull();
    expect(focus!.visible).toBe(true);
    expect(focus!.ring, `focused ${focus!.tag} shows a focus ring`).toBe(true);
  });
}
