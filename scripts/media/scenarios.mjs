// Clips recorded by scripts/media/record.mjs. Each one opens a page in headless Chromium,
// waits for `#app[data-ready="1"]`, runs `act`, and keeps recording until `seconds` have passed.
//
// `page` must be built by Vite (listed in vite.config.ts). A clip whose page is missing from dist/
// is skipped with a note, so a scenario can be listed before its page lands.
// `gif` picks the excerpt for the README GIF: start offset and length in seconds after ready.

/** @typedef {import('@playwright/test').Page} Page */

/**
 * @typedef {object} Scenario
 * @property {string} id            output file stem in docs/media/
 * @property {string} title         caption used in the README and media.json
 * @property {string} page          HTML file in dist/
 * @property {string} query         query string, including the leading "?"
 * @property {number} seconds       clip length after the page is ready
 * @property {{ start: number, length: number }} gif
 * @property {'low' | 'high'} [quality]
 * @property {boolean} [hero]       use this GIF at the top of the README
 * @property {'wall' | 'sim'} [timing]
 *   wall: play back at the speed the page ran. sim: one captured repaint per 50 ms sim step, so a
 *   page that renders slowly under software WebGL still plays at true flight speed. Anything driven
 *   by real time (training workers) then plays faster than it ran, and the README says so.
 * @property {(page: Page) => Promise<void>} [act]
 */

/** @type {Scenario[]} */
export const SCENARIOS = [
  // Woods-to-building missions. demo.html comes from the mission pipeline work; until it is in
  // dist/ these three are skipped. Scenario ids and the finish signal follow that page.
  ...[
    { id: 'mission-find-target', title: 'Woods to building: find the target', scenario: 'find' },
    { id: 'mission-confirm-clear', title: 'Woods to building: confirm the area clear', scenario: 'clear' },
    { id: 'mission-night-entry', title: 'Night entry through an opening', scenario: 'night' },
  ].map((m, i) => ({
    id: m.id,
    title: m.title,
    page: 'demo.html',
    query: `?scenario=${m.scenario}&quality=low`,
    seconds: 55,
    gif: { start: 8, length: 10 },
    quality: /** @type {const} */ ('low'),
    hero: i === 0,
    act: async (page) => {
      // Stop early when the page reports the mission is over.
      await page
        .waitForFunction(() => document.querySelector('#app')?.getAttribute('data-done') === '1', undefined, { timeout: 55_000 })
        .catch(() => undefined);
    },
  })),
  {
    id: 'training',
    title: 'Live PPO training dashboard',
    page: 'index.html',
    query: '?seed=11&quality=low',
    seconds: 40,
    timing: 'sim',
    gif: { start: 20, length: 10 },
    quality: 'low',
    act: async (page) => {
      await page.locator('#train-btn').click();
      await page.waitForFunction(() => (document.querySelector('#rate-num')?.textContent ?? '0') !== '0', undefined, {
        timeout: 30_000,
      });
    },
  },
  {
    id: 'fpv-night',
    title: 'FPV night flight, shipped policy, easy building',
    page: 'index.html',
    query: '?seed=11&stage=0&camera=fpv&tod=22&flash=1&lights=0&pilot=policy&quality=low',
    seconds: 30,
    timing: 'sim',
    gif: { start: 4, length: 10 },
    quality: 'low',
  },
  {
    id: 'mission-editor',
    title: 'Mission editor, live preview, and report',
    page: 'mission.html',
    query: '?seed=11',
    seconds: 40,
    gif: { start: 3, length: 12 },
    act: async (page) => {
      await page.waitForTimeout(3000);
      await page.locator('[data-testid=mode-run]').click();
      await page.locator('[data-testid=speed]').selectOption('4');
      await page.locator('[data-testid=play]').click();
      await page.locator('[data-testid=approval-prompt]').waitFor({ state: 'visible', timeout: 30_000 });
      await page.waitForTimeout(2500);
      await page.locator('[data-testid=approval-allow]').click();
      await page.locator('[data-testid=speed]').selectOption('16');
      await page.locator('[data-testid=mission-report]').waitFor({ state: 'visible', timeout: 60_000 });
      await page.waitForTimeout(1500);
      await page.locator('[data-testid=mission-report]').scrollIntoViewIfNeeded();
    },
  },
];
