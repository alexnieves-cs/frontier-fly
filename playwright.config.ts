import { defineConfig, devices } from '@playwright/test';

export default defineConfig({
  testDir: 'tests/e2e',
  timeout: 180_000,
  expect: { timeout: 20_000 },
  fullyParallel: false,
  retries: 0,
  reporter: [['list']],
  use: {
    baseURL: 'http://127.0.0.1:4173',
    // Local runs use the installed Chrome. CI uses the Playwright Chromium build.
    ...(process.env.CI ? {} : { channel: 'chrome' as const }),
    viewport: { width: 1440, height: 900 },
    launchOptions: {
      args: [
        '--enable-webgl',
        '--ignore-gpu-blocklist',
        '--use-gl=angle',
      ],
    },
  },
  projects: [
    // The outdoor test asserts an fps floor on software WebGL, so it runs alone before the
    // rest instead of sharing the CPU with another rendering page.
    { name: 'outdoor', testMatch: /outdoor\.spec\.ts/, use: { ...devices['Desktop Chrome'] } },
    {
      name: 'chromium',
      testIgnore: /outdoor\.spec\.ts/,
      dependencies: ['outdoor'],
      use: { ...devices['Desktop Chrome'] },
    },
  ],
  webServer: {
    command: 'npm run build && npm run preview -- --host 127.0.0.1 --port 4173 --strictPort',
    url: 'http://127.0.0.1:4173',
    reuseExistingServer: !process.env.CI,
    timeout: 180_000,
  },
});
