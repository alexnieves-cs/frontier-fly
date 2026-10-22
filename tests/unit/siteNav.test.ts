import { readdirSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { siteNavLinks } from '../../src/ui/siteNav';

describe('site nav', () => {
  it('links every page at the repo root, and marks only the current one', () => {
    const pages = readdirSync('.').filter((name) => name.endsWith('.html'));
    const html = siteNavLinks('race');
    for (const page of pages) expect(html).toContain(`href="./${page}"`);
    expect(html.match(/aria-current="page"/g)).toHaveLength(1);
    expect(html).toMatch(/href="\.\/race\.html"[^>]*aria-current="page"/);
  });
});
