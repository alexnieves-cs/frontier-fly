import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';

// One version number for the repository: the npm package, the citation and the Python package move together.
describe('version numbers', () => {
  const npm = (JSON.parse(readFileSync('package.json', 'utf8')) as { version: string }).version;
  it('CITATION.cff matches package.json', () => {
    expect(readFileSync('CITATION.cff', 'utf8')).toMatch(new RegExp(`^version: "?${npm.replace(/\./g, '\\.')}"?$`, 'm'));
  });
  it('the Python package matches package.json', () => {
    expect(readFileSync('python/pyproject.toml', 'utf8')).toMatch(new RegExp(`^version = "${npm.replace(/\./g, '\\.')}"$`, 'm'));
    expect(readFileSync('python/frontierfly/__init__.py', 'utf8')).toContain(`__version__ = "${npm}"`);
  });
});
