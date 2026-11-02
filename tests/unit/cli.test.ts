// The headless CLI (scripts/cli): option parsing, and that its flights are the bench's flights.
import { mkdtempSync, readFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { parseArgs, UsageError } from '../../scripts/cli/args';
import { main } from '../../scripts/cli/frontier-fly';

describe('parseArgs', () => {
  const known = { seed: 'int', out: 'value', log: 'flag' } as const;

  it('reads values, = values, ints and flags', () => {
    const a = parseArgs(['--seed', '42', '--out=x.json', '--log'], known);
    expect(a.int('seed')).toBe(42);
    expect(a.string('out')).toBe('x.json');
    expect(a.flag('log')).toBe(true);
    expect(a.string('missing')).toBeUndefined();
  });

  it('rejects typos, missing values, bad ints and stray words before anything runs', () => {
    expect(() => parseArgs(['--sed', '1'], known)).toThrow(UsageError);
    expect(() => parseArgs(['--seed'], known)).toThrow(/needs a value/);
    expect(() => parseArgs(['--seed', '--log'], known)).toThrow(/needs a value/);
    expect(() => parseArgs(['--seed', '4.5'], known)).toThrow(/whole number/);
    expect(() => parseArgs(['cabin'], known)).toThrow(/unexpected argument/);
    expect(() => parseArgs(['--log=yes'], known)).toThrow(/takes no value/);
  });
});

describe('frontier-fly CLI', () => {
  let stdout: string[];
  let stderr: string[];
  beforeEach(() => {
    stdout = [];
    stderr = [];
    vi.spyOn(process.stdout, 'write').mockImplementation((s) => (stdout.push(String(s)), true));
    vi.spyOn(process.stderr, 'write').mockImplementation((s) => (stderr.push(String(s)), true));
  });
  afterEach(() => vi.restoreAllMocks());

  it('exits 2 with a message on a usage error', async () => {
    expect(await main(['mission', '--scenario', 'moon-base'])).toBe(2);
    expect(stderr.join('')).toMatch(/unknown scenario "moon-base"/);
    expect(await main(['teleport'])).toBe(2);
    expect(await main([])).toBe(2);
    expect(await main(['--help'])).toBe(0);
    expect(stdout.join('')).toMatch(/Usage: frontier-fly <command>/);
  });

  it('flies the cabin mission to the expected outcome and writes its files', async () => {
    const dir = mkdtempSync(join(tmpdir(), 'ff-cli-'));
    const code = await main(['mission', '--scenario', 'cabin-day', '--pilot', 'sensor', '--json', join(dir, 'm.json'), '--track', join(dir, 't.csv')]);
    expect(code).toBe(0);
    expect(stdout.join('')).toMatch(/Outcome: found \(as expected\)/);
    const json = JSON.parse(readFileSync(join(dir, 'm.json'), 'utf8')) as { seed: number; asExpected: boolean; report: { outcome: string } };
    expect(json).toMatchObject({ seed: 20, asExpected: true, report: { outcome: 'found' } });
    const csv = readFileSync(join(dir, 't.csv'), 'utf8').trim().split('\n');
    expect(csv[0]).toBe('t,x,y,z,phase');
    expect(csv.length).toBeGreaterThan(100);
  });

  it('scores the same flights as bench:score (bench/results)', async () => {
    const dir = mkdtempSync(join(tmpdir(), 'ff-cli-'));
    const out = join(dir, 'b.json');
    expect(await main(['bench', '--pilot', 'planner', '--split', 'test', '--tasks', 'room,hard', '--limit', '2', '--out', out])).toBe(0);
    const ours = (JSON.parse(readFileSync(out, 'utf8')) as { partial: boolean; episodes: { task: string; seed: number }[] });
    expect(ours.partial).toBe(true);
    const published = (JSON.parse(readFileSync('bench/results/planner.json', 'utf8')) as { episodes: { task: string; seed: number }[] }).episodes;
    expect(ours.episodes).toHaveLength(4);
    for (const e of ours.episodes) expect(e).toEqual(published.find((p) => p.task === e.task && p.seed === e.seed));
  });

  it('exports a QGroundControl plan', async () => {
    const dir = mkdtempSync(join(tmpdir(), 'ff-cli-'));
    expect(await main(['plan', '--seed', '4242', '--out', join(dir, 'w.plan')])).toBe(0);
    const plan = JSON.parse(readFileSync(join(dir, 'w.plan'), 'utf8')) as { fileType: string; mission: { items: unknown[] } };
    expect(plan.fileType).toBe('Plan');
    expect(plan.mission.items.length).toBeGreaterThan(3);
  });
});
