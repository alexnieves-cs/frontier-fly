import { describe, expect, it } from 'vitest';
import atlas from '../../docs/atlas.json';
import { results } from '../../src/landing/results';
import { buildClaims } from '../../src/ui/tour/claims';
import { decideOnLoad, nextStep, withTour } from '../../src/ui/tour/state';
import { BODY_MAX, END_TITLE, TITLE_MAX, buildSteps } from '../../src/ui/tour/steps';

const steps = buildSteps({ webgl: true });

describe('tour state', () => {
  it('visits Home, the film, the race, then the closing card', () => {
    expect(steps.map((s) => s.id)).toEqual(['home-results', 'film-stage', 'race-lanes']);
    expect(nextStep(steps, 'home-results')).toBe('film-stage');
    expect(nextStep(steps, 'race-lanes')).toBe('end');
  });

  it('leaves out the film without WebGL', () => {
    const flat = buildSteps({ webgl: false });
    expect(flat.map((s) => s.id)).toEqual(['home-results', 'race-lanes']);
    expect(nextStep(flat, 'home-results')).toBe('race-lanes');
    expect(decideOnLoad({ param: 'film-stage', marker: true, page: 'film', steps: flat })).toEqual({ kind: 'strip' });
  });

  it('runs a step only when started from Home, on its own page', () => {
    expect(decideOnLoad({ param: null, marker: true, page: 'film', steps })).toEqual({ kind: 'idle' });
    expect(decideOnLoad({ param: 'film-stage', marker: true, page: 'film', steps })).toEqual({ kind: 'step', id: 'film-stage' });
    expect(decideOnLoad({ param: 'film-stage', marker: false, page: 'film', steps })).toEqual({ kind: 'strip' });
    expect(decideOnLoad({ param: 'film-stage', marker: true, page: 'race', steps })).toEqual({ kind: 'strip' });
    expect(decideOnLoad({ param: 'nonsense', marker: true, page: 'home', steps })).toEqual({ kind: 'strip' });
  });

  it('opens the closing card from a shared link on any page', () => {
    expect(decideOnLoad({ param: 'end', marker: false, page: 'px4', steps })).toEqual({ kind: 'end' });
  });

  it('sets and removes ?tour without touching other parameters', () => {
    expect(withTour('?condition=hard&seed=3', 'race-lanes')).toBe('?condition=hard&seed=3&tour=race-lanes');
    expect(withTour('?tour=end&seed=3', null)).toBe('?seed=3');
    expect(withTour('?tour=end', null)).toBe('');
  });
});

describe('tour copy', () => {
  it('fits a 360 px card: one title line, two body lines', () => {
    for (const s of steps) {
      expect(s.title.length, s.title).toBeLessThanOrEqual(TITLE_MAX);
      expect(s.body.length, s.body).toBeLessThanOrEqual(BODY_MAX);
    }
    expect(END_TITLE.length).toBeLessThanOrEqual(TITLE_MAX);
  });
});

describe('closing card numbers', () => {
  const claims = buildClaims();
  const text = claims.map((c) => `${c.value} ${c.label} ${c.condition}`).join(' ');

  it('come from the homepage results and the failure atlas', () => {
    const r = results();
    const values = claims.map((c) => c.value);
    expect(values).toContain(`${(r.pilots[0]!.value * 100).toFixed(1)}%`);
    expect(values).toContain(`${(atlas.rates.falseClear.importance.p * 100).toFixed(2)}%`);
    expect(values).toContain(`${(atlas.rates.collision.importance.p * 100).toFixed(2)}%`);
    expect(values).toContain(`${(atlas.conditional.falseClearSweepOccupied.p * 100).toFixed(1)}%`);
  });

  it('show the failure rates beside the coverage, each with its conditions', () => {
    expect(claims.filter((c) => c.tone === 'limit').length).toBeGreaterThanOrEqual(3);
    for (const c of claims) expect(c.condition.length).toBeGreaterThan(20);
  });

  it('do not overclaim', () => {
    expect(text).not.toMatch(/real-world|\bsafe\b|AI-powered|state of the art/i);
  });
});
