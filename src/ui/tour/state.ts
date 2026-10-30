// Where a visitor is in the guided tour. The step lives in the URL (?tour=<step>), so it survives reloads, the
// cross-origin-isolation service worker's first-visit reload, Back and bfcache. A sessionStorage marker records that
// the visitor started the tour from the Home button: without it a shared ?tour link shows nothing, except the
// closing card (?tour=end), which is static content rather than the tour itself.
import type { SitePage } from '../siteNav';

export type StepId = 'home-results' | 'film-stage' | 'race-lanes' | 'end';

export interface StepPlace {
  id: StepId;
  /** The page the step is shown on; the closing card shows on whatever page the visitor is on. */
  page: SitePage | 'any';
  needsWebgl?: boolean;
}

export const STEP_IDS: readonly StepId[] = ['home-results', 'film-stage', 'race-lanes', 'end'];

export const isStepId = (v: string | null): v is StepId => v !== null && (STEP_IDS as readonly string[]).includes(v);

export function parseTour(search: string): string | null {
  return new URLSearchParams(search).get('tour');
}

/** The step after `id` in the tour as flown (`steps` already has any WebGL-only step removed). */
export function nextStep(steps: readonly StepPlace[], id: StepId): StepId {
  const i = steps.findIndex((s) => s.id === id);
  return steps[i + 1]?.id ?? 'end';
}

export type LoadDecision = { kind: 'idle' } | { kind: 'strip' } | { kind: 'step'; id: StepId } | { kind: 'end' };

/** What a page load does with its ?tour parameter. */
export function decideOnLoad(i: { param: string | null; marker: boolean; page: SitePage | null; steps: readonly StepPlace[] }): LoadDecision {
  if (i.param === null) return { kind: 'idle' };
  if (i.param === 'end') return { kind: 'end' };
  if (!i.marker || !isStepId(i.param)) return { kind: 'strip' };
  const step = i.steps.find((s) => s.id === i.param);
  if (!step || step.page !== i.page) return { kind: 'strip' };
  return { kind: 'step', id: step.id };
}

/** `search` with ?tour set to `id` (null removes it), keeping every other parameter. */
export function withTour(search: string, id: StepId | null): string {
  const q = new URLSearchParams(search);
  if (id) q.set('tour', id);
  else q.delete('tour');
  const s = q.toString();
  return s ? `?${s}` : '';
}

const MARKER_KEY = 'ff-tour';

/** Storage can throw (private windows, blocked site data); the tour then only lives for the current page. */
export const marker = {
  has(): boolean {
    try {
      return sessionStorage.getItem(MARKER_KEY) !== null;
    } catch {
      return false;
    }
  },
  set(): void {
    try {
      sessionStorage.setItem(MARKER_KEY, JSON.stringify({ v: 1, started: Date.now() }));
    } catch {
      /* the tour still runs on this page */
    }
  },
  clear(): void {
    try {
      sessionStorage.removeItem(MARKER_KEY);
    } catch {
      /* nothing stored */
    }
  },
};
