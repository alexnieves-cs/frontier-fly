// The guided tour's stops and their words. Titles fit one line of a 360 px card (32 characters), bodies two (82).
import type { StepId, StepPlace } from './state';

export interface TourStep extends StepPlace {
  /** data-tour anchor the step points at. */
  anchor?: string;
  /** Page whose readiness the card reports while the 3D view builds. */
  signal?: 'film' | 'race';
  title: string;
  body: string;
  next: string;
  /** The Next label while the page is still building. */
  nextWaiting?: string;
}

export const TITLE_MAX = 32;
export const BODY_MAX = 82;

const STEPS: readonly TourStep[] = [
  {
    id: 'home-results',
    page: 'home',
    anchor: 'results',
    title: 'It all runs in this tab',
    body: 'The sim runs in your browser, no server. Bars: share of unseen buildings searched.',
    next: 'Next: the full mission',
  },
  {
    id: 'film-stage',
    page: 'film',
    anchor: 'film-stage',
    signal: 'film',
    needsWebgl: true,
    title: 'One mission, woods to building',
    body: 'Flown headless in your tab, then replayed in 3D: woods, an opening, room by room.',
    next: 'Next: race the pilots',
    nextWaiting: 'Skip ahead: race',
  },
  {
    id: 'race-lanes',
    page: 'race',
    anchor: 'race-lanes',
    signal: 'race',
    title: 'Four pilots, one building',
    body: 'The policy covers more; the planner collides less. Same building, same seed.',
    next: 'See the results',
    nextWaiting: 'Skip to the results',
  },
];

export const END_TITLE = 'What it does, and where it fails';

/** The tour as flown: without WebGL the film stop is left out. */
export function buildSteps(o: { webgl: boolean }): readonly TourStep[] {
  return STEPS.filter((s) => o.webgl || !s.needsWebgl);
}

export function stepById(steps: readonly TourStep[], id: StepId): TourStep | undefined {
  return steps.find((s) => s.id === id);
}
