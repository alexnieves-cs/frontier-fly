// The guided recruiter tour: a small card docked over the real, live pages (Home results, then the 3D mission
// film, then the pilot race) that ends on a card with the honest numbers. The visitor sets the pace with Next;
// nothing advances on its own, and Next, Skip and Exit always work, even while a page is still building its 3D view.
// Loaded only when a tour is running (siteNav.ts imports it when the URL has ?tour, the Home button on click).
import './tour.css';
import { REPO_URL, currentSitePage, type SitePage } from '../siteNav';
import { buildClaims } from './claims';
import { decideOnLoad, marker, nextStep, parseTour, withTour, type StepId } from './state';
import { END_TITLE, buildSteps, stepById, type TourStep } from './steps';

export const GITHUB_CTA = `${REPO_URL}?ref=tour`;
export const PORTFOLIO_CTA = 'https://alexnieves.me/?from=frontier-fly-tour';

type PageState = 'waiting' | 'ready' | 'failed';

const STATUS_TEXT: Record<PageState, string> = {
  waiting: 'Building the world in your tab…',
  ready: 'Live in your browser.',
  failed: 'The 3D view could not start here. Next still works.',
};

let webgl: boolean | undefined;

export function hasWebgl(): boolean {
  if (webgl === undefined) {
    try {
      const canvas = document.createElement('canvas');
      webgl = !!(canvas.getContext('webgl2') ?? canvas.getContext('webgl'));
    } catch {
      webgl = false;
    }
  }
  return webgl;
}

const reducedMotion = (): boolean => window.matchMedia?.('(prefers-reduced-motion: reduce)').matches ?? false;

function pageState(signal: TourStep['signal']): PageState {
  if (signal === 'film') {
    const s = window.__film?.status;
    return s === 'ready' ? 'ready' : s === 'error' ? 'failed' : 'waiting';
  }
  if (signal === 'race') {
    const s = window.__race?.status;
    return s === 'error' ? 'failed' : s && s !== 'loading' ? 'ready' : 'waiting';
  }
  return 'ready';
}

const esc = (s: string): string => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');

function setUrl(id: StepId | null): void {
  history.replaceState(history.state, '', `${location.pathname}${withTour(location.search, id)}${location.hash}`);
}

const PAGE_FILE: Record<string, string> = { home: './index.html', film: './film.html', race: './race.html' };

class Tour {
  private readonly steps = buildSteps({ webgl: hasWebgl() });
  private card: HTMLElement | null = null;
  private target: HTMLElement | null = null;
  private poll = 0;
  private waitAnnounce = 0;

  constructor(private readonly page: SitePage | null) {
    // Capture, so the check below sees the site menu before the menu's own Escape handler closes it.
    document.addEventListener('keydown', this.onKey, true);
  }

  show(id: StepId): void {
    if (id === 'end') return this.showEnd();
    const step = stepById(this.steps, id);
    if (!step) return this.showEnd();
    setUrl(id);
    const index = this.steps.indexOf(step);
    const card = this.mount(`
      <p class="ff-tour-count">Step ${index + 1} of ${this.steps.length}</p>
      <h2 class="ff-tour-title" tabindex="-1">${esc(step.title)}</h2>
      <p class="ff-tour-body">${esc(step.body)}</p>
      ${step.signal ? `<p class="ff-tour-status" data-state="waiting"><span class="ff-tour-dot" aria-hidden="true"></span><span class="ff-tour-status-text"></span></p>` : ''}
      <div class="ff-tour-actions">
        <button type="button" class="ff-tour-primary" data-tour-next>${esc(step.next)}</button>
        ${this.extraAction(step)}
        ${index < this.steps.length - 1 ? '<button type="button" class="ff-tour-link" data-tour-skip>Skip to results</button>' : ''}
      </div>`);
    card.querySelector('[data-tour-next]')!.addEventListener('click', () => this.advance(step));
    card.querySelector('[data-tour-skip]')?.addEventListener('click', () => this.showEnd());
    card.querySelector('[data-tour-extra]')?.addEventListener('click', () => this.runExtra(step));
    this.focusTitle();
    void this.point(step);
    if (step.signal) this.watch(step);
  }

  /** Under reduced motion the film starts paused and the race runs at 1x: offer playing them as an explicit choice. */
  private extraAction(step: TourStep): string {
    if (!reducedMotion()) return '';
    if (step.id === 'film-stage') return '<button type="button" class="ff-tour-secondary" data-tour-extra>Play the film</button>';
    if (step.id === 'race-lanes') return '<button type="button" class="ff-tour-secondary" data-tour-extra>Run at 16x</button>';
    return '';
  }

  private runExtra(step: TourStep): void {
    if (step.id === 'film-stage') document.querySelector<HTMLButtonElement>('.film-play[aria-pressed="false"]')?.click();
    if (step.id === 'race-lanes') document.querySelector<HTMLButtonElement>('[data-speed="16"]')?.click();
  }

  private advance(step: TourStep): void {
    const next = nextStep(this.steps, step.id);
    const nextStepDef = stepById(this.steps, next);
    if (!nextStepDef || nextStepDef.page === this.page || nextStepDef.page === 'any') return this.show(next);
    // Leave the page with the URL already on the next step, so Back returns here and a reload resumes there.
    location.assign(`${PAGE_FILE[nextStepDef.page]}${withTour('', next)}`);
  }

  showEnd(): void {
    this.stopWatching();
    this.clearTarget();
    setUrl('end');
    const rows = buildClaims()
      .map(
        (c, i) => `
        <li class="ff-tour-claim tone-${c.tone}" aria-describedby="ff-tour-claim-${i}">
          <b>${esc(c.value)}</b> <span>${esc(c.label)}</span>
          <span class="sr-only" id="ff-tour-claim-${i}">${esc(c.condition)}</span>
        </li>`,
      )
      .join('');
    const card = this.mount(
      `
      <h2 class="ff-tour-title" tabindex="-1">${esc(END_TITLE)}</h2>
      <ul class="ff-tour-claims">${rows}</ul>
      <p class="ff-tour-foot">Simulation only, seeded and reproducible. Intervals and test conditions are on the
        <a href="./atlas.html">Safety page</a> and in the <a href="${REPO_URL}/blob/main/docs/REPORT.md" rel="noopener">report</a>.</p>
      <div class="ff-tour-actions">
        <a class="ff-tour-primary" href="${GITHUB_CTA}" rel="noopener">Read the code on GitHub</a>
        <a class="ff-tour-secondary" href="${PORTFOLIO_CTA}" rel="noopener">More of Alex's work</a>
      </div>
      <p class="ff-tour-more">Go deeper: <a href="./how.html">How it works</a> · <a href="./demo.html">Run a rescue mission</a> · <a href="./px4.html">PX4 in the loop</a> ·
        <button type="button" class="ff-tour-link" data-tour-replay>Replay the tour</button></p>`,
      true,
    );
    card.querySelector('[data-tour-replay]')!.addEventListener('click', () => startTour());
    this.focusTitle();
  }

  exit(): void {
    marker.clear();
    setUrl(null);
    this.destroy();
  }

  destroy(): void {
    this.stopWatching();
    this.clearTarget();
    this.card?.remove();
    this.card = null;
    document.removeEventListener('keydown', this.onKey, true);
  }

  private mount(inner: string, end = false): HTMLElement {
    if (!this.card) {
      this.card = document.createElement('section');
      this.card.className = 'ff-tour';
      this.card.setAttribute('aria-label', 'Guided tour');
      if (this.page) this.card.dataset.page = this.page;
      document.body.appendChild(this.card);
    }
    const card = this.card;
    card.classList.toggle('ff-tour-end', end);
    card.classList.remove('collapsed');
    card.innerHTML = `
      <div class="ff-tour-bar">
        <button type="button" class="ff-tour-collapse" aria-expanded="true" data-tour-collapse>Hide</button>
        <button type="button" class="ff-tour-exit" aria-label="Exit tour" data-tour-exit>×</button>
      </div>
      <div class="ff-tour-inner">${inner}</div>
      <p class="sr-only" aria-live="polite" data-tour-live></p>`;
    card.querySelector('[data-tour-exit]')!.addEventListener('click', () => this.exit());
    const collapse = card.querySelector<HTMLButtonElement>('[data-tour-collapse]')!;
    collapse.addEventListener('click', () => {
      const collapsed = card.classList.toggle('collapsed');
      collapse.textContent = collapsed ? 'Show tour' : 'Hide';
      collapse.setAttribute('aria-expanded', String(!collapsed));
    });
    return card;
  }

  private focusTitle(): void {
    this.card?.querySelector<HTMLElement>('.ff-tour-title')?.focus({ preventScroll: true });
  }

  private announce(text: string): void {
    const live = this.card?.querySelector('[data-tour-live]');
    if (live) live.textContent = text;
  }

  /** Ring the step's target once its page has drawn it; the card itself never waits for it. */
  private async point(step: TourStep): Promise<void> {
    if (!step.anchor) return;
    const deadline = performance.now() + 10_000;
    let el: HTMLElement | null = null;
    while (!(el = document.querySelector<HTMLElement>(`[data-tour="${step.anchor}"]`)) && performance.now() < deadline) {
      await new Promise((r) => setTimeout(r, 100));
    }
    if (!el || !this.card) return;
    this.clearTarget();
    this.target = el;
    if (step.page !== 'film') {
      el.classList.add('ff-tour-target');
      el.scrollIntoView({ block: 'start', behavior: reducedMotion() ? 'auto' : 'smooth' });
    }
  }

  private clearTarget(): void {
    this.target?.classList.remove('ff-tour-target');
    this.target = null;
  }

  private watch(step: TourStep): void {
    let last: PageState | null = null;
    const update = () => {
      const state = pageState(step.signal);
      if (state === last) return;
      last = state;
      const status = this.card?.querySelector<HTMLElement>('.ff-tour-status');
      if (status) {
        status.dataset.state = state;
        status.querySelector('.ff-tour-status-text')!.textContent = STATUS_TEXT[state];
      }
      this.card?.setAttribute('data-tour-state', state);
      const next = this.card?.querySelector('[data-tour-next]');
      if (next) next.textContent = state === 'waiting' && step.nextWaiting ? step.nextWaiting : step.next;
      window.clearTimeout(this.waitAnnounce);
      // Say "building" only if the wait is long enough to notice; say when it is live or failed.
      if (state === 'waiting') this.waitAnnounce = window.setTimeout(() => this.announce(STATUS_TEXT.waiting), 1500);
      else this.announce(STATUS_TEXT[state]);
      if (state !== 'waiting') this.stopWatching(false);
    };
    update();
    this.poll = window.setInterval(update, 250);
    if (step.id === 'film-stage') prefetchNext();
  }

  private stopWatching(clearAnnounce = true): void {
    window.clearInterval(this.poll);
    if (clearAnnounce) window.clearTimeout(this.waitAnnounce);
  }

  private readonly onKey = (e: KeyboardEvent): void => {
    if (e.key !== 'Escape' || !this.card) return;
    // Escape closes the site's page menu first.
    if (document.querySelector('.site-menu.open')) return;
    this.exit();
  };
}

/** While the film plays, fetch the race page at idle so the next stop starts sooner. Not on data-saver links. */
function prefetchNext(): void {
  const connection = (navigator as Navigator & { connection?: { saveData?: boolean; effectiveType?: string } }).connection;
  if (connection?.saveData || /2g/.test(connection?.effectiveType ?? '')) return;
  const add = () => {
    if (document.querySelector('link[data-tour-prefetch]')) return;
    const link = document.createElement('link');
    link.rel = 'prefetch';
    link.href = './race.html';
    link.dataset.tourPrefetch = '';
    document.head.appendChild(link);
  };
  if ('requestIdleCallback' in window) window.requestIdleCallback(add, { timeout: 4000 });
  else setTimeout(add, 2000);
}

let active: Tour | null = null;

/** The Home button: start from the first step. Elsewhere (the closing card's "Replay"), go home first. */
export function startTour(): void {
  marker.set();
  const page = currentSitePage();
  if (page !== 'home') {
    location.assign(`./index.html${withTour('', 'home-results')}`);
    return;
  }
  active?.destroy();
  active = new Tour(page);
  active.show('home-results');
}

/** Called on every page load with ?tour in its URL. */
export function resumeTour(page: SitePage | null = currentSitePage()): void {
  const decision = decideOnLoad({ param: parseTour(location.search), marker: marker.has(), page, steps: buildSteps({ webgl: hasWebgl() }) });
  if (decision.kind === 'idle') return;
  if (decision.kind === 'strip') return setUrl(null);
  active?.destroy();
  active = new Tour(page);
  if (decision.kind === 'end') active.showEnd();
  else active.show(decision.id);
}

// bfcache: a page restored by Back runs no scripts, so re-read the URL.
window.addEventListener('pageshow', (e) => {
  if (e.persisted && parseTour(location.search) !== null) resumeTour();
});
