import '../film/film.css';
import './landing.css';
import type { FilmStatus } from '../film/page';
import { savingData } from '../ui/network';
import { prefersReducedMotion, REPO_URL as REPO, siteHeader } from '../ui/siteNav';
import { isWebglError } from '../ui/startError';
import { atlasRates } from '../ui/tour/claims';
import { pct, results, type PilotResult } from './results';

const HERO_SCENARIO = 'cabin-day';

const contacts = (n: number): string => (n === 0 ? 'no wall contacts' : `${n.toFixed(1)} wall contacts per flight`);
/** "3 of 30 flights crashed · 0.4 wall contacts per flight", or just the contacts before the crash rule. */
function safety(p: PilotResult): string {
  if (p.crashes === null) return contacts(p.contacts);
  const crashed = p.crashes === 0 ? 'no crashes' : `${p.crashes} of ${p.episodes} flights crashed`;
  return `${crashed} · ${contacts(p.contacts)}`;
}

function markup(): string {
  const r = results();
  const falseClear = atlasRates().falseClear;
  const bars = r.pilots
    .map(
      (p) => `
        <li class="bar tone-${p.tone}">
          <span class="bar-name">${p.name}<small class="bar-contacts">${safety(p)}</small></span>
          <span class="bar-track"><span class="bar-fill" style="width:${(p.value * 100).toFixed(1)}%"></span></span>
          <span class="bar-value">${pct(p.value)}</span>
        </li>`,
    )
    .join('');
  return `
  <div class="landing">
    ${siteHeader('home', { heading: false })}
    <section class="hero" data-tour="hero" aria-label="Mission replay">
      <div class="hero-film" id="hero-film"></div>
      <div class="hero-poster" id="hero-poster">
        <img class="hero-still" src="./hero/poster.webp" width="960" height="540" alt="" fetchpriority="high" decoding="async" />
        <video class="hero-loop" muted playsinline loop preload="none" aria-hidden="true" tabindex="-1" disablepictureinpicture>
          <source src="./hero/loop.webm" type="video/webm" />
          <source src="./hero/loop.mp4" type="video/mp4" />
        </video>
      </div>
      <p class="hero-badge" id="hero-badge" role="status" aria-live="polite">
        <span class="badge-dot" aria-hidden="true"></span><span id="poster-status">Recorded flight · starting the live sim…</span>
      </p>
      <button type="button" class="hero-live" id="hero-live" hidden>Fly it live in this tab (about 7 MB)</button>
    </section>

    <section class="intro">
      <div class="intro-copy">
        <p class="kicker">AUTONOMOUS SEARCH AND RESCUE · RUNS IN YOUR TAB</p>
        <h1>frontier-fly</h1>
        <p class="pitch">
          A rescue drone that sweeps the woods, finds the building, flies in through an opening it fits, and searches
          room by room, in buildings it has never seen. The physics, the sensors, the learned autopilot and the 3D replay
          above all run in this tab. No server.
        </p>
        <ul class="proof" aria-label="Headline results">
          <li><strong>${pct(r.pilots[0]!.value)}</strong> of ${r.seeds} unseen hard buildings searched${r.pilots[0]!.crashes !== null ? `, ${r.pilots[0]!.crashes === 0 ? 'no' : r.pilots[0]!.crashes} crash${r.pilots[0]!.crashes === 1 ? '' : 'es'}` : ''}</li>
          <li><strong>${pct(r.easy)}</strong> of easy ones</li>
          <li><strong>${pct(Math.min(1, r.nightOverDay))}</strong> as good at night, lights off</li>
        </ul>
        <p class="method">
          The indoor pilot is a small network learned by imitation (behavior cloning + DAgger) from a privileged planner,
          then fine-tuned with PPO on its own flights. <a href="#results-title">See the numbers</a>.
        </p>
      </div>
      <nav class="actions" aria-label="Start here">
        <a class="btn primary" href="./film.html">Watch the full mission</a>
        <button type="button" class="btn tour-start" id="tour-start">Take the 90-second tour</button>
        <a class="btn" href="./race.html">Race the pilots live</a>
        <a class="btn" href="./team.html">Send in a drone team</a>
        <a class="btn" href="./sim.html">Fly it yourself</a>
        <span class="more-links">
          <a href="./demo.html">Run a rescue mission</a>
          <a href="./mission.html">Plan a mission</a>
          <a href="./sim.html?train=1&amp;quality=low">Train it in your tab</a>
          <a href="./outdoor.html">Fly the outdoor site</a>
          <a href="${REPO}" rel="noopener">GitHub</a>
        </span>
      </nav>
    </section>

    <section class="results" data-tour="results" aria-labelledby="results-title">
      <div class="results-head">
        <h2 id="results-title">Indoor coverage on unseen hard buildings</h2>
        <p>Fraction of each building the drone saw before time ran out, and how often it touched a wall. Stage ${r.hardStage}, mean of ${r.seeds} fresh buildings no training or tuning ever saw, daylight.</p>
      </div>
      <div class="results-body">
        <ul class="bars">${bars}
        </ul>
        <div class="tiles">
          <div class="tile"><span class="tile-value">${pct(r.easy)}</span><span class="tile-label">coverage on easy buildings (${r.seeds} fresh)</span></div>
          <div class="tile"><span class="tile-value">${pct(Math.min(1, r.nightOverDay))}</span><span class="tile-label">night coverage relative to day, lights off</span></div>
          <a class="tile tile-limit" href="./atlas.html"><span class="tile-value">${(falseClear.p * 100).toFixed(2)}%</span><span class="tile-label">of flights call a building clear with someone inside (failure atlas)</span></a>
        </div>
      </div>
      <p class="results-foot">
        Every pilot flies the same ${r.seeds} buildings, measured at commit <code>${r.commit}</code>; the guarded policy's 95% interval is
        ${pct(r.pilots[0]!.ci[0])} to ${pct(r.pilots[0]!.ci[1])}.
        ${r.crashSpeed !== null ? `A contact faster than ${r.crashSpeed} m/s is a crash and ends the flight; coverage counts what was flown until then.` : ''}
        ${r.guardSaves ? 'Alone, the policy hits walls hard: it learned before the crash rule, when contacts were free. The lidar guard is what makes it safe.' : ''}
        Night, multi-floor and quadrotor results are in
        <a href="${REPO}/blob/main/docs/REPORT.md" rel="noopener">REPORT.md</a>; method and every condition in
        <a href="${REPO}/blob/main/docs/BENCHMARKS.md" rel="noopener">BENCHMARKS.md</a>.
      </p>
    </section>

    <footer class="foot">
      <span>frontier-fly · civilian search, inspection, and confirm-clear only · MIT</span>
      <span class="foot-links">
        <a href="./mission.html">Plan a mission</a>
        <a href="./px4.html">PX4 in the loop</a>
        <a href="${REPO}/blob/main/docs/ARCHITECTURE.md" rel="noopener">Architecture</a>
        <a href="${REPO}" rel="noopener">Source</a>
      </span>
    </footer>
  </div>`;
}

/** Plays the recorded loop under the poster frame, unless the visitor asked for less motion or to save data. */
function playLoop(video: HTMLVideoElement, poster: HTMLElement): void {
  if (prefersReducedMotion() || savingData()) return;
  video.preload = 'auto';
  video.addEventListener('playing', () => poster.classList.add('moving'), { once: true });
  // Autoplay can be refused (Low Power Mode, some embeds): the poster frame stays, which is fine.
  video.play().catch(() => undefined);
}

function hasWebgl(): boolean {
  try {
    const canvas = document.createElement('canvas');
    return Boolean(canvas.getContext('webgl2') ?? canvas.getContext('webgl'));
  } catch {
    return false;
  }
}

/** Runs `fn` once the page has loaded, the browser is idle, and the hero is on screen. */
function whenSettledAndVisible(target: HTMLElement, fn: () => void): void {
  const onScreen = () => {
    const io = new IntersectionObserver((entries) => {
      if (!entries.some((e) => e.isIntersecting)) return;
      io.disconnect();
      fn();
    });
    io.observe(target);
  };
  const idle = () => {
    if (typeof window.requestIdleCallback === 'function') window.requestIdleCallback(onScreen, { timeout: 1500 });
    else setTimeout(onScreen, 200);
  };
  if (document.readyState === 'complete') idle();
  else window.addEventListener('load', idle, { once: true });
}

function start(root: HTMLElement): void {
  root.innerHTML = markup();
  // The page is usable now; the hero shows the recorded flight while the live replay loads behind it.
  root.dataset.ready = '1';
  const hero = root.querySelector<HTMLElement>('.hero')!;
  const film = root.querySelector<HTMLElement>('#hero-film')!;
  const poster = root.querySelector<HTMLElement>('#hero-poster')!;
  const video = root.querySelector<HTMLVideoElement>('.hero-loop')!;
  const badge = root.querySelector<HTMLElement>('#hero-badge')!;
  const text = root.querySelector<HTMLElement>('#poster-status')!;
  const live = root.querySelector<HTMLButtonElement>('#hero-live')!;
  root.querySelector('#tour-start')!.addEventListener('click', () => void import('../ui/tour').then((m) => m.startTour()));
  // A link straight to the tour's closing card (?tour=end) is about the numbers: the poster frame is enough.
  if (new URLSearchParams(location.search).get('tour') === 'end') {
    badge.hidden = true;
    return;
  }
  playLoop(video, poster);
  const onStatus = (status: FilmStatus, message: string) => {
    if (status === 'flying') text.textContent = 'Recorded flight · flying the mission live in your tab…';
    else if (status === 'building') text.textContent = 'Recorded flight · building the live 3D replay…';
    else if (status === 'ready') {
      poster.classList.add('hidden');
      poster.setAttribute('aria-hidden', 'true');
      badge.classList.add('live');
      text.textContent = 'Live · flown and drawn in your tab';
      // The live replay has taken over: stop decoding the recording.
      window.setTimeout(() => video.pause(), 700);
      window.setTimeout(() => badge.classList.add('faded'), 5000);
    } else {
      // The recording keeps playing; say why it is not live and offer the full film page.
      badge.classList.add('failed');
      if (isWebglError(message)) text.textContent = 'Recorded flight · the live replay needs WebGL, which is off in this browser';
      else text.innerHTML = 'Recorded flight · the live replay could not start here. <a href="./film.html">Open the mission film</a>';
      console.warn('Landing hero:', message);
    }
  };
  // The film (three.js, the physics engine, the mission stack, ~7 MB in all) is most of the page: it waits until
  // the page has loaded and the recorded loop is up, then flies and builds behind the loop and fades in when ready.
  const goLive = () => {
    live.hidden = true;
    if (!hasWebgl()) {
      onStatus('error', 'WebGL is not available');
      return;
    }
    text.textContent = 'Recorded flight · loading the live sim…';
    import('../film/page')
      .then(({ startFilm }) => startFilm(film, { embed: true, scenario: HERO_SCENARIO, onStatus }))
      .catch((error: unknown) => onStatus('error', error instanceof Error ? error.message : String(error)));
  };
  if (savingData()) {
    // Data saver or a 2G link: keep the poster frame, and load the live replay only on request.
    text.textContent = 'Recorded flight · data saver is on';
    live.hidden = false;
    live.addEventListener('click', goLive, { once: true });
    return;
  }
  whenSettledAndVisible(hero, goLive);
}

const root = document.querySelector('#app');
if (root instanceof HTMLElement) start(root);
