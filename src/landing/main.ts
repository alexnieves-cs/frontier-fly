import '../film/film.css';
import './landing.css';
import type { FilmStatus } from '../film/page';
import { REPO_URL as REPO, siteHeader } from '../ui/siteNav';
import { isWebglError, showStartError } from '../ui/startError';
import { pct, results } from './results';

const HERO_SCENARIO = 'cabin-day';

function markup(): string {
  const r = results();
  const bars = r.pilots
    .map(
      (p) => `
        <li class="bar tone-${p.tone}">
          <span class="bar-name">${p.name}</span>
          <span class="bar-track"><span class="bar-fill" style="width:${(p.value * 100).toFixed(1)}%"></span></span>
          <span class="bar-value">${pct(p.value)}</span>
        </li>`,
    )
    .join('');
  return `
  <div class="landing">
    ${siteHeader('home', { heading: false })}
    <section class="hero" data-tour="hero" aria-label="Live mission replay">
      <div class="hero-film" id="hero-film"></div>
      <div class="hero-poster" id="hero-poster" role="status" aria-live="polite">
        <div class="poster-inner">
          <span class="poster-logo" aria-hidden="true"></span>
          <p class="poster-kicker">LIVE IN YOUR BROWSER</p>
          <p class="poster-title">Injured hiker in a cabin</p>
          <p class="poster-status" id="poster-status">Flying the mission headless…</p>
          <span class="poster-bar" aria-hidden="true"><span></span></span>
        </div>
      </div>
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
          <li><strong>${pct(r.pilots[0]!.value)}</strong> of unseen hard buildings searched</li>
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
        <p>Fraction of each building the drone saw before time ran out. Stage ${r.hardStage}, mean of ${r.hardSeeds} held-out seeds, daylight.</p>
      </div>
      <div class="results-body">
        <ul class="bars">${bars}
        </ul>
        <div class="tiles">
          <div class="tile"><span class="tile-value">${pct(r.easy)}</span><span class="tile-label">coverage on easy buildings (${r.easySeeds} seeds)</span></div>
          <div class="tile"><span class="tile-value">${pct(Math.min(1, r.nightOverDay))}</span><span class="tile-label">night coverage relative to day, lights off</span></div>
        </div>
      </div>
      <p class="results-foot">
        A small, deterministic eval: ${r.hardSeeds} hard and ${r.easySeeds} easy seeds, measured ${r.measured} at commit
        <code>${r.commit}</code>. ${r.policyLeads ? 'The policy covers more than its teacher here, but collides more than the sensor-only planner.' : 'The policy still trails its teacher.'}
        Results on 30 fresh buildings per condition, with intervals and collisions, are in
        <a href="${REPO}/blob/main/docs/REPORT.md" rel="noopener">REPORT.md</a>; method and per-seed numbers in
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

function start(root: HTMLElement): void {
  root.innerHTML = markup();
  // The page is usable now; the hero keeps loading behind its poster.
  root.dataset.ready = '1';
  const film = root.querySelector<HTMLElement>('#hero-film')!;
  const poster = root.querySelector<HTMLElement>('#hero-poster')!;
  const text = root.querySelector<HTMLElement>('#poster-status')!;
  const onStatus = (status: FilmStatus, message: string) => {
    if (status === 'flying') text.textContent = 'Flying the mission headless in your tab…';
    else if (status === 'building') text.textContent = 'Building the world…';
    else if (status === 'ready') {
      poster.classList.add('hidden');
      poster.setAttribute('aria-hidden', 'true');
    } else {
      poster.classList.add('failed');
      // Without WebGL the film page fails too, so point at the recorded flights instead.
      if (isWebglError(message)) showStartError(text, message);
      else text.innerHTML = 'The live replay could not start here. <a href="./film.html">Open the mission film</a>';
      console.warn('Landing hero:', message);
    }
  };
  // The film (three.js, the physics engine, the mission stack) is most of the page's code: load it after the
  // landing markup paints, and let the markup paint before the headless flight takes the main thread.
  window.setTimeout(() => {
    import('../film/page')
      .then(({ startFilm }) => startFilm(film, { embed: true, scenario: HERO_SCENARIO, onStatus }))
      .catch((error: unknown) => onStatus('error', error instanceof Error ? error.message : String(error)));
  }, 120);
}

const root = document.querySelector('#app');
if (root instanceof HTMLElement) start(root);
