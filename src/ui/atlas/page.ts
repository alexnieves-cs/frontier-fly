// Failure atlas page: the false-clear and collision rates under the declared operating
// distribution, where the failures concentrate, what the search learned, and every failing flight
// with a link that replays it on the Missions page. Data: docs/atlas.json (scripts/study/atlas-report.ts).
import atlas from '../../../docs/atlas.json';
import { siteHeader } from '../siteNav';

type Event = 'falseClear' | 'collision';
const EVENTS: Event[] = ['falseClear', 'collision'];
const EVENT_LABEL: Record<Event, string> = { falseClear: 'False clear', collision: 'Collision' };
const EVENT_BLURB: Record<Event, string> = {
  falseClear: 'A building declared clear while the person or object it was searching for was inside.',
  collision: 'Any contact with a wall, door frame, tree or other solid. Touching the ground to land does not count.',
};

interface Rate {
  flights: number;
  failures: number;
  p: number;
  ci: number[];
  ess: number;
}
interface Slice {
  dim: string;
  value: string;
  flights: number;
  failures: number;
  rate: number;
  ci: number[];
}
interface Failure {
  event: string;
  source: string;
  code: string;
  seed: number;
  mission: string;
  buildings: number;
  personState: string | null;
  object: string | null;
  decoys: number;
  lights: string;
  hour: number;
  windMps: number;
  fault: string;
  outcome: string;
  targetConf: number | null;
  where: string | null;
  tag: string | null;
}
interface AtlasData {
  mode: string;
  settings: Record<string, number>;
  seeds: { search: Record<string, number>; mc: number; is: Record<string, number> };
  dims: { id: string; label: string; values: string[]; nominal: number[] }[];
  rates: Record<Event, { importance: Rate; monteCarlo: Rate }>;
  conditional: Record<string, Rate>;
  slices: Record<Event, Slice[]>;
  search: Record<Event, { generations: { gen: number; flights: number; failures: number; level: number | null }[]; shift: { dim: string; label: string; top: { value: string; nominal: number; learned: number }[] }[] | null }>;
  flights: { search: number; mc: number; is: Record<string, number> };
  failures: Failure[];
}

const data = atlas as unknown as AtlasData;

declare global {
  interface Window {
    __atlas?: { ready: boolean; failures: number };
  }
}

const esc = (s: string) => s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);
/** A probability as a percentage, with enough digits to show a rare one. */
function pct(p: number): string {
  if (p === 0) return '0%';
  const v = p * 100;
  return `${v >= 10 ? v.toFixed(1) : v >= 1 ? v.toFixed(2) : v.toPrecision(2)}%`;
}
const ci = (c: number[]) => `${pct(c[0]!)} – ${pct(c[1]!)}`;
const words = (s: string) => s.replace(/-/g, ' ');
const MISSION_LABEL: Record<string, string> = {
  'find-person': 'Find a person',
  'sweep-occupied': 'Confirm clear (someone inside)',
  'sweep-empty': 'Confirm clear (empty)',
  'find-object': 'Find an object',
};

/** Replays a case on the Missions page: same seeds, same faults, the operator approving every entry. */
const replayHref = (code: string) => `./demo.html?atlas=${encodeURIComponent(code)}&operator=approve&speed=16&autorun=1`;

export function startAtlas(root: HTMLElement): void {
  root.classList.add('atlas');
  root.innerHTML = `
    ${siteHeader('atlas', { page: 'Failure atlas' })}
    <main class="atlas-main">
      <section class="atlas-intro">
        <p class="atlas-kicker">Safety numbers</p>
        <h2>How often does it get it wrong?</h2>
        <p class="atlas-lede">Two failures matter most for a search-and-rescue drone: telling a rescuer a building is clear when someone is inside, and flying into something. This page measures both on the shipped autopilot across ${data.flights.mc.toLocaleString()} randomly drawn missions, then hunts for the failures with a cross-entropy search and re-estimates each rate with importance sampling. Every failing flight below replays in your browser.</p>
      </section>
      <section class="atlas-rates" aria-label="Rates per flight">${EVENTS.map(rateCard).join('')}</section>
      <p class="atlas-note">${conditionalNote()}</p>
      <section class="atlas-section" aria-labelledby="where-title">
        <div class="atlas-head">
          <h2 id="where-title">Where the failures are</h2>
          <div class="atlas-tabs" role="tablist" aria-label="Event">${EVENTS.map((e, i) => `<button type="button" role="tab" data-event="${e}" aria-selected="${i === 0}">${EVENT_LABEL[e]}</button>`).join('')}</div>
        </div>
        <p class="atlas-sub">Rate among the Monte Carlo flights with each setting, with its 95% interval. Pick a factor:</p>
        <div class="atlas-dims" role="group" aria-label="Factor">${data.dims.map((d, i) => `<button type="button" data-dim="${d.id}" aria-pressed="${i === 0}">${esc(shortLabel(d.label))}</button>`).join('')}</div>
        <div class="atlas-bars" data-testid="atlas-bars"></div>
      </section>
      <section class="atlas-section" aria-labelledby="search-title">
        <h2 id="search-title">What the search learned</h2>
        <p class="atlas-sub">Each generation draws ${data.search.falseClear.generations[0]?.flights ?? 0} missions, keeps the ${Math.round((data.settings.rho ?? 0.1) * 100)}% that came closest to failing, and shifts its odds toward them. Factors it never had evidence for keep their odds. These are the settings it favours most against the declared mix.</p>
        <div class="atlas-learned">${EVENTS.map(learnedPanel).join('')}</div>
      </section>
      <section class="atlas-section" aria-labelledby="flights-title">
        <div class="atlas-head">
          <h2 id="flights-title">The failing flights</h2>
          <div class="atlas-tabs" role="tablist" aria-label="Event">${EVENTS.map((e, i) => `<button type="button" role="tab" data-list="${e}" aria-selected="${i === 0}">${EVENT_LABEL[e]} <span>${data.failures.filter((f) => f.event === e).length}</span></button>`).join('')}</div>
        </div>
        <p class="atlas-sub">Every failure among the ${data.flights.mc.toLocaleString()} randomly drawn missions, the ones a real deployment of this mix would meet. Replay rebuilds the same site, people, light, wind and fault from the flight's seeds and flies it live on the Missions page.</p>
        <div class="atlas-table-wrap"><table class="atlas-table" data-testid="atlas-failures"><thead></thead><tbody></tbody></table></div>
        <button type="button" class="atlas-more" data-testid="atlas-more" hidden>Show more</button>
      </section>
      <section class="atlas-section atlas-method" aria-labelledby="method-title">
        <h2 id="method-title">How it is measured</h2>
        <ul>
          <li><b>The autopilot.</b> The shipped policy flies each building and hands it to the sensor-only planner on a stall or contact, on the quadrotor airframe, with no ground truth in the loop. The operator approves every entry it asks for.</li>
          <li><b>The declared mix.</b> Each mission is a draw over ${data.dims.length} factors: the mission kind, 1 to 3 buildings, the person's pose, the object, decoys, interior lights, the time of day, wind speed, direction, gusts and arrival, and a sensor or airframe fault on 40% of flights with its timing, severity and bearing. The site layout and where things are placed come from seeds. The rates are rates under this mix; the factor breakdown above lets you weigh it differently.</li>
          <li><b>The search.</b> The cross-entropy method climbs a score that reaches 1 exactly on the failure: for a false clear, a target the drone saw but never confirmed, plus how many buildings it declared clear; for a collision, how close the drone came to a solid. Each kept flight is weighted by how much likelier it is under the declared mix than under the search's own odds. A first false-clear search rewarded never seeing the target at all and wandered off toward camera dropouts and strong wind, so it was re-run with the score above. ${data.flights.search.toLocaleString()} flights, seeds ${[...new Set(Object.values(data.seeds.search))].join(' and ')}+.</li>
          <li><b>The estimate.</b> Fresh flights drawn ${Math.round((data.settings.alpha ?? 0.3) * 100)}% from the declared mix and the rest from what the search learned, each weighted back to the declared mix (no weight above ${(1 / (data.settings.alpha ?? 0.3)).toFixed(1)}). Plain Monte Carlo on its own seeds (${data.seeds.mc}+) checks it. No seed block was used for anything else, and nothing was tuned on them.</li>
        </ul>
        <p><a href="https://github.com/alexnieves-cs/frontier-fly/blob/main/docs/ATLAS.md">docs/ATLAS.md</a> has the tables; <a href="https://github.com/alexnieves-cs/frontier-fly/tree/main/src/eval/atlas">src/eval/atlas</a> has the code.</p>
      </section>
    </main>`;

  let event: Event = 'falseClear';
  let dim = data.dims[0]!.id;
  let list: Event = 'falseClear';
  let shown = 40;
  const bars = root.querySelector<HTMLElement>('[data-testid="atlas-bars"]')!;
  const table = root.querySelector<HTMLTableElement>('[data-testid="atlas-failures"]')!;
  const more = root.querySelector<HTMLButtonElement>('[data-testid="atlas-more"]')!;

  const paintBars = () => {
    const d = data.dims.find((x) => x.id === dim)!;
    const rows = (data.slices[event] ?? []).filter((s) => s.dim === dim);
    const top = Math.max(0.0001, ...rows.map((s) => s.ci[1]!));
    bars.innerHTML = rows.length
      ? rows
          .map((s) => {
            const at = (x: number) => `${((x / top) * 100).toFixed(2)}%`;
            return `<div class="atlas-bar">
              <span class="atlas-bar-label">${esc(d.id === 'mission' ? (MISSION_LABEL[s.value] ?? s.value) : words(s.value))}</span>
              <span class="atlas-bar-track" aria-hidden="true"><i class="fill" style="width:${at(s.rate)}"></i><i class="ci" style="left:${at(s.ci[0]!)};width:${at(s.ci[1]! - s.ci[0]!)}"></i></span>
              <span class="atlas-bar-value">${pct(s.rate)} <small>${s.failures}/${s.flights}</small></span>
            </div>`;
          })
          .join('')
      : '<p class="atlas-sub">No flights.</p>';
  };

  const paintList = () => {
    const rows = data.failures.filter((f) => f.event === list);
    const head =
      list === 'falseClear'
        ? ['Seed', 'Mission', 'Target', 'Fault', 'Wind', 'Time', 'Lights', 'Outcome', 'Best look', '']
        : ['Seed', 'Mission', 'Buildings', 'Fault', 'Wind', 'Time', 'Where', 'Hit', '', ''];
    table.tHead!.innerHTML = `<tr>${head.map((h) => `<th scope="col">${h}</th>`).join('')}</tr>`;
    table.tBodies[0]!.innerHTML = rows
      .slice(0, shown)
      .map((f) => {
        const time = `${String(Math.floor(f.hour)).padStart(2, '0')}:${String(Math.round((f.hour % 1) * 60)).padStart(2, '0')}`;
        const cells =
          list === 'falseClear'
            ? [f.seed, MISSION_LABEL[f.mission] ?? f.mission, f.personState ? `person, ${f.personState}` : (f.object ?? '—'), words(f.fault), `${f.windMps} m/s`, time, f.lights, words(f.outcome), f.targetConf === null ? '—' : f.targetConf === 0 ? 'never seen' : pct(f.targetConf)]
            : [f.seed, MISSION_LABEL[f.mission] ?? f.mission, f.buildings, words(f.fault), `${f.windMps} m/s`, time, f.where ? words(f.where) : '—', f.tag ?? '—', ''];
        return `<tr>${cells.map((c) => `<td>${esc(String(c))}</td>`).join('')}<td><a class="atlas-replay" href="${replayHref(f.code)}">Replay</a></td></tr>`;
      })
      .join('');
    more.hidden = rows.length <= shown;
    more.textContent = `Show all ${rows.length}`;
  };

  root.querySelectorAll<HTMLButtonElement>('[data-event]').forEach((b) =>
    b.addEventListener('click', () => {
      event = b.dataset.event as Event;
      root.querySelectorAll('[data-event]').forEach((x) => x.setAttribute('aria-selected', String(x === b)));
      paintBars();
    }),
  );
  root.querySelectorAll<HTMLButtonElement>('[data-dim]').forEach((b) =>
    b.addEventListener('click', () => {
      dim = b.dataset.dim!;
      root.querySelectorAll('[data-dim]').forEach((x) => x.setAttribute('aria-pressed', String(x === b)));
      paintBars();
    }),
  );
  root.querySelectorAll<HTMLButtonElement>('[data-list]').forEach((b) =>
    b.addEventListener('click', () => {
      list = b.dataset.list as Event;
      shown = 40;
      root.querySelectorAll('[data-list]').forEach((x) => x.setAttribute('aria-selected', String(x === b)));
      paintList();
    }),
  );
  more.addEventListener('click', () => {
    shown = Infinity;
    paintList();
  });
  paintBars();
  paintList();
  root.dataset.ready = '1';
  window.__atlas = { ready: true, failures: data.failures.length };
}

function shortLabel(label: string): string {
  return label.replace(/ \(.*\)$/, '').replace(/, (m\/s|deg)$/, '');
}

function rateCard(e: Event): string {
  const r = data.rates[e];
  return `<article class="atlas-rate" data-testid="rate-${e}">
    <h3>${EVENT_LABEL[e]}</h3>
    <p class="atlas-big">${pct(r.importance.p)}<small> per flight</small></p>
    <p class="atlas-ci">95% interval ${ci(r.importance.ci)}</p>
    <p class="atlas-blurb">${EVENT_BLURB[e]}</p>
    <dl>
      <div><dt>Importance sampling</dt><dd>${r.importance.failures} failing of ${r.importance.flights.toLocaleString()} flights · effective sample ${Math.round(r.importance.ess).toLocaleString()}</dd></div>
      <div><dt>Plain Monte Carlo check</dt><dd>${pct(r.monteCarlo.p)} (${ci(r.monteCarlo.ci)}) · ${r.monteCarlo.failures} of ${r.monteCarlo.flights.toLocaleString()}</dd></div>
    </dl>
  </article>`;
}

function conditionalNote(): string {
  const any = data.conditional.falseClearWithTarget!;
  const sweep = data.conditional.falseClearSweepOccupied!;
  return `A sweep of an empty site cannot false-clear, so the rate per flight mixes in missions that can't fail that way. Counting only missions with someone or something inside, the false-clear rate is ${pct(any.p)} (${ci(any.ci)}); on "confirm the site is clear" sweeps with a person inside it is ${pct(sweep.p)} (${ci(sweep.ci)}, ${sweep.failures} of ${sweep.flights}).`;
}

function learnedPanel(e: Event): string {
  const s = data.search[e];
  const moved = (s.shift ?? []).filter((d) => d.top[0] && Math.abs(d.top[0].learned - d.top[0].nominal) > 0.02);
  return `<article class="atlas-panel">
    <h3>${EVENT_LABEL[e]}</h3>
    ${
      moved.length
        ? `<ul class="atlas-shift">${moved
            .map(
              (d) =>
                `<li><span>${esc(shortLabel(d.label))}</span><b>${esc(d.dim === 'mission' ? (MISSION_LABEL[d.top[0]!.value] ?? d.top[0]!.value) : words(d.top[0]!.value))}</b><small>${pct(d.top[0]!.nominal)} → ${pct(d.top[0]!.learned)}</small></li>`,
            )
            .join('')}</ul>`
        : '<p class="atlas-sub">The search found no factor that moves this failure.</p>'
    }
    <table class="atlas-gens"><thead><tr><th scope="col">Generation</th><th scope="col">Failing</th><th scope="col">Elite level</th></tr></thead><tbody>${s.generations
      .map((g) => `<tr><td>${g.gen}</td><td>${g.failures}/${g.flights} (${pct(g.failures / Math.max(1, g.flights))})</td><td>${g.level === null ? '—' : g.level.toFixed(2)}</td></tr>`)
      .join('')}</tbody></table>
  </article>`;
}
