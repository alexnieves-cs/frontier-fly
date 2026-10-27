import { cells as BENCH_CELLS } from '../../../docs/team-bench.json';
import { POLICY_DT } from '../../shared/constants';
import { RNG } from '../../shared/rng';
import { TEAM_CONDITIONS, teamEpisodeSetup, type TeamCondition } from '../../team/conditions';
import { TeamSim, type Comms, type Role } from '../../team/run';
import { CoverageChart } from '../race/chart';
import { DRONE_COLORS, TeamMap } from './draw';
import { prefersReducedMotion, siteHeader } from '../siteNav';

const EVAL_SEED_BASE: Record<string, number> = { hard: 70001, large: 71001 };
const SPEEDS = [
  { id: '1', label: '1×', rate: 20 },
  { id: '4', label: '4×', rate: 80 },
  { id: '16', label: '16×', rate: 320 },
  { id: 'max', label: 'Max', rate: 0 },
] as const;
type SpeedId = (typeof SPEEDS)[number]['id'];
const RADIOS = [
  { id: 'normal', label: 'Normal (0 dBm)' },
  { id: 'weak', label: 'Weak (−10 dBm)' },
  { id: 'perfect', label: 'Perfect' },
] as const;
type RadioId = (typeof RADIOS)[number]['id'];

export interface TeamSetup {
  condition: string;
  seed: number;
  drones: number;
  radio: RadioId;
  relay: boolean;
}

export interface TeamPageState {
  status: 'loading' | 'running' | 'paused' | 'finished' | 'error';
  setup: TeamSetup | null;
  step: number;
  coverage: number;
  baseKnown: number;
  frames: number;
}

declare global {
  interface Window {
    __team?: TeamPageState;
  }
}

const pct = (v: number, digits = 1) => `${(v * 100).toFixed(digits)}%`;
const esc = (s: string) => s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

function conditionById(id: string | null): TeamCondition {
  return TEAM_CONDITIONS.find((c) => c.id === id) ?? TEAM_CONDITIONS[1]!;
}

/** The bench arm a setup corresponds to, if the bench ran it. */
export function armId(s: TeamSetup): string {
  if (s.drones === 1) return 'solo';
  if (s.radio === 'perfect') return s.drones === 3 && !s.relay ? 'team3-perfect' : '';
  if (s.radio === 'weak') return s.drones === 3 ? (s.relay ? 'team3-weak-relay' : 'team3-weak') : '';
  if (s.relay) return s.drones === 3 ? 'team3-relay' : '';
  return `team${s.drones}`;
}

function readSetup(): { setup: TeamSetup; speed: SpeedId } {
  const q = new URLSearchParams(location.search);
  const condition = conditionById(q.get('condition'));
  const seedParam = Number(q.get('seed'));
  const seed = Number.isInteger(seedParam) && seedParam > 0 ? seedParam : EVAL_SEED_BASE[condition.id]!;
  const drones = Math.max(1, Math.min(4, Number(q.get('drones')) || 3));
  const radio = (RADIOS.find((r) => r.id === q.get('radio'))?.id ?? 'normal') as RadioId;
  const relay = q.get('relay') === '1' && drones >= 2;
  const speed = (SPEEDS.find((s) => s.id === q.get('speed'))?.id ?? (prefersReducedMotion() ? '1' : '16')) as SpeedId;
  return { setup: { condition: condition.id, seed, drones, radio, relay }, speed };
}

function markup(): string {
  const conditions = TEAM_CONDITIONS.map((c) => `<option value="${c.id}">${esc(c.label)}</option>`).join('');
  const sizes = [1, 2, 3, 4].map((n) => `<button type="button" class="seg" data-drones="${n}" aria-pressed="false">${n}</button>`).join('');
  const radios = RADIOS.map((r) => `<button type="button" class="seg" data-radio="${r.id}" aria-pressed="false">${esc(r.label)}</button>`).join('');
  const speeds = SPEEDS.map((s) => `<button type="button" class="seg" data-speed="${s.id}" aria-pressed="false">${s.label}</button>`).join('');
  return `
  <div class="race team">
    ${siteHeader('team', { heading: false, className: 'race-top' })}

    <section class="race-intro">
      <p class="kicker">DRONE TEAM · RUNNING IN YOUR TAB</p>
      <h1>Several drones, one building, one weak radio.</h1>
      <p class="pitch">Each drone maps with its own lidar and shares what it has seen over a radio that every wall weakens. Teammates split the frontiers between them, and what reaches the base outside is all the operator gets to see.
      Nothing here is prerecorded: the team is simulated live in this tab, with the same code as the published benchmark.</p>
    </section>

    <section class="race-controls" aria-label="Team setup">
      <label class="ctl">
        <span class="ctl-label">Building type</span>
        <select id="team-condition">${conditions}</select>
      </label>
      <div class="ctl">
        <span class="ctl-label">Drones</span>
        <div class="segs" role="group" aria-label="Team size">${sizes}</div>
      </div>
      <div class="ctl">
        <span class="ctl-label">Radio</span>
        <div class="segs" role="group" aria-label="Radio">${radios}</div>
      </div>
      <div class="ctl">
        <span class="ctl-label">Relay</span>
        <div class="segs" role="group" aria-label="Relay">
          <button type="button" class="seg" id="team-relay" aria-pressed="false" title="The last drone to launch parks where it still reaches the base">Last drone relays</button>
        </div>
      </div>
      <div class="ctl">
        <span class="ctl-label">Building</span>
        <div class="stepper">
          <button type="button" class="seg icon" id="team-prev" aria-label="Previous building">‹</button>
          <span class="stepper-value" id="team-building">–</span>
          <button type="button" class="seg icon" id="team-next" aria-label="Next building">›</button>
          <button type="button" class="seg" id="team-random" title="A building the benchmark never flew">Random</button>
        </div>
      </div>
      <div class="ctl">
        <span class="ctl-label">Speed</span>
        <div class="segs" role="group" aria-label="Playback speed">${speeds}</div>
      </div>
      <div class="ctl ctl-actions">
        <button type="button" class="btn ghost" id="team-pause">Pause</button>
        <button type="button" class="btn primary" id="team-restart">Restart</button>
      </div>
    </section>

    <section class="team-maps">
      <figure class="team-map">
        <figcaption><b>Truth</b><span class="muted">Floor flown by each drone; radio links</span></figcaption>
        <canvas id="team-truth" aria-label="Top-down map of the building and the team"></canvas>
      </figure>
      <figure class="team-map">
        <figcaption><b>What the base sees</b><span class="muted">Floor reported to the base; each drone where it last reported</span></figcaption>
        <canvas id="team-base" aria-label="The team map as the base station knows it"></canvas>
      </figure>
    </section>

    <section class="team-deck">
      <div class="team-panel">
        <div class="team-readouts">
          <div><span class="ctl-label">Time</span><b id="team-time">0.0 s</b></div>
          <div><span class="ctl-label">Searched</span><b id="team-cov">0.0%</b></div>
          <div><span class="ctl-label">Base knows</span><b id="team-known">0.0%</b></div>
          <div><span class="ctl-label">Packets through</span><b id="team-deliv">–</b></div>
        </div>
        <ul class="team-legend" id="team-legend"></ul>
        <p class="muted team-ref" id="team-ref"></p>
      </div>
      <div class="team-panel">
        <span class="ctl-label">Searched (solid) and known at the base (faint)</span>
        <canvas class="team-chart" id="team-chart" aria-label="Coverage over time"></canvas>
      </div>
    </section>
  </div>`;
}

export function startTeam(root: HTMLElement): void {
  const state: TeamPageState = { status: 'loading', setup: null, step: 0, coverage: 0, baseKnown: 0, frames: 0 };
  window.__team = state;
  root.innerHTML = markup();
  const $ = <T extends Element>(sel: string) => root.querySelector(sel) as T;
  const launch = readSetup();
  let setup = launch.setup;
  let speed = launch.speed;
  let sim: TeamSim | null = null;
  let truthMap: TeamMap | null = null;
  let baseMap: TeamMap | null = null;
  const chart = new CoverageChart($<HTMLCanvasElement>('#team-chart'));
  let paused = false;
  let generation = 0;
  let carry = 0;
  let last = 0;

  const syncControls = () => {
    $<HTMLSelectElement>('#team-condition').value = setup.condition;
    root.querySelectorAll<HTMLButtonElement>('[data-drones]').forEach((b) => b.setAttribute('aria-pressed', String(Number(b.dataset.drones) === setup.drones)));
    root.querySelectorAll<HTMLButtonElement>('[data-radio]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.radio === setup.radio)));
    root.querySelectorAll<HTMLButtonElement>('[data-speed]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.speed === speed)));
    const relayBtn = $<HTMLButtonElement>('#team-relay');
    relayBtn.setAttribute('aria-pressed', String(setup.relay));
    relayBtn.disabled = setup.drones < 2 || setup.radio === 'perfect';
    const base = EVAL_SEED_BASE[setup.condition]!;
    const idx = setup.seed - base;
    $('#team-building').textContent = idx >= 0 && idx < 30 ? `Bench #${idx + 1} of 30` : `Seed ${setup.seed}`;
    $<HTMLButtonElement>('#team-pause').textContent = paused ? 'Resume' : 'Pause';
    const q = new URLSearchParams({ condition: setup.condition, seed: String(setup.seed), drones: String(setup.drones), radio: setup.radio, relay: setup.relay ? '1' : '0', speed });
    history.replaceState(null, '', `?${q.toString()}`);
  };

  const legend = () => {
    if (!sim) return;
    const v = sim.view();
    const r = sim.result();
    $('#team-legend').innerHTML = v.pos
      .map((_, i) => {
        const role = v.role[i] === 'relay' ? (v.relay[i] === 'hold' ? 'relay, holding the link' : v.relay[i] === 'return' ? 'relay, backing up to the link' : 'relay, exploring') : 'explorer';
        const status = !v.launched[i] ? 'waiting to launch' : v.done[i] ? 'landed' : role;
        return `<li><i style="background:${DRONE_COLORS[i % DRONE_COLORS.length]}"></i>Drone ${i + 1} <span class="muted">${esc(status)} · flew ${pct(r.perDrone[i] ?? 0)}</span></li>`;
      })
      .join('');
  };

  const reference = () => {
    const arm = armId(setup);
    const cell = BENCH_CELLS.find((c) => c.condition === setup.condition && c.arm === arm);
    $('#team-ref').textContent = cell
      ? `Benchmark, 30 held-out buildings of this type: ${cell.label} searched ${pct(cell.coverage)} on average, and the base knew ${pct(cell.baseKnown)} at the end.`
      : 'The benchmark did not run this combination.';
  };

  const draw = () => {
    if (!sim || !truthMap || !baseMap) return;
    const v = sim.view();
    truthMap.draw(v, sim.firstBy, null, setup.radio === 'perfect' ? null : sim.radio);
    // The base only knows where each drone was when its last packet arrived.
    const heard = sim.baseNode.heard;
    const baseView = {
      ...v,
      pos: v.pos.map((p, i) => heard[i]?.pos ?? p),
      goal: v.goal.map((_, i) => heard[i]?.goal ?? null),
      launched: v.launched.map((_, i) => heard[i] !== null),
    };
    baseMap.draw(baseView, sim.firstBy, sim.baseNode.visited, null);
    chart.draw();
    $('#team-time').textContent = `${(v.step * POLICY_DT).toFixed(1)} s`;
    $('#team-cov').textContent = pct(v.coverage);
    $('#team-known').textContent = pct(v.baseKnown);
    const p = sim.packets;
    $('#team-deliv').textContent = p.sent ? pct(p.delivered / p.sent, 0) : '–';
    state.step = v.step;
    state.coverage = v.coverage;
    state.baseKnown = v.baseKnown;
    state.frames += 1;
  };

  const restart = async () => {
    const gen = ++generation;
    state.status = 'loading';
    syncControls();
    reference();
    sim?.dispose();
    sim = null;
    const cond = conditionById(setup.condition);
    const { spec, envSeed, cfg } = teamEpisodeSetup(cond, setup.seed);
    const roles: Role[] = Array.from({ length: setup.drones }, (_, i) => (setup.relay && i === setup.drones - 1 ? 'relay' : 'explore'));
    const comms: Comms = setup.radio === 'perfect' ? 'perfect' : 'radio';
    const next = await TeamSim.create(spec, envSeed, cfg, { drones: setup.drones, comms, roles, radio: setup.radio === 'weak' ? { txPower: -10 } : undefined }, setup.seed);
    if (gen !== generation) {
      next.dispose();
      return;
    }
    sim = next;
    truthMap = new TeamMap($<HTMLCanvasElement>('#team-truth'), spec);
    baseMap = new TeamMap($<HTMLCanvasElement>('#team-base'), spec);
    chart.reset([...DRONE_COLORS.slice(0, 1), 'rgba(86, 224, 255, 0.35)'], (cfg.maxSteps * POLICY_DT) | 0);
    state.setup = { ...setup };
    state.status = paused ? 'paused' : 'running';
    carry = 0;
    last = performance.now();
    legend();
    draw();
    root.dataset.ready = '1';
  };

  const tick = (now: number) => {
    requestAnimationFrame(tick);
    if (!sim || state.status === 'loading') return;
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    if (paused || sim.done) return;
    const rate = SPEEDS.find((s) => s.id === speed)!.rate;
    const t0 = performance.now();
    let steps = 0;
    if (rate === 0) {
      while (performance.now() - t0 < 14 && sim.step()) steps += 1;
    } else {
      carry += dt * rate;
      while (carry >= 1 && performance.now() - t0 < 30) {
        carry -= 1;
        steps += 1;
        if (!sim.step()) break;
      }
    }
    if (steps > 0) {
      const t = sim.stepCount * POLICY_DT;
      chart.push(0, t, sim.coverage);
      chart.push(1, t, sim.baseKnown);
      draw();
      if (state.frames % 10 === 0 || sim.done) legend();
    }
    if (sim.done && state.status !== 'finished') {
      state.status = 'finished';
      legend();
    }
  };

  const change = (patch: Partial<TeamSetup>) => {
    setup = { ...setup, ...patch };
    if (setup.drones < 2 || setup.radio === 'perfect') setup.relay = false;
    void restart();
  };

  $<HTMLSelectElement>('#team-condition').addEventListener('change', (e) => {
    const id = (e.target as HTMLSelectElement).value;
    change({ condition: id, seed: EVAL_SEED_BASE[id]! });
  });
  root.querySelectorAll<HTMLButtonElement>('[data-drones]').forEach((b) => b.addEventListener('click', () => change({ drones: Number(b.dataset.drones) })));
  root.querySelectorAll<HTMLButtonElement>('[data-radio]').forEach((b) => b.addEventListener('click', () => change({ radio: b.dataset.radio as RadioId })));
  root.querySelectorAll<HTMLButtonElement>('[data-speed]').forEach((b) =>
    b.addEventListener('click', () => {
      speed = b.dataset.speed as SpeedId;
      syncControls();
    }),
  );
  $('#team-relay').addEventListener('click', () => change({ relay: !setup.relay }));
  $('#team-prev').addEventListener('click', () => change({ seed: Math.max(1, setup.seed - 1) }));
  $('#team-next').addEventListener('click', () => change({ seed: setup.seed + 1 }));
  // Above every bench and tuning block, so a random building is one nothing was ever tuned or scored on.
  $('#team-random').addEventListener('click', () => change({ seed: 900_000 + new RNG(Date.now() >>> 0).int(99_999) }));
  $('#team-restart').addEventListener('click', () => void restart());
  $('#team-pause').addEventListener('click', () => {
    paused = !paused;
    if (sim && !sim.done) state.status = paused ? 'paused' : 'running';
    syncControls();
  });

  void restart().catch((err: unknown) => {
    state.status = 'error';
    $('#team-ref').textContent = `Could not start: ${String(err)}`;
  });
  requestAnimationFrame(tick);
}
