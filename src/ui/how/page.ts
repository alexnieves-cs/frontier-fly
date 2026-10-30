// How-it-works page: the autopilot explained in seven short chapters, each next to something live
// or measured. The explorer in chapter 2 flies real generated buildings (./explore.ts); every
// number comes from the study JSON the README and the report are generated from.
import { cells as studyCells } from '../../../docs/study.json';
import { arms as ablationArms } from '../../../docs/ablations.json';
import { hardPooled } from '../../../docs/rl-finetune.json';
import { conditional, rates, slices } from '../../../docs/atlas.json';
import { HIDDEN, OBS_DIM } from '../../shared/constants';
import { prefersReducedMotion, REPO_URL, siteHeader } from '../siteNav';
import { Explorer, SEEN, STAGES, VOXEL_SIZE } from './explore';

const pct = (v: number, digits = 1) => `${(v * 100).toFixed(digits)}%`;
const REPORT_PDF = './frontier-fly-report.pdf';

declare global {
  interface Window {
    __how?: { ready: boolean; coverage: number };
  }
}

interface StudyCell {
  condition: string;
  lighting: string;
  pilot: string;
  episodes: number;
  coverage: number;
  coverageCi: number[];
  collisionsPerEpisode: number;
}

function studyCell(pilot: string, lighting = 'day'): StudyCell {
  return (studyCells as StudyCell[]).find((c) => c.condition === 'hard' && c.lighting === lighting && c.pilot === pilot)!;
}

/** The observation the network reads, by block, in the order of src/sim/OBS_LAYOUT.md. */
const OBS_BLOCKS = [
  { n: 32, label: 'Horizontal lidar', note: '32 beams, every 11.25°, range to 8 m' },
  { n: 8, label: 'Tilted lidar', note: 'up and down 26°, every 45°' },
  { n: 12, label: 'Motion and attitude', note: 'velocity, yaw, height, floor, time left' },
  { n: 12, label: 'Where to go next', note: 'direction and distance to the nearest frontier and the biggest unexplored cluster' },
  { n: 512, label: 'Local map, 8 m', note: 'where it has flown, and what it has seen but not flown, at 0.5 m' },
  { n: 512, label: 'Coarse map, 32 m', note: 'the same two layers at 2 m' },
];

const PHASES = [
  ['Sweep', 'Fly coverage lanes across the geofenced woods at a safe height, scanning for buildings.'],
  ['Inspect', 'Circle the building and measure every opening with a depth-camera model.'],
  ['Enter', 'Pick an opening it clears with margin, ask the operator first if the mission says to, and fly through.'],
  ['Search', 'The policy explores room by room; when it stalls, the sensor-only planner takes over.'],
  ['Confirm', 'A target seen several times with enough confidence is confirmed. If nothing is, the building is declared clear.'],
  ['Return', 'Fly home, land, and file a report: what was found, where, coverage, battery, and every decision.'],
] as const;

function bars(rows: { label: string; value: number; note: string; tone?: string }[]): string {
  return `<ul class="how-bars">${rows
    .map(
      (r) => `<li class="${r.tone ?? ''}">
        <span class="how-bar-label">${r.label}</span>
        <span class="how-bar-track"><span style="width:${(r.value * 100).toFixed(1)}%"></span></span>
        <span class="how-bar-value">${pct(r.value)}</span>
        <span class="how-bar-note">${r.note}</span>
      </li>`,
    )
    .join('')}</ul>`;
}

function markup(): string {
  const policy = studyCell('policy');
  const night = studyCell('policy', 'night-dark');
  const teacher = studyCell('teacher');
  const planner = studyCell('classical');
  const bc = ablationArms.find((a) => a.id === 'bc')!.hard.policyDay;
  const dagger = ablationArms.find((a) => a.id === 'dagger')!.hard.policyDay;
  const fc = rates.falseClear.importance;
  const pose = (v: string) => slices.falseClear.find((s) => s.dim === 'personState' && s.value === v)!;
  const obsTotal = OBS_BLOCKS.reduce((a, b) => a + b.n, 0);
  return `
  <div class="how">
    ${siteHeader('how', { page: 'How it works' })}
    <main class="how-main">
      <p class="how-kicker">A search-and-rescue autopilot, explained</p>
      <h2 class="how-title">How a drone searches a building it has never seen</h2>
      <p class="how-lede">
        frontier-fly flies a simulated drone from the treeline into a building and through it, room by room, looking for
        a missing person. Everything below runs or comes from measurements made in this repository. Seven short steps.
      </p>

      <section class="how-chapter" id="world">
        <div class="how-copy">
          <p class="how-step">1 · The world</p>
          <h3>Every building comes from a seed</h3>
          <p>
            A seed number grows a floor plan: rooms split from a rectangle, doors and hallways joined until every room is
            reachable, then furniture. The same seed always builds the same building, so any flight can be replayed
            exactly and every benchmark can be rerun. The drone trains on one range of seeds and is scored on others it
            has never seen.
          </p>
          <p>
            Pick a stage and roll a new seed: this is the same generator the simulator uses. Stage 3, with hallways and
            loops, is the "hard building" in every result below.
          </p>
        </div>
        <div class="how-visual" aria-label="A generated floor plan explored by a simplified frontier explorer">
          <div class="how-controls">
            <label>Building <select data-stage>${Object.entries(STAGES)
              .map(([k, v]) => `<option value="${k}"${k === '3' ? ' selected' : ''}>${v.label}</option>`)
              .join('')}</select></label>
            <button type="button" data-seed>New seed</button>
            <button type="button" data-play aria-pressed="true">Pause</button>
            <label class="how-check"><input type="checkbox" data-truth /> Show the floor plan</label>
          </div>
          <canvas class="how-canvas" data-canvas width="660" height="480"></canvas>
          <p class="how-readout" data-readout role="status" aria-live="off"></p>
        </div>
      </section>

      <section class="how-chapter" id="sensing">
        <div class="how-copy">
          <p class="how-step">2 · Sensing</p>
          <h3>It sees with lidar, not a camera</h3>
          <p>
            The drone has no floor plan. It sweeps 32 laser beams around itself (the cyan lines) and builds its own map:
            space a beam passed through is free, the point it stopped is a wall. Free space it has seen but not yet flown
            through is a <em>frontier</em>, the edge of what it knows.
          </p>
          <p>
            Exploring is going to frontiers until there are none left. The map above is a simplified version of that,
            written for this page: it ignores furniture, inertia and sensor noise. Because lidar doesn't care about light,
            the trained pilot searches ${pct(night.coverage)} of hard buildings at night with the lights off, against
            ${pct(policy.coverage)} by day.
          </p>
        </div>
        <div class="how-visual how-legend">
          <ul>
            <li><i class="sw unknown"></i> Never seen</li>
            <li><i class="sw free"></i> Seen free: a frontier until flown</li>
            <li><i class="sw visited"></i> Flown through: counts as covered</li>
            <li><i class="sw wall"></i> Wall hit by a beam</li>
            <li><i class="sw path"></i> Shortest path to the nearest frontier</li>
          </ul>
        </div>
      </section>

      <section class="how-chapter" id="network">
        <div class="how-copy">
          <p class="how-step">3 · The pilot</p>
          <h3>A small network turns ${OBS_DIM} numbers into a velocity</h3>
          <p>
            Twenty times a second the pilot reads ${OBS_DIM} numbers: the lidar, its motion, which way the nearest frontier
            is, and two maps centered on itself. A network of ${HIDDEN.join(' and ')} neurons turns them into four
            outputs: forward, sideways and vertical speed, and a turn rate. A flight controller does the rest.
          </p>
          <p>
            It is small enough to run in a browser tab at thousands of steps a second, which is what lets the benchmarks
            fly thousands of buildings.
          </p>
        </div>
        <div class="how-visual">
          <div class="how-obs" role="img" aria-label="The ${OBS_DIM} observation inputs by block">
            ${OBS_BLOCKS.map((b) => `<span style="flex:${Math.max(b.n, 24)}" title="${b.n} numbers: ${b.note}"><b>${b.label}</b><small>${b.n}</small></span>`).join('')}
          </div>
          <ul class="how-obs-notes">${OBS_BLOCKS.map((b) => `<li><b>${b.label}</b> (${b.n}): ${b.note}</li>`).join('')}</ul>
          <p class="how-net"><code>${obsTotal}</code> → <code>${HIDDEN[0]}</code> → <code>${HIDDEN[1]}</code> → <code>4</code> velocity commands</p>
        </div>
      </section>

      <section class="how-chapter" id="training">
        <div class="how-copy">
          <p class="how-step">4 · Learning</p>
          <h3>It learned by copying, then by practice</h3>
          <p>
            First, <b>imitation</b>: a teacher that is allowed to see the whole explored map flies the buildings, and the
            network learns to copy it from its own sensors (behavior cloning). Then <b>DAgger</b>: the network flies and
            the teacher labels every state it gets itself into, so it learns to recover from its own mistakes.
          </p>
          <p>
            Finally <b>PPO</b>, reinforcement learning on its own flights, rewarded for new ground covered and penalized for
            collisions. That is when it passed its teacher: ${pct(policy.coverage)} of fresh hard buildings against the
            teacher's ${pct(teacher.coverage)}. It also collides more, ${policy.collisionsPerEpisode.toFixed(1)} times per
            flight, where the sensor-only planner covers ${pct(planner.coverage)} and never touches a wall. Missions use
            both: the policy first, the planner when it stalls.
          </p>
        </div>
        <div class="how-visual">
          ${bars([
            { label: 'Behavior cloning', value: bc.coverage, note: `${bc.episodes} fresh hard buildings, ablation run` },
            { label: '+ DAgger', value: dagger.coverage, note: `same buildings, best of 6 rounds on validation seeds` },
            { label: '+ PPO (shipped)', value: hardPooled.ppo800.coverage, note: `${hardPooled.episodes} held-out hard flights`, tone: 'accent' },
            { label: 'Teacher (sees the map)', value: teacher.coverage, note: `${teacher.episodes} fresh hard buildings`, tone: 'muted' },
            { label: 'Sensor-only planner', value: planner.coverage, note: `same buildings, no collisions`, tone: 'muted' },
          ])}
          <p class="how-foot">Coverage of the reachable floor before time runs out. The rows use different seed sets, named beside each bar; the ${policy.episodes}-building study in the report puts every pilot on the same seeds.</p>
        </div>
      </section>

      <section class="how-chapter" id="mission">
        <div class="how-copy">
          <p class="how-step">5 · The mission</p>
          <h3>From the treeline to the last room and home</h3>
          <p>
            Exploring a building is one part of a mission. The mission layer around it decides where to fly outdoors,
            which opening is safe, when to ask the operator, and when a search is done. Nothing in it reads the simulator's
            ground truth: it works from the drone's own maps, pose estimate and detections.
          </p>
          <p><a class="how-cta" href="./film.html">Watch a full mission in 3D</a> <a class="how-cta ghost" href="./demo.html">Run one yourself</a></p>
        </div>
        <div class="how-visual">
          <ol class="how-phases">${PHASES.map(([t, d]) => `<li><b>${t}</b><span>${d}</span></li>`).join('')}</ol>
        </div>
      </section>

      <section class="how-chapter" id="failure">
        <div class="how-copy">
          <p class="how-step">6 · Where it fails</p>
          <h3>Sometimes it calls a building clear with someone inside</h3>
          <p>
            To measure rare failures honestly, a search algorithm hunts for the conditions that cause them, and
            importance sampling turns what it finds into a rate with an error bar. Over a declared mix of missions,
            light, wind and faults, the drone declares a building clear while a person is inside on
            <b>${pct(fc.p, 2)}</b> of flights (95% interval ${pct(fc.ci[0]!, 2)} to ${pct(fc.ci[1]!, 2)}), and on
            ${pct(conditional.falseClearSweepOccupied.p)} of sweeps where someone is actually there.
          </p>
          <p>
            Almost all are a person who is moving or hiding. The detector does see them, but averaged over several
            views the confidence never reaches the confirm bar, so the contact is dropped. That is a flaw in the
            confirmation rule, found by measuring, and it is why this is a research simulator and not a product.
          </p>
          <p><a class="how-cta" href="./atlas.html">See every failing flight</a></p>
        </div>
        <div class="how-visual">
          ${bars([
            { label: 'Walking', value: pose('walking').rate, note: `${pose('walking').failures} of ${pose('walking').flights} flights`, tone: 'danger' },
            { label: 'Hiding', value: pose('hiding').rate, note: `${pose('hiding').failures} of ${pose('hiding').flights}`, tone: 'danger' },
            { label: 'Standing', value: pose('standing').rate, note: `${pose('standing').failures} of ${pose('standing').flights}`, tone: 'danger' },
            { label: 'Sitting', value: pose('sitting').rate, note: `${pose('sitting').failures} of ${pose('sitting').flights}`, tone: 'danger' },
            { label: 'Lying', value: pose('lying').rate, note: `${pose('lying').failures} of ${pose('lying').flights}`, tone: 'danger' },
          ])}
          <p class="how-foot">False clears per flight by the person's pose, from ${rates.falseClear.monteCarlo.flights.toLocaleString('en-US')} plain Monte Carlo flights. Bars are drawn on a 0 to 100% scale.</p>
        </div>
      </section>

      <section class="how-chapter" id="measure">
        <div class="how-copy">
          <p class="how-step">7 · Measuring</p>
          <h3>Every number can be rerun</h3>
          <p>
            Results come from seeds no model or setting was tuned on, with 95% intervals. Each study is a script that writes
            a JSON file, and the README, the report and this page are generated from those files, so a number can't drift
            from the measurement. Misses are reported as misses.
          </p>
          <p>
            <a class="how-cta" href="${REPORT_PDF}">Read the tech report (PDF)</a>
            <a class="how-cta ghost" href="${REPO_URL}" rel="noopener">Source on GitHub</a>
          </p>
        </div>
        <div class="how-visual how-facts">
          <dl>
            <div><dt>Runs in</dt><dd>Your browser tab: physics, sensors, training, rendering</dd></div>
            <div><dt>Physics</dt><dd>Rapier, 400 Hz, kinematic or 6-DoF quadrotor</dd></div>
            <div><dt>Learning</dt><dd>TensorFlow.js, BC + DAgger + PPO</dd></div>
            <div><dt>Checked against</dt><dd>PX4 SITL flies exported plans in CI</dd></div>
            <div><dt>Scope</dt><dd>Civilian search, inspection and confirm-clear only</dd></div>
          </dl>
        </div>
      </section>
    </main>
  </div>`;
}

const COLORS = {
  unknown: '#0b1218',
  free: 'rgba(86, 224, 255, 0.16)',
  visited: 'rgba(61, 255, 154, 0.42)',
  wall: '#eef4f7',
  truth: 'rgba(255, 255, 255, 0.10)',
  beam: 'rgba(86, 224, 255, 0.35)',
  path: '#ffd166',
  drone: '#56e0ff',
};

function draw(ctx: CanvasRenderingContext2D, e: Explorer, truth: boolean): void {
  const { width, height } = ctx.canvas;
  const scale = Math.min(width / (e.vw * VOXEL_SIZE), height / (e.vd * VOXEL_SIZE));
  const ox = (width - e.vw * VOXEL_SIZE * scale) / 2;
  const oy = (height - e.vd * VOXEL_SIZE * scale) / 2;
  const px = VOXEL_SIZE * scale;
  ctx.fillStyle = '#05080b';
  ctx.fillRect(0, 0, width, height);
  for (let v = 0; v < e.open.length; v++) {
    const vx = v % e.vw;
    const vy = (v - vx) / e.vw;
    const s = e.seen[v];
    let fill = COLORS.unknown;
    if (e.visited[v]) fill = COLORS.visited;
    else if (s === SEEN.FREE) fill = COLORS.free;
    else if (s === SEEN.WALL) fill = COLORS.wall;
    else if (truth && !e.open[v]) fill = COLORS.truth;
    ctx.fillStyle = fill;
    ctx.fillRect(ox + vx * px, oy + vy * px, Math.ceil(px), Math.ceil(px));
  }
  const at = (x: number, y: number): [number, number] => [ox + x * scale, oy + y * scale];
  ctx.strokeStyle = COLORS.beam;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (const [bx, by] of e.beams) {
    ctx.moveTo(...at(e.x, e.y));
    ctx.lineTo(...at(bx, by));
  }
  ctx.stroke();
  if (e.path.length > 1) {
    ctx.strokeStyle = COLORS.path;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(...at(e.x, e.y));
    for (const v of e.path) ctx.lineTo(...at(...e.center(v)));
    ctx.stroke();
  }
  ctx.fillStyle = COLORS.drone;
  ctx.beginPath();
  ctx.arc(...at(e.x, e.y), Math.max(4, scale * 0.3), 0, Math.PI * 2);
  ctx.fill();
}

export function startHow(root: HTMLElement): void {
  root.innerHTML = markup();
  const canvas = root.querySelector<HTMLCanvasElement>('[data-canvas]')!;
  const ctx = canvas.getContext('2d')!;
  const readout = root.querySelector<HTMLElement>('[data-readout]')!;
  const stageSelect = root.querySelector<HTMLSelectElement>('[data-stage]')!;
  const playBtn = root.querySelector<HTMLButtonElement>('[data-play]')!;
  const truthBox = root.querySelector<HTMLInputElement>('[data-truth]')!;
  let seed = 4242;
  let explorer = new Explorer(seed, Number(stageSelect.value));
  let simTime = 0;
  let playing = !prefersReducedMotion();
  let visible = true;
  let last = 0;
  let owed = 0;
  const api = { ready: true, coverage: 0 };
  window.__how = api;

  const show = () => {
    draw(ctx, explorer, truthBox.checked);
    api.coverage = explorer.coverage;
    readout.textContent = `Seed ${seed} · ${explorer.vw * VOXEL_SIZE} × ${explorer.vd * VOXEL_SIZE} m · ${Math.round(simTime)} s flown · ${pct(explorer.coverage)} covered${explorer.done ? ' · no frontiers left' : ''}`;
    playBtn.textContent = explorer.done ? 'Replay' : playing ? 'Pause' : 'Play';
    playBtn.setAttribute('aria-pressed', String(playing && !explorer.done));
  };
  const reset = (nextSeed: number) => {
    seed = nextSeed;
    explorer = new Explorer(seed, Number(stageSelect.value));
    simTime = 0;
    owed = 0;
    show();
  };
  const frame = (now: number) => {
    const dt = last ? Math.min(0.1, (now - last) / 1000) : 0;
    last = now;
    if (playing && visible && !explorer.done) {
      // Four seconds of flight per second of wall time, in fixed 0.1 s steps; the remainder carries to the next frame.
      owed += dt * 4;
      while (owed >= 0.1) {
        owed -= 0.1;
        explorer.step(0.1);
        simTime += 0.1;
      }
      show();
    }
    requestAnimationFrame(frame);
  };

  stageSelect.addEventListener('change', () => reset(seed));
  root.querySelector('[data-seed]')!.addEventListener('click', () => reset(1 + Math.floor(Math.random() * 99_999)));
  playBtn.addEventListener('click', () => {
    if (explorer.done) {
      reset(seed);
      playing = true;
    } else playing = !playing;
    show();
  });
  truthBox.addEventListener('change', show);
  new IntersectionObserver(([entry]) => {
    visible = Boolean(entry?.isIntersecting);
  }).observe(canvas);

  show();
  requestAnimationFrame(frame);
  root.dataset.ready = '1';
}
