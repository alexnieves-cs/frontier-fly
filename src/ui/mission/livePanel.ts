import type { AutonomyReport, Phase, RunnerSnapshot } from '../../mission/autonomy/contracts';
import { reportRows } from '../../mission/autonomy/report';
import { MISSION_TYPE_LABELS } from '../../mission/types';

export interface LiveActions {
  play: () => void;
  pause: () => void;
  restart: () => void;
  stop: () => void;
  allow: () => void;
  deny: () => void;
  speed: (value: number) => void;
}

export interface LiveView {
  snapshot: RunnerSnapshot | null;
  playing: boolean;
  /** Set while the flight is being built. */
  preparing: boolean;
  report: AutonomyReport | null;
  /** Building label for an id. */
  label: (buildingId: string | null) => string | null;
  /** Shown under the heading: how the flight differs from the plan. */
  note: string | null;
}

export interface LivePanel {
  element: HTMLElement;
  update(view: LiveView): void;
}

export const PHASE_LABELS: Record<Phase, string> = {
  preflight: 'Preflight',
  coverage: 'Searching outdoors',
  approach: 'Approaching a building',
  inspect: 'Circling a building',
  entry_select: 'Choosing an opening',
  await_operator: 'Waiting for the operator',
  fly_through: 'Flying through an opening',
  indoor_explore: 'Exploring indoors',
  confirm: 'Confirming a contact',
  exit: 'Leaving the building',
  return_home: 'Returning home',
  land: 'Landing',
  recover: 'Recovering from a fault',
  done: 'Landed',
};

const OUTCOME_LABELS: Record<AutonomyReport['outcome'], string> = {
  found: 'Target found and confirmed',
  clear: 'Area confirmed clear',
  not_found: 'Searched everything, target not found',
  incomplete: 'Stopped at a budget before the search finished',
  aborted: 'Stopped by the operator',
  landed: 'Landed early',
};

const LOG_LINES = 8;

function clock(sec: number): string {
  const whole = Math.max(0, Math.floor(sec));
  const minutes = Math.floor(whole / 60);
  return `${minutes}:${String(whole % 60).padStart(2, '0')}`;
}

function classLabel(classId: string): string {
  return classId.replace(/_/g, ' ');
}

export function createLivePanel(actions: LiveActions): LivePanel {
  const element = document.createElement('div');
  element.innerHTML = `
    <h2>Flight</h2>
    <p class="quiet">The same autonomy as the demo and the benchmarks: the drone searches with its simulated camera, maps buildings with lidar, and confirms contacts before it reports them.</p>
    <p class="quiet" data-testid="flight-note" hidden></p>
    <div class="playback">
      <button type="button" data-testid="play">Play</button>
      <button type="button" data-testid="pause">Pause</button>
      <button type="button" data-testid="restart">Restart</button>
      <button type="button" data-testid="stop">Stop</button>
      <label class="snap">Speed
        <select data-testid="speed">
          <option value="1">1×</option>
          <option value="4">4×</option>
          <option value="16">16×</option>
          <option value="64">64×</option>
        </select>
      </label>
    </div>
    <p class="quiet">Elapsed <span data-testid="mission-clock">0:00</span> · <span data-testid="mission-phase">Preparing</span></p>
    <section data-testid="approval-prompt" hidden>
      <h3>Entry requested</h3>
      <p data-approval="where"></p>
      <p data-approval="what"></p>
      <div class="row-actions">
        <button type="button" class="primary" data-testid="approval-allow">Allow entry</button>
        <button type="button" data-testid="approval-deny">Stay outside</button>
      </div>
    </section>
    <h2>Confirmed</h2>
    <ol class="log" data-testid="detection-log"></ol>
    <h2>Flight log</h2>
    <ol class="log" data-testid="flight-log"></ol>
    <section data-testid="mission-report" hidden></section>
  `;

  const q = <T extends HTMLElement>(selector: string): T => element.querySelector<T>(selector)!;
  const play = q<HTMLButtonElement>('[data-testid=play]');
  const pause = q<HTMLButtonElement>('[data-testid=pause]');
  const restart = q<HTMLButtonElement>('[data-testid=restart]');
  const stop = q<HTMLButtonElement>('[data-testid=stop]');
  const speed = q<HTMLSelectElement>('[data-testid=speed]');
  const note = q<HTMLElement>('[data-testid=flight-note]');
  const clockEl = q<HTMLElement>('[data-testid=mission-clock]');
  const phaseEl = q<HTMLElement>('[data-testid=mission-phase]');
  const approval = q<HTMLElement>('[data-testid=approval-prompt]');
  const where = q<HTMLElement>('[data-approval=where]');
  const what = q<HTMLElement>('[data-approval=what]');
  const confirmed = q<HTMLOListElement>('[data-testid=detection-log]');
  const log = q<HTMLOListElement>('[data-testid=flight-log]');
  const report = q<HTMLElement>('[data-testid=mission-report]');
  play.addEventListener('click', actions.play);
  pause.addEventListener('click', actions.pause);
  restart.addEventListener('click', actions.restart);
  stop.addEventListener('click', actions.stop);
  q<HTMLButtonElement>('[data-testid=approval-allow]').addEventListener('click', actions.allow);
  q<HTMLButtonElement>('[data-testid=approval-deny]').addEventListener('click', actions.deny);
  speed.addEventListener('change', () => actions.speed(Number(speed.value)));

  let confirmedKey = '';
  let logKey = '';
  let reportKey = '';

  const update = (view: LiveView): void => {
    const { snapshot, playing } = view;
    const pending = snapshot?.pendingEntry ?? null;
    const done = !snapshot || snapshot.done;
    play.disabled = view.preparing || playing || done || pending !== null;
    pause.disabled = !playing;
    restart.disabled = view.preparing;
    stop.disabled = view.preparing || done;
    note.hidden = !view.note;
    note.textContent = view.note ?? '';
    clockEl.textContent = snapshot ? clock(snapshot.tSec) : '0:00';
    phaseEl.textContent = view.preparing ? 'Building the site and loading the indoor policy…' : snapshot ? PHASE_LABELS[snapshot.phase] : '';

    approval.hidden = !pending;
    if (pending) {
      const label = view.label(pending.buildingId) ?? 'a building';
      where.textContent = `The drone wants to fly into ${label} through opening ${pending.openingId} (${pending.kind.replace(/_/g, ' ')}).`;
      what.textContent = `${pending.clearWidth.toFixed(2)} × ${pending.clearHeight.toFixed(2)} m clear, ${pending.clearanceM.toFixed(2)} m of margin, safety score ${Math.round(pending.safety * 100)}%.`;
    }

    const contacts = snapshot?.confirmed ?? [];
    const nextConfirmed = contacts.map((contact) => contact.contactId).join('|');
    if (nextConfirmed !== confirmedKey || !snapshot) {
      confirmedKey = nextConfirmed;
      confirmed.replaceChildren();
      if (contacts.length === 0) {
        const empty = document.createElement('li');
        empty.textContent = 'Nothing confirmed yet.';
        confirmed.append(empty);
      }
      for (const contact of contacts) {
        const item = document.createElement('li');
        const place = view.label(contact.buildingId) ?? 'outdoors';
        item.textContent = `${clock(contact.tSec)}  ${classLabel(contact.classId)}  ${Math.round(contact.confidence * 100)}%  ${contact.views} views  ${place}`;
        confirmed.append(item);
      }
    }

    const lines = snapshot?.log.slice(-LOG_LINES) ?? [];
    const nextLog = `${snapshot?.log.length ?? 0}:${lines[lines.length - 1]?.tSec ?? 0}`;
    if (nextLog !== logKey) {
      logKey = nextLog;
      log.replaceChildren();
      for (const entry of lines) {
        const item = document.createElement('li');
        item.dataset.tone = entry.tone;
        item.textContent = `${clock(entry.tSec)}  ${entry.message}`;
        log.append(item);
      }
    }

    if (!view.report) {
      report.hidden = true;
      reportKey = '';
      return;
    }
    const key = `${view.report.outcome}:${view.report.durationSec.toFixed(1)}`;
    if (key === reportKey) return;
    reportKey = key;
    report.hidden = false;
    report.className = 'report';
    report.dataset.outcome = view.report.outcome;
    report.replaceChildren(renderReport(view.report));
  };

  return { element, update };
}

function renderReport(report: AutonomyReport): HTMLElement {
  const root = document.createElement('div');
  const title = document.createElement('h3');
  title.textContent = report.missionName;
  const outcome = document.createElement('p');
  outcome.className = 'outcome';
  outcome.textContent = `${OUTCOME_LABELS[report.outcome]}. ${MISSION_TYPE_LABELS[report.missionType]}.`;
  const metrics = document.createElement('div');
  metrics.className = 'metrics';
  // The same rows the benchmarks report, scored against the simulator's truth after the flight.
  const skip = new Set(['Mission', 'Type', 'Outcome']);
  for (const row of reportRows(report)) {
    if (skip.has(row.label)) continue;
    const cell = document.createElement('div');
    cell.className = 'metric';
    const label = document.createElement('span');
    label.textContent = row.label;
    const value = document.createElement('strong');
    value.textContent = row.value;
    cell.append(label, value);
    metrics.append(cell);
  }
  root.append(title, outcome, metrics);
  for (const text of report.notes) {
    const p = document.createElement('p');
    p.className = 'notes';
    p.textContent = text;
    root.append(p);
  }
  return root;
}
