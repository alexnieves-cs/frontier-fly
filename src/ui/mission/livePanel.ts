import { MISSION_TYPE_LABELS, TARGET_KIND_LABELS } from '../../mission/types';
import type { MissionReport, SessionSnapshot } from '../../mission/session';

export interface LiveActions {
  play: () => void;
  pause: () => void;
  restart: () => void;
  stop: () => void;
  allow: () => void;
  deny: () => void;
  speed: (value: number) => void;
}

export interface LivePanel {
  element: HTMLElement;
  update(snapshot: SessionSnapshot | null, playing: boolean): void;
}

function clock(sec: number): string {
  const whole = Math.max(0, Math.floor(sec));
  const minutes = Math.floor(whole / 60);
  return `${minutes}:${String(whole % 60).padStart(2, '0')}`;
}

function duration(sec: number): string {
  const whole = Math.max(0, Math.round(sec));
  const minutes = Math.floor(whole / 60);
  const seconds = whole % 60;
  return minutes > 0 ? `${minutes} min ${seconds} s` : `${seconds} s`;
}

export function createLivePanel(actions: LiveActions): LivePanel {
  const element = document.createElement('div');
  element.innerHTML = `
    <h2>Preview</h2>
    <p class="quiet">A lawnmower sweep over the site with the simulated camera. It is not the trained policy.</p>
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
        </select>
      </label>
    </div>
    <p class="quiet">Elapsed <span data-testid="mission-clock">0:00</span></p>
    <section data-testid="approval-prompt" hidden>
      <h3>Entry requested</h3>
      <p data-approval="where"></p>
      <p data-approval="what"></p>
      <div class="row-actions">
        <button type="button" class="primary" data-testid="approval-allow">Allow entry</button>
        <button type="button" data-testid="approval-deny">Stay outside</button>
      </div>
    </section>
    <h2>Detections</h2>
    <ol class="log" data-testid="detection-log"></ol>
    <section data-testid="mission-report" hidden></section>
  `;

  const play = element.querySelector<HTMLButtonElement>('[data-testid=play]')!;
  const pause = element.querySelector<HTMLButtonElement>('[data-testid=pause]')!;
  const restart = element.querySelector<HTMLButtonElement>('[data-testid=restart]')!;
  const stop = element.querySelector<HTMLButtonElement>('[data-testid=stop]')!;
  const speed = element.querySelector<HTMLSelectElement>('[data-testid=speed]')!;
  const clockEl = element.querySelector<HTMLElement>('[data-testid=mission-clock]')!;
  const approval = element.querySelector<HTMLElement>('[data-testid=approval-prompt]')!;
  const where = element.querySelector<HTMLElement>('[data-approval=where]')!;
  const what = element.querySelector<HTMLElement>('[data-approval=what]')!;
  const log = element.querySelector<HTMLOListElement>('[data-testid=detection-log]')!;
  const report = element.querySelector<HTMLElement>('[data-testid=mission-report]')!;
  play.addEventListener('click', actions.play);
  pause.addEventListener('click', actions.pause);
  restart.addEventListener('click', actions.restart);
  stop.addEventListener('click', actions.stop);
  element.querySelector<HTMLButtonElement>('[data-testid=approval-allow]')!.addEventListener('click', actions.allow);
  element.querySelector<HTMLButtonElement>('[data-testid=approval-deny]')!.addEventListener('click', actions.deny);
  speed.addEventListener('change', () => actions.speed(Number(speed.value)));

  let logKey = '';
  let reportKey = '';

  const update = (snapshot: SessionSnapshot | null, playing: boolean): void => {
    play.disabled = !snapshot || playing || snapshot.status === 'awaiting_approval' || snapshot.status === 'complete' || snapshot.status === 'aborted';
    pause.disabled = !playing;
    restart.disabled = !snapshot;
    stop.disabled = !snapshot || snapshot.status === 'complete' || snapshot.status === 'aborted';
    clockEl.textContent = snapshot ? clock(snapshot.tSec) : '0:00';

    const pending = snapshot?.pendingApproval ?? null;
    approval.hidden = !pending;
    if (pending) {
      where.textContent = pending.structureLabel
        ? `The aircraft would cross into ${pending.structureLabel} to confirm the contact.`
        : 'The aircraft would cross into a structure to confirm the contact.';
      what.textContent = `${TARGET_KIND_LABELS[pending.kind]}, ${pending.label}, ${Math.round(pending.confidence * 100)}% confidence.`;
    }

    const revealed = snapshot?.detections.filter((detection) => detection.revealed) ?? [];
    const nextKey = revealed.map((detection) => `${detection.id}:${detection.disposition}`).join('|');
    if (nextKey !== logKey) {
      logKey = nextKey;
      log.replaceChildren();
      if (revealed.length === 0) {
        const empty = document.createElement('li');
        empty.textContent = 'No contacts yet.';
        log.append(empty);
      }
      for (const detection of revealed) {
        const item = document.createElement('li');
        const disposition = detection.disposition === 'pending' ? 'awaiting approval' : detection.disposition;
        item.textContent = `${clock(detection.tSec)}  ${TARGET_KIND_LABELS[detection.kind]}  ${detection.label}  ${Math.round(detection.confidence * 100)}%  ${disposition}`;
        log.append(item);
      }
    }

    if (!snapshot?.report) {
      report.hidden = true;
      reportKey = '';
      return;
    }
    const key = `${snapshot.report.outcome}:${snapshot.report.elapsedSec.toFixed(1)}:${snapshot.report.detectionCount}`;
    if (key === reportKey) return;
    reportKey = key;
    report.hidden = false;
    report.className = 'report';
    report.replaceChildren(renderReport(snapshot.report));
  };

  return { element, update };
}

function renderReport(report: MissionReport): HTMLElement {
  const root = document.createElement('div');
  const title = document.createElement('h3');
  title.textContent = report.missionName;
  const outcome = document.createElement('p');
  outcome.className = 'outcome';
  outcome.textContent = `${outcomeLabel(report)}. ${MISSION_TYPE_LABELS[report.missionType]}.`;
  const metrics = document.createElement('div');
  metrics.className = 'metrics';
  for (const [label, value] of [
    ['Elapsed', duration(report.elapsedSec)],
    ['Battery used', `${Math.round(report.batteryUsedPct)}%`],
    ['Coverage', `${Math.round(report.coverageRatio * 100)}%`],
    ['Detections', String(report.detectionCount)],
    ['Entries allowed', String(report.entriesApproved)],
    ['Kept outside', String(report.entriesDenied)],
  ] as const) {
    const cell = document.createElement('div');
    cell.className = 'metric';
    const span = document.createElement('span');
    span.textContent = label;
    const strong = document.createElement('strong');
    strong.textContent = value;
    cell.append(span, strong);
    metrics.append(cell);
  }
  const targets = document.createElement('div');
  const targetHeading = document.createElement('h2');
  targetHeading.textContent = 'Targets';
  targets.append(targetHeading);
  for (const target of report.targets) {
    const row = document.createElement('p');
    row.textContent = `${target.label}: ${target.found} of ${target.minCount}${target.met ? ', met' : ''}`;
    targets.append(row);
  }
  const placeholders = document.createElement('div');
  placeholders.className = 'placeholders';
  const heading = document.createElement('h4');
  heading.textContent = 'Not measured in this preview';
  const list = document.createElement('dl');
  for (const label of ['Path efficiency', 'Sensor health', 'Comms dropouts', 'False-positive rate']) {
    const row = document.createElement('div');
    const dt = document.createElement('dt');
    dt.textContent = label;
    const dd = document.createElement('dd');
    dd.textContent = 'Not measured';
    row.append(dt, dd);
    list.append(row);
  }
  placeholders.append(heading, list);
  const notes = document.createElement('div');
  for (const note of report.notes) {
    const p = document.createElement('p');
    p.className = 'notes';
    p.textContent = note;
    notes.append(p);
  }
  root.append(title, outcome, metrics, targets, placeholders, notes);
  return root;
}

function outcomeLabel(report: MissionReport): string {
  if (report.outcome === 'complete') return 'Sweep finished';
  if (report.outcome === 'aborted') return 'Stopped by the operator';
  if (report.budgetCause === 'battery') return 'Stopped at the battery budget';
  return 'Stopped at the time budget';
}
