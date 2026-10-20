import { mockDetectionSource } from '../../mission/detectionMock';
import type { DetectionSource } from '../../mission/detections';
import { outdoorWorldProvider, perceptionDetectionSource, woodsSampleMission } from '../../mission/outdoorWorld';
import { canonicalMission, canonicalMissionJson, decodeMissionHash, missionToHash } from '../../mission/serialize';
import { createSession, type MissionSession, type SessionSnapshot } from '../../mission/session';
import { blankMission, sampleMission, suggestedTargets } from '../../mission/sample';
import {
  ENTRY_POLICIES,
  MISSION_SCHEMA_VERSION,
  MISSION_TYPE_LABELS,
  MISSION_TYPES,
  TARGET_KIND_LABELS,
  TARGET_KINDS,
  type EntryPolicy,
  type Mission,
  type MissionIssue,
  type MissionType,
  type Point,
  type TargetKind,
} from '../../mission/types';
import { validateMission } from '../../mission/validate';
import type { WorldProvider, WorldSnapshot } from '../../mission/world';
import { mockWorldProvider } from '../../mission/worldMock';
import { createExportPanel } from './exportPanel';
import { createLivePanel } from './livePanel';
import { createMissionMap, type MapLayers, type MapSelection, type MapTool } from './map';

const STORAGE_KEY = 'frontier-fly.mission.draft';

function el<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  options: { className?: string; text?: string; testid?: string; attrs?: Record<string, string> } = {},
): HTMLElementTagNameMap[K] {
  const node = document.createElement(tag);
  if (options.className) node.className = options.className;
  if (options.text != null) node.textContent = options.text;
  if (options.testid) node.dataset.testid = options.testid;
  if (options.attrs) {
    for (const [key, value] of Object.entries(options.attrs)) node.setAttribute(key, value);
  }
  return node;
}

function seedFrom(url: URL): number {
  const parsed = Number(url.searchParams.get('seed') ?? '11');
  if (!Number.isInteger(parsed) || parsed < 0 || parsed > 4294967295) return 11;
  return parsed;
}

function readStored(): Mission | null {
  try {
    const raw = sessionStorage.getItem(STORAGE_KEY);
    if (!raw) return null;
    const data: unknown = JSON.parse(raw);
    if (!data || typeof data !== 'object') return null;
    if ((data as { schemaVersion?: unknown }).schemaVersion !== MISSION_SCHEMA_VERSION) return null;
    return data as Mission;
  } catch {
    return null;
  }
}

function pathLabel(path: string): string {
  if (path.startsWith('geofence')) return 'Geofence';
  if (path.startsWith('noFlyZones')) return 'No-fly';
  if (path.startsWith('altitude')) return 'Altitude';
  if (path.startsWith('targets')) return 'Targets';
  if (path.startsWith('startPose.z')) return 'Start altitude';
  if (path.startsWith('startPose')) return 'Start';
  if (path.startsWith('budgets.battery')) return 'Battery';
  if (path.startsWith('budgets.time')) return 'Time';
  if (path === 'requiredConfidence') return 'Confidence';
  if (path === 'entryPolicy') return 'Entry';
  if (path === 'missionType') return 'Mission type';
  if (path === 'name') return 'Name';
  if (path === 'seed') return 'Seed';
  if (path === 'schemaVersion' || path === 'url') return 'Link';
  return 'Mission';
}

function nextIdent(prefix: string, used: string[]): string {
  let n = 1;
  while (used.includes(`${prefix}-${n}`)) n += 1;
  return `${prefix}-${n}`;
}

export function mountMissionEditor(root: HTMLElement): void {
  const url = new URL(location.href);
  // `?site=mock` keeps the fixed practice site and scripted detections for tests.
  const mockSite = url.searchParams.get('site') === 'mock';
  const provider: WorldProvider = mockSite ? mockWorldProvider : outdoorWorldProvider;
  const detector: DetectionSource = mockSite ? mockDetectionSource : perceptionDetectionSource;
  const sampleFor = (seed: number): Mission => (mockSite ? sampleMission(seed) : woodsSampleMission(seed));
  let bannerText: string | null = null;
  let mission = loadInitial(url, (message) => {
    bannerText = message;
  });
  let world: WorldSnapshot = provider.load(mission.seed >>> 0);
  let mode: 'plan' | 'run' = 'plan';
  let tool: MapTool = 'edit';
  let snap = true;
  let draft: Point[] | null = null;
  let selection: MapSelection | null = null;
  let fieldIssues: MissionIssue[] = [];
  let session: MissionSession | null = null;
  let playing = false;
  let speed = 1;
  let raf = 0;
  let zoneSignature = '';
  let targetSignature = '';

  root.className = 'mission';
  root.dataset.ready = '0';
  root.dataset.site = provider.id;

  const saveStatus = el('p', { className: 'save-status', testid: 'save-status', text: '' });
  const saveButton = el('button', { className: 'primary', text: 'Save link', testid: 'mission-save', attrs: { type: 'button' } });
  const planButton = el('button', { text: 'Plan', testid: 'mode-plan', attrs: { type: 'button', role: 'tab', 'aria-selected': 'true' } });
  const runButton = el('button', { text: 'Run', testid: 'mode-run', attrs: { type: 'button', role: 'tab', 'aria-selected': 'false' } });
  const header = el('header', { className: 'top' });
  const titles = el('div');
  titles.append(el('p', { className: 'eyebrow', text: 'Frontier Fly' }), el('h1', { text: 'Mission editor' }));
  const brand = el('div', { className: 'brand' });
  brand.append(el('span', { className: 'mark', attrs: { 'aria-hidden': 'true' } }), titles);
  const modes = el('div', { className: 'modes', attrs: { role: 'tablist' } });
  modes.append(planButton, runButton);
  const schema = el('p', { className: 'schema', testid: 'schema-version', text: `Schema v${MISSION_SCHEMA_VERSION}` });
  const sim = el('a', { className: 'sim-link', text: 'Open sim', attrs: { href: './sim.html' } });
  header.append(brand, modes, schema, el('span', { className: 'spacer' }), saveStatus, saveButton, sim);

  const nameInput = el('input', { attrs: { type: 'text', id: 'mission-name', maxlength: '80', autocomplete: 'off' }, testid: 'mission-name' });
  const seedInput = el('input', { attrs: { type: 'number', id: 'mission-seed', min: '0', step: '1' }, testid: 'mission-seed' });
  const typeSelect = el('select', { attrs: { id: 'mission-type' }, testid: 'mission-type' });
  for (const type of MISSION_TYPES) {
    const option = el('option', { text: MISSION_TYPE_LABELS[type], attrs: { value: type } });
    typeSelect.append(option);
  }
  const altMin = el('input', { attrs: { type: 'number', id: 'altitude-min', min: '0', max: '120', step: '0.5' }, testid: 'altitude-min' });
  const altMax = el('input', { attrs: { type: 'number', id: 'altitude-max', min: '0', max: '120', step: '0.5' }, testid: 'altitude-max' });
  const confidence = el('input', { attrs: { type: 'number', id: 'confidence', min: '0', max: '100', step: '1' }, testid: 'confidence' });
  const battery = el('input', { attrs: { type: 'number', id: 'battery', min: '1', max: '100', step: '1' }, testid: 'battery' });
  const timeMin = el('input', { attrs: { type: 'number', id: 'time-min', min: '0.5', max: '120', step: '0.5' }, testid: 'time-min' });
  const startZ = el('input', { attrs: { type: 'number', id: 'start-z', min: '0', max: '120', step: '0.5' }, testid: 'start-z' });
  const startYaw = el('input', { attrs: { type: 'number', id: 'start-yaw', min: '-180', max: '180', step: '1' }, testid: 'start-yaw' });
  const notes = el('textarea', { attrs: { id: 'mission-notes', maxlength: '500' } });
  const policyRadios = ENTRY_POLICIES.map((policy) => {
    const input = el('input', { attrs: { type: 'radio', name: 'entry', value: policy }, testid: `policy-${policy}` });
    const label = el('label', { className: 'choice' });
    label.append(input, document.createTextNode(policyText(policy)));
    return { policy, input, label };
  });

  const fields = el('fieldset');
  fields.append(
    el('h2', { text: 'Mission' }),
    labeled('Name', nameInput, 'name'),
    labeled('Seed', seedInput, 'seed'),
    labeled('Type', typeSelect, 'missionType'),
    el('h2', { text: 'Altitude band' }),
    pair(labeled('Minimum, m', altMin, 'altitude'), labeled('Maximum, m', altMax, 'altitude')),
    labeled('Required confidence, %', confidence, 'requiredConfidence'),
    el('h2', { text: 'Entry into structures' }),
    ...policyRadios.map((item) => item.label),
    errorSlot('entryPolicy'),
    el('h2', { text: 'Budgets' }),
    pair(labeled('Battery, %', battery, 'budgets.batteryPct'), labeled('Time, min', timeMin, 'budgets.timeSec')),
    el('h2', { text: 'Start' }),
    pair(labeled('Altitude, m', startZ, 'startPose.z'), labeled('Heading, deg', startYaw, 'startPose')),
    errorSlot('startPose'),
    el('h2', { text: 'Targets' }),
  );
  const targetList = el('div', { testid: 'target-list' });
  const addTarget = el('button', { text: 'Add target', testid: 'add-target', attrs: { type: 'button' } });
  const suggest = el('button', { text: 'Use suggested targets', attrs: { type: 'button' } });
  const targetActions = el('div', { className: 'row-actions' });
  targetActions.append(addTarget, suggest);
  fields.append(targetList, targetActions, errorSlot('targets'));
  fields.append(el('h2', { text: 'Notes' }), labeled('Notes', notes, 'notes'));
  const loadSample = el('button', { text: 'Load sample for this seed', testid: 'load-sample', attrs: { type: 'button' } });
  fields.append(loadSample, el('p', { className: 'help', text: 'G geofence, N no-fly, S start, E edit. Enter closes a ring. Delete removes a corner.' }));

  const rail = el('aside', { className: 'rail' });
  rail.append(fields);

  const toolButtons: Record<Exclude<MapTool, 'view'>, HTMLButtonElement> = {
    geofence: el('button', { text: 'Geofence', testid: 'tool-geofence', attrs: { type: 'button' } }),
    nofly: el('button', { text: 'No-fly', testid: 'tool-nofly', attrs: { type: 'button' } }),
    start: el('button', { text: 'Start', testid: 'tool-start', attrs: { type: 'button' } }),
    edit: el('button', { text: 'Edit', testid: 'tool-edit', attrs: { type: 'button' } }),
  };
  const snapInput = el('input', { attrs: { type: 'checkbox', id: 'snap' }, testid: 'snap' });
  snapInput.checked = true;
  const snapLabel = el('label', { className: 'snap', attrs: { for: 'snap' } });
  snapLabel.append(snapInput, document.createTextNode('Snap 1 m'));
  const fitButton = el('button', { text: 'Fit', testid: 'fit-map', attrs: { type: 'button' } });
  const readout = el('span', { className: 'readout', testid: 'map-readout', text: 'E —   N —' });
  const toolbar = el('div', { className: 'toolbar' });
  toolbar.append(toolButtons.geofence, toolButtons.nofly, toolButtons.start, toolButtons.edit, snapLabel, fitButton, readout);
  const legend = el('p', { className: 'legend' });
  legend.append(swatch('fence', 'Geofence'), swatch('nofly', 'No-fly'), swatch('building', 'Building'), swatch('track', 'Track'));
  const canvas = el('canvas', { testid: 'mission-map', attrs: { 'aria-hidden': 'true' } });
  const frame = el('div', { className: 'map-frame' });
  const hud = el('div', { className: 'hud', testid: 'live-metrics' });
  hud.hidden = true;
  const hudTime = hudCell('Elapsed');
  const hudBattery = hudCell('Battery');
  const hudCover = hudCell('Coverage');
  hud.append(hudTime.cell, hudBattery.cell, hudCover.cell);
  frame.append(canvas, hud);
  const summary = el('p', { className: 'summary', testid: 'mission-summary' });
  const mapError = el('p', { className: 'map-error', testid: 'map-error' });
  const stage = el('section', { className: 'stage' });
  stage.append(toolbar, legend, frame, summary, mapError);

  const banner = el('p', { className: 'banner', testid: 'link-banner' });
  banner.hidden = true;
  const issueList = el('div', { testid: 'mission-issues' });
  const zoneList = el('div', { testid: 'nofly-list' });
  const siteList = el('ul', { className: 'site-list', testid: 'site-list' });
  const linkField = el('input', {
    className: 'link',
    testid: 'mission-link',
    attrs: { type: 'text', readonly: 'true', placeholder: 'Save to create a shareable link.' },
  });
  const planSide = el('div', { testid: 'plan-side' });
  planSide.append(
    el('h2', { text: 'Check' }),
    banner,
    issueList,
    el('h2', { text: 'No-fly zones' }),
    zoneList,
    el('h2', { text: 'Site context' }),
    el('p', {
      className: 'quiet',
      text: mockSite
        ? 'Mock practice site for this seed.'
        : 'Generated woods for this seed: buildings, trails, and tree canopy. Detections come from the simulated camera.',
    }),
    siteList,
    el('h2', { text: 'Share' }),
    linkField,
  );
  const exporter = createExportPanel({
    mockSite,
    mission: () => mission,
    onImport: (next) => {
      mission = next;
      draft = null;
      selection = null;
      syncPlan();
      map.fit(viewPoints());
    },
    onPreview: () => map.setLayers(layers()),
  });
  planSide.append(exporter.element);
  const side = el('aside', { className: 'side' });

  const workspace = el('div', { className: 'workspace' });
  workspace.append(rail, stage, side);
  root.replaceChildren(header, workspace);

  const map = createMissionMap(canvas, {
    onPoint: (point) => {
      if (mode !== 'plan' || (tool !== 'geofence' && tool !== 'nofly')) return;
      const next = [...(draft ?? []), point];
      if (next.length > 64) return;
      draft = next;
      syncPlan();
    },
    onClose: () => commitDraft(),
    onPlaceStart: (point) => {
      if (mode !== 'plan') return;
      mission.startPose = { ...mission.startPose, x: point.x, y: point.y };
      selection = { kind: 'start' };
      syncPlan();
    },
    onMoveVertex: (hit, point) => {
      if (mode !== 'plan') return;
      if (hit.kind === 'start') mission.startPose = { ...mission.startPose, x: point.x, y: point.y };
      else if (hit.kind === 'geofence' && hit.vertex != null && mission.geofence.points[hit.vertex]) {
        mission.geofence.points[hit.vertex] = point;
      } else if (hit.kind === 'nofly' && hit.vertex != null) {
        const zone = mission.noFlyZones.find((item) => item.id === hit.zoneId);
        if (zone && zone.polygon.points[hit.vertex]) zone.polygon.points[hit.vertex] = point;
      }
      syncPlan();
    },
    onInsertVertex: (hit, point) => {
      if (mode !== 'plan' || hit.vertex == null) return;
      if (hit.kind === 'geofence') mission.geofence.points.splice(hit.vertex, 0, point);
      if (hit.kind === 'nofly') {
        const zone = mission.noFlyZones.find((item) => item.id === hit.zoneId);
        zone?.polygon.points.splice(hit.vertex, 0, point);
      }
      selection = hit;
      syncPlan();
    },
    onSelect: (hit) => {
      selection = hit;
      syncPlan();
    },
    onHover: (point) => {
      readout.textContent = point ? `E ${point.x.toFixed(1)} m   N ${point.y.toFixed(1)} m` : 'E —   N —';
    },
    onReady: () => {
      root.dataset.ready = '1';
    },
  });

  const live = createLivePanel({
    play: () => play(),
    pause: () => pause(),
    restart: () => restart(),
    stop: () => halt(),
    allow: () => answer(true),
    deny: () => answer(false),
    speed: (value) => {
      if (value === 1 || value === 4 || value === 16) speed = value;
    },
  });
  side.append(planSide, live.element);

  bindNumber(seedInput, 'seed', 'Seed', (value) => {
    mission.seed = Math.round(value);
    world = provider.load(mission.seed >>> 0);
  });
  bindNumber(altMin, 'altitude', 'Minimum altitude', (value) => {
    mission.altitude = { ...mission.altitude, minM: value };
  });
  bindNumber(altMax, 'altitude', 'Maximum altitude', (value) => {
    mission.altitude = { ...mission.altitude, maxM: value };
  });
  bindNumber(confidence, 'requiredConfidence', 'Confidence', (value) => {
    mission.requiredConfidence = value / 100;
  });
  bindNumber(battery, 'budgets.batteryPct', 'Battery', (value) => {
    mission.budgets = { ...mission.budgets, batteryPct: value };
  });
  bindNumber(timeMin, 'budgets.timeSec', 'Time', (value) => {
    mission.budgets = { ...mission.budgets, timeSec: Math.round(value * 60) };
  });
  bindNumber(startZ, 'startPose.z', 'Start altitude', (value) => {
    mission.startPose = { ...mission.startPose, z: value };
  });
  bindNumber(startYaw, 'startPose', 'Heading', (value) => {
    mission.startPose = { ...mission.startPose, yawRad: (value * Math.PI) / 180 };
  });
  nameInput.addEventListener('input', () => {
    mission.name = nameInput.value;
    syncPlan();
  });
  notes.addEventListener('input', () => {
    mission.notes = notes.value;
    syncPlan();
  });
  typeSelect.addEventListener('change', () => {
    mission.missionType = typeSelect.value as MissionType;
    syncPlan();
  });
  for (const item of policyRadios) {
    item.input.addEventListener('change', () => {
      if (item.input.checked) mission.entryPolicy = item.policy;
      syncPlan();
    });
  }
  for (const input of [seedInput, altMin, altMax, confidence, battery, timeMin, startZ, startYaw]) {
    input.addEventListener('wheel', (event) => event.preventDefault(), { passive: false });
  }
  addTarget.addEventListener('click', () => {
    if (mission.targets.length >= 16) return;
    const used = mission.targets.map((target) => target.id);
    mission.targets.push({ id: nextIdent('tgt', used), kind: 'object', label: 'New target', minCount: 1, minConfidence: null });
    targetSignature = '';
    syncPlan();
  });
  suggest.addEventListener('click', () => {
    mission.targets = suggestedTargets(mission.missionType);
    targetSignature = '';
    syncPlan();
  });
  loadSample.addEventListener('click', () => {
    mission = sampleFor(mission.seed || 11);
    world = provider.load(mission.seed);
    draft = null;
    selection = null;
    fieldIssues = [];
    zoneSignature = '';
    targetSignature = '';
    bannerText = null;
    syncPlan();
    map.fit(viewPoints());
  });
  saveButton.addEventListener('click', () => save());
  planButton.addEventListener('click', () => setMode('plan'));
  runButton.addEventListener('click', () => setMode('run'));
  for (const [name, button] of Object.entries(toolButtons) as [Exclude<MapTool, 'view'>, HTMLButtonElement][]) {
    button.addEventListener('click', () => setTool(name));
  }
  snapInput.addEventListener('change', () => {
    snap = snapInput.checked;
    map.setSnap(snap, 1);
  });
  fitButton.addEventListener('click', () => map.fit(viewPoints()));
  window.addEventListener('keydown', (event) => {
    if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 's') {
      event.preventDefault();
      save();
      return;
    }
    if (event.target instanceof HTMLElement && event.target.closest('input, textarea, select')) return;
    if (mode !== 'plan') return;
    const key = event.key.toLowerCase();
    if (key === 'g') setTool('geofence');
    else if (key === 'n') setTool('nofly');
    else if (key === 's') setTool('start');
    else if (key === 'e') setTool('edit');
    else if (key === 'enter') commitDraft();
    else if (key === 'escape') {
      draft = null;
      selection = null;
      syncPlan();
    } else if (key === 'delete' || key === 'backspace') {
      event.preventDefault();
      deleteSelection();
    }
  });

  map.setSnap(true, 1);
  setTool('edit');
  syncPlan();
  map.fit(viewPoints());
  if (bannerText) showBanner(bannerText);

  function loadInitial(pageUrl: URL, onBanner: (message: string) => void): Mission {
    if (pageUrl.hash.startsWith('#m=')) {
      const decoded = decodeMissionHash(pageUrl.hash);
      if (decoded.ok && decoded.mission) return decoded.mission;
      onBanner(decoded.issues[0]?.message ?? 'This link could not be read.');
    }
    if (pageUrl.searchParams.get('blank') === '1') return blankMission(seedFrom(pageUrl));
    if (!pageUrl.searchParams.has('seed')) {
      const stored = readStored();
      if (stored) return stored;
    }
    return sampleFor(seedFrom(pageUrl));
  }

  function viewPoints(): Point[] {
    if (mission.geofence.points.length >= 3) return mission.geofence.points;
    const footprints = world.footprints.flatMap((footprint) => footprint.polygon);
    return footprints.length > 0 ? footprints : [
      { x: 0, y: 0 },
      { x: 90, y: 60 },
    ];
  }

  function setTool(next: Exclude<MapTool, 'view'>): void {
    if (mode !== 'plan') return;
    if (next !== tool) draft = null;
    tool = next;
    map.setTool(next);
    for (const [name, button] of Object.entries(toolButtons)) button.setAttribute('aria-pressed', name === next ? 'true' : 'false');
    syncPlan();
  }

  function setMode(next: 'plan' | 'run'): void {
    if (next === 'plan') {
      pause();
      mode = 'plan';
      map.setTool(tool === 'view' ? 'edit' : tool);
      showMode();
      syncPlan();
      return;
    }
    const validation = validateMission(mission);
    const errors = combinedIssues(validation.issues).filter((issue) => issue.severity === 'error');
    if (!validation.ok || errors.length > 0) {
      showBanner('Fix the errors in the plan before running the preview.');
      mode = 'plan';
      showMode();
      syncPlan();
      return;
    }
    pause();
    mode = 'run';
    mission = canonicalMission(validation.mission ?? mission);
    session = createSession(mission, world, detector);
    map.setTool('view');
    showMode();
    syncPlan();
    syncLive();
  }

  function showMode(): void {
    planButton.setAttribute('aria-selected', mode === 'plan' ? 'true' : 'false');
    runButton.setAttribute('aria-selected', mode === 'run' ? 'true' : 'false');
    fields.disabled = mode !== 'plan';
    for (const button of Object.values(toolButtons)) button.disabled = mode !== 'plan';
    planSide.hidden = mode !== 'plan';
    live.element.hidden = mode !== 'run';
    hud.hidden = mode !== 'run';
  }

  function commitDraft(): void {
    if (!draft || draft.length < 3 || mode !== 'plan') return;
    if (tool === 'nofly') {
      if (mission.noFlyZones.length >= 12) return;
      const used = mission.noFlyZones.map((zone) => zone.id);
      mission.noFlyZones.push({
        id: nextIdent('nf', used),
        name: `No-fly ${mission.noFlyZones.length + 1}`,
        polygon: { points: draft.map((point) => ({ ...point })) },
        reason: 'Marked by the operator.',
      });
      zoneSignature = '';
    } else {
      mission.geofence = { points: draft.map((point) => ({ ...point })) };
    }
    draft = null;
    tool = 'edit';
    map.setTool('edit');
    for (const [name, button] of Object.entries(toolButtons)) button.setAttribute('aria-pressed', name === 'edit' ? 'true' : 'false');
    syncPlan();
  }

  function deleteSelection(): void {
    if (!selection || selection.vertex == null) return;
    if (selection.kind === 'geofence' && mission.geofence.points.length > 3) {
      mission.geofence.points.splice(selection.vertex, 1);
    } else if (selection.kind === 'nofly') {
      const zone = mission.noFlyZones.find((item) => item.id === selection?.zoneId);
      if (zone && zone.polygon.points.length > 3) zone.polygon.points.splice(selection.vertex, 1);
    }
    selection = null;
    syncPlan();
  }

  function combinedIssues(structural: MissionIssue[]): MissionIssue[] {
    return [...fieldIssues, ...structural];
  }

  function syncPlan(): void {
    const validation = validateMission(mission);
    const issues = combinedIssues(validation.issues);
    const errors = issues.filter((issue) => issue.severity === 'error');
    saveButton.disabled = errors.length > 0;
    setValue(nameInput, mission.name);
    setValue(seedInput, String(mission.seed));
    if (document.activeElement !== typeSelect) typeSelect.value = mission.missionType;
    setValue(altMin, String(mission.altitude.minM));
    setValue(altMax, String(mission.altitude.maxM));
    setValue(confidence, String(Math.round(mission.requiredConfidence * 100)));
    setValue(battery, String(mission.budgets.batteryPct));
    setValue(timeMin, String(mission.budgets.timeSec / 60));
    setValue(startZ, String(mission.startPose.z));
    setValue(startYaw, ((mission.startPose.yawRad * 180) / Math.PI).toFixed(1));
    setValue(notes, mission.notes);
    for (const item of policyRadios) item.input.checked = mission.entryPolicy === item.policy;
    paintErrors(issues);
    issueList.replaceChildren();
    if (issues.length === 0) {
      issueList.append(el('p', { className: 'quiet', text: 'No errors. The mission can be saved and shared.' }));
    } else {
      for (const issue of issues) {
        const row = el('p', { className: issue.severity === 'warning' ? 'issue warning' : 'issue' });
        const label = el('strong', { text: `${pathLabel(issue.path)}. ` });
        row.append(label, document.createTextNode(issue.message));
        issueList.append(row);
      }
    }
    const geometry = issues.find((issue) => issue.severity === 'error' && /geofence|noFly|startPose/.test(issue.path));
    mapError.textContent = geometry ? geometry.message : '';
    const corners = mission.geofence.points.length;
    const zones = mission.noFlyZones.length;
    summary.textContent = `${corners >= 3 ? `Geofence ${corners} corners.` : 'Geofence is not closed.'} ${
      zones === 0 ? 'No no-fly zones.' : zones === 1 ? '1 no-fly zone.' : `${zones} no-fly zones.`
    } Start E ${mission.startPose.x.toFixed(1)} m, N ${mission.startPose.y.toFixed(1)} m.`;
    root.dataset.corners = String(corners);
    root.dataset.draft = String(draft?.length ?? 0);
    document.title = `Mission editor — ${mission.name || 'Untitled'}`;
    renderTargets();
    renderZones();
    renderSite();
    exporter.refresh();
    map.setLayers(layers());
    if (mode === 'plan') persist();
  }

  function layers(): MapLayers {
    const liveSnap: SessionSnapshot | null = mode === 'run' && session ? session.snapshot() : null;
    return {
      footprints: world.footprints,
      geofence: mission.geofence.points,
      noFly: mission.noFlyZones.map((zone) => ({
        id: zone.id,
        points: zone.polygon.points,
        selected: selection?.kind === 'nofly' && selection.zoneId === zone.id,
      })),
      draft,
      draftKind: tool === 'nofly' ? 'nofly' : 'geofence',
      start: { x: mission.startPose.x, y: mission.startPose.y, yawRad: mission.startPose.yawRad },
      plan: liveSnap?.plan ?? (mode === 'plan' ? (exporter.preview() ?? []) : []),
      track: liveSnap?.track ?? [],
      drone: liveSnap ? liveSnap.pose : null,
      coverage: liveSnap?.coverage ?? null,
      detections: (liveSnap?.detections ?? [])
        .filter((detection) => detection.revealed)
        .map((detection) => ({ x: detection.x, y: detection.y, kind: detection.kind, label: detection.label })),
      selection,
    };
  }

  function renderTargets(): void {
    const signature = mission.targets.map((target) => target.id).join('|');
    if (signature === targetSignature) return;
    targetSignature = signature;
    targetList.replaceChildren();
    for (const target of mission.targets) {
      const card = el('div', { className: 'target' });
      const kind = el('select');
      for (const value of TARGET_KINDS) kind.append(el('option', { text: TARGET_KIND_LABELS[value], attrs: { value } }));
      kind.value = target.kind;
      kind.addEventListener('change', () => {
        target.kind = kind.value as TargetKind;
        syncPlan();
      });
      const label = el('input', { attrs: { type: 'text', maxlength: '80' } });
      label.value = target.label;
      label.addEventListener('input', () => {
        target.label = label.value;
        syncPlan();
      });
      const count = el('input', { attrs: { type: 'number', min: '1', max: '50', step: '1' } });
      count.value = String(target.minCount);
      count.addEventListener('input', () => {
        const parsed = Number(count.value);
        if (Number.isInteger(parsed)) target.minCount = parsed;
        syncPlan();
      });
      const remove = el('button', { text: 'Remove', attrs: { type: 'button' } });
      remove.disabled = mission.targets.length === 1;
      remove.addEventListener('click', () => {
        mission.targets = mission.targets.filter((item) => item.id !== target.id);
        targetSignature = '';
        syncPlan();
      });
      card.append(labeled('Kind', kind, `targets.${target.id}`), labeled('Label', label, `targets.${target.id}.label`), labeled('Minimum count', count, `targets.${target.id}.count`), remove);
      targetList.append(card);
    }
  }

  function renderZones(): void {
    const signature = `${mission.noFlyZones.map((zone) => zone.id).join('|')}:${selection?.zoneId ?? ''}`;
    if (signature === zoneSignature) return;
    zoneSignature = signature;
    zoneList.replaceChildren();
    if (mission.noFlyZones.length === 0) {
      zoneList.append(el('p', { className: 'quiet', text: 'None. Use No-fly to draw one.' }));
      return;
    }
    for (const zone of mission.noFlyZones) {
      const card = el('div', { className: selection?.zoneId === zone.id ? 'zone selected' : 'zone' });
      const name = el('input', { attrs: { type: 'text', maxlength: '80' } });
      name.value = zone.name;
      name.addEventListener('input', () => {
        zone.name = name.value;
        syncPlan();
      });
      const reason = el('input', { attrs: { type: 'text', maxlength: '160' } });
      reason.value = zone.reason;
      reason.addEventListener('input', () => {
        zone.reason = reason.value;
        syncPlan();
      });
      const remove = el('button', { text: 'Delete zone', attrs: { type: 'button' } });
      remove.addEventListener('click', () => {
        mission.noFlyZones = mission.noFlyZones.filter((item) => item.id !== zone.id);
        zoneSignature = '';
        if (selection?.zoneId === zone.id) selection = null;
        syncPlan();
      });
      card.addEventListener('click', () => {
        selection = { kind: 'nofly', zoneId: zone.id };
        zoneSignature = '';
        syncPlan();
      });
      card.append(labeled('Name', name, `zone.${zone.id}`), labeled('Reason', reason, `zone.${zone.id}.reason`), remove);
      zoneList.append(card);
    }
  }

  function renderSite(): void {
    siteList.replaceChildren();
    const title = el('li');
    title.append(el('span', { text: world.siteName }), el('span', { text: `seed ${world.seed}` }));
    siteList.append(title);
    const canopy = world.footprints.filter((footprint) => footprint.kind === 'canopy');
    if (canopy.length > 0) {
      const top = Math.max(...canopy.map((footprint) => footprint.heightM ?? 0));
      const row = el('li');
      row.append(el('span', { text: `Tree canopy, ${canopy.length} stands` }), el('span', { text: `${top} m` }));
      siteList.append(row);
    }
    for (const footprint of world.footprints) {
      if (footprint.kind === 'canopy') continue;
      const row = el('li');
      const height = footprint.heightM == null ? footprint.kind : `${footprint.heightM} m`;
      row.append(el('span', { text: footprint.label }), el('span', { text: height }));
      siteList.append(row);
    }
  }

  function paintErrors(issues: MissionIssue[]): void {
    const slots = root.querySelectorAll<HTMLElement>('[data-for]');
    slots.forEach((slot) => {
      const path = slot.dataset.for ?? '';
      const match = issues.filter((issue) => issue.severity === 'error' && (issue.path === path || issue.path.startsWith(`${path}.`) || issue.path.startsWith(`${path}[`)));
      slot.textContent = match.map((issue) => issue.message).join(' ');
    });
    const controls: [HTMLElement, string][] = [
      [nameInput, 'name'],
      [seedInput, 'seed'],
      [typeSelect, 'missionType'],
      [altMin, 'altitude'],
      [altMax, 'altitude'],
      [confidence, 'requiredConfidence'],
      [battery, 'budgets.batteryPct'],
      [timeMin, 'budgets.timeSec'],
      [startZ, 'startPose.z'],
      [startYaw, 'startPose'],
    ];
    for (const [control, path] of controls) {
      const invalid = issues.some((issue) => issue.severity === 'error' && (issue.path === path || issue.path.startsWith(`${path}.`)));
      control.setAttribute('aria-invalid', invalid ? 'true' : 'false');
    }
  }

  function persist(): void {
    try {
      sessionStorage.setItem(STORAGE_KEY, canonicalMissionJson(mission));
    } catch {
      /* ignore quota */
    }
  }

  function save(): void {
    const validation = validateMission(mission);
    if (!validation.ok || !validation.mission) {
      saveStatus.textContent = 'Fix the errors before saving.';
      syncPlan();
      return;
    }
    mission = canonicalMission(validation.mission);
    const next = new URL(location.href);
    next.searchParams.delete('blank');
    next.searchParams.set('seed', String(mission.seed));
    next.hash = missionToHash(mission);
    history.replaceState(null, '', next);
    linkField.value = next.toString();
    saveStatus.textContent = 'Link updated.';
    zoneSignature = '';
    targetSignature = '';
    syncPlan();
    void navigator.clipboard?.writeText(next.toString()).then(
      () => {
        saveStatus.textContent = 'Link copied.';
      },
      () => {
        saveStatus.textContent = 'Link updated in the address bar.';
      },
    );
  }

  function showBanner(message: string): void {
    banner.hidden = false;
    banner.textContent = message;
  }

  function play(): void {
    if (!session || mode !== 'run') return;
    if (session.snapshot().status === 'ready') session.start();
    if (session.snapshot().status !== 'running') {
      syncLive();
      return;
    }
    playing = true;
    let last = performance.now();
    const tick = (now: number): void => {
      const dt = Math.min(0.05, (now - last) / 1000);
      last = now;
      session?.step(dt * speed);
      syncLive();
      if (playing && session?.snapshot().status === 'running') raf = requestAnimationFrame(tick);
      else playing = false;
    };
    raf = requestAnimationFrame(tick);
    syncLive();
  }

  function pause(): void {
    playing = false;
    if (raf) cancelAnimationFrame(raf);
    raf = 0;
    if (mode === 'run') syncLive();
  }

  function restart(): void {
    pause();
    session = createSession(mission, world, detector);
    syncLive();
  }

  function halt(): void {
    pause();
    session?.abort();
    syncLive();
  }

  function answer(allow: boolean): void {
    if (!session) return;
    if (allow) session.approve();
    else session.deny();
    syncLive();
    play();
  }

  function syncLive(): void {
    const snapshot = session?.snapshot() ?? null;
    live.update(snapshot, playing);
    if (!snapshot) return;
    hudTime.value.textContent = clock(snapshot.tSec);
    hudBattery.value.textContent = `${Math.round(snapshot.batteryRemainingPct)}%`;
    hudCover.value.textContent = `${Math.round(snapshot.coverage.ratio * 100)}%`;
    map.setLayers(layers());
  }

  function bindNumber(input: HTMLInputElement, path: string, label: string, apply: (value: number) => void): void {
    input.addEventListener('input', () => {
      fieldIssues = fieldIssues.filter((issue) => issue.path !== path);
      const parsed = Number(input.value);
      if (input.value.trim() === '' || !Number.isFinite(parsed)) {
        fieldIssues.push({ path, message: `${label} needs a number.`, severity: 'error' });
      } else {
        apply(parsed);
      }
      syncPlan();
    });
  }
}

function labeled(text: string, control: HTMLElement, path: string): HTMLLabelElement {
  const wrap = el('label', { className: 'field' });
  const id = control.id || `${path}-${text}`.replace(/[^a-z0-9-]+/gi, '-').toLowerCase();
  control.id = id;
  wrap.htmlFor = id;
  wrap.append(el('span', { text }), control, errorSlot(path));
  return wrap;
}

function errorSlot(path: string): HTMLElement {
  return el('small', { className: 'field-error', attrs: { 'data-for': path } });
}

function pair(a: HTMLElement, b: HTMLElement): HTMLElement {
  const wrap = el('div', { className: 'pair' });
  wrap.append(a, b);
  return wrap;
}

function swatch(kind: string, text: string): HTMLElement {
  const node = el('span');
  const mark = document.createElement('i');
  mark.className = kind;
  node.append(mark, document.createTextNode(text));
  return node;
}

function hudCell(label: string): { cell: HTMLElement; value: HTMLElement } {
  const cell = el('div');
  const value = el('strong', { text: '—' });
  cell.append(el('span', { text: label }), value);
  return { cell, value };
}

function policyText(policy: EntryPolicy): string {
  if (policy === 'auto') return 'Auto, enter to confirm';
  if (policy === 'ask') return 'Ask, wait for an operator';
  return 'Never, stay outside';
}

function clock(sec: number): string {
  const whole = Math.max(0, Math.floor(sec));
  return `${Math.floor(whole / 60)}:${String(whole % 60).padStart(2, '0')}`;
}

function setValue(input: HTMLInputElement | HTMLTextAreaElement, value: string): void {
  if (document.activeElement === input) return;
  if (input.value !== value) input.value = value;
}
