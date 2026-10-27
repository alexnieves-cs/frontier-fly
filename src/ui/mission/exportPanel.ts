import { buildMissionWorld } from '../../mission/autonomy/world';
import type { MissionWorld } from '../../mission/autonomy/contracts';
import {
  buildFlightPlan,
  exportStem,
  type AltitudeRef,
  type Finish,
  type Firmware,
  type FlightPlan,
} from '../../mission/export/flightPlan';
import { mavlinkJson, qgcPlanJson, toWpl } from '../../mission/export/formats';
import { GEO_ORIGINS, isValidOrigin, type GeoOriginId, type GeoPoint } from '../../mission/export/geo';
import { importQgcPlan } from '../../mission/export/importPlan';
import { routeFromCoverage, type MissionRoute } from '../../mission/export/route';
import type { Mission, Point } from '../../mission/types';
import { validateMission } from '../../mission/validate';

/**
 * "Export to QGroundControl": turn the mission into a .plan, a QGC WPL 110
 * waypoint file or MAVLink mission items, and read a .plan's fences back in.
 * The panel is a closed <details> until opened; only then does it plan the
 * route and draw it on the map.
 */

type Format = 'plan' | 'wpl' | 'mavlink';

const FORMATS: Record<Format, { label: string; ext: string; mime: string }> = {
  plan: { label: 'QGroundControl plan (.plan)', ext: 'plan', mime: 'application/json' },
  wpl: { label: 'Waypoint file, QGC WPL 110 (.waypoints)', ext: 'waypoints', mime: 'text/plain' },
  mavlink: { label: 'MAVLink mission items (.json)', ext: 'mavlink.json', mime: 'application/json' },
};

export interface ExportPanelOptions {
  /** The mission as it stands in the editor. */
  mission: () => Mission;
  /** A valid mission read from a .plan; the editor adopts it. */
  onImport: (mission: Mission) => void;
  /** The route changed, or the panel opened or closed. Null hides the preview. */
  onPreview: (points: Point[] | null) => void;
}

export interface ExportPanel {
  element: HTMLElement;
  /** Call after every mission edit. Cheap while the panel is closed. */
  refresh(): void;
  /** Route points for the map, or null while the panel is closed. */
  preview(): Point[] | null;
}

function el<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  options: { className?: string; text?: string; testid?: string; attrs?: Record<string, string> } = {},
): HTMLElementTagNameMap[K] {
  const node = document.createElement(tag);
  if (options.className) node.className = options.className;
  if (options.text != null) node.textContent = options.text;
  if (options.testid) node.dataset.testid = options.testid;
  if (options.attrs) for (const [key, value] of Object.entries(options.attrs)) node.setAttribute(key, value);
  return node;
}

function field(text: string, control: HTMLElement): HTMLLabelElement {
  const wrap = el('label', { className: 'field' });
  wrap.htmlFor = control.id;
  wrap.append(el('span', { text }), control);
  return wrap;
}

function select<T extends string>(id: string, options: [T, string][], value: T): HTMLSelectElement {
  const node = el('select', { testid: id, attrs: { id } });
  for (const [key, label] of options) node.append(el('option', { text: label, attrs: { value: key } }));
  node.value = value;
  return node;
}

function minutes(sec: number): string {
  const m = Math.floor(sec / 60);
  const s = Math.round(sec % 60);
  return m ? `${m} min ${s} s` : `${s} s`;
}

export function createExportPanel(options: ExportPanelOptions): ExportPanel {
  const details = el('details', { className: 'export', testid: 'export-panel' });
  const summaryLine = el('summary', { text: 'Export to QGroundControl' });
  const format = select<Format>(
    'export-format',
    (Object.keys(FORMATS) as Format[]).map((key) => [key, FORMATS[key].label]),
    'plan',
  );
  const firmware = select<Firmware>('export-firmware', [
    ['px4', 'PX4'],
    ['ardupilot', 'ArduPilot'],
  ], 'px4');
  const origin = select<GeoOriginId | 'custom'>(
    'export-origin',
    [
      ['px4-sitl', 'PX4 SITL home (Zurich)'],
      ['ardupilot-sitl', 'ArduPilot SITL home (Canberra)'],
      ['custom', 'Custom latitude and longitude'],
    ],
    'px4-sitl',
  );
  const lat = el('input', { testid: 'export-lat', attrs: { id: 'export-lat', type: 'number', step: '0.000001', min: '-85', max: '85' } });
  const lon = el('input', { testid: 'export-lon', attrs: { id: 'export-lon', type: 'number', step: '0.000001', min: '-180', max: '180' } });
  const amsl = el('input', { testid: 'export-amsl', attrs: { id: 'export-amsl', type: 'number', step: '1', min: '-400', max: '8800' } });
  const custom = el('div', { className: 'pair' });
  custom.append(field('Latitude', lat), field('Longitude', lon));
  const customAlt = field('Home altitude, m AMSL', amsl);
  const altitude = select<AltitudeRef>('export-altitude', [
    ['relative', 'Relative to home'],
    ['terrain', 'Above terrain'],
  ], 'relative');
  const finish = select<Finish>('export-finish', [
    ['rtl', 'Return to launch'],
    ['land', 'Fly home and land'],
  ], 'rtl');
  const video = el('input', { testid: 'export-video', attrs: { id: 'export-video', type: 'checkbox' } });
  const videoLabel = el('label', { className: 'choice' });
  videoLabel.append(video, document.createTextNode('Record video during the sweep'));
  const stats = el('p', { className: 'quiet', testid: 'export-summary' });
  const warnings = el('ul', { className: 'export-warnings', testid: 'export-warnings' });
  const download = el('button', { className: 'primary', text: 'Download', testid: 'export-download', attrs: { type: 'button' } });
  const importInput = el('input', {
    testid: 'export-import',
    attrs: { id: 'export-import', type: 'file', accept: '.plan,application/json' },
  });
  const importLabel = el('label', { className: 'import-button', text: 'Import .plan fence' });
  importLabel.htmlFor = importInput.id;
  const importStatus = el('p', { className: 'quiet', testid: 'export-import-status', attrs: { role: 'status' } });
  const actions = el('div', { className: 'row-actions' });
  actions.append(download, importInput, importLabel);

  details.append(
    summaryLine,
    el('p', {
      className: 'quiet',
      text: 'Takeoff, the planned sweep and the return, with the geofence and no-fly zones as fences. Confirmation looks, building entry and the indoor search are decided in flight, so they are not in the file.',
    }),
    field('Format', format),
    pair(field('Autopilot', firmware), field('Altitudes', altitude)),
    field('Home on the map', origin),
    custom,
    customAlt,
    field('Finish', finish),
    videoLabel,
    stats,
    warnings,
    actions,
    importStatus,
  );

  const preset = (id: GeoOriginId): void => {
    const o = GEO_ORIGINS[id];
    lat.value = String(o.latDeg);
    lon.value = String(o.lonDeg);
    amsl.value = String(o.altM);
  };
  preset('px4-sitl');

  let world: { seed: number; value: MissionWorld } | null = null;
  let cache: { key: string; route: MissionRoute | null; error: string | null } | null = null;
  let current: FlightPlan | null = null;

  function worldFor(seed: number): MissionWorld {
    if (!world || world.seed !== seed) world = { seed, value: buildMissionWorld({ seed }) };
    return world.value;
  }

  function route(mission: Mission): { route: MissionRoute | null; error: string | null } {
    const key = JSON.stringify(mission);
    if (cache?.key === key) return cache;
    let next: { route: MissionRoute | null; error: string | null };
    if (!validateMission(mission).ok) {
      next = { route: null, error: 'Fix the mission errors before exporting.' };
    } else {
      try {
        next = { route: routeFromCoverage(mission, worldFor(mission.seed >>> 0)), error: null };
      } catch (error) {
        next = { route: null, error: error instanceof Error ? error.message : String(error) };
      }
    }
    cache = { key, ...next };
    return cache;
  }

  function home(): GeoPoint | null {
    const point = { latDeg: Number(lat.value), lonDeg: Number(lon.value), altM: Number(amsl.value) };
    return isValidOrigin(point) ? point : null;
  }

  function refresh(): void {
    const isCustom = origin.value === 'custom';
    custom.hidden = !isCustom;
    customAlt.hidden = !isCustom;
    if (!details.open) return;
    current = null;
    warnings.replaceChildren();
    const result = route(options.mission());
    const pad = home();
    if (!result.route || !pad) {
      stats.textContent = result.error ?? 'Enter a latitude within 85 degrees, a longitude within 180 and a sensible altitude.';
      download.disabled = true;
      details.dataset.items = '0';
      options.onPreview(null);
      return;
    }
    current = buildFlightPlan(options.mission(), result.route, {
      home: pad,
      firmware: firmware.value as Firmware,
      altitudeRef: altitude.value as AltitudeRef,
      finish: finish.value as Finish,
      video: video.checked,
    });
    const s = current.summary;
    const source = current.source === 'coverage' ? 'the autonomy stack\'s terrain-following sweep' : 'the editor\'s sweep preview';
    stats.textContent = `${s.items} mission items, ${s.waypoints} waypoints from ${source}: ${Math.round(s.sweepM)} m of sweep and ${Math.round(s.transitM)} m of connectors, about ${minutes(s.estimatedSec)} of flight. Highest point ${s.maxAltM.toFixed(1)} m above home.`;
    for (const warning of current.warnings) warnings.append(el('li', { text: warning }));
    download.disabled = false;
    details.dataset.items = String(s.items);
    options.onPreview(preview());
  }

  function preview(): Point[] | null {
    if (!details.open) return null;
    const mission = options.mission();
    const r = cache?.key === JSON.stringify(mission) ? cache.route : null;
    if (!r) return null;
    const points: Point[] = [{ x: r.home.x, y: r.home.y }];
    for (const leg of r.legs) for (const p of leg.points) points.push({ x: p.x, y: p.y });
    points.push({ x: r.home.x, y: r.home.y });
    return points;
  }

  function save(): void {
    if (!current) return;
    const kind = format.value as Format;
    const text = kind === 'plan' ? qgcPlanJson(current) : kind === 'wpl' ? toWpl(current) : mavlinkJson(current);
    const name = `${exportStem(current)}.${FORMATS[kind].ext}`;
    const url = URL.createObjectURL(new Blob([text], { type: FORMATS[kind].mime }));
    const link = el('a', { attrs: { href: url, download: name } });
    document.body.append(link);
    link.click();
    link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 0);
    details.dataset.lastDownload = name;
  }

  async function load(file: File): Promise<void> {
    const text = await file.text();
    const result = importQgcPlan(text, options.mission());
    if (result.ok && result.mission) {
      const { counts } = result;
      const zones = counts.exclusion === 1 ? '1 no-fly zone' : `${counts.exclusion} no-fly zones`;
      importStatus.textContent = `Imported the geofence and ${zones} from ${file.name}. The plan's home sits on the start marker.`;
      importStatus.dataset.state = 'ok';
      options.onImport(result.mission);
    } else {
      const first = result.issues.find((issue) => issue.severity === 'error') ?? result.issues[0];
      importStatus.textContent = `Could not import ${file.name}: ${first?.message ?? 'unknown problem.'}`;
      importStatus.dataset.state = 'error';
    }
  }

  for (const control of [format, firmware, altitude, finish, video, lat, lon, amsl]) {
    control.addEventListener('change', () => refresh());
  }
  origin.addEventListener('change', () => {
    if (origin.value !== 'custom') preset(origin.value as GeoOriginId);
    refresh();
  });
  details.addEventListener('toggle', () => {
    refresh();
    if (!details.open) options.onPreview(null);
  });
  download.addEventListener('click', () => save());
  importInput.addEventListener('change', () => {
    const file = importInput.files?.[0];
    importInput.value = '';
    if (file) void load(file);
  });
  refresh();

  return { element: details, refresh, preview };
}

function pair(a: HTMLElement, b: HTMLElement): HTMLElement {
  const wrap = el('div', { className: 'pair' });
  wrap.append(a, b);
  return wrap;
}
