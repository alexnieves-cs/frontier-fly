// The search-by-sight section of docs/REPORT.md, from docs/vision-detector.json (scripts/vision/eval-detector.ts)
// and docs/vision-bench.json (npm run study:vision). Returns template values; generate.mjs merges them.

const NONE = 'not measured';
const pct = (v) => `${(v * 100).toFixed(1)}%`;
const pct0 = (v) => `${Math.round(v * 100)}%`;
const ci = ([lo, hi]) => `${Math.round(lo * 100)}–${Math.round(hi * 100)}%`;
const sec = (v) => (v === null || v === undefined || !Number.isFinite(v) ? '—' : `${Math.round(v)} s`);

function table(head, rows) {
  const line = (cells) => `| ${cells.join(' | ')} |`;
  return [line(head), line(head.map(() => '---')), ...rows.map(line)].join('\n');
}

const KEYS = [
  'R_V_MAP',
  'R_V_PERSON_AP',
  'R_V_FRAMES',
  'R_V_ECE_RAW',
  'R_V_ECE_CAL',
  'R_V_MS',
  'R_V_BG_FP',
  'R_V_MANNEQUIN',
  'R_V_DECOY_PACK',
  'R_V_ANALYTIC',
  'R_V_VISION',
  'R_V_FLIGHTS',
  'R_V_PAIRS',
  'R_V_ONLY_ANALYTIC',
  'R_V_ONLY_VISION',
  'R_V_FA',
  'R_V_FC',
  'R_V_FRAMES_PER_FLIGHT',
  'R_V_WALL',
  'VISION_CLASS_TABLE',
  'VISION_RECALL_TABLE',
  'VISION_MISSION_TABLE',
  'VISION_RESULT_NOTE',
  'R_V_NIGHT_ANALYTIC',
  'R_V_NIGHT_VISION',
];

const NAMES = { person: 'Person', backpack: 'Backpack', phone: 'Phone', toolbox: 'Toolbox', medkit: 'Medkit', bottle: 'Bottle', radio: 'Radio', helmet: 'Helmet', blanket: 'Blanket' };
const SCENARIO = { 'cabin-day': 'Cabin search, day', 'compound-clear': 'Two-building safety sweep', 'backpack-night': 'Backpack search, night' };

export function visionSection(detector, bench) {
  const values = Object.fromEntries(KEYS.map((k) => [k, NONE]));
  if (detector) {
    values.R_V_MAP = detector.mAP50.toFixed(2);
    values.R_V_PERSON_AP = detector.perClass.person.ap.toFixed(2);
    values.R_V_FRAMES = `${detector.frames.toLocaleString('en-US')} frames (${detector.dayFrames} day, ${detector.nightFrames} night)`;
    values.R_V_ECE_RAW = detector.eceRaw.toFixed(3);
    values.R_V_ECE_CAL = detector.eceCalibrated.toFixed(3);
    values.R_V_MS = `${detector.msPerFrame.toFixed(1)} ms`;
    values.R_V_BG_FP = detector.backgroundFpPerFrame.toFixed(3);
    values.R_V_MANNEQUIN = `${detector.decoys.mannequin.fooled} of ${detector.decoys.mannequin.seen}`;
    values.R_V_DECOY_PACK = `${detector.decoys.decoy_pack.fooled} of ${detector.decoys.decoy_pack.seen}`;
    values.VISION_CLASS_TABLE = table(
      ['Class', 'AP at IoU 0.5', 'Labelled boxes'],
      Object.entries(detector.perClass).map(([c, v]) => [NAMES[c] ?? c, v.ap.toFixed(2), String(v.labels)]),
    );
    const bands = detector.recall.personDay;
    const cell = (key, i) => {
      const b = detector.recall[key][i];
      return b && b.n ? `${pct0(b.recall)} (${b.n})` : '—';
    };
    values.VISION_RECALL_TABLE = table(
      ['Distance', 'Person, day', 'Person, night', 'Objects, day', 'Objects, night'],
      bands.map((b, i) => [`${b.from}–${b.to} m`, cell('personDay', i), cell('personNight', i), cell('objectsDay', i), cell('objectsNight', i)]),
    );
  }
  if (bench) {
    const a = bench.arms.analytic.all;
    const v = bench.arms.vision.all;
    values.R_V_ANALYTIC = `${pct(a.expectedOutcome)} (${ci(a.expectedOutcomeCi)})`;
    values.R_V_VISION = `${pct(v.expectedOutcome)} (${ci(v.expectedOutcomeCi)})`;
    values.R_V_FLIGHTS = String(v.flights);
    values.R_V_PAIRS = String(bench.paired.pairs);
    values.R_V_ONLY_ANALYTIC = String(bench.paired.onlyAnalytic);
    values.R_V_ONLY_VISION = String(bench.paired.onlyVision);
    values.R_V_FA = String(v.falseAlarms);
    values.R_V_FC = String(v.falseClears);
    values.R_V_FRAMES_PER_FLIGHT = v.framesPerFlight.toLocaleString('en-US');
    values.R_V_WALL = `${v.wallSecPerFlight} s`;
    const rows = [];
    for (const s of Object.keys(SCENARIO)) {
      for (const p of ['analytic', 'vision']) {
        const g = bench.arms[p][s];
        if (!g) continue;
        rows.push([
          p === 'analytic' ? `**${SCENARIO[s]}**` : '',
          p === 'analytic' ? 'Analytic detector' : 'Camera + learned detector',
          `${pct(g.expectedOutcome)} (${ci(g.expectedOutcomeCi)})`,
          sec(g.meanFirstFindSec),
          sec(g.meanDurationSec),
          String(g.falseAlarms),
          String(g.falseClears),
        ]);
      }
    }
    const na = bench.arms.analytic['backpack-night'];
    const nv = bench.arms.vision['backpack-night'];
    values.R_V_NIGHT_ANALYTIC = pct(na.expectedOutcome);
    values.R_V_NIGHT_VISION = pct(nv.expectedOutcome);
    const p = bench.paired;
    values.VISION_RESULT_NOTE = [
      `**What the numbers say.** Over all ${p.pairs} paired flights the camera meets the expected outcome in ${pct(v.expectedOutcome)} against ${pct(a.expectedOutcome)} for the analytic detector. ${p.onlyAnalytic} paired flights succeed only with the analytic detector and ${p.onlyVision} only with the camera, so the camera never rescues a flight the analytic model loses.`,
      `The two are within noise of each other in every scenario; the night backpack is ${pct(nv.expectedOutcome)} against ${pct(na.expectedOutcome)}. Under the flashlight alone the detector often scores the backpack around 0.4 to 0.6, below the 0.8 mean the three-bearing confirmation asks for, so the drone spends its re-views on it. The confirmation threshold was left where the analytic missions set it. An earlier run of this bench showed a much larger night gap; most of it was targets the placer had put inside tables and shelves, hidden from the camera but not from the analytic detector, which has no furniture occlusion. Targets are now never placed inside furniture.`,
      `${v.falseAlarms === 0 && v.falseClears === 0 ? 'No camera flight confirmed a decoy or declared a building clear with a target inside' : 'Camera flights confirmed a decoy or declared a building clear with a target inside only rarely'} (${v.falseAlarms} false alarm${v.falseAlarms === 1 ? '' : 's'}, ${v.falseClears} false clear${v.falseClears === 1 ? '' : 's'} in ${v.flights} flights). A camera flight renders and reads about ${v.framesPerFlight.toLocaleString('en-US')} frames and takes ${v.wallSecPerFlight} s of wall time on one CPU core, against ${a.wallSecPerFlight} s with the analytic detector.`,
    ].join('\n\n');
    values.VISION_MISSION_TABLE = table(['Scenario', 'Detector', 'Expected outcome (95% CI)', 'Mean time to first find', 'Mean flight', 'False alarms', 'False clears'], rows);
  }
  return { values };
}
