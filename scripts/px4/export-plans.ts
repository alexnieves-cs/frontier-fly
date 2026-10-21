/**
 * Writes the QGroundControl .plan files that the PX4 flight check flies.
 *
 *   npx vite-node scripts/px4/export-plans.ts -- [--out test-results/px4/plans] [--set ci|full]
 *
 * Each plan comes from the same code path as the mission editor's Download
 * button (planner route, buildFlightPlan, toQgcPlan) with the PX4 SITL home,
 * so PX4 flies exactly what a user would export. A sidecar `<stem>.meta.json`
 * carries the plan summary (estimated time, waypoint count) for the report.
 *
 * Seeds: the woods are the export tests' seeds (11, 4242, 5150). Nothing is
 * tuned on them; the check is pass or fail.
 */
import { mkdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { buildMissionWorld } from '../../src/mission/autonomy/world';
import { woodsSampleMission } from '../../src/mission/outdoorWorld';
import { buildFlightPlan, exportStem, type ExportOptions } from '../../src/mission/export/flightPlan';
import { qgcPlanJson } from '../../src/mission/export/formats';
import { GEO_ORIGINS } from '../../src/mission/export/geo';
import { routeFromCoverage, routeFromPreview, type MissionRoute } from '../../src/mission/export/route';
import { sampleMission } from '../../src/mission/sample';
import type { Mission } from '../../src/mission/types';

const args = process.argv.slice(2);
const opt = (name: string): string | undefined => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const outDir = opt('out') ?? 'test-results/px4/plans';
const set = opt('set') ?? 'ci';

const PX4: ExportOptions = {
  home: GEO_ORIGINS['px4-sitl'],
  firmware: 'px4',
  altitudeRef: 'relative',
  finish: 'rtl',
  video: false,
};

interface Case {
  id: string;
  label: string;
  /** The sim world the plan came from, so the bridge can fly it over the same terrain. */
  site: { kind: 'woods'; seed: number } | { kind: 'practice' };
  mission: Mission;
  route: MissionRoute;
  options: ExportOptions;
}

function woods(seed: number): { mission: Mission; route: MissionRoute } {
  // The mission editor's default woods mission, so these are the plans its Download button writes.
  const world = buildMissionWorld({ seed });
  const mission = woodsSampleMission(seed);
  return { mission, route: routeFromCoverage(mission, world) };
}

const cases: Case[] = [];
{
  const mission = sampleMission(11);
  cases.push({
    id: 'practice-land',
    label: 'Practice site, land at the pad',
    site: { kind: 'practice' },
    mission,
    route: routeFromPreview(mission),
    options: { ...PX4, finish: 'land' },
  });
}
for (const seed of set === 'full' ? [11, 4242, 5150] : [4242]) {
  cases.push({ id: `woods-${seed}-rtl`, label: `Woods seed ${seed}, return to launch`, site: { kind: 'woods', seed }, ...woods(seed), options: PX4 });
}
if (set === 'full') {
  cases.push({
    id: 'woods-11-land-video',
    label: 'Woods seed 11, video on, land at the pad',
    site: { kind: 'woods', seed: 11 },
    ...woods(11),
    options: { ...PX4, finish: 'land', video: true },
  });
}

mkdirSync(outDir, { recursive: true });
for (const c of cases) {
  const plan = buildFlightPlan(c.mission, c.route, c.options);
  const file = `${c.id}.plan`;
  writeFileSync(join(outDir, file), qgcPlanJson(plan));
  const meta = {
    id: c.id,
    label: c.label,
    file,
    stem: exportStem(plan),
    seed: plan.seed,
    site: c.site,
    /** The pad in the sim frame (x east, y north, meters). */
    pad: { x: c.route.home.x, y: c.route.home.y, groundZ: c.route.home.groundZ },
    home: plan.home,
    finish: c.options.finish,
    video: c.options.video,
    cruiseSpeedMps: plan.cruiseSpeedMps,
    summary: plan.summary,
    warnings: plan.warnings,
    noFlyZones: plan.fence.filter((f) => !f.inclusion).length,
  };
  writeFileSync(join(outDir, `${c.id}.meta.json`), `${JSON.stringify(meta, null, 2)}\n`);
  console.log(
    `${file}: ${plan.summary.waypoints} waypoints, ${plan.summary.items} items, ~${plan.summary.estimatedSec} s, ${meta.noFlyZones} no-fly zones`,
  );
}
