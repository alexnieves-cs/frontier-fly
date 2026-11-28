// Two ways in: two linked drones against one drone, and against two drones that share nothing,
// on the cabin search with a second way into the cabin. Writes docs/two-ways-in.json.
//
//   vite-node scripts/study/two-ways-in.ts -- --seeds 60
//   vite-node scripts/study/two-ways-in.ts -- --seeds 60 --shard 0 --of 4 --out /tmp/t0.jsonl
//   vite-node scripts/study/two-ways-in.ts -- --merge /tmp/t0.jsonl /tmp/t1.jsonl ... --out docs/two-ways-in.json
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { runDuo, TWO_WAYS_IN, type DuoResult } from '../../src/mission/autonomy/duo';
import { requirementsMet } from '../../src/mission/autonomy/planner/confirm';
import { createPerceptionSource } from '../../src/mission/autonomy/perception';
import { runMission } from '../../src/mission/autonomy/runner';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

type Arm = 'solo' | 'duo' | 'duo_no_link';
export const ARMS: Arm[] = ['solo', 'duo', 'duo_no_link'];

interface Flight {
  seed: number;
  arm: Arm;
  found: boolean;
  tFoundSec: number | null;
  tDoneSec: number;
  /** Openings flown in by, one per drone. */
  entries: (number | null)[];
  distinctEntries: boolean | null;
  minSeparationM: number | null;
  closeSec: number | null;
  collisions: number;
  landedHome: boolean;
}

async function fly(seed: number, arm: Arm): Promise<Flight> {
  const worldOptions = { ...TWO_WAYS_IN.world, seed };
  if (arm === 'solo') {
    const world = buildMissionWorld(worldOptions);
    const mission = missionForWorld(world, TWO_WAYS_IN.mission);
    const rec = await runMission(mission, world, createPerceptionSource(world), { ...TWO_WAYS_IN.runner });
    let tFound: number | null = null;
    for (let k = 1; k <= rec.confirmed.length && tFound === null; k++) {
      if (requirementsMet(mission, rec.confirmed.slice(0, k))) tFound = rec.confirmed[k - 1]!.tSec;
    }
    const go = rec.entries.filter((e) => e.decision === 'auto' || e.decision === 'approved');
    const inside = rec.phases.some((p) => p.phase === 'indoor_explore');
    return {
      seed,
      arm,
      found: tFound !== null,
      tFoundSec: tFound,
      tDoneSec: rec.tSec,
      entries: [inside && go.length ? go[go.length - 1]!.openingId : null],
      distinctEntries: null,
      minSeparationM: null,
      closeSec: null,
      collisions: rec.collisions.length,
      landedHome: rec.landedHome,
    };
  }
  const r: DuoResult = await runDuo(worldOptions, TWO_WAYS_IN.mission, { ...TWO_WAYS_IN.runner }, { noLink: arm === 'duo_no_link' });
  return {
    seed,
    arm,
    found: r.found,
    tFoundSec: r.tFoundSec,
    tDoneSec: r.tDoneSec,
    entries: r.entries,
    distinctEntries: r.distinctEntries,
    minSeparationM: Number.isFinite(r.minSeparationM) ? r.minSeparationM : null,
    closeSec: r.closeSec,
    collisions: r.collisions,
    landedHome: r.landedHome,
  };
}

function median(xs: number[]): number | null {
  if (!xs.length) return null;
  const s = [...xs].sort((a, b) => a - b);
  const m = s.length >> 1;
  return s.length % 2 ? s[m]! : (s[m - 1]! + s[m]!) / 2;
}

function round(v: number | null, d = 1): number | null {
  return v === null ? null : Math.round(v * 10 ** d) / 10 ** d;
}

/** Per arm: success, time to find and to finish, entries, separation; and the paired speedup of duo over solo. */
export function report(flights: Flight[]) {
  const arms = Object.fromEntries(
    ARMS.map((arm) => {
      const f = flights.filter((x) => x.arm === arm);
      const found = f.filter((x) => x.found);
      const duoish = arm !== 'solo';
      return [
        arm,
        {
          flights: f.length,
          foundRate: round(found.length / Math.max(1, f.length), 3),
          medianFoundSec: round(median(found.map((x) => x.tFoundSec!))),
          medianDoneSec: round(median(f.map((x) => x.tDoneSec))),
          landedHomeRate: round(f.filter((x) => x.landedHome).length / Math.max(1, f.length), 3),
          collisions: f.reduce((n, x) => n + x.collisions, 0),
          ...(duoish
            ? {
                // Of the flights where both drones went in, how many used two different openings.
                bothEntered: f.filter((x) => x.entries.every((e) => e !== null)).length,
                distinctEntryRate: round(
                  f.filter((x) => x.distinctEntries).length / Math.max(1, f.filter((x) => x.entries.every((e) => e !== null)).length),
                  3,
                ),
                // One drone found the person before the other had gone in, so only one entry was needed.
                oneEntered: f.filter((x) => x.entries.filter((e) => e !== null).length === 1).length,
                minSeparationM: round(Math.min(...f.map((x) => x.minSeparationM ?? Infinity)), 2),
                medianMinSeparationM: round(median(f.map((x) => x.minSeparationM ?? Infinity).filter(Number.isFinite)), 2),
                flightsWithCloseTime: f.filter((x) => (x.closeSec ?? 0) > 0).length,
              }
            : {}),
        },
      ];
    }),
  );
  // Paired on seeds both arms found: duo time to find over solo's.
  const paired = (other: Arm) => {
    const ratios: number[] = [];
    for (const d of flights.filter((x) => x.arm === other && x.found)) {
      const s = flights.find((x) => x.arm === 'solo' && x.seed === d.seed && x.found);
      if (s) ratios.push(d.tFoundSec! / s.tFoundSec!);
    }
    return { seeds: ratios.length, medianRatio: round(median(ratios), 3), fasterShare: round(ratios.filter((r) => r < 1).length / Math.max(1, ratios.length), 3) };
  };
  return { arms, findTimeVsSolo: { duo: paired('duo'), duo_no_link: paired('duo_no_link') } };
}

if (process.argv.includes('--merge')) {
  const rest = process.argv.slice(process.argv.indexOf('--merge') + 1);
  const files = rest.filter((a, i) => !a.startsWith('--') && !(i > 0 && rest[i - 1]!.startsWith('--')));
  const flights: Flight[] = files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as Flight),
  );
  flights.sort((a, b) => a.seed - b.seed || ARMS.indexOf(a.arm) - ARMS.indexOf(b.arm));
  const commit = execSync('git rev-parse --short HEAD').toString().trim();
  const out = {
    what: 'Two ways in: cabin search with two ways into the cabin, flown by one drone, two linked drones, and two drones that share nothing. Simulation only.',
    commit,
    seeds: [...new Set(flights.map((f) => f.seed))].length,
    ...report(flights),
    flights,
  };
  writeFileSync(arg('out', 'docs/two-ways-in.json'), JSON.stringify(out, null, 1) + '\n');
  console.log(JSON.stringify({ ...out, flights: undefined }, null, 1));
} else {
  const seeds = Number(arg('seeds', '60'));
  const base = Number(arg('base', '100'));
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', '/tmp/two-ways-in.jsonl');
  const lines: string[] = [];
  for (let k = shard; k < seeds; k += of) {
    for (const arm of ARMS) {
      const f = await fly(base + k, arm);
      lines.push(JSON.stringify(f));
      console.log(`${f.seed} ${f.arm} found=${f.found} t=${round(f.tFoundSec)} entries=${f.entries.join('/')} sep=${round(f.minSeparationM, 2)}`);
    }
  }
  writeFileSync(out, lines.join('\n') + '\n');
}
