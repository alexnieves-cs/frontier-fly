/**
 * A mission flown through PX4 SITL (scripts/px4/mission.ts --film), saved so the film page can
 * replay it: `film.html?px4=<url of the .film.json>`. The file carries the atlas case, not the
 * world: the world is rebuilt here from the same seeds, so it is the world PX4 flew in.
 */
import { caseSetup, type AtlasCase } from '../eval/atlas/space';
import type { AutonomyReport, ConfirmedContact, LogEntry, RunRecord, V3 } from '../mission/autonomy/contracts';
import { createPerceptionSource } from '../mission/autonomy/perception';
import type { LoadedScenario, Scenario } from '../mission/autonomy/scenarios';
import { buildMissionWorld, missionForWorld } from '../mission/autonomy/world';
import type { FlightFrame, FlightLog } from './flight';

export const PX4_FILM_SCHEMA = 'frontier-fly/px4-film@1';

/** Facts about the PX4 flight that hold for the whole run, for the HUD and the report card. */
export interface Px4FlightFacts {
  /** e.g. "PX4 v1.15.4 SITL". */
  autopilot: string;
  /** Lockstep sensor rate, Hz. */
  lockstepHz: number;
  /** Offboard setpoint rate, Hz. */
  offboardHz: number;
  /** Indoor navigation source: 'vision' feeds EKF2 external odometry; 'gps' keeps a fix. */
  indoorNav: 'vision' | 'gps';
  gnssDeniedSec: number;
  visionSec: number;
  ekfHorizErrRmsM: number | null;
  ekfHorizErrMaxM: number;
  lockstepTimeouts: number;
  /** PX4 parameters the run set beyond the airframe's own (id → value). */
  params: Record<string, number>;
  wallSec: number;
}

export interface Px4FilmFile {
  schema: typeof PX4_FILM_SCHEMA;
  id: string;
  title: string;
  blurb: string;
  case: AtlasCase;
  /** Mission mode the runner flew (missionModeOptions). */
  mode: string;
  dt: number;
  frames: FlightFrame[];
  plans: V3[][];
  confirmed: ConfirmedContact[];
  log: LogEntry[];
  record: RunRecord;
  report: AutonomyReport;
  explorer: FlightLog['explorer'];
  px4: Px4FlightFacts;
}

/** The PX4 flight's facts, when a FlightLog came from a PX4 film file. */
const facts = new WeakMap<FlightLog, Px4FlightFacts>();

export function px4Facts(log: FlightLog): Px4FlightFacts | undefined {
  return facts.get(log);
}

export function flightLogFromPx4(file: Px4FilmFile): FlightLog {
  if (file.schema !== PX4_FILM_SCHEMA) throw new Error(`Not a PX4 film file (schema ${String(file.schema)})`);
  const setup = caseSetup(file.case);
  const world = buildMissionWorld(setup.world);
  const mission = missionForWorld(world, setup.mission);
  const scenario = {
    id: file.id,
    title: file.title,
    blurb: file.blurb,
    tags: ['PX4 SITL'],
    world: setup.world,
    mission: setup.mission,
    runner: {},
    expect: { outcome: file.report.outcome, landedHome: true },
  } as unknown as Scenario;
  const loaded: LoadedScenario = { scenario, world, mission, source: createPerceptionSource(world), options: { dt: file.dt } };
  const log: FlightLog = {
    loaded,
    frames: file.frames,
    plans: file.plans,
    confirmed: file.confirmed,
    log: file.log,
    record: file.record,
    report: file.report,
    dt: file.dt,
    explorer: file.explorer,
  };
  facts.set(log, file.px4);
  return log;
}

export async function fetchPx4Flight(url: string): Promise<FlightLog> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`PX4 flight ${url}: HTTP ${res.status}`);
  return flightLogFromPx4((await res.json()) as Px4FilmFile);
}
