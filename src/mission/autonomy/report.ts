/* eslint-disable @typescript-eslint/no-unused-vars */
import type { AutonomyReport, LiveDetectionSource, RunRecord } from './contracts';

// STARTER: shape only. The safety/report workstream replaces it.
export function buildReport(run: RunRecord, _source: LiveDetectionSource): AutonomyReport {
  return {
    missionName: run.mission.name,
    missionType: run.mission.missionType,
    outcome: run.outcome,
    durationSec: run.tSec,
    timeToFindSec: null,
    outdoorCoverage: 0,
    indoorCoverage: 0,
    perBuildingCoverage: {},
    falseClears: 0,
    falseAlarms: 0,
    missed: 0,
    collisions: run.collisions.length,
    batteryUsedPct: run.batteryStartPct - run.batteryEndPct,
    landedHome: run.landedHome,
    entries: run.entries,
    safetyEvents: run.safetyEvents.length,
    notes: [],
  };
}
