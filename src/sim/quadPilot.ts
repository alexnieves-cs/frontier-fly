/**
 * The default quadrotor explorer (docs/QUAD_PICKER.md, "The default quad pilot"): the sensor
 * planner with braking filter and trajectory tracker, its next frontier goal chosen by the frozen
 * learned picker, and a more cautious braking model than QUAD_BRAKING.
 *
 * On report-only seeds 89401-89732 (664 flights, flown once) it covered 54.2% with 1 crash,
 * against 45.9% and 30 crashes for the earlier planner-only quad pilot (classicalBrakeTrack).
 */
import frozen from './quadPickerDefault.json';
import type { BrakingModel } from './lidarSafety';
import { MlpPicker, type MlpWeights } from './picker';
import type { SensorPlannerOptions } from './sensorPlanner';

/**
 * Braking model the default quad pilot plans with: 0.25 s reaction (QUAD_BRAKING: 0.21) and
 * 4 m/s² deceleration (QUAD_BRAKING: 5). Chosen on practice seeds 66001-66160 and 66701-66764;
 * it cut the picker's crashes from 18 to 5 of 448 for half a point of coverage.
 */
export const QUAD_PICKER_BRAKING: BrakingModel = { reaction: frozen.braking.reaction, decel: frozen.braking.decel };

/** Frozen picker weights (results/picker/default.json). */
export const QUAD_PICKER_WEIGHTS = frozen.weights as unknown as MlpWeights;

/** Planner options of the default quad pilot. Each call makes its own picker (it keeps per-episode scratch). */
export function quadPilotOptions(extra: SensorPlannerOptions = {}): SensorPlannerOptions {
  return {
    braking: QUAD_PICKER_BRAKING,
    tracker: {},
    edgeBonus: frozen.planner.edgeBonus,
    momentum: frozen.planner.momentum,
    replanEvery: frozen.planner.replanEvery,
    picker: new MlpPicker(QUAD_PICKER_WEIGHTS, 0),
    ...extra,
  };
}
