import { DEFAULT_BATTERY } from '../mission/autonomy/battery';
import { cellVoltage, MISSION_PACK, motorSpeedScale, packCurrent, type PackConfig } from '../mission/autonomy/pack';
import { missionQuadParams } from '../mission/autonomy/quadVehicle';
import { AeroModel } from '../sim/dynamics/aero';
import { GustTable, type GustSpec } from '../sim/dynamics/gust';
import type { DynParams } from '../shared/types';
import { FrontierPlant } from './frontierPlant';
import { eulerOf } from './quad';

/**
 * The air, the pack and the surfaces around the physics validation's vehicle,
 * shared by both of its frontier-fly simulators: PX4 flying the plant through
 * the bridge (PhysicsRig below) and frontier-fly's own controller
 * (scripts/physics/native.ts). Same gust table, same pack, same aerodynamics.
 */
export interface RigOptions {
  wind?: GustSpec;
  /** Fixed state of charge, 0..1. Unset: the motors keep their full top speed (the shipped model). */
  soc?: number;
  /** Near-surface aerodynamics (aero.ts). Unset: the shipped linear ground effect. */
  aero?: { ceilingM?: number };
}

const GUST_SPAN_SEC = 600;

export class RigEnvironment {
  readonly params: DynParams;
  readonly rotorRadiusM: number;
  readonly gust: GustTable | null;
  readonly aero: AeroModel | null;
  readonly pack: PackConfig = MISSION_PACK;
  /** Pack current and loaded voltage of the last power update. */
  currentA = 0;
  packV = 0;

  constructor(readonly options: RigOptions) {
    this.params = missionQuadParams();
    this.rotorRadiusM = DEFAULT_BATTERY.rotorDiameterM / 2;
    this.gust = options.wind ? new GustTable(options.wind, GUST_SPAN_SEC) : null;
    const ceiling = options.aero?.ceilingM;
    this.aero =
      options.aero || options.soc !== undefined
        ? new AeroModel({
            rotorRadiusM: this.rotorRadiusM,
            ground: options.aero !== undefined,
            ceiling: ceiling !== undefined,
            clearance: (_x, _y, z) => ({ below: z, above: ceiling === undefined ? Infinity : ceiling - z }),
          })
        : null;
  }

  /**
   * Wind, world ENU m/s. Before the maneuver starts (t null) only the steady
   * mean blows, so the hover settles into it; from t = 0 the gusts play.
   */
  windEnu(t: number | null): [number, number, number] {
    const g = this.gust;
    if (!g) return [0, 0, 0];
    if (t === null) {
      const d = (g.spec.dirDeg * Math.PI) / 180;
      return [g.spec.meanMps * Math.cos(d), g.spec.meanMps * Math.sin(d), 0];
    }
    const [e, n] = g.at(t);
    return [e, n, 0];
  }

  /**
   * Electrical power for the rotor speeds, W: momentum theory per rotor with
   * battery.ts's figure of merit, motor efficiency and avionics, so a hover
   * draws what the mission's energy model charges for one.
   */
  powerW(rotorRadS: ArrayLike<number>): number {
    const c = DEFAULT_BATTERY;
    const area = Math.PI * this.rotorRadiusM * this.rotorRadiusM;
    let shaft = 0;
    for (let i = 0; i < 4; i++) {
      const t = this.params.kf * (rotorRadS[i] ?? 0) ** 2;
      shaft += t ** 1.5 / Math.sqrt(2 * c.airDensity * area) / c.figureOfMerit;
    }
    return shaft / c.electricalEfficiency + c.avionicsW;
  }

  /** Update the motors' top speed from the pack under the current load. */
  updatePack(rotorRadS: ArrayLike<number>): void {
    const soc = this.options.soc;
    if (soc === undefined || !this.aero) return;
    const i = packCurrent(this.pack, soc, this.powerW(rotorRadS));
    this.currentA = Number.isFinite(i) ? i : 0;
    this.packV = this.pack.cells * cellVoltage(this.pack, soc, this.currentA);
    this.aero.motorMaxScale = motorSpeedScale(this.pack, soc, this.currentA);
  }
}

/** A row of the 50 Hz state log. */
export type RigRow = number[];
export const RIG_COLUMNS = ['tUs', 'n', 'e', 'd', 'vn', 've', 'vd', 'rollDeg', 'pitchDeg', 'yawDeg', 'u0', 'u1', 'u2', 'u3', 'windN', 'windE', 'packV', 'maxScale'];

/** FrontierPlant plus its environment, for the PX4 bridge (scripts/px4/sim-node.ts --plant frontier). */
export class PhysicsRig {
  readonly env: RigEnvironment;
  readonly plant: FrontierPlant;
  private readonly rows: RigRow[] = [];
  private nextUs = 0;

  constructor(options: RigOptions) {
    this.env = new RigEnvironment(options);
    this.plant = new FrontierPlant(this.env.params, this.env.aero);
  }

  /** Before each physics step: the wind at maneuver time t (null before the maneuver), and the pack. */
  beforeStep(t: number | null): void {
    const w = this.env.windEnu(t);
    this.plant.wind = [w[1], w[0], -w[2]];
    this.env.updatePack(this.plant.state.motors);
  }

  /** Log the state at 50 Hz of PX4's clock. */
  record(px4Us: number, u: ArrayLike<number>): void {
    if (px4Us <= 0 || px4Us < this.nextUs) return;
    this.nextUs = px4Us + 20_000;
    const s = this.plant.s;
    const e = eulerOf(s.q);
    const r2 = (v: number): number => Math.round(v * 1000) / 1000;
    const deg = 180 / Math.PI;
    this.rows.push([
      px4Us,
      ...s.pos.map(r2),
      ...s.vel.map(r2),
      r2(e.roll * deg),
      r2(e.pitch * deg),
      r2(e.yaw * deg),
      ...Array.from({ length: 4 }, (_, i) => r2(u[i] ?? 0)),
      r2(this.plant.wind[0]),
      r2(this.plant.wind[1]),
      r2(this.env.packV),
      r2(this.env.aero?.motorMaxScale ?? 1),
    ]);
  }

  log(): { columns: string[]; rows: RigRow[] } {
    return { columns: RIG_COLUMNS, rows: this.rows };
  }
}
