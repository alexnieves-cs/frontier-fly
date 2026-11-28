/**
 * The whole autonomous mission through PX4 SITL.
 *
 * The mission runner (src/mission/autonomy/runner.ts) is synchronous: every 50 ms tick it asks its
 * vehicle to fly one velocity command and reads back where the airframe went. Px4Vehicle
 * (src/mission/autonomy/px4Vehicle.ts) answers that by handing the command to PX4 as an offboard
 * velocity setpoint and letting PX4's estimator and controllers fly frontier-fly's own rigid body
 * (FrontierPlant) for that long, in lockstep. PX4 talks over sockets, which need an event loop, so the
 * PX4 side lives in a worker thread (missionLink.worker.ts) and the two meet in shared memory: the
 * runner writes a command and blocks on Atomics.wait until the worker has flown it.
 *
 * Frames on this page are PX4's: NED from the pad, body FRD.
 */

/** Int32 control words. */
/** The lockstep HIL step, µs (250 Hz). A link advance flies a whole number of these. */
export const LINK_STEP_US = 4000;

export const CTL = {
  /** Request counter, bumped by the runner for each command. */
  REQ: 0,
  /** Last request the worker finished. */
  ACK: 1,
  /** Link phase, one of PHASE. */
  PHASE: 2,
  /** What the request asks for, one of OP. */
  OP: 3,
  LENGTH: 8,
} as const;

export const PHASE = { BOOT: 0, LISTENING: 1, READY: 2, FAILED: 3, CLOSED: 4 } as const;
export const OP = { ADVANCE: 1, STOP: 2 } as const;

/** Float64 slots, command half (runner to worker). */
export const CMD = {
  DT: 0,
  VEL: 1, // 3: north, east, down, m/s
  YAW_RATE: 4, // rad/s about down
  WIND: 5, // 3: NED, m/s
  TELEPORT: 8, // 1 to move the airframe first
  TP_POS: 9, // 3
  TP_VEL: 12, // 3
  THRUST: 15, // 4: per-motor thrust authority 0..1
  GPS_FIX: 19, // 1 with a fix, 0 denied
  EV_ON: 20, // 1 to send external vision this request
  EV_POS: 21, // 3
  EV_VEL: 24, // 3
  EV_POS_VAR: 27,
  EV_VEL_VAR: 28,
  EV_RESET: 29, // reset counter: bump when the estimate jumps
} as const;

/** Float64 slots, state half (worker to runner). */
export const ST = {
  BASE: 64,
  SIM_TIME: 64,
  POS: 65, // 3 NED
  VEL: 68, // 3 NED
  Q: 71, // 4 wxyz, body FRD to NED
  OMEGA: 75, // 3 body FRD
  SPECIFIC_FORCE: 78, // 3 body FRD
  ARMED: 81,
  CUSTOM_MODE: 82,
  ON_GROUND: 83,
  MOTORS: 84, // 4
  EKF_POS: 88, // 3: PX4's own estimate (LOCAL_POSITION_NED)
  EKF_VEL: 91, // 3
  LANDED_STATE: 94,
  TIMEOUTS: 95,
  SETPOINTS: 96,
  EV_SENT: 97,
} as const;

export const DATA_LENGTH = 128;

/** PX4's custom mode word: main mode in bits 16-23, sub mode in 24-31. */
export function px4Mode(customMode: number): string {
  const main = (customMode >>> 16) & 0xff;
  const sub = (customMode >>> 24) & 0xff;
  const mains: Record<number, string> = { 1: 'manual', 2: 'altctl', 3: 'posctl', 4: 'auto', 5: 'acro', 6: 'offboard', 7: 'stabilized', 8: 'rattitude' };
  const autos: Record<number, string> = { 1: 'ready', 2: 'takeoff', 3: 'loiter', 4: 'mission', 5: 'rtl', 6: 'land', 8: 'follow', 9: 'precland' };
  const m = mains[main] ?? `mode ${main}`;
  return main === 4 ? `auto.${autos[sub] ?? sub}` : m;
}

/** A PX4 parameter to set before arming. */
export interface Px4Param {
  id: string;
  value: number;
  type: 'int32' | 'real32';
}

/** What the worker is started with. */
export interface MissionLinkInit {
  ctl: SharedArrayBuffer;
  data: SharedArrayBuffer;
  /** TCP port PX4's simulator_mavlink connects to. */
  hilPort: number;
  /** UDP port PX4's onboard MAVLink instance sends to (PX4 SITL: 14540) and the one it listens on (14580). */
  offboardLocalPort: number;
  offboardRemotePort: number;
  home: { latDeg: number; lonDeg: number; altM: number };
  params: Px4Param[];
  /** Thrust and drag of the airframe the plant flies (DynParams), passed through as JSON. */
  dyn: unknown;
  /** Ground under the pad-relative NED position, as down: sampled on a grid by the runner side. */
  ground: { originEast: number; originNorth: number; step: number; nx: number; ny: number; down: Float32Array } | null;
  seed: number;
  /** PX4's MAVLink system id (instance N boots as N+1). Default 1. */
  sysId?: number;
}

/** One line from PX4 or the link, for the report. */
export interface LinkEvent {
  simSec: number;
  kind: 'statustext' | 'link' | 'mode' | 'arm';
  text: string;
  severity?: number;
}
