import type { BrakingModel } from '../sim/lidarSafety';

/**
 * The real airframe this module plans for: a Bitcraze Crazyflie 2.1+ with a
 * Flow deck v2 (downward ToF + optical flow, for height and velocity) and a
 * Multi-ranger deck (five VL53L1x ToF sensors: front, back, left, right, up).
 *
 * Numbers come from the Bitcraze product pages and datasheets
 * (https://www.bitcraze.io/products/), rounded toward the conservative side.
 * Recheck them against the exact hardware before a flight; docs/CRAZYFLIE.md
 * says where each one is used.
 */
export const CRAZYFLIE = {
  /** Motor to motor, meters (2.1+, brushed). */
  motorSpan: 0.092,
  /** Half the span plus a 45 mm prop: the radius of the spinning disc, meters. */
  propRadius: 0.07,
  /** Take-off mass with both decks and the stock battery, kg (27 g frame + ~3.3 g decks + battery margin). */
  massKg: 0.033,
  /** Usable flight time with Flow + Multi-ranger, seconds (Bitcraze quotes ~7 min bare; decks cost ~1-2 min). */
  flightTimeSec: 300,
  /** Battery voltage to land at under load, volts (1S LiPo). */
  landVoltage: 3.4,
  /** Lowest resting voltage to allow a take-off, volts. */
  takeoffVoltage: 3.9,
} as const;

/** The Multi-ranger deck as the planner sees it. */
export const MULTIRANGER = {
  /** VL53L1x long-distance mode, meters. Anything at or past this is "no return". */
  maxRange: 4,
  /** Shortest reliable reading, meters. */
  minRange: 0.03,
  /** 1-sigma range noise, meters (datasheet: a few cm at 2-3 m indoors). */
  sigma: 0.02,
  /** Readings per second per sensor the stock firmware logs (range.* at 100 Hz internally; we log at 20 Hz). */
  rateHz: 20,
} as const;

/** The Flow deck v2: height from a downward VL53L1x, velocity from a PMW3901 flow sensor. */
export const FLOW_DECK = {
  /** Height range the flow sensor tracks texture in, meters (datasheet: 0.08 to 3 m). */
  minHeight: 0.1,
  maxHeight: 3,
  /** Position drift of the flow + IMU estimate, fraction of distance flown (Bitcraze forum reports 2-5% on good texture). */
  driftFraction: 0.04,
} as const;

/** How fast the explorer may fly a Crazyflie indoors, m/s. Bitcraze's Multi-ranger demos use 0.2 to 0.5. */
export const CF_SPEED = 0.35;
/** Cruise height, meters: low enough that a dropped drone lands softly, high enough for the flow sensor. */
export const CF_HEIGHT = 0.5;
/** Fastest yaw the explorer commands, rad/s (90 deg/s). */
export const CF_YAW_RATE = Math.PI / 2;
/**
 * Stopping model for the braking filter. Reaction covers the 20 Hz log, the
 * radio round trip and the planner step (~0.2 s measured on cflib links,
 * plus margin); deceleration is what the stock PID velocity loop reaches
 * with its default limits, kept low on purpose.
 */
export const CF_BRAKING: BrakingModel = { reaction: 0.3, decel: 1.5 };
