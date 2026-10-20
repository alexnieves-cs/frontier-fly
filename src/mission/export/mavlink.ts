/**
 * The slice of the MAVLink common dialect the exporters use. Values are from
 * message_definitions/v1.0/common.xml (https://mavlink.io/en/messages/common.html).
 */

/** MAV_CMD */
export const MAV_CMD = {
  NAV_WAYPOINT: 16,
  NAV_RETURN_TO_LAUNCH: 20,
  NAV_LAND: 21,
  NAV_TAKEOFF: 22,
  DO_CHANGE_SPEED: 178,
  VIDEO_START_CAPTURE: 2500,
  VIDEO_STOP_CAPTURE: 2501,
  NAV_FENCE_POLYGON_VERTEX_INCLUSION: 5001,
  NAV_FENCE_POLYGON_VERTEX_EXCLUSION: 5002,
  NAV_RALLY_POINT: 5100,
} as const;

export type MavCmd = (typeof MAV_CMD)[keyof typeof MAV_CMD];

/** MAV_FRAME */
export const MAV_FRAME = {
  /** Global, altitude above mean sea level. */
  GLOBAL: 0,
  /** Not a position: the item is a command. */
  MISSION: 2,
  /** Global, altitude relative to the home position. */
  GLOBAL_RELATIVE_ALT: 3,
  /** Global, altitude above terrain from the vehicle's terrain database. */
  GLOBAL_TERRAIN_ALT: 10,
} as const;

export type MavFrame = (typeof MAV_FRAME)[keyof typeof MAV_FRAME];

/** MAV_AUTOPILOT */
export const MAV_AUTOPILOT = { ARDUPILOTMEGA: 3, PX4: 12 } as const;

/** MAV_TYPE */
export const MAV_TYPE = { QUADROTOR: 2 } as const;

/** MAV_MISSION_TYPE */
export const MAV_MISSION_TYPE = { MISSION: 0, FENCE: 1, RALLY: 2 } as const;

/** DO_CHANGE_SPEED param1 speed type. */
export const SPEED_TYPE_GROUND = 1;

/** Commands whose params 5 to 7 are a position. */
export const NAV_COMMANDS: ReadonlySet<number> = new Set([
  MAV_CMD.NAV_WAYPOINT,
  MAV_CMD.NAV_LAND,
  MAV_CMD.NAV_TAKEOFF,
  MAV_CMD.NAV_FENCE_POLYGON_VERTEX_INCLUSION,
  MAV_CMD.NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
  MAV_CMD.NAV_RALLY_POINT,
]);

export const MAV_CMD_NAMES: Record<number, string> = Object.fromEntries(
  Object.entries(MAV_CMD).map(([name, value]) => [value, `MAV_CMD_${name}`]),
);
