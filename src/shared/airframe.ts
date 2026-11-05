/**
 * The one airframe every part of frontier-fly flies: the rigid-body dynamics and flight controller
 * (sim/dynamics), the collision sphere (shared/constants.ts DRONE_RADIUS), the battery and energy
 * model (mission/autonomy/battery.ts), and the 3D mesh (viz/droneMesh.ts).
 *
 * It is a ducted indoor quad of the size and mass of the DJI Avata (2022, published specs):
 * 410 g takeoff weight, 180 x 180 x 80 mm with ducts, 2.9-inch props, 4S 2420 mAh / 35.71 Wh pack,
 * 18 min rated flight time. A ducted frame is the airframe class built to fly through a 1 m doorway
 * and bump a wall at walking pace (the crash rule, shared/constants.ts CRASH_SPEED).
 *
 * Where frontier-fly departs from that reference, the comment on the field says why. PX4 SITL runs
 * its own stock model (px4/quad.ts IRIS) when PX4 flies the exported mission plans; that model
 * belongs to PX4's simulator, not to this airframe.
 */
export const AIRFRAME = {
  /** All-up mass, kg. 40 g over the reference for the lidar ring and the onboard computer. */
  massKg: 0.45,
  /** Body inertia about x, y, z, kg m^2 (a 0.45 kg X frame with 0.1 m arms). */
  inertia: [0.0022, 0.0024, 0.0036] as readonly [number, number, number],
  /** Motor axis to frame center, m (a 200 mm motor-to-motor diagonal). */
  armM: 0.1,
  /** Propeller diameter, m (3 inch). */
  propDiameterM: 0.0762,
  /** Duct wall plus clearance around each prop, m. */
  ductWallM: 0.006,
  /**
   * Radius of the sphere every collision and clearance check uses, m: arm + prop radius + duct wall
   * is 0.144 m, rounded up. The reference frame's half-diagonal is 0.127 m.
   */
  collisionRadiusM: 0.15,
  /** First-order motor lag, s. */
  motorTauS: 0.03,
  /** Battery: 4S LiPo, the reference pack. */
  pack: { cells: 4, mAh: 2420, nominalV: 14.76, capacityWh: 35.71 } as const,
  /** Parasite drag area, m^2: about 0.015 m^2 of frontal area (180 x 80 mm) at Cd ~1 for a bluff ducted body. */
  cdaM2: 0.015,
};

/** Total rotor count. */
export const ROTORS = 4;
