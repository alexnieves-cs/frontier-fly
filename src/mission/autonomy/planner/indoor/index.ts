/**
 * Indoor exploration, in the building-local frame (world minus shell.origin). This is the surface the
 * mission phases use: the visit's frame (grid.ts), the drone's own maps (sensing.ts), navigation
 * queries behind IndoorNav (nav.ts), and the explorers (explorers.ts, plus the oracle reference in
 * oracle.ts). Grid internals (cells, grid paths, floor-plan line of sight) stay in grid.ts: a phase
 * that needs one is missing an IndoorNav method, and eslint.config.js keeps phases from importing it.
 *
 * Missions fly the trained policy and the sensor-only explorer, which read only the drone's own maps.
 * The frontier explorer in oracle.ts plans on the true floor plan, so it is kept only as the 'oracle'
 * reference.
 */
export { INDOOR_FLY_Z, type IndoorGrid, floorOf, indoorGrid, toLocal, toWorld } from './grid';
export { INDOOR_FILTER_HORIZON, IndoorSensing, type LidarWorld, shellLidar } from './sensing';
export { type IndoorNav, oracleNav, sensorNav } from './nav';
export { type ActingPolicy, type PolicyCommand, PolicyExplorer, SensorExplorer, bestViewpoint } from './explorers';
export { FrontierExplorer, type IndoorGoal, IndoorVisibility } from './oracle';
