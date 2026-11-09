/**
 * Frontier-fly's physics as PX4's simulator, headless, over TCP.
 *
 *   npx vite-node scripts/px4/sim-node.ts -- [--port 4560] [--speed 1] [--plan file.plan] [--truth out.json]
 *
 * Listens where PX4 SITL's simulator_mavlink connects (TCP 4560, airframe
 * 10016_none_iris) and runs the lockstep loop in src/px4/session.ts. With
 * --plan, the matching .meta.json from scripts/px4/export-plans.ts puts the
 * pad on the plan's home and, for a woods plan, gives the ground the sim
 * world's terrain, so PX4 flies the route over the same hills the mission
 * planner saw. --truth writes the true track (1 Hz) and the closest approach
 * to any tree, for the flight check report.
 */
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { createServer } from 'node:net';
import { buildMissionWorld } from '../../src/mission/autonomy/world';
import { GEO_ORIGINS, type GeoPoint } from '../../src/mission/export/geo';
import { Px4SimSession } from '../../src/px4/session';
import { PhysicsRig, type RigOptions } from '../../src/px4/rig';
import { sampleHeight } from '../../src/world/outdoor/terrain';
import type { TreeInstance } from '../../src/world/outdoor/types';

const args = process.argv.slice(2);
const opt = (name: string): string | undefined => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const port = Number(opt('port') ?? 4560);
const speed = Number(opt('speed') ?? 1);
const planPath = opt('plan');
const truthPath = opt('truth');
/**
 * --plant frontier puts PX4 on frontier-fly's own rigid body (the mission quad) instead of the Iris
 * model; --rig takes a JSON RigOptions (wind, pack state of charge, near-surface aerodynamics) and
 * --log writes a 50 Hz state log stamped with PX4's clock. Used by the physics validation
 * (scripts/physics/run.sh).
 */
const plantKind = opt('plant') ?? 'iris';
const rigJson = opt('rig');
const logPath = opt('log');

interface Meta {
  site: { kind: 'woods'; seed: number } | { kind: 'practice' };
  pad: { x: number; y: number; groundZ: number };
  home: GeoPoint;
}

let home: GeoPoint = GEO_ORIGINS['px4-sitl'];
let groundDown: (north: number, east: number) => number = () => 0;
let trees: TreeInstance[] = [];
let pad = { x: 0, y: 0, groundZ: 0 };
if (planPath) {
  const meta = JSON.parse(readFileSync(planPath.replace(/\.plan$/u, '.meta.json'), 'utf8')) as Meta;
  home = meta.home;
  pad = meta.pad;
  if (meta.site.kind === 'woods') {
    const world = buildMissionWorld({ seed: meta.site.seed });
    const terrain = world.outdoor.terrain;
    groundDown = (north, east) => -(sampleHeight(terrain, pad.x + east, pad.y + north) - pad.groundZ);
    trees = world.outdoor.trees.filter((t) => t.kind !== 'undergrowth');
  }
}

/** Distance from a sim-frame point to a tree's trunk cylinder or canopy sphere, whichever is closer. */
function treeClearance(x: number, y: number, z: number): number {
  let best = Infinity;
  for (const t of trees) {
    const dxy = Math.hypot(x - t.x, y - t.y);
    if (dxy > 20) continue;
    const top = t.z + t.trunkHeight;
    const trunkDz = z > top ? z - top : z < t.z ? t.z - z : 0;
    const trunk = Math.hypot(Math.max(0, dxy - t.trunkRadius), trunkDz);
    const canopy = Math.hypot(dxy, z - (t.z + t.canopyZ)) - t.canopyRadius;
    best = Math.min(best, trunk, canopy);
  }
  return best;
}

const truth: number[][] = [];
let minTreeClearance = Infinity;
let maxGroundTouchSpeed = 0;

const server = createServer((socket) => {
  socket.setNoDelay(true);
  console.log(`PX4 connected from ${socket.remoteAddress}:${socket.remotePort}`);
  const rig = plantKind === 'frontier' ? new PhysicsRig(rigJson ? (JSON.parse(rigJson) as RigOptions) : {}) : null;
  const session = new Px4SimSession((bytes) => socket.write(bytes), { home, lockstepTimeoutMs: 100, quad: rig?.plant });
  session.quad.groundDown = groundDown;
  const markerPath = logPath ? `${logPath}.t0` : null;
  let gustT0Us: number | null = null;
  let nextMarkerCheck = 0;
  let open = true;
  activeRig = rig;
  socket.on('data', (chunk: Buffer) => session.receive(chunk));
  socket.on('close', () => {
    open = false;
    session.abort();
  });
  socket.on('error', () => {
    open = false;
  });
  void (async () => {
    const wall0 = performance.now();
    let nextLog = 0;
    let nextTruth = 0;
    let wasAirborne = false;
    while (open) {
      const vz = session.quad.s.vel[2];
      if (rig) {
        const px4Us = session.lastControls?.timeUsec ?? 0;
        if (markerPath && gustT0Us === null && session.simTimeSec >= nextMarkerCheck) {
          nextMarkerCheck = session.simTimeSec + 0.5;
          if (existsSync(markerPath)) gustT0Us = Number(readFileSync(markerPath, 'utf8'));
        }
        rig.beforeStep(gustT0Us === null ? null : (px4Us - gustT0Us) * 1e-6);
      }
      await session.step();
      if (rig && logPath) rig.record(session.lastControls?.timeUsec ?? 0, session.motors);
      const t = session.simTimeSec;
      const s = session.quad.s;
      if (!s.onGround) wasAirborne = true;
      if (wasAirborne && s.onGround) maxGroundTouchSpeed = Math.max(maxGroundTouchSpeed, vz);
      const [n, e, d] = s.pos;
      const z = pad.groundZ - d;
      if (!s.onGround && trees.length) minTreeClearance = Math.min(minTreeClearance, treeClearance(pad.x + e, pad.y + n, z));
      if (t >= nextTruth) {
        truth.push([Math.round(t * 10) / 10, +e.toFixed(2), +n.toFixed(2), +(-d).toFixed(2), session.armed ? 1 : 0]);
        nextTruth += 1;
      }
      if (t >= nextLog) {
        const st = session.stats;
        console.log(
          `t=${t.toFixed(0)}s armed=${session.armed} ned=(${n.toFixed(1)}, ${e.toFixed(1)}, ${d.toFixed(1)}) ` +
            `motors=${Array.from(session.motors, (m) => m.toFixed(2)).join(',')} timeouts=${st.timeouts} badCrc=${st.badCrc}`,
        );
        nextLog += 10;
        writeTruth();
      }
      if (speed > 0) {
        const ahead = t * 1000 - (performance.now() - wall0) * speed;
        if (ahead > 5) await new Promise((r) => setTimeout(r, ahead / speed));
      }
    }
    console.log(`PX4 disconnected after ${session.simTimeSec.toFixed(1)} s, ${session.stats.timeouts} lockstep timeouts`);
  })();
});

function writeLog(): void {
  if (!logPath || !activeRig) return;
  writeFileSync(logPath, `${JSON.stringify(activeRig.log())}\n`);
}
let activeRig: PhysicsRig | null = null;

function writeTruth(): void {
  writeLog();
  if (!truthPath) return;
  writeFileSync(
    truthPath,
    `${JSON.stringify({
      columns: ['t', 'east', 'north', 'up', 'armed'],
      track: truth,
      minTreeClearanceM: Number.isFinite(minTreeClearance) ? +minTreeClearance.toFixed(2) : null,
      touchdownSpeedMps: +maxGroundTouchSpeed.toFixed(2),
    })}\n`,
  );
}
for (const signal of ['SIGTERM', 'SIGINT'] as const) {
  process.on(signal, () => {
    writeTruth();
    process.exit(0);
  });
}

// Loopback only: PX4 SITL connects from this machine, and nothing on the network should reach the port.
server.listen(port, '127.0.0.1', () => console.log(`waiting for PX4 on TCP ${port}`));
