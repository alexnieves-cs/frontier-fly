// Runs one submitted FrontierBench pilot in a worker thread (see sandbox.ts for the protocol).
// The worker has its own heap and its own module instances: the pilot cannot reach the scorer's
// env, the building it is flying or the scorer's code. It sees only what the scorer copies into
// the shared buffers, and the scorer reads back only the four action numbers.
import { receiveMessageOnPort, workerData } from 'node:worker_threads';
import { createServer } from 'vite';
import { ViteNodeRunner } from 'vite-node/client';
import { ViteNodeServer } from 'vite-node/server';

const { port, file, ctrl: ctrlBuf, meta: metaBuf, obs: obsBuf, lidar: lidarBuf, grids: gridBuf, action: actionBuf, layout } = workerData;
const ctrl = new Int32Array(ctrlBuf);
const meta = new Float64Array(metaBuf);
const action = new Float32Array(actionBuf);
const M = layout.meta;
const CMD = layout.cmd;

function reply(ok, message) {
  if (!ok) port.postMessage({ error: String(message) });
  Atomics.store(ctrl, 2, ok ? 0 : 1);
  Atomics.add(ctrl, 1, 1);
  Atomics.notify(ctrl, 1);
}

let pilot;
let server;
try {
  server = await createServer({ optimizeDeps: { noDiscovery: true, include: [] }, server: { middlewareMode: true, hmr: false, watch: null }, logLevel: 'error' });
  const node = new ViteNodeServer(server);
  const runner = new ViteNodeRunner({
    root: server.config.root,
    base: server.config.base,
    fetchModule: (id) => node.fetchModule(id),
    resolveId: (id, importer) => node.resolveId(id, importer),
  });
  const mod = await runner.executeFile(file);
  if (typeof mod.default !== 'function') throw new Error('the pilot module has no default export that makes a pilot');
  pilot = await mod.default();
  if (!pilot || typeof pilot.act !== 'function' || typeof pilot.reset !== 'function') throw new Error('the pilot needs reset() and act()');
  port.postMessage({ ready: true, name: String(pilot.name) });
} catch (e) {
  port.postMessage({ error: e instanceof Error ? (e.stack ?? e.message) : String(e) });
}

// From here on the worker only answers the scorer, one command at a time.
let seq = 0;
for (;;) {
  Atomics.wait(ctrl, 0, seq);
  seq = Atomics.load(ctrl, 0);
  const cmd = Atomics.load(ctrl, 3);
  if (cmd === CMD.quit || !pilot) break;
  try {
    if (cmd === CMD.reset) {
      pilot.reset({ speedH: meta[M.speedH], maxSteps: meta[M.maxSteps], dynamics: meta[M.dynamics] ? 'quad' : 'kinematic' });
    } else if (cmd === CMD.act) {
      const cells = meta[M.cells];
      const frame = {
        seen: new Uint8Array(gridBuf, 0, cells),
        visited: new Uint8Array(gridBuf, layout.gridCap, cells),
        ...(meta[M.hasOccupied] ? { occupied: new Uint8Array(gridBuf, 2 * layout.gridCap, cells) } : {}),
        vw: meta[M.vw],
        vd: meta[M.vd],
        floors: meta[M.floors],
        floorHeight: meta[M.floorHeight],
        pos: [meta[M.pos], meta[M.pos + 1], meta[M.pos + 2]],
        yaw: meta[M.yaw],
        lidar: new Float64Array(lidarBuf, 0, meta[M.lidarLen]),
        alt: meta[M.alt],
        ...(meta[M.hasVel] ? { vel: [meta[M.vel], meta[M.vel + 1]] } : {}),
      };
      const a = pilot.act(new Float32Array(obsBuf, 0, meta[M.obsLen]), frame);
      if (!a || a.length !== 4) throw new Error('act() must return four numbers');
      for (let i = 0; i < 4; i++) action[i] = a[i];
    }
    reply(true);
  } catch (e) {
    reply(false, e instanceof Error ? (e.stack ?? e.message) : e);
  }
}
// Drain anything left so the port closes cleanly, then let the thread end.
while (receiveMessageOnPort(port)) {
  /* nothing */
}
port.close();
await server?.close();
