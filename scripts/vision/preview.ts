// Renders a few camera frames from the scenario worlds to PNG, for eyeballing the camera model.
//   npx vite-node scripts/vision/preview.ts -- <outDir>
import { mkdirSync, writeFileSync } from 'node:fs';
import { cameraLookingAt, fpvCameraFromDrone } from '../../src/perception/camera';
import { SCENARIOS } from '../../src/mission/autonomy/scenarios';
import { buildMissionWorld } from '../../src/mission/autonomy/world';
import { VisionScene } from '../../src/vision/scene';
import { loadHumanoidPosesSync } from '../../src/vision/node';
import { encodePng } from '../../src/vision/png';
import { resolveScene } from '../../src/targets/motion';
import type { V3 } from '../../src/mission/autonomy/contracts';

const out = process.argv.slice(2).filter((a) => a !== '--')[0] ?? 'test-results/vision-preview';
mkdirSync(out, { recursive: true });
const poses = loadHumanoidPosesSync();
for (const scenario of SCENARIOS) {
  const world = buildMissionWorld(scenario.world);
  const scene = new VisionScene(world, poses);
  const lighting = { timeOfDay: world.timeOfDay, flashlightOn: world.flashlightOn, sensorMode: world.sensorMode };
  let k = 0;
  for (const b of world.buildings) {
    for (const t of resolveScene(b.scene.targets, 0)) {
      const c: V3 = [t.center[0] + b.shell.origin[0], t.center[1] + b.shell.origin[1], t.center[2] + b.shell.origin[2]];
      for (const [d, yawOff] of [[2.2, 0], [4, 0.6]] as const) {
        const ang = Math.atan2(t.feet[1] - b.scene.targets[0]!.feet[1], 1) + yawOff + k;
        const eye: V3 = [c[0] - Math.cos(ang) * d, c[1] - Math.sin(ang) * d, b.shell.origin[2] + t.floor * b.shell.spec.floorHeight + 1.2];
        const view = cameraLookingAt(eye, c, [0, 0, 1], (110 * Math.PI) / 180, 4 / 3);
        const t0 = performance.now();
        const f = scene.render(view, 0, lighting, 1234 + k);
        const ms = performance.now() - t0;
        const name = `${scenario.id}-${t.classId}-${d}m.png`;
        writeFileSync(`${out}/${name}`, encodePng(f.width, f.height, f.rgb));
        const px = f.inst.reduce((a, v) => a + (v > 0 ? 1 : 0), 0);
        console.log(name, `${ms.toFixed(1)} ms`, `gain ${f.gain.toFixed(2)}`, `target px ${px}`, f.truth.map((x) => x.classId).join(','));
        k += 1;
      }
    }
  }
  // Outdoor FPV from home.
  const view = fpvCameraFromDrone([world.home[0], world.home[1], world.home[2] + 1.5], world.homeYaw, 4 / 3);
  const f = scene.render(view, 0, lighting, 99);
  writeFileSync(`${out}/${scenario.id}-home.png`, encodePng(f.width, f.height, f.rgb));
}
