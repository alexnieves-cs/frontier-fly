/**
 * Signal check for observation v2: does the sensor-only BFS waypoint point the
 * way the privileged frontier teacher commands? Runs the teacher on stage-3
 * buildings and compares body-frame headings once the lidar has looked around.
 * Usage: npx vite-node scripts/obs-agreement.ts [-- seed seed ...]
 */
import { generateBuilding } from '../src/gen/building';
import { Curriculum } from '../src/rl/curriculum';
import { OBS } from '../src/shared/constants';
import { RNG } from '../src/shared/rng';
import { createEnv, defaultEpisodeConfig } from '../src/sim/env';
import { frontierAction } from '../src/sim/frontier';
import { lightingPreset } from '../src/sim/lighting';

const WARMUP = 20;
const seeds = process.argv.slice(2).filter((a) => /^\d+$/.test(a)).map(Number);

async function main() {
  const env = await createEnv();
  let total = 0;
  let within45 = 0;
  let within90 = 0;
  let noFrontier = 0;
  const farN = [0, 0];
  const farA = [0, 0];
  for (const seed of seeds.length ? seeds : [4242, 5150, 80, 85]) {
    const params = new Curriculum().genParams(3, new RNG(seed, 'eval-3'));
    const spec = generateBuilding(params);
    const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
    const cfg = defaultEpisodeConfig(3, nRooms);
    cfg.maxSteps = 2400;
    cfg.stallSteps = 500;
    cfg.maxCollisions = 16;
    cfg.dynamics = 'kinematic';
    cfg.lighting = lightingPreset('day', nRooms);
    let obs = env.reset(spec, params.seed, cfg);
    let n = 0;
    let a45 = 0;
    let a90 = 0;
    let coverage = 0;
    for (let t = 0; t < cfg.maxSteps; t++) {
      const pose = env.snapshot();
      const act = frontierAction(spec, env.visitedGrid, pose.pos, pose.yaw);
      const speed = Math.hypot(act[0]!, act[1]!);
      if (t >= WARMUP && speed > 0.2) {
        if (obs[OBS.hasFrontier]! < 0.5) noFrontier += 1;
        else {
          // The teacher's yaw command is err / (π/3) clamped at ±0.8, where err is
          // its goal bearing in the body frame; unclamped it is the exact heading.
          const yawCmd = act[3]!;
          const teacher = Math.abs(yawCmd) < 0.79 ? (yawCmd * Math.PI) / 3 : Math.atan2(act[1]!, act[0]!);
          const ours = Math.atan2(obs[OBS.wpDir + 1]!, obs[OBS.wpDir]!);
          let d = Math.abs(teacher - ours) % (Math.PI * 2);
          if (d > Math.PI) d = Math.PI * 2 - d;
          n += 1;
          const far = obs[OBS.frontierDist]! * 8 >= 1 ? 1 : 0;
          farN[far] += 1;
          if (d < Math.PI / 4) farA[far] += 1;
          if (d < Math.PI / 4) a45 += 1;
          if (d < Math.PI / 2) a90 += 1;
        }
      }
      const step = env.step(act);
      obs = step.obs;
      coverage = step.info.coverage;
      if (step.done) break;
    }
    console.log(
      `stage-3 seed ${seed}: ${n} moving steps, agree <45° ${((100 * a45) / Math.max(1, n)).toFixed(1)}%, <90° ${((100 * a90) / Math.max(1, n)).toFixed(1)}%, teacher coverage ${(coverage * 100).toFixed(1)}%`,
    );
    total += n;
    within45 += a45;
    within90 += a90;
  }
  console.log(
    `overall: ${total} steps, agree <45° ${((100 * within45) / Math.max(1, total)).toFixed(1)}%, <90° ${((100 * within90) / Math.max(1, total)).toFixed(1)}%, no-frontier while teacher moving ${noFrontier}`,
  );
  const pct = (a: number, b: number) => ((100 * a) / Math.max(1, b)).toFixed(1);
  console.log(
    `by target distance: < 1 m ${farN[0]} steps, <45° ${pct(farA[0]!, farN[0]!)}%; >= 1 m ${farN[1]} steps, <45° ${pct(farA[1]!, farN[1]!)}%`,
  );
}

void main();
