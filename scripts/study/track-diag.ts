// Trajectory-tracking diagnostics for the sensor planner on the planner-tuning buildings
// (9101+), never on study seeds. Compares the kinematic airframe with the quadrotor and
// reports where the quad's time goes: mean flown speed, commanded speed, tracking error,
// share of steps nearly stopped, distance flown, collisions.
//
//   vite-node scripts/study/track-diag.ts -- [--n 8] [--speed 2] [--variants kin,quad,quadClean] [--pilots plain,brake,track,trackBrake] [--tk accel=3,corner=0.4] [--mom 1]
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { classicalPilot } from '../../src/eval/pilots';
import { RNG } from '../../src/shared/rng';
import { defaultDynParams, sampleDynParams, sampleNoise, zeroNoise } from '../../src/sim/dynamics/dr';
import { createEnv } from '../../src/sim/env';
import { QUAD_BRAKING, type SensorPlannerOptions } from '../../src/sim/sensorPlanner';
import { MAX_SPEED_H } from '../../src/shared/constants';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const n = Number(arg('n', '8'));
const base = Number(arg('base', '9101'));
const speed = Number(arg('speed', '2'));
const variants = arg('variants', 'kin,quad').split(',');
const pilots = arg('pilots', 'plain').split(',');
const tk = Object.fromEntries(
  arg('tk', '')
    .split(',')
    .filter(Boolean)
    .map((kv) => [kv.split('=')[0]!, Number(kv.split('=')[1])]),
);
const env = await createEnv();

const mom = Number(arg('mom', '0'));
const pilotOpts: Record<string, SensorPlannerOptions> = {
  plain: { speed, momentum: mom },
  brake: { speed, braking: QUAD_BRAKING, momentum: mom },
  track: { speed, tracker: tk, momentum: mom },
  trackBrake: { speed, braking: QUAD_BRAKING, tracker: tk, momentum: mom },
  plainNoFace: { speed, momentum: mom, faceTravel: false },
  brakeNoFace: { speed, braking: QUAD_BRAKING, momentum: mom, faceTravel: false },
  trackNoFace: { speed, tracker: tk, momentum: mom, faceTravel: false },
  trackBrakeNoFace: { speed, braking: QUAD_BRAKING, tracker: tk, momentum: mom, faceTravel: false },
};

for (const variant of variants)
  for (const pilotName of pilots) {
    let cov = 0;
    let coll = 0;
    let vSum = 0;
    let cmdSum = 0;
    let errSum = 0;
    let slow = 0;
    let steps = 0;
    let dist = 0;
    let banned = 0;
    let fresh = 0;
    for (let i = 0; i < n; i++) {
      const seed = base + i;
      const { spec, envSeed, cfg } = evalEpisodeSetup(3, seed, 'day');
      if (variant !== 'kin') {
        const rng = new RNG(seed, 'study-dr');
        cfg.dynamics = 'quad';
        cfg.dyn = sampleDynParams(rng, defaultDynParams(), 1);
        cfg.noise = sampleNoise(rng, 1, false);
        if (variant === 'quadClean') cfg.noise = zeroNoise();
        if (variant === 'quadNom') {
          cfg.dyn = defaultDynParams();
          cfg.noise = zeroNoise();
        }
      }
      cfg.speedH = speed;
      const p = classicalPilot(pilotOpts[pilotName]!);
      p.reset();
      let obs = env.reset(spec, envSeed, cfg);
      let info = { coverage: 0, collisions: 0 };
      let prev = env.snapshot().pos;
      for (let t = 0; t < cfg.maxSteps; t++) {
        const a = p.act(obs, env);
        const s = env.step(a);
        const st = env.snapshot();
        const v = Math.hypot(st.vel[0], st.vel[1]);
        const cmd = Math.hypot(a[0]!, a[1]!) * speed;
        const cy = Math.cos(st.yaw);
        const sy = Math.sin(st.yaw);
        const cx = (cy * a[0]! - sy * a[1]!) * speed;
        const cyw = (sy * a[0]! + cy * a[1]!) * speed;
        errSum += Math.hypot(cx - st.vel[0], cyw - st.vel[1]);
        vSum += v;
        cmdSum += cmd;
        if (v < 0.3) slow += 1;
        dist += Math.hypot(st.pos[0] - prev[0], st.pos[1] - prev[1]);
        prev = st.pos;
        steps += 1;
        obs = s.obs;
        info = s.info;
        if (s.done) break;
      }
      banned += p.planner.bannedCount;
      fresh += info.coverage * spec.reachableVoxels;
      cov += info.coverage;
      coll += info.collisions;
    }
    console.log(
      `${variant}/${pilotName} v=${speed}: coverage ${((cov / n) * 100).toFixed(1)}%  coll/ep ${(coll / n).toFixed(2)}  mean speed ${(vSum / steps).toFixed(2)}  cmd ${(cmdSum / steps).toFixed(2)}  track err ${(errSum / steps).toFixed(2)}  slow ${((slow / steps) * 100).toFixed(0)}%  dist/ep ${(dist / n).toFixed(0)} m  banned/ep ${(banned / n).toFixed(1)}  new voxels per voxel flown ${(fresh / (dist / 0.5)).toFixed(2)}  (cap ${MAX_SPEED_H})`,
    );
  }
env.dispose();
