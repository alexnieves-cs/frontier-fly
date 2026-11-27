/** Where does the quad student crash? Flies a checkpoint alone on practice seeds and prints the last steps before each crash. */
import { readFileSync } from 'node:fs';
import { ACT_DIM, MAX_SPEED_H, OBS } from '../../src/shared/constants';
import { parseCheckpoint } from '../../src/rl/checkpoint';
import { QuadStudentPilot } from '../../src/rl/quadStudent';
import { quadPilot } from '../../src/eval/pilots';
import { createEnv } from '../../src/sim/env';
import { episodeSetup } from '../../src/train/pickerJob';

const ck = parseCheckpoint(readFileSync(process.argv[2]!));
const n = Number(process.argv[3] ?? 6);
const env = await createEnv();
for (let i = 0; i < n; i++) {
  const e = { stage: 3, seed: 180001 + i, lighting: 'day' as const, quad: 1 };
  const s = episodeSetup(e);
  const st = new QuadStudentPilot(ck);
  st.reset();
  const ex = quadPilot();
  ex.reset();
  let obs = env.reset(s.spec, s.envSeed, s.cfg);
  const hist: string[] = [];
  let cov: number;
  for (let t = 0; t < s.cfg.maxSteps; t++) {
    const a = st.act(obs);
    const yaw = env.snapshot().yaw;
    const f = a[0]! * MAX_SPEED_H, l = a[1]! * MAX_SPEED_H;
    ex.planner.holdCommand(Math.cos(yaw) * f - Math.sin(yaw) * l, Math.sin(yaw) * f + Math.cos(yaw) * l);
    const lab = ex.act(obs, env);
    let minL = 1;
    for (let k = 0; k < 32; k++) minL = Math.min(minL, obs[OBS.lidar + k]!);
    const v = [obs[OBS.vel]! * 2, obs[OBS.vel + 1]! * 2];
    hist.push(`t${t} v(${v[0]!.toFixed(2)},${v[1]!.toFixed(2)}) alt ${(obs[OBS.alt]!).toFixed(2)} ceil ${(obs[OBS.ceil]!).toFixed(2)} vz ${obs[OBS.vel + 2]!.toFixed(2)} minL ${(minL * 8).toFixed(2)} fwdL ${(obs[OBS.lidar]! * 8).toFixed(2)} tilt [${Array.from(obs.subarray(OBS.tilt, OBS.tilt + 8)).map((x) => (x * 8).toFixed(1))}] pos [${env.snapshot().pos.map((x) => x.toFixed(2))}] roll ${obs[OBS.roll]!.toFixed(2)} pitch ${obs[OBS.pitch]!.toFixed(2)} stu [${Array.from(a).map((x) => x.toFixed(2))}] exp [${Array.from(lab.subarray(0, ACT_DIM)).map((x) => x.toFixed(2))}]`);
    const step = env.step(a);
    obs = step.obs;
    cov = step.info.coverage;
    if (step.done) {
      console.log(`seed ${e.seed} t ${t} cov ${(cov * 100).toFixed(1)} crashed ${step.info.crashed} impact ${step.info.impactSpeed.toFixed(2)}`);
      if (step.info.crashed) console.log(hist.slice(-12).join('\n'));
      break;
    }
  }
}
