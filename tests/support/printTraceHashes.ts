/** Prints the trace hash of every TRACE_CASE. Run against an old checkout to record goldens. */
import { createEnv } from '../../src/sim/env';
import { runTrace, TRACE_CASES } from './envTrace';

const out: Record<string, string> = {};
for (const c of TRACE_CASES) {
  const env = await createEnv();
  out[c.name] = runTrace(env, c).hash;
}
console.log(JSON.stringify(out, null, 2));
