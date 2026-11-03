// Entry point of the headless CLI: `npm run cli -- <command>`, or dist-cli/frontier-fly.mjs once bundled.
import { main } from './frontier-fly';

main(process.argv.slice(2)).then(
  (code) => {
    process.exitCode = code;
  },
  (e: unknown) => {
    process.stderr.write(`frontier-fly: ${(e as Error).stack ?? String(e)}\n`);
    process.exitCode = 1;
  },
);
