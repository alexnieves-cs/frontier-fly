/**
 * Writes the shipped checkpoint without its value head (public/checkpoints/policy.bin),
 * the file the pages that fly the policy load. Run it after replacing pretrained.bin;
 * a unit test fails while the two disagree.
 *
 *   npm run pack:policy
 */
import { readFileSync, writeFileSync } from 'node:fs';
import { actorOnly, parseCheckpoint, serializeCheckpointBinary, SHIPPED_CHECKPOINT_PATH, SHIPPED_POLICY_PATH } from '../src/rl/checkpoint';

const bytes = serializeCheckpointBinary(actorOnly(parseCheckpoint(readFileSync(SHIPPED_CHECKPOINT_PATH))));
writeFileSync(SHIPPED_POLICY_PATH, bytes);
console.log(`wrote ${SHIPPED_POLICY_PATH}: ${(bytes.length / 1e6).toFixed(2)} MB (from ${(readFileSync(SHIPPED_CHECKPOINT_PATH).length / 1e6).toFixed(2)} MB)`);
