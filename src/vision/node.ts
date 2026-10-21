// Node-only loaders for the vision assets (benches, training, tests). The browser fetches the same files.
import { readFileSync } from 'node:fs';
import type { DetectorBundle, DetectorMeta } from './detector';
import { parseHumanoidPoses, type HumanoidPoses } from './humanoid';

export const HUMANOID_POSES_PATH = 'public/assets/targets/humanoid-poses.bin';

let cached: HumanoidPoses | null = null;

export function loadHumanoidPosesSync(path = HUMANOID_POSES_PATH): HumanoidPoses {
  if (cached && path === HUMANOID_POSES_PATH) return cached;
  const buf = readFileSync(path);
  const poses = parseHumanoidPoses(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength));
  if (path === HUMANOID_POSES_PATH) cached = poses;
  return poses;
}

export const DETECTOR_DIR_PATH = 'public/models/vision-detector';

/** Loads the shipped detector from disk (tfjs-node or the cpu backend, whichever is active). */
export async function loadDetectorFromDisk(dir = DETECTOR_DIR_PATH): Promise<DetectorBundle> {
  const tf = await import('@tensorflow/tfjs');
  const json = JSON.parse(readFileSync(`${dir}/model.json`, 'utf8')) as {
    modelTopology: object;
    weightsManifest: Array<{ weights: import('@tensorflow/tfjs').io.WeightsManifestEntry[] }>;
  };
  const buf = readFileSync(`${dir}/weights.bin`);
  const weightData = buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
  const model = await tf.loadLayersModel(tf.io.fromMemory({ modelTopology: json.modelTopology, weightSpecs: json.weightsManifest[0]!.weights, weightData }));
  const meta = JSON.parse(readFileSync(`${dir}/meta.json`, 'utf8')) as DetectorMeta;
  return { model, calibration: meta.calibration, minScore: meta.minScore };
}

/** Uses tfjs-node when it is installed (benches); the cpu backend otherwise (CI). */
export async function useNativeTf(): Promise<string> {
  const tf = await import('@tensorflow/tfjs');
  try {
    const name = '@tensorflow/tfjs-node';
    await import(/* @vite-ignore */ name);
    await tf.setBackend('tensorflow');
  } catch {
    await tf.setBackend('cpu');
  }
  await tf.ready();
  return tf.getBackend();
}
