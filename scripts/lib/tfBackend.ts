import { setTrainerBackend } from '../../src/rl/model';

/**
 * Use the native TensorFlow backend for training when `@tensorflow/tfjs-node`
 * is installed (`npm install --no-save @tensorflow/tfjs-node@4.22.0`). It is
 * not a dependency, so CI and the browser build never download it; without it
 * training falls back to the pure-JS cpu backend, which is ~50x slower.
 */
export async function useFastestBackend(): Promise<string> {
  try {
    const name = '@tensorflow/tfjs-node';
    await import(/* @vite-ignore */ name);
    setTrainerBackend('tensorflow');
    return 'tensorflow';
  } catch {
    return 'cpu';
  }
}
