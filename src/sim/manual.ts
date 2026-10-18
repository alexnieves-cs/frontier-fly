/** Keyboard fly mapping. Body frame: +vx forward, +vy left, +vz up, +yawRate CCW. */
const ZERO = new Float32Array(4);

export function actionFromKeys(keys: ReadonlySet<string>): Float32Array {
  const action = new Float32Array(4);
  if (keys.has('KeyW') || keys.has('ArrowUp')) action[0] += 1;
  if (keys.has('KeyS') || keys.has('ArrowDown')) action[0] -= 1;
  if (keys.has('KeyA')) action[1] += 1;
  if (keys.has('KeyD')) action[1] -= 1;
  if (keys.has('Space')) action[2] += 1;
  if (keys.has('ShiftLeft') || keys.has('ShiftRight') || keys.has('KeyC')) action[2] -= 1;
  if (keys.has('KeyQ') || keys.has('ArrowLeft')) action[3] -= 1;
  if (keys.has('KeyE') || keys.has('ArrowRight')) action[3] += 1;
  for (let i = 0; i < 4; i++) action[i] = Math.max(-1, Math.min(1, action[i]!));
  return action;
}

export function zeroAction(): Float32Array {
  return new Float32Array(ZERO);
}
