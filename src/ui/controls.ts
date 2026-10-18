import type { CameraMode, QualityPreset, SensorMode } from '../shared/types';

export interface LaunchParams {
  seed: number;
  camera: CameraMode;
  tod: number;
  shot: 'day' | 'night' | 'fpv' | 'dusk' | 'targets' | 'targets-room' | 'targets-night' | null;
  train: boolean;
  stage: number | 'auto';
  flashlight: boolean;
  lights: boolean;
  quality: QualityPreset;
  sensor: SensorMode;
  pilot: 'keys' | 'heuristic' | 'policy';
}

export function readLaunchParams(search = window.location.search): LaunchParams {
  const q = new URLSearchParams(search);
  const shotRaw = q.get('shot');
  const shot =
    shotRaw === 'day' ||
    shotRaw === 'night' ||
    shotRaw === 'fpv' ||
    shotRaw === 'dusk' ||
    shotRaw === 'targets' ||
    shotRaw === 'targets-room' ||
    shotRaw === 'targets-night'
      ? shotRaw
      : null;
  const cameraRaw = q.get('camera');
  let camera: CameraMode = cameraRaw === 'fpv' || cameraRaw === 'overview' || cameraRaw === 'chase' ? cameraRaw : 'chase';
  let tod = Number(q.get('tod') ?? '13');
  let flashlight = q.get('flash') === '1';
  let lights = q.get('lights') !== '0';
  if (shot === 'day') {
    tod = 13;
    camera = 'chase';
    flashlight = false;
    lights = true;
  } else if (shot === 'dusk') {
    tod = 18.5;
    camera = 'chase';
  } else if (shot === 'night') {
    tod = 22.5;
    camera = 'chase';
    flashlight = true;
    lights = false;
  } else if (shot === 'fpv') {
    tod = 22;
    camera = 'fpv';
    flashlight = true;
    lights = false;
  } else if (shot === 'targets' || shot === 'targets-room') {
    tod = 13;
    camera = 'fpv';
    flashlight = false;
    lights = true;
  } else if (shot === 'targets-night') {
    tod = 22.5;
    camera = 'fpv';
    flashlight = true;
    lights = false;
  }
  const stageRaw = q.get('stage');
  const stage = stageRaw == null || stageRaw === 'auto' ? 'auto' : Math.max(0, Math.min(5, Number(stageRaw) || 0));
  const quality: QualityPreset = q.get('quality') === 'low' ? 'low' : 'high';
  const sensorRaw = q.get('sensor');
  let sensor: SensorMode = sensorRaw === 'ir' || sensorRaw === 'depth' ? sensorRaw : 'rgb';
  if (shot === 'night' || shot === 'fpv') sensor = q.get('sensor') === 'ir' ? 'ir' : 'rgb';
  const pilotRaw = q.get('pilot');
  const pilot = pilotRaw === 'policy' || pilotRaw === 'keys' ? pilotRaw : shot ? 'heuristic' : 'keys';
  return {
    seed: Math.max(1, Number(q.get('seed') ?? '42') || 42),
    camera,
    tod: Number.isFinite(tod) ? tod : 13,
    shot,
    train: q.get('train') === '1',
    stage,
    flashlight,
    lights,
    quality,
    sensor,
    pilot,
  };
}

export function writeLaunchParams(params: LaunchParams): void {
  const q = new URLSearchParams();
  q.set('seed', String(params.seed));
  q.set('camera', params.camera);
  q.set('tod', params.tod.toFixed(2));
  if (params.stage !== 'auto') q.set('stage', String(params.stage));
  if (params.train) q.set('train', '1');
  if (params.shot) q.set('shot', params.shot);
  const url = `${window.location.pathname}?${q.toString()}`;
  window.history.replaceState(null, '', url);
}
