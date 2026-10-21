import { describe, expect, it } from 'vitest';
import type { V3 } from '../../src/mission/autonomy/contracts';
import { SCENARIOS } from '../../src/mission/autonomy/scenarios';
import { buildMissionWorld } from '../../src/mission/autonomy/world';
import { cameraLookingAt, fpvCameraFromDrone, projectAabb } from '../../src/perception/camera';
import { resolveScene } from '../../src/targets/motion';
import { frameLabels, sampleViews, trainingWorldOptions } from '../../src/vision/dataset';
import { decodeOutput, GRID_H, GRID_W, NUM_CLASSES, OUT_CHANNELS, type FrameDetector } from '../../src/vision/detector';
import { poseIndex } from '../../src/vision/humanoid';
import { loadHumanoidPosesSync } from '../../src/vision/node';
import { defaultPalette } from '../../src/vision/palette';
import { Raster } from '../../src/vision/raster';
import { VisionScene } from '../../src/vision/scene';
import { boxRange, createVisionSource } from '../../src/vision/source';

const poses = loadHumanoidPosesSync();
const day = { timeOfDay: 13, flashlightOn: false, sensorMode: 'rgb' as const };

/** The cabin scenario's lying person, and an FPV pose 2.5 m from it looking at it. */
function cabinView() {
  const scenario = SCENARIOS.find((s) => s.id === 'cabin-day')!;
  const world = buildMissionWorld(scenario.world);
  const b = world.buildings[0]!;
  const person = resolveScene(b.scene.targets, 0).find((t) => t.classId === 'person')!;
  const c: V3 = [person.center[0] + b.shell.origin[0], person.center[1] + b.shell.origin[1], person.center[2] + b.shell.origin[2]];
  // First bearing whose camera position is an empty cell.
  for (let k = 0; k < 16; k++) {
    const th = (k / 16) * Math.PI * 2;
    const eye: V3 = [c[0] - Math.cos(th) * 2.5, c[1] - Math.sin(th) * 2.5, b.shell.origin[2] + 1.3];
    const lx = Math.floor(eye[0] - b.shell.origin[0]);
    const ly = Math.floor(eye[1] - b.shell.origin[1]);
    const floor = b.shell.spec.floors[0]!;
    if (lx < 1 || ly < 1 || lx >= floor.w - 1 || ly >= floor.d - 1 || floor.cells[ly * floor.w + lx] !== 0) continue;
    return { world, person, c, view: cameraLookingAt(eye, c, [0, 0, 1], (110 * Math.PI) / 180, 4 / 3) };
  }
  throw new Error('no free bearing around the cabin person');
}

describe('vision: rasterizer', () => {
  it('covers the same pixels the analytic projection predicts for a box', () => {
    const r = new Raster(160, 120);
    const view = cameraLookingAt([0, -4, 1], [0, 0, 0.8], [0, 0, 1], (70 * Math.PI) / 180, 4 / 3);
    r.begin(view);
    const p = defaultPalette().crate;
    r.box(-0.4, -0.3, 0, 0.4, 0.3, 1.6, p, 1);
    let minX = 160;
    let maxX = -1;
    let minY = 120;
    let maxY = -1;
    for (let i = 0; i < r.tri.length; i++) {
      if (r.tri[i]! < 0) continue;
      const x = i % 160;
      const y = Math.floor(i / 160);
      minX = Math.min(minX, x);
      maxX = Math.max(maxX, x);
      minY = Math.min(minY, y);
      maxY = Math.max(maxY, y);
    }
    const box = projectAabb(view, { min: [-0.4, -0.3, 0], max: [0.4, 0.3, 1.6] })!.bbox;
    expect(minX / 160).toBeCloseTo(box.x, 1);
    expect((maxX + 1) / 160).toBeCloseTo(box.x + box.w, 1);
    expect(minY / 120).toBeCloseTo(box.y, 1);
    expect((maxY + 1) / 120).toBeCloseTo(box.y + box.h, 1);
  });

  it('keeps the nearest surface and clips geometry behind the camera', () => {
    const r = new Raster(32, 24);
    const view = cameraLookingAt([0, 0, 1], [0, 5, 1], [0, 0, 1], Math.PI / 2, 4 / 3);
    r.begin(view);
    const p = defaultPalette().crate;
    r.quad(-50, 8, -50, 50, 8, -50, 50, 8, 50, -50, 8, 50, p, 2);
    r.quad(-50, 4, -50, 50, 4, -50, 50, 4, 50, -50, 4, 50, p, 3);
    // A wall that straddles the camera plane: only the part in front may draw.
    r.quad(-1, -3, -10, -1, 3, -10, -1, 3, 10, -1, -3, 10, p, 4);
    const center = 12 * 32 + 16;
    expect(r.inst[r.tri[center]!]).toBe(3);
    expect(1 / r.invZ[center]!).toBeCloseTo(4, 3);
    let left = 0;
    for (let y = 0; y < 24; y++) if (r.inst[r.tri[y * 32]!] === 4) left++;
    expect(left).toBeGreaterThan(0);
  });
});

describe('vision: people', () => {
  it('bakes every person state, standing about 1.75 m tall, walking in six frames', () => {
    expect(poses.names).toEqual(['standing', 'sitting', 'lying', 'hiding', 'walking0', 'walking1', 'walking2', 'walking3', 'walking4', 'walking5']);
    let top = 0;
    for (let i = 0; i < poses.verts; i++) top = Math.max(top, poses.positions[i * 3 + 2]!);
    expect(top).toBeGreaterThan(1.65);
    expect(top).toBeLessThan(1.85);
    const frames = new Set([0, 0.2, 0.4, 0.6, 0.8, 1.0].map((t) => poseIndex(poses, 'walking', 'p1', t)));
    expect(frames.size).toBeGreaterThan(3);
    expect(poseIndex(poses, 'lying', 'p1', 3)).toBe(2);
  });
});

describe('vision: camera scene', () => {
  it('renders the same frame twice for the same pose, time and noise seed', () => {
    const { world, view } = cabinView();
    const scene = new VisionScene(world, poses);
    const a = scene.render(view, 1, day, 7);
    const b = scene.render(view, 1, day, 7);
    expect(Buffer.from(a.rgb).equals(Buffer.from(b.rgb))).toBe(true);
    const c = scene.render(view, 1, day, 8);
    expect(Buffer.from(a.rgb).equals(Buffer.from(c.rgb))).toBe(false);
  });

  it('labels the person it sees and measures its range from the depth image', () => {
    const { world, person, c, view } = cabinView();
    const scene = new VisionScene(world, poses);
    const frame = scene.render(view, 0, day, 1);
    const labels = frameLabels(frame);
    const seen = labels.find((l) => l.targetId === person.id);
    expect(seen).toBeDefined();
    expect(seen!.pixels).toBeGreaterThan(50);
    const range = boxRange(frame, seen!);
    expect(range).not.toBeNull();
    // The depth image sees the body's surface: compare with the person's own pixels, not its box center.
    const k = frame.truth.findIndex((t) => t.id === person.id) + 1;
    let sum = 0;
    let n = 0;
    for (let i = 0; i < frame.inst.length; i++) {
      if (frame.inst[i] !== k) continue;
      sum += frame.depth[i]!;
      n++;
    }
    expect(Math.abs(range! - sum / n)).toBeLessThan(0.35);
    expect(range!).toBeLessThan(Math.hypot(c[0] - view.position[0], c[1] - view.position[1], c[2] - view.position[2]) + 0.5);
  });

  it('gets darker and noisier at night without lights, and the flashlight lifts it', () => {
    const { world, view } = cabinView();
    for (const b of world.buildings) b.interiorLightsOn.fill(0);
    const scene = new VisionScene(world, poses);
    const dark = scene.render(view, 0, { timeOfDay: 23, flashlightOn: false, sensorMode: 'rgb' }, 1);
    const lit = scene.render(view, 0, { timeOfDay: 23, flashlightOn: true, sensorMode: 'rgb' }, 1);
    const dayFrame = scene.render(view, 0, day, 1);
    expect(dark.gain).toBeGreaterThan(dayFrame.gain * 2);
    expect(lit.gain).toBeLessThan(dark.gain);
  });

  it('samples training views from the fresh seed block with people, objects and decoys', () => {
    const opts = trainingWorldOptions(66001);
    expect(opts.seed).toBe(66001);
    const world = buildMissionWorld(SCENARIOS[0]!.world);
    const views = sampleViews(world, 40, 66001);
    expect(views.length).toBe(40);
    expect(new Set(views.map((v) => v.kind)).size).toBeGreaterThan(2);
  });
});

describe('vision: decoding and contacts', () => {
  it('decodes a heatmap peak into a box with its offset and size', () => {
    const out = new Float32Array(GRID_H * GRID_W * OUT_CHANNELS).fill(-8);
    const at = (iy: number, ix: number, k: number) => (iy * GRID_W + ix) * OUT_CHANNELS + k;
    out[at(7, 10, 0)] = 3;
    out[at(7, 11, 0)] = 1;
    out[at(7, 10, NUM_CLASSES)] = 0.5;
    out[at(7, 10, NUM_CLASSES + 1)] = 0.5;
    out[at(7, 10, NUM_CLASSES + 2)] = Math.log(2);
    out[at(7, 10, NUM_CLASSES + 3)] = Math.log(4);
    for (let iy = 0; iy < GRID_H; iy++) for (let ix = 0; ix < GRID_W; ix++) for (let k = NUM_CLASSES; k < OUT_CHANNELS; k++) if (!(iy === 7 && ix === 10)) out[at(iy, ix, k)] = 0;
    const found = decodeOutput(out, { minScore: 0.3 });
    expect(found).toHaveLength(1);
    expect(found[0]!.classId).toBe('person');
    expect(found[0]!.bbox.x + found[0]!.bbox.w / 2).toBeCloseTo(10.5 / GRID_W, 4);
    expect(found[0]!.bbox.w).toBeCloseTo(2 / GRID_W, 4);
    expect(found[0]!.bbox.h).toBeCloseTo(4 / GRID_H, 4);
  });

  it('turns a detector box into a contact at the target, scored against the instance it covers', () => {
    const { world, person, c, view } = cabinView();
    const scene = new VisionScene(world, poses);
    const label = frameLabels(scene.render(view, 0, day, 1)).find((l) => l.targetId === person.id)!;
    // A stand-in detector that reports the true box: checks the plumbing, not the network.
    const stub: FrameDetector = {
      detect: () => [{ classId: 'person', confidence: 0.9, score: 0.9, bbox: { x: label.x, y: label.y, w: label.w, h: label.h } }],
      dispose: () => {},
    };
    const source = createVisionSource(world, poses, stub);
    const pose = { pos: [view.position[0], view.position[1], view.position[2]] as V3, yaw: Math.atan2(c[1] - view.position[1], c[0] - view.position[0]), vel: [0, 0, 0] as V3 };
    // observe() uses the FPV camera from the pose; aim it with the same yaw.
    const fpv = fpvCameraFromDrone(pose.pos, pose.yaw, 4 / 3);
    expect(fpv.position[2]).toBeCloseTo(view.position[2] + 0.02, 5);
    const obs = source.observe(pose, 0);
    expect(obs.contacts.length).toBeGreaterThan(0);
    const contact = obs.contacts[0]!;
    expect(contact.buildingId).toBe(world.buildings[0]!.id);
    if (contact.truth.kind === 'tp') {
      expect(contact.truth.targetId).toBe(person.id);
      expect(Math.hypot(contact.estimate[0] - c[0], contact.estimate[1] - c[1])).toBeLessThan(1.8);
    }
  });
});
