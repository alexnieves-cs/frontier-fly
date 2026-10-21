// Bakes the CC0 KayKit humanoid (public/assets/targets/humanoid.glb) into posed triangle soups for the
// software camera (src/vision). One pose per still clip and WALK_FRAMES poses across the walk cycle,
// already scaled to HUMANOID_SCALE and turned the way TargetView turns the rig at yaw 0. The mesh is
// simplified by vertex clustering so a person costs about 2k triangles to rasterize.
//
//   node scripts/vision/bake-humanoid.mjs
//
// Writes public/assets/targets/humanoid-poses.bin: a u32 header length, a JSON header, then the
// index, part and position arrays it describes. Positions are sim frame (x, y horizontal, z up),
// millimetres, feet at the origin, facing +x.
import { Buffer } from 'node:buffer';
import console from 'node:console';
import { readFileSync, writeFileSync } from 'node:fs';
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';

const SRC = 'public/assets/targets/humanoid.glb';
const OUT = 'public/assets/targets/humanoid-poses.bin';
// Mirrors src/targets/assets.ts.
const SOURCE_HEIGHT = 2.204;
const TARGET_HEIGHT = 1.75;
const SCALE = TARGET_HEIGHT / SOURCE_HEIGHT;
const CLIPS = { standing: 'Idle_A', sitting: 'Sit_Chair_Idle', lying: 'Lie_Idle', walking: 'Walking_A', hiding: 'Sit_Floor_Idle' };
const WALK_FRAMES = 6;
const CLUSTER_M = 0.03;
const PART = { Head: 0, Body: 1, Arm: 2, Leg: 3 };

// GLTFLoader looks for `self` in Node; the embedded texture fails to decode and is skipped.
globalThis.self = globalThis;
const warn = console.warn;
console.warn = (...args) => (String(args[0]).includes("Couldn't load texture") ? undefined : warn(...args));

const buf = readFileSync(SRC);
const gltf = await new GLTFLoader().parseAsync(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength), '');
const root = new THREE.Group();
const model = gltf.scene;
model.scale.setScalar(SCALE);
root.add(model);
// TargetView: rotation.y = PI/2 - yaw for the rig. Yaw 0 here; the runtime turns by +yaw about sim z.
root.rotation.set(0, Math.PI / 2, 0);

const meshes = [];
model.traverse((obj) => {
  if (!obj.isSkinnedMesh) return;
  const key = Object.keys(PART).find((k) => obj.name.includes(k));
  meshes.push({ mesh: obj, part: PART[key] ?? 1 });
});

function pose(clipName, time) {
  const clip = gltf.animations.find((c) => c.name === clipName);
  const mixer = new THREE.AnimationMixer(model);
  mixer.clipAction(clip).play();
  mixer.setTime(time);
  root.updateMatrixWorld(true);
  const out = [];
  const v = new THREE.Vector3();
  for (const { mesh, part } of meshes) {
    const pos = mesh.geometry.attributes.position;
    mesh.skeleton.update();
    for (let i = 0; i < pos.count; i++) {
      mesh.getVertexPosition(i, v);
      v.applyMatrix4(mesh.matrixWorld);
      // three (x, y up, z) -> sim (x, y = three z, z up). TargetView sets position (x, z, y).
      out.push([v.x, v.z, v.y, part]);
    }
    mesh.updateMatrixWorld();
  }
  mixer.stopAllAction();
  mixer.uncacheRoot(model);
  return out;
}

function trianglesOf() {
  const tris = [];
  let base = 0;
  for (const { mesh } of meshes) {
    const geo = mesh.geometry;
    const n = geo.attributes.position.count;
    const index = geo.index ? geo.index.array : Array.from({ length: n }, (_, i) => i);
    for (let i = 0; i < index.length; i += 3) tris.push([base + index[i], base + index[i + 1], base + index[i + 2]]);
    base += n;
  }
  return tris;
}

const poses = [
  ['standing', pose(CLIPS.standing, 0)],
  ['sitting', pose(CLIPS.sitting, 0)],
  ['lying', pose(CLIPS.lying, 0)],
  ['hiding', pose(CLIPS.hiding, 0)],
];
const walk = gltf.animations.find((c) => c.name === CLIPS.walking);
for (let k = 0; k < WALK_FRAMES; k++) poses.push([`walking${k}`, pose(CLIPS.walking, (walk.duration * k) / WALK_FRAMES)]);

// Feet at the origin: each pose's lowest vertex sits on z = 0, centered on the standing pose's x/y.
const rest = poses[0][1];
const cx = rest.reduce((a, p) => a + p[0], 0) / rest.length;
const cy = rest.reduce((a, p) => a + p[1], 0) / rest.length;
for (const [, verts] of poses) {
  const minZ = Math.min(...verts.map((p) => p[2]));
  for (const p of verts) {
    p[0] -= cx;
    p[1] -= cy;
    p[2] -= minZ;
  }
}

// Vertex clustering on the rest pose, kept per part so arms do not weld to the body.
const clusterOf = new Map();
const members = [];
const map = new Int32Array(rest.length);
rest.forEach((p, i) => {
  const key = `${p[3]}:${Math.floor(p[0] / CLUSTER_M)}:${Math.floor(p[1] / CLUSTER_M)}:${Math.floor(p[2] / CLUSTER_M)}`;
  let id = clusterOf.get(key);
  if (id === undefined) {
    id = members.length;
    clusterOf.set(key, id);
    members.push([]);
  }
  members[id].push(i);
  map[i] = id;
});
const tris = [];
const seen = new Set();
for (const [a, b, c] of trianglesOf()) {
  const t = [map[a], map[b], map[c]];
  if (t[0] === t[1] || t[1] === t[2] || t[0] === t[2]) continue;
  const key = [...t].sort((x, y) => x - y).join(',');
  if (seen.has(key)) continue;
  seen.add(key);
  tris.push(t);
}
const nVerts = members.length;
const parts = new Uint8Array(nVerts);
members.forEach((list, id) => (parts[id] = rest[list[0]][3]));
const positions = new Int16Array(poses.length * nVerts * 3);
poses.forEach(([, verts], k) => {
  members.forEach((list, id) => {
    for (let d = 0; d < 3; d++) {
      const mean = list.reduce((acc, i) => acc + verts[i][d], 0) / list.length;
      positions[(k * nVerts + id) * 3 + d] = Math.round(mean * 1000);
    }
  });
});
const indices = new Uint16Array(tris.flat());

const header = {
  source: 'KayKit Rig_Medium mannequin and clips (CC0-1.0), Kay Lousberg. See CREDITS.md.',
  poses: poses.map(([name]) => name),
  verts: nVerts,
  tris: tris.length,
  parts: ['head', 'body', 'arm', 'leg'],
  units: 'mm',
};
const headerBytes = Buffer.from(JSON.stringify(header), 'utf8');
const pad = (n) => (4 - (n % 4)) % 4;
const headPad = pad(4 + headerBytes.length);
const chunks = [
  Buffer.from(new Uint32Array([headerBytes.length + headPad]).buffer),
  headerBytes,
  Buffer.alloc(headPad, 0x20),
  Buffer.from(indices.buffer),
  Buffer.alloc(pad(indices.byteLength)),
  Buffer.from(parts.buffer),
  Buffer.alloc(pad(parts.byteLength)),
  Buffer.from(positions.buffer),
];
writeFileSync(OUT, Buffer.concat(chunks));
console.log(`${OUT}: ${poses.length} poses, ${nVerts} verts, ${tris.length} tris, ${Buffer.concat(chunks).length} bytes`);
