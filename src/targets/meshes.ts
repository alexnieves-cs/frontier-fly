import * as THREE from 'three';
import { GLTFLoader, type GLTF } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { clone as cloneSkeleton } from 'three/examples/jsm/utils/SkeletonUtils.js';
import { HUMANOID, HUMANOID_SCALE } from './assets';
import { JOINT_NAMES, samplePersonMotion } from './skeleton';
import type { PersonState, PlacedTarget, ResolvedTarget, Vec3 } from './types';

const SHIRT: Record<PersonState, string> = {
  standing: '#e2b08a',
  sitting: '#4c8dff',
  lying: '#e25b4a',
  walking: '#3cba6a',
  hiding: '#6d7c8a',
};

interface Recorded {
  root: THREE.Group;
  mixer: THREE.AnimationMixer | null;
  clip: string;
  rigged: boolean;
}

function placeRoot(feet: Vec3, yaw: number, obj: THREE.Object3D, rigged: boolean): void {
  obj.position.set(feet[0], feet[2], feet[1]);
  obj.rotation.set(0, rigged ? Math.PI / 2 - yaw : -yaw, 0);
}

function propMaterial(classId: string): THREE.MeshStandardMaterial {
  const color = classId === 'medkit'
    ? '#f4f7fb'
    : classId === 'phone'
      ? '#2a3344'
      : classId === 'toolbox' || classId === 'decoy_case'
        ? '#d4893a'
        : classId === 'backpack' || classId === 'decoy_pack'
          ? '#3d6b4f'
          : classId === 'bottle'
            ? '#7fd0ea'
            : classId === 'radio'
              ? '#243044'
              : classId === 'helmet'
                ? '#f0d36a'
                : classId === 'blanket'
                  ? '#c46a62'
                  : classId === 'coat'
                    ? '#c4a574'
                    : '#cfd3d8';
  return new THREE.MeshStandardMaterial({ color, roughness: 0.62, metalness: 0.04 });
}

function addCrate(parent: THREE.Object3D, box: { min: Vec3; max: Vec3 }): void {
  const crate = new THREE.Mesh(
    new THREE.BoxGeometry(box.max[0] - box.min[0], box.max[2] - box.min[2], box.max[1] - box.min[1]),
    new THREE.MeshStandardMaterial({ color: '#8a5a32', roughness: 0.82 }),
  );
  crate.position.set(
    (box.min[0] + box.max[0]) / 2,
    (box.min[2] + box.max[2]) / 2,
    (box.min[1] + box.max[1]) / 2,
  );
  crate.castShadow = true;
  parent.add(crate);
}

function stickFigure(person: PlacedTarget, time: number): THREE.Group {
  const group = new THREE.Group();
  const motion = samplePersonMotion(person, time);
  const state = person.state === 'static' ? 'standing' : person.state;
  const mat = new THREE.MeshStandardMaterial({ color: SHIRT[state], roughness: 0.55 });
  const geo = new THREE.SphereGeometry(0.055, 8, 8);
  for (const name of JOINT_NAMES) {
    const joint = motion?.joints.find((item) => item.name === name);
    if (!joint) continue;
    const ball = new THREE.Mesh(geo, mat);
    ball.position.set(joint.position[0], joint.position[2], joint.position[1]);
    group.add(ball);
  }
  return group;
}

/**
 * Draws placed people and objects. People prefer the CC0 KayKit rig and
 * fall back to the analytic skeleton when the file has not loaded.
 */
export class TargetView {
  readonly group = new THREE.Group();
  loaded = false;
  private gltf: GLTF | null = null;
  private readonly records = new Map<string, Recorded>();
  private readonly crates = new THREE.Group();
  private readonly loading: Promise<void>;
  private placed = new Map<string, PlacedTarget>();

  constructor() {
    this.group.add(this.crates);
    this.loading = this.load();
  }

  wait(): Promise<void> {
    return this.loading;
  }

  private async load(): Promise<void> {
    try {
      this.gltf = await new GLTFLoader().loadAsync(HUMANOID.url);
    } catch (error) {
      console.warn('humanoid rig unavailable', error);
      this.gltf = null;
    }
    this.loaded = true;
    for (const record of this.records.values()) this.group.remove(record.root);
    this.records.clear();
  }

  sync(resolved: ResolvedTarget[], placed: PlacedTarget[], time: number): void {
    this.placed = new Map(placed.map((item) => [item.id, item]));
    const live = new Set(resolved.map((item) => item.id));
    for (const [id, record] of this.records) {
      if (live.has(id)) continue;
      this.group.remove(record.root);
      this.records.delete(id);
    }
    this.crates.clear();
    for (const target of resolved) {
      const clip = this.clipName(target);
      const rigged = Boolean(this.gltf && clip);
      let record = this.records.get(target.id);
      if (!record || record.clip !== clip || record.rigged !== rigged) {
        if (record) this.group.remove(record.root);
        record = this.make(target, clip, rigged, time);
        this.records.set(target.id, record);
        this.group.add(record.root);
      }
      this.place(record, target, time);
      for (const box of target.occluders) addCrate(this.crates, box);
    }
  }

  private clipName(target: ResolvedTarget): string {
    if (target.classId === 'mannequin') return HUMANOID.clips.standing;
    if (target.classId === 'person' && target.state !== 'static') return HUMANOID.clips[target.state];
    return '';
  }

  private clipOf(name: string): THREE.AnimationClip | null {
    return this.gltf?.animations.find((clip) => clip.name === name) ?? null;
  }

  private make(target: ResolvedTarget, clipName: string, rigged: boolean, time: number): Recorded {
    const root = new THREE.Group();
    let mixer: THREE.AnimationMixer | null = null;
    if (rigged && this.gltf && clipName) {
      const model = cloneSkeleton(this.gltf.scene);
      model.scale.setScalar(HUMANOID_SCALE);
      const plastic = target.classId === 'mannequin';
      const state = target.state === 'static' ? 'standing' : target.state;
      const tint = plastic ? '#d5dae1' : SHIRT[state];
      model.traverse((obj: THREE.Object3D) => {
        const mesh = obj as THREE.Mesh;
        if (!mesh.isMesh) return;
        const source = Array.isArray(mesh.material) ? mesh.material[0] : mesh.material;
        if (!source) return;
        const mat = source.clone() as THREE.MeshStandardMaterial;
        if (mat.color) mat.color.set(tint);
        if (plastic) {
          mat.map = null;
          mat.roughness = 0.32;
        }
        mesh.material = mat;
        mesh.castShadow = true;
        mesh.frustumCulled = false;
      });
      const chest = model.getObjectByName('chest');
      if (chest && !plastic) {
        const shirt = new THREE.Mesh(
          new THREE.BoxGeometry(0.42, 0.22, 0.26),
          new THREE.MeshStandardMaterial({ color: tint, roughness: 0.7 }),
        );
        shirt.position.set(0, 0.02, 0.06);
        chest.add(shirt);
      }
      root.add(model);
      const clip = this.clipOf(clipName);
      if (clip) {
        mixer = new THREE.AnimationMixer(model);
        const action = mixer.clipAction(clip);
        action.play();
        mixer.setTime(time % Math.max(clip.duration, 0.01));
      }
    } else if (target.classId === 'person') {
      const source = this.placed.get(target.id);
      if (source) root.add(stickFigure(source, time));
    } else {
      const source = this.placed.get(target.id);
      const hx = source?.hx ?? 0.2;
      const hy = source?.hy ?? 0.2;
      const height = source?.height ?? 0.4;
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(hx * 2, height, hy * 2), propMaterial(target.classId));
      mesh.position.y = height / 2;
      mesh.castShadow = true;
      root.add(mesh);
      if (target.classId === 'medkit') {
        const cross = new THREE.Mesh(
          new THREE.BoxGeometry(Math.min(0.09, hx), 0.03, Math.min(0.16, hy * 2)),
          new THREE.MeshStandardMaterial({ color: '#d64545' }),
        );
        cross.position.y = height + 0.02;
        root.add(cross);
      }
      // A real pack has a dark top flap and straps and a toolbox has a handle; their look-alike decoys
      // do not. The drone camera (src/vision/scene.ts) draws the same details.
      const detail = new THREE.MeshStandardMaterial({ color: '#1d2a22', roughness: 0.7 });
      if (target.classId === 'backpack') {
        const flap = new THREE.Mesh(new THREE.BoxGeometry(hx * 2.12, height * 0.28, hy * 2.12), detail);
        flap.position.y = height * 0.86;
        root.add(flap);
        for (const side of [-0.5, 0.5]) {
          const strap = new THREE.Mesh(new THREE.BoxGeometry(hx * 2.16, height * 0.95, 0.05), detail);
          strap.position.set(0, height / 2, side * hy * 1.2);
          root.add(strap);
        }
      } else if (target.classId === 'toolbox') {
        const handle = new THREE.Mesh(new THREE.BoxGeometry(hx * 1.2, 0.08, 0.04), detail);
        handle.position.y = height + 0.04;
        root.add(handle);
      }
    }
    if (target.classId === 'person' && target.state === 'sitting') {
      const chair = new THREE.Mesh(
        new THREE.BoxGeometry(0.48, 0.42, 0.46),
        new THREE.MeshStandardMaterial({ color: '#6b4630', roughness: 0.8 }),
      );
      chair.position.y = 0.21;
      root.add(chair);
    }
    return { root, mixer, clip: clipName, rigged };
  }

  private place(record: Recorded, target: ResolvedTarget, time: number): void {
    if (record.rigged || target.classId !== 'person') placeRoot(target.feet, target.yaw, record.root, record.rigged);
    else {
      record.root.position.set(0, 0, 0);
      record.root.rotation.set(0, 0, 0);
      const source = this.placed.get(target.id);
      record.root.clear();
      if (source) record.root.add(stickFigure(source, time));
    }
    if (!record.mixer || !record.clip) return;
    const clip = this.clipOf(record.clip);
    if (!clip) return;
    const action = record.mixer.clipAction(clip);
    action.play();
    const phase = hashPhase(target.id);
    record.mixer.setTime((time + phase) % Math.max(clip.duration, 0.01));
  }
}

function hashPhase(id: string): number {
  let h = 0;
  for (let i = 0; i < id.length; i++) h = (h * 33 + id.charCodeAt(i)) >>> 0;
  return (h % 1000) / 700;
}
