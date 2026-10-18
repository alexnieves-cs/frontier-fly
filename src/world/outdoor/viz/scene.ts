import * as THREE from 'three';
import { sampleHeight } from '../terrain';
import type { OutdoorWorld, Solid } from '../types';
import { makeOutdoorDrone, type OutdoorDrone } from './drone';

export type OutdoorCamera = 'fpv' | 'chase';

const base = import.meta.env.BASE_URL;

/** Instanced forest, terrain, and shell colliders. LOD buckets follow the camera. */
export class OutdoorView {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(58, 1, 0.08, 240);
  private readonly sun: THREE.DirectionalLight;
  private readonly drone: OutdoorDrone;
  private readonly loader = new THREE.TextureLoader();
  private world: OutdoorWorld | null = null;
  private mode: OutdoorCamera = 'chase';
  private spin = 0;
  private frames = 0;
  private fps = 0;
  private lastFps = performance.now();
  private readonly buckets: InstancedBucket[] = [];
  private buildingRoot: THREE.Group | null = null;
  private terrainMesh: THREE.Mesh | null = null;
  private forestX = Number.NaN;
  private forestY = Number.NaN;

  constructor(canvas: HTMLCanvasElement) {
    const soft = softwareWebgl();
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: !soft, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(soft ? 1 : Math.min(window.devicePixelRatio || 1, 1.5));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.AgXToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    this.renderer.shadowMap.enabled = !soft;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    // Trees and shells do not move, so the shadow map is rendered once per world.
    this.renderer.shadowMap.autoUpdate = false;
    this.renderer.shadowMap.needsUpdate = !soft;
    this.scene.background = new THREE.Color('#9ec8ea');
    this.scene.fog = new THREE.Fog('#b7d4ee', 28, 92);
    this.sun = new THREE.DirectionalLight('#fff4dd', 2.6);
    this.sun.position.set(48, 62, 18);
    this.sun.castShadow = !soft;
    this.sun.shadow.mapSize.set(512, 512);
    this.sun.shadow.camera.near = 10;
    this.sun.shadow.camera.far = 140;
    this.sun.shadow.camera.left = -40;
    this.sun.shadow.camera.right = 40;
    this.sun.shadow.camera.top = 40;
    this.sun.shadow.camera.bottom = -40;
    this.sun.shadow.bias = -0.0004;
    this.scene.add(this.sun, this.sun.target);
    this.scene.add(new THREE.HemisphereLight('#d5e8ff', '#3d4a32', 0.72));
    this.scene.add(new THREE.AmbientLight('#ffffff', 0.18));
    this.drone = makeOutdoorDrone();
    this.scene.add(this.drone.group);
    this.resize();
  }

  get framesPerSecond(): number {
    return this.fps;
  }

  setWorld(world: OutdoorWorld): void {
    this.disposeWorld();
    this.world = world;
    this.terrainMesh = this.buildTerrain(world);
    this.scene.add(this.terrainMesh);
    this.buildingRoot = this.buildShells(world);
    this.scene.add(this.buildingRoot);
    this.buckets.push(...buildForest(world, this.loader));
    for (const bucket of this.buckets) this.scene.add(bucket.mesh);
    const mid = world.size / 2;
    this.sun.target.position.set(mid, sampleHeight(world.terrain, mid, mid), mid);
    this.sun.position.set(mid + 28, 64, mid + 10);
    this.sun.shadow.camera.position.copy(this.sun.position);
    this.sun.shadow.camera.lookAt(mid, 4, mid);
    this.renderer.shadowMap.needsUpdate = true;
    this.forestX = Number.NaN;
    this.forestY = Number.NaN;
    this.placeForest(world, world.spawn[0], world.spawn[1]);
  }

  setCamera(mode: OutdoorCamera): void {
    this.mode = mode;
    this.camera.fov = mode === 'fpv' ? 104 : 58;
    this.camera.updateProjectionMatrix();
  }

  update(pos: [number, number, number], yaw: number, rotors: number): void {
    const world = this.world;
    if (!world) return;
    this.drone.group.visible = this.mode !== 'fpv';
    this.drone.group.position.set(pos[0], pos[2], pos[1]);
    this.drone.group.rotation.set(0, -yaw, 0);
    this.spin += rotors;
    for (const rotor of this.drone.rotors) rotor.rotation.y = this.spin;
    if (this.mode === 'fpv') this.placeFpv(pos, yaw);
    else this.placeChase(world, pos, yaw);
    this.placeForest(world, pos[0], pos[1]);
    this.renderer.render(this.scene, this.camera);
    this.frames += 1;
    const now = performance.now();
    if (now - this.lastFps > 500) {
      this.fps = (this.frames * 1000) / (now - this.lastFps);
      this.frames = 0;
      this.lastFps = now;
    }
  }

  resize(): void {
    const canvas = this.renderer.domElement;
    const width = canvas.clientWidth || 1;
    const height = canvas.clientHeight || 1;
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  dispose(): void {
    this.disposeWorld();
    this.renderer.dispose();
  }

  private placeFpv(pos: [number, number, number], yaw: number): void {
    const nx = Math.cos(yaw);
    const ny = Math.sin(yaw);
    this.camera.position.set(pos[0] + nx * 0.12, pos[2] + 0.04, pos[1] + ny * 0.12);
    this.camera.up.set(0, 1, 0);
    this.camera.lookAt(pos[0] + nx * 8, pos[2] - 0.85, pos[1] + ny * 8);
  }

  private placeChase(world: OutdoorWorld, pos: [number, number, number], yaw: number): void {
    const nx = Math.cos(yaw);
    const ny = Math.sin(yaw);
    const x = pos[0] - nx * 8.5;
    const y = pos[1] - ny * 8.5;
    let z = pos[2] + 3.3;
    const ground = sampleHeight(world.terrain, x, y) + 0.7;
    if (z < ground) z = ground;
    this.camera.position.set(x, z, y);
    this.camera.up.set(0, 1, 0);
    this.camera.lookAt(pos[0] + nx * 4, pos[2] + 1.1, pos[1] + ny * 4);
  }

  private placeForest(world: OutdoorWorld, camX: number, camY: number): void {
    if (Math.hypot(camX - this.forestX, camY - this.forestY) < 0.05) return;
    this.forestX = camX;
    this.forestY = camY;
    for (const bucket of this.buckets) bucket.fill(world, camX, camY);
  }

  private buildTerrain(world: OutdoorWorld): THREE.Mesh {
    const n = world.terrain.resolution;
    const cell = world.size / (n - 1);
    const positions = new Float32Array(n * n * 3);
    const colors = new Float32Array(n * n * 3);
    const uvs = new Float32Array(n * n * 2);
    for (let j = 0; j < n; j++) {
      for (let i = 0; i < n; i++) {
        const k = j * n + i;
        const h = world.terrain.heights[k]!;
        positions[k * 3] = i * cell;
        positions[k * 3 + 1] = h;
        positions[k * 3 + 2] = j * cell;
        uvs[k * 2] = i / 6;
        uvs[k * 2 + 1] = j / 6;
        const slope = Math.abs((world.terrain.heights[Math.min(n - 1, j + 1) * n + i]! - h) / cell);
        const rock = Math.min(1, slope * 2.2);
        colors[k * 3] = 0.32 + rock * 0.25;
        colors[k * 3 + 1] = 0.48 - rock * 0.12;
        colors[k * 3 + 2] = 0.22 - rock * 0.08;
      }
    }
    const indices = new Uint32Array((n - 1) * (n - 1) * 6);
    let t = 0;
    for (let j = 0; j < n - 1; j++) {
      for (let i = 0; i < n - 1; i++) {
        const v00 = j * n + i;
        const v10 = v00 + 1;
        const v01 = v00 + n;
        const v11 = v01 + 1;
        indices[t++] = v00;
        indices[t++] = v10;
        indices[t++] = v01;
        indices[t++] = v01;
        indices[t++] = v10;
        indices[t++] = v11;
      }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geo.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
    geo.setIndex(new THREE.BufferAttribute(indices, 1));
    geo.computeVertexNormals();
    const texture = this.loader.load(`${base}outdoor/textures/ground-color.jpg`);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.wrapS = THREE.RepeatWrapping;
    texture.wrapT = THREE.RepeatWrapping;
    const mat = new THREE.MeshStandardMaterial({ map: texture, vertexColors: true, roughness: 0.92, metalness: 0 });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.receiveShadow = true;
    return mesh;
  }

  private buildShells(world: OutdoorWorld): THREE.Group {
    const root = new THREE.Group();
    const mats: Record<string, THREE.MeshStandardMaterial> = {
      wall: new THREE.MeshStandardMaterial({ color: '#d5cfc4', roughness: 0.78 }),
      floor: new THREE.MeshStandardMaterial({ color: '#8b6844', roughness: 0.72 }),
      roof: new THREE.MeshStandardMaterial({ color: '#3c4450', roughness: 0.58, metalness: 0.08 }),
      jamb: new THREE.MeshStandardMaterial({ color: '#241c16', roughness: 0.62 }),
      door: new THREE.MeshStandardMaterial({ color: '#7c4b2c', roughness: 0.55 }),
    };
    const geo = new THREE.BoxGeometry(1, 1, 1);
    for (const solid of world.colliders) {
      if (solid.shape !== 'cuboid') continue;
      const mat = mats[solid.tag] ?? mats.wall!;
      const mesh = new THREE.Mesh(geo, mat);
      mesh.castShadow = solid.tag !== 'floor';
      mesh.receiveShadow = true;
      placeCuboid(mesh, solid);
      root.add(mesh);
    }
    for (const shell of world.buildings) {
      const floor = shell.spec.floors[0]!;
      const light = new THREE.PointLight('#ffd7ae', 8, Math.max(floor.w, floor.d) * 1.4, 1.4);
      light.position.set(
        shell.origin[0] + (floor.w * shell.spec.cellSize) / 2,
        shell.origin[2] + 2.1,
        shell.origin[1] + (floor.d * shell.spec.cellSize) / 2,
      );
      root.add(light);
    }
    return root;
  }

  private disposeWorld(): void {
    this.terrainMesh?.geometry.dispose();
    if (this.terrainMesh) this.scene.remove(this.terrainMesh);
    if (this.buildingRoot) this.scene.remove(this.buildingRoot);
    this.terrainMesh = null;
    this.buildingRoot = null;
    for (const bucket of this.buckets) {
      this.scene.remove(bucket.mesh);
      bucket.mesh.geometry.dispose();
    }
    this.buckets.length = 0;
  }
}

function placeCuboid(mesh: THREE.Mesh, solid: Extract<Solid, { shape: 'cuboid' }>): void {
  const ax = rotateQuat(solid, [1, 0, 0]);
  const ay = rotateQuat(solid, [0, 1, 0]);
  const az = rotateQuat(solid, [0, 0, 1]);
  const basis = new THREE.Matrix4().makeBasis(
    new THREE.Vector3(ax[0], ax[2], ax[1]),
    new THREE.Vector3(az[0], az[2], az[1]),
    new THREE.Vector3(ay[0], ay[2], ay[1]),
  );
  basis.scale(new THREE.Vector3(solid.hx * 2, solid.hz * 2, solid.hy * 2));
  basis.setPosition(solid.cx, solid.cz, solid.cy);
  mesh.matrixAutoUpdate = false;
  mesh.matrix.copy(basis);
}

function rotateQuat(solid: Extract<Solid, { shape: 'cuboid' }>, v: [number, number, number]): [number, number, number] {
  const { qx: x, qy: y, qz: z, qw: w } = solid;
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx),
  ];
}

interface InstancedBucket {
  mesh: THREE.InstancedMesh;
  fill(world: OutdoorWorld, camX: number, camY: number): void;
}

function buildForest(world: OutdoorWorld, loader: THREE.TextureLoader): InstancedBucket[] {
  const bark = loader.load(`${base}outdoor/textures/bark-color.jpg`);
  bark.colorSpace = THREE.SRGBColorSpace;
  bark.wrapS = THREE.RepeatWrapping;
  bark.wrapT = THREE.RepeatWrapping;
  const trunkMat = new THREE.MeshStandardMaterial({ map: bark, color: '#c49a6c', roughness: 0.86 });
  const pineMat = new THREE.MeshStandardMaterial({ color: '#ffffff', roughness: 0.82 });
  const leafMat = new THREE.MeshStandardMaterial({ color: '#ffffff', roughness: 0.78 });
  const bushMat = new THREE.MeshStandardMaterial({ color: '#ffffff', roughness: 0.85 });
  const count = Math.max(1, world.trees.length);
  const trunk = cylinder(0.5, 1, 7);
  const pine0 = cone(1, 1, 8);
  const pine1 = cone(1, 1, 6);
  const pine2 = cone(1, 1, 4);
  const ball0 = new THREE.SphereGeometry(1, 10, 8);
  const ball1 = new THREE.SphereGeometry(1, 7, 5);
  const ball2 = new THREE.SphereGeometry(1, 5, 4);
  const buckets = [
    bucket(trunk, trunkMat, count, (tree, camX, camY) => tree.kind !== 'undergrowth' && tree.trunkHeight > 0 && treeLod(tree, camX, camY) < 2, (tree) => ({
      x: tree.x,
      y: tree.y,
      z: tree.z + tree.trunkHeight / 2,
      sx: tree.trunkRadius * 2,
      sy: tree.trunkHeight,
      sz: tree.trunkRadius * 2,
      color: [0.72, 0.55, 0.36] as [number, number, number],
    })),
    bucket(pine0, pineMat, count, (tree, camX, camY) => tree.kind === 'pine' && treeLod(tree, camX, camY) === 0, pinePose),
    bucket(pine1, pineMat, count, (tree, camX, camY) => tree.kind === 'pine' && treeLod(tree, camX, camY) === 1, pinePose),
    bucket(pine2, pineMat, count, (tree, camX, camY) => tree.kind === 'pine' && treeLod(tree, camX, camY) === 2, pinePose),
    bucket(ball0, leafMat, count, (tree, camX, camY) => tree.kind === 'broadleaf' && treeLod(tree, camX, camY) === 0, leafPose),
    bucket(ball1, leafMat, count, (tree, camX, camY) => tree.kind === 'broadleaf' && treeLod(tree, camX, camY) === 1, leafPose),
    bucket(ball2, leafMat, count, (tree, camX, camY) => tree.kind === 'broadleaf' && treeLod(tree, camX, camY) === 2, leafPose),
    bucket(ball1, bushMat, count, (tree) => tree.kind === 'undergrowth', (tree) => ({
      x: tree.x,
      y: tree.y,
      z: tree.z + tree.canopyZ,
      sx: tree.canopyRadius,
      sy: tree.canopyRadius * 0.7,
      sz: tree.canopyRadius,
      color: [0.35 + tree.tint * 0.1, 0.48, 0.22] as [number, number, number],
    })),
  ];
  return buckets;
}

function treeLod(tree: { x: number; y: number }, camX: number, camY: number): 0 | 1 | 2 {
  const dist = Math.hypot(tree.x - camX, tree.y - camY);
  if (dist < 18) return 0;
  if (dist < 36) return 1;
  return 2;
}

function pinePose(tree: OutdoorWorld['trees'][number]): Pose {
  const height = tree.canopyRadius * 2.15;
  return {
    x: tree.x,
    y: tree.y,
    z: tree.z + tree.trunkHeight * 0.46 + height / 2,
    sx: tree.canopyRadius * 0.92,
    sy: height,
    sz: tree.canopyRadius * 0.92,
    color: [0.13 + tree.tint * 0.05, 0.32 + tree.tint * 0.12, 0.16],
  };
}

function leafPose(tree: OutdoorWorld['trees'][number]): Pose {
  return {
    x: tree.x,
    y: tree.y,
    z: tree.z + tree.canopyZ,
    sx: tree.canopyRadius,
    sy: tree.canopyRadius * 0.82,
    sz: tree.canopyRadius,
    color: [0.28 + tree.tint * 0.12, 0.42 + tree.tint * 0.08, 0.16],
  };
}

interface Pose {
  x: number;
  y: number;
  z: number;
  sx: number;
  sy: number;
  sz: number;
  color: [number, number, number];
}

function bucket(
  geo: THREE.BufferGeometry,
  mat: THREE.Material,
  capacity: number,
  accept: (tree: OutdoorWorld['trees'][number], camX: number, camY: number) => boolean,
  pose: (tree: OutdoorWorld['trees'][number]) => Pose,
): InstancedBucket {
  const mesh = new THREE.InstancedMesh(geo, mat, capacity);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.count = 0;
  mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  const dummy = new THREE.Object3D();
  const color = new THREE.Color();
  return {
    mesh,
    fill(world, camX, camY) {
      let n = 0;
      for (const tree of world.trees) {
        if (!accept(tree, camX, camY)) continue;
        const item = pose(tree);
        dummy.position.set(item.x, item.z, item.y);
        dummy.rotation.set(0, tree.yaw, 0);
        dummy.scale.set(item.sx, item.sy, item.sz);
        dummy.updateMatrix();
        mesh.setMatrixAt(n, dummy.matrix);
        color.setRGB(item.color[0], item.color[1], item.color[2]);
        mesh.setColorAt(n, color);
        n += 1;
      }
      mesh.count = n;
      mesh.instanceMatrix.needsUpdate = true;
      if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    },
  };
}

function softwareWebgl(): boolean {
  const probe = document.createElement('canvas');
  const gl = probe.getContext('webgl2');
  if (!gl) return true;
  const debug = gl.getExtension('WEBGL_debug_renderer_info');
  const name = debug ? String(gl.getParameter(debug.UNMASKED_RENDERER_WEBGL)) : '';
  gl.getExtension('WEBGL_lose_context')?.loseContext();
  return /swiftshader|llvmpipe|softpipe|software/i.test(name);
}

function cylinder(radius: number, height: number, segments: number): THREE.CylinderGeometry {
  return new THREE.CylinderGeometry(radius, radius * 1.15, height, segments);
}

function cone(radius: number, height: number, segments: number): THREE.ConeGeometry {
  return new THREE.ConeGeometry(radius, height, segments);
}
