import * as THREE from 'three';
import { Sky } from 'three/addons/objects/Sky.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { buildingAabbs } from '../gen/parts';
import type { MissionBuilding, MissionWorld, V3 } from '../mission/autonomy/contracts';
import { gimbalFootprint } from '../mission/autonomy/perception';
import { CELL } from '../shared/constants';
import { RNG } from '../shared/rng';
import { skyFromTime } from '../sim/lighting';
import { TargetView } from '../targets/meshes';
import { resolveScene } from '../targets/motion';
import { buildDrone, type DroneRig } from '../viz/droneMesh';
import { buildProps } from '../viz/props';
import { applyPbr, makeMaterials, type MaterialSet } from '../viz/three/materials';
import { loadPbrSet } from '../viz/textures';
import { sampleHeight } from '../world/outdoor/terrain';
import type { OutdoorWorld, Solid, TreeInstance } from '../world/outdoor/types';
import type { FlightLog } from './flight';
import { curtainTexture, glowTexture, groundDetailTexture, hatchTexture, scanGrid, shingleTexture, sidingTexture } from './textures';

/** Weathered paint colours for exterior siding (cedar, sage, slate, oxblood). */
const SIDING_PAINTS = ['#8c6a4f', '#7d8b74', '#76828c', '#7e4a3f'];

const base = import.meta.env.BASE_URL;

const PINE = '#24492c';
const LEAF = '#4f6f2a';
const BUSH = '#5b6e33';

/** Sim frame (x east, y north, z up) to Three (x, y up, z). */
export function toThree(p: V3 | [number, number, number], out = new THREE.Vector3()): THREE.Vector3 {
  return out.set(p[0], p[2], p[1]);
}

export interface FilmState {
  t: number;
  pos: V3;
  yaw: number;
  vel: V3;
  /** Frame index into the flight log. */
  index: number;
  inside: string | null;
  coverage: boolean;
}

interface BuiltShell {
  building: MissionBuilding;
  root: THREE.Group;
  roof: THREE.Object3D[];
  ceilings: THREE.Object3D[];
  targets: TargetView;
  lights: THREE.PointLight[];
}

/** Everything that draws the mission world. Owns no camera logic. */
export class FilmScene {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly night: boolean;
  /** Building meshes the director keeps the camera in front of. */
  readonly occluders: THREE.Object3D[] = [];
  private readonly world: MissionWorld;
  private readonly log: FlightLog;
  private readonly sun = new THREE.DirectionalLight('#fff4dd', 3);
  private readonly hemi = new THREE.HemisphereLight('#cfe3ff', '#3b4a2e', 0.6);
  private readonly drone: DroneRig;
  private readonly droneRoot = new THREE.Group();
  private readonly navLights: THREE.Sprite[] = [];
  private readonly shells: BuiltShell[] = [];
  private outdoorTargets: TargetView | null = null;
  private trail!: THREE.Line;
  private trailPositions!: Float32Array;
  private trailColors!: Float32Array;
  private plan!: THREE.Line;
  private planIndex = -1;
  private footprint!: THREE.Mesh;
  private coverageCanvas!: HTMLCanvasElement;
  private coverageTexture!: THREE.CanvasTexture;
  private coveragePainted = 0;
  private readonly markers = new THREE.Group();
  private markerPool: THREE.Mesh[] = [];
  private homePad!: THREE.Group;
  private spin = 0;
  private cutaway = 0;
  private time = 0;

  constructor(canvas: HTMLCanvasElement, log: FlightLog) {
    this.log = log;
    this.world = log.loaded.world;
    this.night = this.world.timeOfDay < 6.5 || this.world.timeOfDay > 19.5;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(1);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = this.night ? 1.25 : 0.9;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.drone = buildDrone();
  }

  /** Build the whole world. Resolves once textures and the humanoid rig are loaded. */
  async load(): Promise<void> {
    const outdoor = this.world.outdoor;
    this.buildSky();
    this.scene.add(this.hemi);
    this.buildTerrain(outdoor);
    this.buildFarForest(outdoor);
    this.buildForest(outdoor.trees, false);
    this.buildFence(outdoor);
    const mats = makeMaterials(this.world.seed, 'high');
    try {
      applyPbr(mats, await loadPbrSet(`${base}textures/`));
    } catch {
      /* canvas fallbacks stay */
    }
    // Under the film's brighter sun, white plaster blows out; use a warm grey.
    if (mats.wall instanceof THREE.MeshStandardMaterial) mats.wall.color.set('#c4bcb0');
    for (const building of this.world.buildings) this.shells.push(this.buildShell(building, mats));
    this.buildOpenings(outdoor.colliders);
    if (this.world.outdoorScene.targets.length) {
      this.outdoorTargets = new TargetView();
      this.scene.add(this.outdoorTargets.group);
    }
    this.buildDroneRig();
    this.buildOverlays();
    await Promise.all([...this.shells.map((s) => s.targets.wait()), this.outdoorTargets?.wait()]);
    this.syncTargets(0);
  }

  resize(width: number, height: number): void {
    this.renderer.setSize(width, height, false);
  }

  /** 0 = closed building, 1 = roof and ceilings hidden so the camera sees in. */
  setCutaway(amount: number): void {
    this.cutaway = amount;
    for (const shell of this.shells) {
      for (const roof of shell.roof) roof.visible = amount < 0.5;
      for (const ceiling of shell.ceilings) ceiling.visible = amount < 0.5;
    }
  }

  get cutawayAmount(): number {
    return this.cutaway;
  }

  update(state: FilmState): void {
    this.time = state.t;
    const p = toThree(state.pos);
    this.droneRoot.position.copy(p);
    this.droneRoot.rotation.set(0, -state.yaw, 0);
    // Bank into the velocity like a real multirotor.
    const speed = Math.hypot(state.vel[0], state.vel[1]);
    const fwd = state.vel[0] * Math.cos(state.yaw) + state.vel[1] * Math.sin(state.yaw);
    const side = -state.vel[0] * Math.sin(state.yaw) + state.vel[1] * Math.cos(state.yaw);
    this.drone.group.rotation.set(Math.max(-0.35, Math.min(0.35, side * 0.09)), 0, Math.max(-0.35, Math.min(0.35, -fwd * 0.09)));
    this.spin += 0.9 + speed * 0.1;
    for (const rotor of this.drone.rotors) rotor.rotation.y = this.spin;
    this.drone.updateWash(state.t, 0.6 + speed * 0.1);
    const blink = Math.floor(state.t * 1.5) % 2 === 0;
    const strobe = state.t % 1.2 < 0.08;
    this.navLights.forEach((sprite, i) => {
      if (i < 2) sprite.material.opacity = 0.95;
      else sprite.material.opacity = (i === 2 ? strobe : blink) ? 1 : 0.08;
    });
    this.updateTrail(state.index);
    this.updatePlan(state.index);
    // Planner paths are outdoor routes; inside they only slice through walls.
    this.plan.visible = this.plan.geometry.drawRange.count > 1 && !state.inside;
    this.updateFootprint(state);
    this.updateCoverage(state.index);
    this.updateMarkers(state);
    this.syncTargets(state.t);
  }

  render(camera: THREE.Camera): void {
    this.renderer.render(this.scene, camera);
  }

  /** Ground height in sim coordinates. */
  groundAt(x: number, y: number): number {
    return sampleHeight(this.world.outdoor.terrain, x, y);
  }

  /** Building the drone is in, or nearest building to a point. */
  buildingById(id: string): MissionBuilding | undefined {
    return this.world.buildings.find((b) => b.id === id);
  }

  // ---------------------------------------------------------------- build

  private buildSky(): void {
    const sky = skyFromTime(this.world.timeOfDay);
    const sunDir = new THREE.Vector3(sky.sunDir[0], sky.sunDir[2], sky.sunDir[1]).normalize();
    const mid = this.world.outdoor.size / 2;
    if (!this.night) {
      const dome = new Sky();
      dome.scale.setScalar(4000);
      const u = dome.material.uniforms;
      u['turbidity']!.value = 6;
      u['rayleigh']!.value = 1.6;
      u['mieCoefficient']!.value = 0.004;
      u['mieDirectionalG']!.value = 0.82;
      u['sunPosition']!.value.copy(sunDir);
      // Environment light from the same sky so the shading matches the backdrop.
      const pmrem = new THREE.PMREMGenerator(this.renderer);
      const envScene = new THREE.Scene();
      const envSky = new Sky();
      envSky.scale.setScalar(1000);
      for (const key of Object.keys(u)) envSky.material.uniforms[key]!.value = u[key]!.value;
      envScene.add(envSky);
      this.scene.environment = pmrem.fromScene(envScene, 0.04).texture;
      this.scene.environmentIntensity = 0.3;
      pmrem.dispose();
      this.scene.add(dome);
      this.scene.fog = new THREE.FogExp2('#c4d3dc', 0.0055);
      this.sun.color.setRGB(sky.sunColor[0], sky.sunColor[1], sky.sunColor[2]);
      this.sun.intensity = 2.8;
      this.hemi.intensity = 0.45;
    } else {
      // Moonlit night: deep blue gradient, stars, a cool key light.
      const gradient = document.createElement('canvas');
      gradient.width = 4;
      gradient.height = 256;
      const g = gradient.getContext('2d')!;
      const grad = g.createLinearGradient(0, 0, 0, 256);
      grad.addColorStop(0, '#02040b');
      grad.addColorStop(0.55, '#0a1426');
      grad.addColorStop(1, '#1b2a40');
      g.fillStyle = grad;
      g.fillRect(0, 0, 4, 256);
      const tex = new THREE.CanvasTexture(gradient);
      tex.colorSpace = THREE.SRGBColorSpace;
      const dome = new THREE.Mesh(
        new THREE.SphereGeometry(1500, 32, 16),
        new THREE.MeshBasicMaterial({ map: tex, side: THREE.BackSide, fog: false, depthWrite: false }),
      );
      dome.position.set(mid, 0, mid);
      this.scene.add(dome);
      const rng = new RNG(this.world.seed, 'stars');
      const starPos: number[] = [];
      for (let i = 0; i < 1600; i++) {
        const az = rng.range(0, Math.PI * 2);
        const el = Math.asin(rng.range(0.08, 1));
        starPos.push(mid + Math.cos(az) * Math.cos(el) * 1200, Math.sin(el) * 1200, mid + Math.sin(az) * Math.cos(el) * 1200);
      }
      const stars = new THREE.Points(
        new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(starPos, 3)),
        new THREE.PointsMaterial({ color: '#dfe8ff', size: 2.2, sizeAttenuation: false, fog: false, transparent: true, opacity: 0.85 }),
      );
      this.scene.add(stars);
      this.scene.fog = new THREE.FogExp2('#0c1626', 0.012);
      sunDir.set(-0.45, 0.75, 0.35).normalize();
      this.sun.color.set('#9fb8ff');
      this.sun.intensity = 1.1;
      this.hemi.color.set('#4a5f8c');
      this.hemi.groundColor.set('#141b20');
      this.hemi.intensity = 0.7;
    }
    this.sun.position.set(mid + sunDir.x * 90, sunDir.y * 90, mid + sunDir.z * 90);
    this.sun.target.position.set(mid, 0, mid);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    const cam = this.sun.shadow.camera;
    cam.left = -62;
    cam.right = 62;
    cam.top = 62;
    cam.bottom = -62;
    cam.near = 10;
    cam.far = 220;
    this.sun.shadow.bias = -0.00035;
    this.sun.shadow.normalBias = 0.04;
    this.scene.add(this.sun, this.sun.target);
  }

  private buildTerrain(outdoor: OutdoorWorld): void {
    const terrain = outdoor.terrain;
    const n = terrain.resolution;
    const cell = outdoor.size / (n - 1);
    const rng = new RNG(outdoor.seed, 'film-ground');
    const blotch: number[] = [];
    for (let i = 0; i < 24; i++) blotch.push(rng.range(0, outdoor.size), rng.range(0, outdoor.size), rng.range(5, 14), rng.range(-1, 1));
    const positions = new Float32Array(n * n * 3);
    const colors = new Float32Array(n * n * 3);
    const uvs = new Float32Array(n * n * 2);
    const uv2 = new Float32Array(n * n * 2);
    const grass = new THREE.Color('#7f9a55');
    const moss = new THREE.Color('#5d7a3c');
    const dirt = new THREE.Color('#9a8565');
    const tmp = new THREE.Color();
    for (let j = 0; j < n; j++) {
      for (let i = 0; i < n; i++) {
        const k = j * n + i;
        const x = i * cell;
        const y = j * cell;
        positions[k * 3] = x;
        positions[k * 3 + 1] = terrain.heights[k]!;
        positions[k * 3 + 2] = y;
        uvs[k * 2] = x / 4;
        uvs[k * 2 + 1] = y / 4;
        uv2[k * 2] = x / outdoor.size;
        uv2[k * 2 + 1] = y / outdoor.size;
        let wet = 0;
        for (let b = 0; b < blotch.length; b += 4) {
          const d = Math.hypot(x - blotch[b]!, y - blotch[b + 1]!) / blotch[b + 2]!;
          if (d < 1) wet += (1 - d * d) * blotch[b + 3]!;
        }
        tmp.copy(grass).lerp(moss, Math.max(0, Math.min(1, 0.5 + wet * 0.5)));
        let path = 0;
        for (const c of outdoor.corridors) {
          const d = segmentDistance(x, y, c.ax, c.ay, c.bx, c.by);
          path = Math.max(path, 1 - Math.min(1, Math.max(0, (d - c.halfWidth * 0.35) / (c.halfWidth * 0.9))));
        }
        for (const c of outdoor.clearings) {
          const d = Math.hypot(x - c.x, y - c.y);
          path = Math.max(path, 0.45 * (1 - Math.min(1, Math.max(0, (d - c.inner * 0.6) / (c.outer - c.inner * 0.6 + 1e-3)))));
        }
        tmp.lerp(dirt, path * 0.85);
        colors[k * 3] = tmp.r;
        colors[k * 3 + 1] = tmp.g;
        colors[k * 3 + 2] = tmp.b;
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
        // Counter-clockwise seen from above, so the faces point up.
        indices[t++] = v00;
        indices[t++] = v01;
        indices[t++] = v10;
        indices[t++] = v10;
        indices[t++] = v01;
        indices[t++] = v11;
      }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geo.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
    geo.setAttribute('uv1', new THREE.BufferAttribute(uv2, 2));
    geo.setIndex(new THREE.BufferAttribute(indices, 1));
    geo.computeVertexNormals();
    const loader = new THREE.TextureLoader();
    const map = loader.load(`${base}outdoor/textures/ground-color.jpg`);
    map.colorSpace = THREE.SRGBColorSpace;
    map.wrapS = THREE.RepeatWrapping;
    map.wrapT = THREE.RepeatWrapping;
    map.anisotropy = 8;
    const detail = groundDetailTexture(outdoor.seed);
    const mat = new THREE.MeshStandardMaterial({ map, vertexColors: true, roughness: 0.95, metalness: 0, color: '#e6ecd9' });
    mat.onBeforeCompile = (shader) => {
      shader.uniforms['detailMap'] = { value: detail };
      shader.fragmentShader = shader.fragmentShader
        .replace('#include <common>', '#include <common>\nuniform sampler2D detailMap;')
        .replace(
          '#include <map_fragment>',
          `#ifdef USE_MAP
            vec4 sampledDiffuseColor = texture2D( map, vMapUv );
            vec4 far = texture2D( map, vMapUv * 0.23 + 0.31 );
            sampledDiffuseColor.rgb = mix(sampledDiffuseColor.rgb, far.rgb, 0.45);
            sampledDiffuseColor.rgb *= 0.75 + 0.5 * texture2D( detailMap, vMapUv * 2.7 ).r;
            diffuseColor *= sampledDiffuseColor;
          #endif`,
        );
    };
    const mesh = new THREE.Mesh(geo, mat);
    mesh.receiveShadow = true;
    this.scene.add(mesh);

    // Coverage layer: same surface, lifted a little, alpha from a canvas the run paints.
    this.coverageCanvas = document.createElement('canvas');
    this.coverageCanvas.width = 256;
    this.coverageCanvas.height = 256;
    this.coverageTexture = new THREE.CanvasTexture(this.coverageCanvas);
    const overlayGeo = geo.clone();
    overlayGeo.setAttribute('uv', new THREE.BufferAttribute(uv2, 2));
    const overlay = new THREE.Mesh(
      overlayGeo,
      new THREE.MeshBasicMaterial({
        color: '#7ff4ff',
        map: scanGrid(outdoor.size),
        alphaMap: this.coverageTexture,
        opacity: this.night ? 0.22 : 0.55,
        transparent: true,
        depthWrite: false,
        polygonOffset: true,
        polygonOffsetFactor: -2,
        polygonOffsetUnits: -2,
        toneMapped: false,
      }),
    );
    overlay.renderOrder = 2;
    this.scene.add(overlay);

    // Outer apron so the horizon is forest, not the edge of the world.
    const apron = new THREE.Mesh(
      new THREE.RingGeometry(outdoor.size * 0.68, 900, 64, 1),
      new THREE.MeshStandardMaterial({ color: this.night ? '#1d2a1c' : '#4f6a35', roughness: 1 }),
    );
    apron.rotation.x = -Math.PI / 2;
    apron.position.set(outdoor.size / 2, Math.min(...terrain.heights) - 0.6, outdoor.size / 2);
    apron.receiveShadow = true;
    this.scene.add(apron);
  }

  private buildFarForest(outdoor: OutdoorWorld): void {
    const rng = new RNG(outdoor.seed, 'far-forest');
    const trees: TreeInstance[] = [];
    const mid = outdoor.size / 2;
    const groundZ = Math.min(...outdoor.terrain.heights) - 0.6;
    for (let i = 0; i < 2600; i++) {
      const r = outdoor.size * 0.72 + Math.pow(rng.next(), 0.7) * 170;
      const a = rng.range(0, Math.PI * 2);
      const pine = rng.bool(0.62);
      const scale = rng.range(0.8, 1.35);
      trees.push({
        x: mid + Math.cos(a) * r,
        y: mid + Math.sin(a) * r,
        z: groundZ,
        yaw: rng.range(0, Math.PI * 2),
        kind: pine ? 'pine' : 'broadleaf',
        trunkRadius: 0.22 * scale,
        trunkHeight: (pine ? 3.2 : 3.6) * scale,
        canopyRadius: (pine ? 2.1 : 2.6) * scale,
        canopyZ: (pine ? 5.2 : 5.4) * scale,
        lod: 2,
        tint: rng.next(),
      });
    }
    this.buildForest(trees, true);
  }

  private buildForest(trees: TreeInstance[], far: boolean): void {
    const loader = new THREE.TextureLoader();
    const bark = loader.load(`${base}outdoor/textures/bark-color.jpg`);
    bark.colorSpace = THREE.SRGBColorSpace;
    bark.wrapS = THREE.RepeatWrapping;
    bark.wrapT = THREE.RepeatWrapping;
    const trunkMat = new THREE.MeshStandardMaterial({ map: bark, color: '#b59775', roughness: 0.92 });
    const foliage = new THREE.MeshStandardMaterial({ color: '#ffffff', roughness: 0.86, flatShading: true });
    const rng = new RNG(this.world.seed, far ? 'far-shapes' : 'tree-shapes');
    const pineGeo = pineCanopy(rng, far ? 6 : 9);
    const leafGeo = broadleafCanopy(rng, far ? 0 : 1);
    const bushGeo = broadleafCanopy(rng, 0);
    const trunkGeo = new THREE.CylinderGeometry(0.5, 0.62, 1, far ? 5 : 8, 1);
    trunkGeo.translate(0, 0.5, 0);
    const dummy = new THREE.Object3D();
    const color = new THREE.Color();
    const make = (geo: THREE.BufferGeometry, mat: THREE.Material, list: TreeInstance[], pose: (tree: TreeInstance) => void, tint?: (tree: TreeInstance) => void) => {
      if (!list.length) return;
      const mesh = new THREE.InstancedMesh(geo, mat, list.length);
      list.forEach((tree, i) => {
        pose(tree);
        dummy.updateMatrix();
        mesh.setMatrixAt(i, dummy.matrix);
        if (tint) {
          tint(tree);
          mesh.setColorAt(i, color);
        }
      });
      mesh.castShadow = !far;
      mesh.receiveShadow = true;
      mesh.frustumCulled = false;
      this.scene.add(mesh);
      if (!far) this.occluders.push(mesh);
    };
    const night = this.night ? 0.55 : 1;
    const withTrunk = trees.filter((t) => t.kind !== 'undergrowth' && t.trunkHeight > 0);
    make(trunkGeo, trunkMat, withTrunk, (tree) => {
      dummy.position.set(tree.x, tree.z - 0.2, tree.y);
      dummy.rotation.set(0, tree.yaw, 0);
      dummy.scale.set(tree.trunkRadius * 2, tree.trunkHeight + (tree.kind === 'pine' ? tree.canopyRadius * 1.2 : 0.6), tree.trunkRadius * 2);
    });
    make(pineGeo, foliage, trees.filter((t) => t.kind === 'pine'), (tree) => {
      const height = tree.canopyRadius * 2.6;
      dummy.position.set(tree.x, tree.z + tree.trunkHeight * 0.55, tree.y);
      dummy.rotation.set(0, tree.yaw, 0);
      dummy.scale.set(tree.canopyRadius, height, tree.canopyRadius);
    }, (tree) => color.set(PINE).offsetHSL((tree.tint - 0.5) * 0.04, 0, (tree.tint - 0.5) * 0.08).multiplyScalar(night));
    make(leafGeo, foliage, trees.filter((t) => t.kind === 'broadleaf'), (tree) => {
      dummy.position.set(tree.x, tree.z + tree.canopyZ, tree.y);
      dummy.rotation.set(0, tree.yaw, 0);
      dummy.scale.set(tree.canopyRadius, tree.canopyRadius * 0.85, tree.canopyRadius);
    }, (tree) => color.set(LEAF).offsetHSL((tree.tint - 0.5) * 0.06, 0, (tree.tint - 0.5) * 0.1).multiplyScalar(night));
    make(bushGeo, foliage, trees.filter((t) => t.kind === 'undergrowth'), (tree) => {
      dummy.position.set(tree.x, tree.z + tree.canopyZ * 0.8, tree.y);
      dummy.rotation.set(0, tree.yaw, 0);
      dummy.scale.set(tree.canopyRadius, tree.canopyRadius * 0.65, tree.canopyRadius);
    }, (tree) => color.set(BUSH).offsetHSL(0, 0, (tree.tint - 0.5) * 0.08).multiplyScalar(night));
  }

  private buildFence(outdoor: OutdoorWorld): void {
    const poly = outdoor.geofence.polygon;
    const curtain = curtainTexture();
    const positions: number[] = [];
    const uvs: number[] = [];
    let run = 0;
    const height = 9;
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i]!;
      const b = poly[(i + 1) % poly.length]!;
      const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
      const za = this.groundAt(a[0], a[1]);
      const zb = this.groundAt(b[0], b[1]);
      positions.push(a[0], za, a[1], b[0], zb, b[1], b[0], zb + height, b[1], a[0], za, a[1], b[0], zb + height, b[1], a[0], za + height, a[1]);
      const u0 = run / 2;
      const u1 = (run + len) / 2;
      uvs.push(u0, 0, u1, 0, u1, 1, u0, 0, u1, 1, u0, 1);
      run += len;
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geo.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
    const fence = new THREE.Mesh(
      geo,
      new THREE.MeshBasicMaterial({
        color: '#5ef0dc',
        alphaMap: curtain,
        transparent: true,
        opacity: this.night ? 0.35 : 0.6,
        side: THREE.DoubleSide,
        depthWrite: false,
        fog: false,
      }),
    );
    fence.renderOrder = 3;
    this.scene.add(fence);
    const hatch = hatchTexture();
    for (const zone of outdoor.noFly) {
      const shape = new THREE.Shape(zone.polygon.map(([x, y]) => new THREE.Vector2(x, -y)));
      const top = Math.min(zone.maxZ, 7);
      const extrude = new THREE.ExtrudeGeometry(shape, { depth: top, bevelEnabled: false });
      extrude.rotateX(-Math.PI / 2);
      const z0 = this.groundAt(zone.polygon[0]![0], zone.polygon[0]![1]) - 0.5;
      const box = new THREE.Mesh(
        extrude,
        new THREE.MeshBasicMaterial({ color: '#ff4f5e', transparent: true, opacity: 0.07, depthWrite: false, side: THREE.DoubleSide, fog: false }),
      );
      box.position.y = z0;
      box.renderOrder = 3;
      this.scene.add(box);
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(extrude), new THREE.LineBasicMaterial({ color: '#ff6b77', transparent: true, opacity: 0.45, fog: false }));
      edges.position.y = z0;
      this.scene.add(edges);
      const floorHatch = new THREE.Mesh(
        new THREE.ShapeGeometry(shape),
        new THREE.MeshBasicMaterial({ color: '#ff4f5e', alphaMap: hatch, transparent: true, opacity: 0.5, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -3, fog: false }),
      );
      floorHatch.rotation.x = -Math.PI / 2;
      floorHatch.position.y = z0 + 0.62;
      const uv = floorHatch.geometry.getAttribute('uv');
      for (let i = 0; i < uv.count; i++) uv.setXY(i, uv.getX(i) / 3, uv.getY(i) / 3);
      this.scene.add(floorHatch);
    }
  }

  private buildShell(building: MissionBuilding, mats: MaterialSet): BuiltShell {
    const shell = building.shell;
    const spec = shell.spec;
    const root = new THREE.Group();
    root.position.set(shell.origin[0], shell.origin[2], shell.origin[1]);
    this.scene.add(root);
    this.occluders.push(root);
    const floor0 = spec.floors[0]!;
    const W = floor0.w * spec.cellSize;
    const D = floor0.d * spec.cellSize;
    const H = spec.floorHeight;
    const top = spec.floors.length * H;
    const paint = SIDING_PAINTS[(this.world.seed + shell.id) % SIDING_PAINTS.length]!;
    const siding = new THREE.MeshStandardMaterial({ map: sidingTexture(this.world.seed + shell.id, paint), roughness: 0.82, metalness: 0 });
    siding.map!.repeat.set(1 / 1.2, 1 / 1.2);
    // Section-cut wall tops in a dark poché tone so rooms read clearly from above.
    const cap = new THREE.MeshStandardMaterial({ color: this.night ? '#2a2f37' : '#454b54', roughness: 0.85 });
    const eps = 0.02;
    for (const box of buildingAabbs(spec)) {
      if (box.kind === 'slab') {
        if (box.cz > top - 0.2) continue;
        const slab = new THREE.Mesh(new THREE.BoxGeometry(box.hx * 2, 0.1, box.hy * 2), mats.floor);
        tileUv(slab.geometry as THREE.BoxGeometry, box.hx * 2, box.hy * 2, 1.5);
        slab.position.set(box.cx, box.cz, box.cy);
        slab.receiveShadow = true;
        root.add(slab);
        continue;
      }
      const inner = box.kind === 'window' ? mats.glass : mats.wall;
      // BoxGeometry groups: +x, -x, +y, -y, +z, -z (Three axes; Three z is sim y).
      const faces = [
        box.cx + box.hx >= W - eps ? siding : inner,
        box.cx - box.hx <= eps ? siding : inner,
        cap,
        inner,
        box.cy + box.hy >= D - eps ? siding : inner,
        box.cy - box.hy <= eps ? siding : inner,
      ];
      const geo = new THREE.BoxGeometry(box.hx * 2, box.hz * 2, box.hy * 2);
      tileUv(geo, box.hx * 2, box.hy * 2, 1, box.hz * 2);
      const mesh = new THREE.Mesh(geo, faces);
      mesh.position.set(box.cx, box.cz, box.cy);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      root.add(mesh);
    }
    // Door leaves on interior doorways.
    for (let f = 0; f < spec.floors.length; f++) {
      const floor = spec.floors[f]!;
      for (let z = 1; z < floor.d - 1; z++) {
        for (let x = 1; x < floor.w - 1; x++) {
          if (floor.cells[z * floor.w + x] !== CELL.DOOR) continue;
          const vertical = floor.cells[z * floor.w + x - 1] === CELL.WALL;
          const leaf = new THREE.Mesh(new THREE.BoxGeometry(0.04, H * 0.8, 0.8), mats.door);
          const cx = (x + 0.5) * spec.cellSize;
          const cy = (z + 0.5) * spec.cellSize;
          leaf.position.set(cx + (vertical ? 0 : 0.45), f * H + H * 0.4, cy + (vertical ? 0.45 : 0));
          leaf.rotation.y = vertical ? Math.PI / 2 : 0;
          leaf.castShadow = true;
          root.add(leaf);
        }
      }
    }
    const ceilings: THREE.Object3D[] = [];
    spec.floors.forEach((floor, f) => {
      const ceil = new THREE.Mesh(new THREE.BoxGeometry(floor.w * spec.cellSize - 0.1, 0.06, floor.d * spec.cellSize - 0.1), mats.ceiling);
      ceil.position.set(W / 2, (f + 1) * H - 0.04, D / 2);
      ceil.receiveShadow = true;
      ceil.castShadow = true;
      root.add(ceil);
      ceilings.push(ceil);
    });
    root.add(buildProps(spec, mats));
    // Room lights follow the scenario's per-room switch.
    const lights: THREE.PointLight[] = [];
    let index = 0;
    spec.floors.forEach((floor, f) => {
      for (const room of floor.rooms) {
        const on = building.interiorLightsOn[index++] === 1;
        if (!on) continue;
        const light = new THREE.PointLight('#ffd9b0', 6, Math.max(room.w, room.d) * spec.cellSize * 1.6, 1.6);
        light.position.set((room.x + room.w / 2) * spec.cellSize, f * H + H - 0.35, (room.z + room.d / 2) * spec.cellSize);
        root.add(light);
        lights.push(light);
        const fixture = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.18, 0.04, 16), new THREE.MeshBasicMaterial({ color: '#fff3dc' }));
        fixture.position.copy(light.position).setY(f * H + H - 0.1);
        root.add(fixture);
      }
    });
    const roof = this.buildRoof(shell, root, W, D, top);
    const targets = new TargetView();
    root.add(targets.group);
    return { building, root, roof, ceilings, targets, lights };
  }

  private buildRoof(shell: MissionBuilding['shell'], root: THREE.Group, W: number, D: number, top: number): THREE.Object3D[] {
    const hatch = shell.openings.some((o) => o.kind === 'roof_hatch');
    const parts: THREE.Object3D[] = [];
    const shingles = new THREE.MeshStandardMaterial({ map: shingleTexture(this.world.seed + shell.id * 7), roughness: 0.78, side: THREE.DoubleSide });
    if (hatch) {
      const slab = new THREE.Mesh(new THREE.BoxGeometry(W, 0.12, D), shingles);
      slab.position.set(W / 2, top + 0.06, D / 2);
      slab.castShadow = true;
      root.add(slab);
      return [slab];
    }
    const o = 0.4;
    const x0 = -o;
    const x1 = W + o;
    const z0 = -o;
    const z1 = D + o;
    const rise = Math.min(2.2, Math.min(W, D) * 0.28);
    const y0 = top + 0.06;
    const ridgeAlongX = W >= D;
    const pos: number[] = [];
    const uv: number[] = [];
    const tri = (a: number[], b: number[], c: number[]) => {
      pos.push(...a, ...b, ...c);
      for (const p of [a, b, c]) uv.push((p[0]! + p[2]!) / 3, (p[1]! - y0) * 1.4 + (ridgeAlongX ? p[2]! : p[0]!) * 0.05);
    };
    if (ridgeAlongX) {
      const inset = (z1 - z0) / 2;
      const ra = [x0 + inset, y0 + rise, (z0 + z1) / 2];
      const rb = [x1 - inset, y0 + rise, (z0 + z1) / 2];
      tri([x0, y0, z0], ra, [x1, y0, z0]);
      tri([x1, y0, z0], ra, rb);
      tri([x0, y0, z1], [x1, y0, z1], ra);
      tri([x1, y0, z1], rb, ra);
      tri([x0, y0, z0], [x0, y0, z1], ra);
      tri([x1, y0, z0], rb, [x1, y0, z1]);
    } else {
      const inset = (x1 - x0) / 2;
      const ra = [(x0 + x1) / 2, y0 + rise, z0 + inset];
      const rb = [(x0 + x1) / 2, y0 + rise, z1 - inset];
      tri([x0, y0, z0], [x0, y0, z1], ra);
      tri([x0, y0, z1], rb, ra);
      tri([x1, y0, z0], ra, [x1, y0, z1]);
      tri([x1, y0, z1], ra, rb);
      tri([x0, y0, z0], ra, [x1, y0, z0]);
      tri([x0, y0, z1], [x1, y0, z1], rb);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
    geo.computeVertexNormals();
    const roof = new THREE.Mesh(geo, shingles);
    roof.castShadow = true;
    roof.receiveShadow = true;
    root.add(roof);
    parts.push(roof);
    const fascia = new THREE.Mesh(new THREE.BoxGeometry(W + 2 * o, 0.18, D + 2 * o), new THREE.MeshStandardMaterial({ color: '#e9e2d4', roughness: 0.7 }));
    fascia.position.set(W / 2, y0 - 0.02, D / 2);
    fascia.castShadow = true;
    root.add(fascia);
    parts.push(fascia);
    return parts;
  }

  private buildOpenings(colliders: Solid[]): void {
    const trim = new THREE.MeshStandardMaterial({ color: '#f1ece2', roughness: 0.6 });
    const door = new THREE.MeshStandardMaterial({ color: '#6d4429', roughness: 0.55 });
    const geo = new THREE.BoxGeometry(1, 1, 1);
    for (const solid of colliders) {
      if (solid.shape !== 'cuboid' || (solid.tag !== 'jamb' && solid.tag !== 'door')) continue;
      const mesh = new THREE.Mesh(geo, solid.tag === 'door' ? door : trim);
      // The sim-to-Three axis swap is a reflection, so the rotation angle flips sign.
      const q = new THREE.Quaternion(-solid.qx, -solid.qz, -solid.qy, solid.qw);
      mesh.position.set(solid.cx, solid.cz, solid.cy);
      mesh.quaternion.copy(q);
      mesh.scale.set(solid.hx * 2, solid.hz * 2, solid.hy * 2);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.scene.add(mesh);
    }
  }

  private buildDroneRig(): void {
    this.drone.group.scale.setScalar(1.6);
    this.droneRoot.add(this.drone.group);
    const glow = glowTexture();
    const colors = ['#ff3344', '#33ff88', '#ffffff', '#ffffff'];
    const offsets: [number, number, number][] = [
      [-0.02, 0.02, -0.17],
      [-0.02, 0.02, 0.17],
      [-0.13, 0.05, 0],
      [0.14, -0.04, 0],
    ];
    colors.forEach((c, i) => {
      const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: glow, color: c, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, fog: false }));
      sprite.scale.setScalar((i === 2 ? 0.3 : 0.17) * (this.night ? 0.7 : 1));
      sprite.position.set(...offsets[i]!);
      this.droneRoot.add(sprite);
      this.navLights.push(sprite);
    });
    if (this.world.flashlightOn || this.night) {
      const spot = new THREE.SpotLight('#fff1d6', 40, 24, 0.42, 0.45, 1.6);
      spot.position.set(0.16, -0.02, 0);
      spot.target.position.set(4, -1.6, 0);
      spot.castShadow = true;
      spot.shadow.mapSize.set(512, 512);
      spot.shadow.camera.near = 0.2;
      this.droneRoot.add(spot, spot.target);
    }
    this.scene.add(this.droneRoot);
  }

  private buildOverlays(): void {
    const frames = this.log.frames;
    this.trailPositions = new Float32Array(frames.length * 3);
    this.trailColors = new Float32Array(frames.length * 3);
    const c = new THREE.Color();
    frames.forEach((frame, i) => {
      toThree(frame.pos).toArray(this.trailPositions, i * 3);
      c.set(phaseColor(frame.phase));
      c.toArray(this.trailColors, i * 3);
    });
    const trailGeo = new THREE.BufferGeometry();
    trailGeo.setAttribute('position', new THREE.BufferAttribute(this.trailPositions, 3));
    trailGeo.setAttribute('color', new THREE.BufferAttribute(this.trailColors, 3));
    trailGeo.setDrawRange(0, 0);
    this.trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.95, fog: false, toneMapped: false }));
    this.trail.frustumCulled = false;
    this.trail.renderOrder = 4;
    this.scene.add(this.trail);
    const planGeo = new THREE.BufferGeometry();
    planGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 512), 3));
    this.plan = new THREE.Line(planGeo, new THREE.LineDashedMaterial({ color: '#ffffff', dashSize: 0.8, gapSize: 0.6, transparent: true, opacity: 0.55, fog: false, toneMapped: false }));
    this.plan.frustumCulled = false;
    this.plan.renderOrder = 4;
    this.scene.add(this.plan);
    this.footprint = new THREE.Mesh(
      new THREE.CircleGeometry(1, 48),
      new THREE.MeshBasicMaterial({ color: '#7cf3ff', transparent: true, opacity: 0.18, depthWrite: false, fog: false, toneMapped: false }),
    );
    this.footprint.rotation.x = -Math.PI / 2;
    this.footprint.renderOrder = 3;
    this.scene.add(this.footprint);
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(0.97, 1, 64),
      new THREE.MeshBasicMaterial({ color: '#b8fbff', transparent: true, opacity: 0.7, depthWrite: false, fog: false, toneMapped: false }),
    );
    ring.position.z = 0.01;
    this.footprint.add(ring);
    this.scene.add(this.markers);
    // Home pad.
    const home = this.world.home;
    this.homePad = new THREE.Group();
    const pad = new THREE.Mesh(new THREE.CylinderGeometry(1.1, 1.1, 0.05, 40), new THREE.MeshStandardMaterial({ color: '#2a3138', roughness: 0.6 }));
    pad.receiveShadow = true;
    this.homePad.add(pad);
    const h = new THREE.Mesh(
      new THREE.RingGeometry(0.75, 0.85, 40),
      new THREE.MeshBasicMaterial({ color: '#f5c542', side: THREE.DoubleSide, toneMapped: false }),
    );
    h.rotation.x = -Math.PI / 2;
    h.position.y = 0.03;
    this.homePad.add(h);
    const letter = new THREE.Group();
    for (const [w, d, x, z] of [
      [0.08, 0.6, -0.18, 0],
      [0.08, 0.6, 0.18, 0],
      [0.36, 0.08, 0, 0],
    ] as const) {
      const bar = new THREE.Mesh(new THREE.PlaneGeometry(w, d), new THREE.MeshBasicMaterial({ color: '#f5c542', toneMapped: false }));
      bar.rotation.x = -Math.PI / 2;
      bar.position.set(x, 0.031, z);
      letter.add(bar);
    }
    this.homePad.add(letter);
    this.homePad.position.set(home[0], this.groundAt(home[0], home[1]) + 0.02, home[1]);
    this.scene.add(this.homePad);
  }

  // --------------------------------------------------------------- update

  private updateTrail(index: number): void {
    this.trail.geometry.setDrawRange(0, index + 1);
  }

  private updatePlan(index: number): void {
    const frame = this.log.frames[index]!;
    if (frame.plan === this.planIndex) return;
    this.planIndex = frame.plan;
    const plan = this.log.plans[frame.plan] ?? [];
    const attr = this.plan.geometry.getAttribute('position') as THREE.BufferAttribute;
    const n = Math.min(plan.length, attr.count);
    for (let i = 0; i < n; i++) attr.setXYZ(i, plan[i]![0], plan[i]![2], plan[i]![1]);
    attr.needsUpdate = true;
    this.plan.geometry.setDrawRange(0, n);
    this.plan.computeLineDistances();
  }

  private updateFootprint(state: FilmState): void {
    const ground = this.groundAt(state.pos[0], state.pos[1]);
    const agl = state.pos[2] - ground;
    const show = state.coverage && !state.inside && agl > 3;
    this.footprint.visible = show;
    if (!show) return;
    const fp = gimbalFootprint(agl);
    const r = Math.min(15, Math.max(fp.halfAlongM, fp.halfAcrossM));
    this.footprint.position.set(state.pos[0], ground + 0.15, state.pos[1]);
    this.footprint.scale.setScalar(r);
  }

  private updateCoverage(index: number): void {
    // Paint every outdoor coverage tick up to this frame onto the overlay canvas.
    if (index < this.coveragePainted) {
      const g = this.coverageCanvas.getContext('2d')!;
      g.clearRect(0, 0, 256, 256);
      this.coveragePainted = 0;
    }
    if (index === this.coveragePainted) return;
    const g = this.coverageCanvas.getContext('2d')!;
    const size = this.world.outdoor.size;
    const s = 256 / size;
    g.fillStyle = '#ffffff';
    for (let i = this.coveragePainted; i <= index; i += 2) {
      const frame = this.log.frames[i]!;
      if (frame.phase !== 'coverage' || frame.inside) continue;
      const ground = this.groundAt(frame.pos[0], frame.pos[1]);
      const agl = frame.pos[2] - ground;
      if (agl < 3) continue;
      const fp = gimbalFootprint(agl);
      const r = Math.min(15, Math.max(fp.halfAlongM, fp.halfAcrossM));
      g.beginPath();
      g.arc(frame.pos[0] * s, frame.pos[1] * s, r * s, 0, Math.PI * 2);
      g.fill();
    }
    this.coveragePainted = index;
    this.coverageTexture.needsUpdate = true;
  }

  private updateMarkers(state: FilmState): void {
    // Confirmed contacts: a beacon ring and a light pillar at the estimate.
    const confirmed = this.log.confirmed.filter((c) => c.tSec <= state.t);
    while (this.markerPool.length < confirmed.length) {
      const group = new THREE.Mesh(
        new THREE.RingGeometry(0.42, 0.5, 48),
        new THREE.MeshBasicMaterial({ color: '#3dff9a', transparent: true, opacity: 0.95, side: THREE.DoubleSide, depthWrite: false, depthTest: false, toneMapped: false }),
      );
      group.rotation.x = -Math.PI / 2;
      group.renderOrder = 9;
      const pillar = new THREE.Mesh(
        new THREE.CylinderGeometry(0.05, 0.05, 6, 8, 1, true),
        new THREE.MeshBasicMaterial({ color: '#3dff9a', transparent: true, opacity: 0.35, depthWrite: false, blending: THREE.AdditiveBlending, toneMapped: false }),
      );
      pillar.rotation.x = Math.PI / 2;
      pillar.position.z = -3;
      group.add(pillar);
      this.markers.add(group);
      this.markerPool.push(group);
    }
    this.markerPool.forEach((marker, i) => {
      const c = confirmed[i];
      marker.visible = Boolean(c);
      if (!c) return;
      const age = state.t - c.tSec;
      const pulse = 1 + 0.25 * Math.sin(age * 5) * Math.exp(-age * 0.3);
      marker.position.set(c.estimate[0], c.estimate[2] + 0.05, c.estimate[1]);
      marker.scale.setScalar(pulse * (c.truth.kind === 'tp' ? 1 : 0.7));
    });
  }

  private syncTargets(t: number): void {
    for (const shell of this.shells) shell.targets.sync(resolveScene(shell.building.scene.targets, t), shell.building.scene.targets, t);
    if (this.outdoorTargets) this.outdoorTargets.sync(resolveScene(this.world.outdoorScene.targets, t), this.world.outdoorScene.targets, t);
  }

  /** World-space points the director frames, in Three coordinates. */
  shellCenter(id: string): THREE.Vector3 | null {
    const building = this.buildingById(id);
    if (!building) return null;
    const spec = building.shell.spec;
    const f = spec.floors[0]!;
    return new THREE.Vector3(
      building.shell.origin[0] + (f.w * spec.cellSize) / 2,
      building.shell.origin[2] + 1.5,
      building.shell.origin[1] + (f.d * spec.cellSize) / 2,
    );
  }

  shellSize(id: string): number {
    const building = this.buildingById(id);
    if (!building) return 10;
    const f = building.shell.spec.floors[0]!;
    return Math.max(f.w, f.d) * building.shell.spec.cellSize;
  }

  get timeSec(): number {
    return this.time;
  }
}

export function phaseColor(phase: string): string {
  switch (phase) {
    case 'coverage':
      return '#56e0ff';
    case 'approach':
    case 'inspect':
      return '#ffd166';
    case 'entry_select':
    case 'await_operator':
    case 'fly_through':
      return '#ff9f43';
    case 'indoor_explore':
    case 'confirm':
      return '#3dff9a';
    case 'exit':
    case 'return_home':
    case 'land':
      return '#c3a6ff';
    default:
      return '#ffffff';
  }
}

function segmentDistance(px: number, py: number, ax: number, ay: number, bx: number, by: number): number {
  const dx = bx - ax;
  const dy = by - ay;
  const len2 = dx * dx + dy * dy || 1;
  const t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / len2));
  return Math.hypot(px - (ax + dx * t), py - (ay + dy * t));
}

function tileUv(geo: THREE.BoxGeometry, sx: number, sz: number, scale: number, sy = 1): void {
  // Per face, scale UVs to meters so textures do not stretch across long walls.
  const uv = geo.getAttribute('uv') as THREE.BufferAttribute;
  const faceDims: [number, number][] = [
    [sz, sy],
    [sz, sy],
    [sx, sz],
    [sx, sz],
    [sx, sy],
    [sx, sy],
  ];
  for (let face = 0; face < 6; face++) {
    const [u, v] = faceDims[face]!;
    for (let i = face * 4; i < face * 4 + 4; i++) uv.setXY(i, (uv.getX(i) * u) / scale, (uv.getY(i) * v) / scale);
  }
  uv.needsUpdate = true;
}

/** Stacked, jittered cones: one pine canopy, unit radius and unit height, base at y = 0. */
function pineCanopy(rng: RNG, segments: number): THREE.BufferGeometry {
  const tiers = 4;
  const parts: THREE.BufferGeometry[] = [];
  for (let i = 0; i < tiers; i++) {
    const t = i / tiers;
    const radius = 1 - t * 0.62;
    const height = 0.42 - t * 0.04;
    const cone = new THREE.ConeGeometry(radius, height, segments, 1, true);
    cone.translate(0, t * 0.62 + height / 2, 0);
    jitter(cone, rng, 0.07);
    parts.push(cone.toNonIndexed());
  }
  const merged = mergeGeometries(parts)!;
  merged.computeVertexNormals();
  return merged;
}

/** A few overlapping blobs: one broadleaf canopy of about unit radius. */
function broadleafCanopy(rng: RNG, detail: number): THREE.BufferGeometry {
  const parts: THREE.BufferGeometry[] = [];
  const blobs = 4;
  for (let i = 0; i < blobs; i++) {
    const r = i === 0 ? 0.8 : rng.range(0.45, 0.62);
    const g = new THREE.IcosahedronGeometry(r, detail);
    const a = (i / blobs) * Math.PI * 2 + rng.range(-0.4, 0.4);
    const off = i === 0 ? 0 : 0.42;
    g.translate(Math.cos(a) * off, i === 0 ? 0 : rng.range(-0.15, 0.25), Math.sin(a) * off);
    jitter(g, rng, 0.08);
    parts.push(g.toNonIndexed());
  }
  const merged = mergeGeometries(parts)!;
  merged.computeVertexNormals();
  return merged;
}

function jitter(geo: THREE.BufferGeometry, rng: RNG, amount: number): void {
  const pos = geo.getAttribute('position') as THREE.BufferAttribute;
  // Same displacement for coincident vertices so the mesh stays closed.
  const seen = new Map<string, [number, number, number]>();
  for (let i = 0; i < pos.count; i++) {
    const key = `${pos.getX(i).toFixed(4)},${pos.getY(i).toFixed(4)},${pos.getZ(i).toFixed(4)}`;
    let d = seen.get(key);
    if (!d) {
      d = [rng.range(-amount, amount), rng.range(-amount, amount) * 0.6, rng.range(-amount, amount)];
      seen.set(key, d);
    }
    pos.setXYZ(i, pos.getX(i) + d[0], pos.getY(i) + d[1], pos.getZ(i) + d[2]);
  }
}
