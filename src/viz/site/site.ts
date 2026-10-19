import * as THREE from 'three';
import { Sky } from 'three/addons/objects/Sky.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { buildingAabbs } from '../../gen/parts';
import { CELL } from '../../shared/constants';
import { RNG } from '../../shared/rng';
import { skyFromTime } from '../../sim/lighting';
import { sampleHeight } from '../../world/outdoor/terrain';
import type { BuildingShell, OutdoorWorld, Solid, TreeInstance } from '../../world/outdoor/types';
import { buildProps } from '../props';
import type { MaterialSet } from '../three/materials';
import { curtainTexture, groundDetailTexture, hatchTexture, shingleTexture, sidingTexture } from './textures';

/** Weathered paint colours for exterior siding (cedar, sage, slate, oxblood). */
const SIDING_PAINTS = ['#8c6a4f', '#7d8b74', '#76828c', '#7e4a3f'];
const PINE = '#24492c';
const LEAF = '#4f6f2a';
const BUSH = '#5b6e33';

const base = import.meta.env.BASE_URL;

export interface SiteOptions {
  seed: number;
  /** Hours, 0-24. Before 6:30 or after 19:30 is a moonlit night. */
  timeOfDay: number;
  /** `low` drops shadows, the sky environment map and most far trees (software GL, phones). */
  quality: 'high' | 'low';
}

export interface SiteShell {
  /** At the shell origin; add interior extras (lights, targets) to it. */
  root: THREE.Group;
  roof: THREE.Object3D[];
  ceilings: THREE.Object3D[];
  width: number;
  depth: number;
  height: number;
}

/**
 * The outdoor search site as a finished-looking world: sky and matched fog, lit
 * terrain with paths and clearings, an instanced forest out to the horizon, the
 * geofence curtain, no-fly volumes, and opening trim. Shared by the mission film
 * and the live outdoor views. Everything it adds sits under `root`, except the
 * fog and environment map it sets on the scene.
 */
export class SiteDressing {
  readonly root = new THREE.Group();
  readonly night: boolean;
  readonly sun = new THREE.DirectionalLight('#fff4dd', 3);
  readonly hemi = new THREE.HemisphereLight('#cfe3ff', '#3b4a2e', 0.6);
  /** Site trees, for cameras that must not end up inside a canopy. */
  readonly occluders: THREE.Object3D[] = [];
  terrain!: THREE.Mesh;
  private envMap: THREE.Texture | null = null;

  constructor(
    private readonly renderer: THREE.WebGLRenderer,
    private readonly scene: THREE.Scene,
    private readonly outdoor: OutdoorWorld,
    private readonly opts: SiteOptions,
  ) {
    this.night = opts.timeOfDay < 6.5 || opts.timeOfDay > 19.5;
  }

  /** Sky, terrain, forest, fence and opening trim. Buildings are added with `buildShell`. */
  build(): void {
    this.scene.add(this.root);
    this.buildSky();
    this.buildTerrain(this.outdoor);
    this.buildFarForest(this.outdoor);
    this.buildForest(this.outdoor.trees, false);
    this.buildFence(this.outdoor);
    this.buildOpenings(this.outdoor.colliders);
    this.cheapen(this.root);
  }

  /** Ground height in sim coordinates. */
  groundAt(x: number, y: number): number {
    return sampleHeight(this.outdoor.terrain, x, y);
  }

  /** Detach and free everything this site built. */
  dispose(): void {
    this.scene.remove(this.root);
    this.root.traverse((o) => {
      const mesh = o as THREE.Mesh;
      mesh.geometry?.dispose();
      const mats = Array.isArray(mesh.material) ? mesh.material : mesh.material ? [mesh.material] : [];
      for (const m of mats) m.dispose();
    });
    if (this.scene.environment === this.envMap) this.scene.environment = null;
    this.envMap?.dispose();
    this.scene.fog = null;
  }

  private buildSky(): void {
    const high = this.opts.quality === 'high';
    const sky = skyFromTime(this.opts.timeOfDay);
    const sunDir = new THREE.Vector3(sky.sunDir[0], sky.sunDir[2], sky.sunDir[1]).normalize();
    const mid = this.outdoor.size / 2;
    if (!this.night) {
      const dome: THREE.Object3D = high ? new Sky() : gradientDome(mid, ['#5f8fbf', '#a9c6dc', '#dfe7ea']);
      // The physical sky on high quality; a painted gradient on software GL.
      if (dome instanceof Sky) {
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
        this.envMap = this.scene.environment;
        pmrem.dispose();
      }
      // Drawn after the opaque scene, so the depth test skips the sky behind terrain and trees.
      dome.renderOrder = 1;
      this.root.add(dome);
      this.scene.fog = new THREE.FogExp2('#c4d3dc', 0.0055);
      this.sun.color.setRGB(sky.sunColor[0], sky.sunColor[1], sky.sunColor[2]);
      this.sun.intensity = 2.8;
      // Without the sky environment map, the hemisphere light carries all the fill.
      this.hemi.intensity = high ? 0.45 : 1.1;
    } else {
      // Moonlit night: deep blue gradient, stars, a cool key light.
      const dome = gradientDome(mid, ['#02040b', '#0a1426', '#1b2a40']);
      dome.renderOrder = 1;
      this.root.add(dome);
      const rng = new RNG(this.opts.seed, 'stars');
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
      this.root.add(stars);
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
    this.sun.castShadow = high;
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
    this.root.add(this.sun, this.sun.target, this.hemi);
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
    this.root.add(mesh);
    this.terrain = mesh;

    // Outer apron so the horizon is forest, not the edge of the world.
    const apron = new THREE.Mesh(
      new THREE.RingGeometry(outdoor.size * 0.68, 900, 64, 1),
      new THREE.MeshStandardMaterial({ color: this.night ? '#1d2a1c' : '#4f6a35', roughness: 1 }),
    );
    apron.rotation.x = -Math.PI / 2;
    apron.position.set(outdoor.size / 2, Math.min(...terrain.heights) - 0.6, outdoor.size / 2);
    apron.receiveShadow = true;
    this.root.add(apron);
  }

  private buildFarForest(outdoor: OutdoorWorld): void {
    const rng = new RNG(outdoor.seed, 'far-forest');
    const trees: TreeInstance[] = [];
    const mid = outdoor.size / 2;
    const groundZ = Math.min(...outdoor.terrain.heights) - 0.6;
    for (let i = 0, n = this.opts.quality === 'high' ? 2600 : 320; i < n; i++) {
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
    const rng = new RNG(this.opts.seed, far ? 'far-shapes' : 'tree-shapes');
    const low = this.opts.quality !== 'high';
    const pineGeo = pineCanopy(rng, far || low ? 6 : 9);
    const leafGeo = broadleafCanopy(rng, far || low ? 0 : 1);
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
      mesh.castShadow = !far && this.opts.quality === 'high';
      mesh.receiveShadow = true;
      mesh.frustumCulled = false;
      this.root.add(mesh);
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
        opacity: this.night ? 0.35 : 0.38,
        side: THREE.DoubleSide,
        depthWrite: false,
        fog: false,
      }),
    );
    fence.renderOrder = 3;
    this.root.add(fence);
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
      this.root.add(box);
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(extrude), new THREE.LineBasicMaterial({ color: '#ff6b77', transparent: true, opacity: 0.45, fog: false }));
      edges.position.y = z0;
      this.root.add(edges);
      const floorHatch = new THREE.Mesh(
        new THREE.ShapeGeometry(shape),
        new THREE.MeshBasicMaterial({ color: '#ff4f5e', alphaMap: hatch, transparent: true, opacity: 0.5, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -3, fog: false }),
      );
      floorHatch.rotation.x = -Math.PI / 2;
      floorHatch.position.y = z0 + 0.62;
      const uv = floorHatch.geometry.getAttribute('uv');
      for (let i = 0; i < uv.count; i++) uv.setXY(i, uv.getX(i) / 3, uv.getY(i) / 3);
      this.root.add(floorHatch);
    }
  }

  /**
   * Walls, floors, door leaves, ceilings and roof of one building shell, in a group
   * placed at the shell origin (local axes: Three x = sim x, z = sim y).
   */
  buildShell(shell: BuildingShell, mats: MaterialSet): SiteShell {
    const spec = shell.spec;
    const root = new THREE.Group();
    root.position.set(shell.origin[0], shell.origin[2], shell.origin[1]);
    this.root.add(root);
    const floor0 = spec.floors[0]!;
    const W = floor0.w * spec.cellSize;
    const D = floor0.d * spec.cellSize;
    const H = spec.floorHeight;
    const top = spec.floors.length * H;
    const paint = SIDING_PAINTS[(this.opts.seed + shell.id) % SIDING_PAINTS.length]!;
    const siding = new THREE.MeshStandardMaterial({ map: sidingTexture(this.opts.seed + shell.id, paint), roughness: 0.82, metalness: 0 });
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
    const roof = this.buildRoof(shell, root, W, D, top);
    this.cheapen(root);
    return { root, roof, ceilings, width: W, depth: D, height: top };
  }

  /**
   * On low quality, swap physically based materials for Lambert ones with the
   * same colour and textures. Software GL is fill-bound, and per-pixel PBR
   * lighting cost it about four times the frame rate.
   */
  private cheapen(root: THREE.Object3D): void {
    if (this.opts.quality === 'high') return;
    root.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (!mesh.isMesh) return;
      if (Array.isArray(mesh.material)) mesh.material = mesh.material.map((m) => this.lambert(m));
      else mesh.material = this.lambert(mesh.material);
    });
  }

  private readonly lamberts = new Map<THREE.Material, THREE.Material>();

  private lambert(m: THREE.Material): THREE.Material {
    if (!(m instanceof THREE.MeshStandardMaterial)) return m;
    // Anisotropic filtering is several texture reads per pixel in software.
    if (m.map) m.map.anisotropy = 1;
    let out = this.lamberts.get(m);
    if (!out) {
      out = new THREE.MeshLambertMaterial({
        color: m.color,
        map: m.map,
        alphaMap: m.alphaMap,
        emissive: m.emissive,
        emissiveMap: m.emissiveMap,
        emissiveIntensity: m.emissiveIntensity,
        vertexColors: m.vertexColors,
        flatShading: m.flatShading,
        transparent: m.transparent,
        opacity: m.opacity,
        side: m.side,
        depthWrite: m.depthWrite,
        alphaTest: m.alphaTest,
        polygonOffset: m.polygonOffset,
        polygonOffsetFactor: m.polygonOffsetFactor,
        polygonOffsetUnits: m.polygonOffsetUnits,
      });
      this.lamberts.set(m, out);
    }
    return out;
  }

  private buildRoof(shell: BuildingShell, root: THREE.Group, W: number, D: number, top: number): THREE.Object3D[] {
    const hatch = shell.openings.some((o) => o.kind === 'roof_hatch');
    const parts: THREE.Object3D[] = [];
    const shingles = new THREE.MeshStandardMaterial({ map: shingleTexture(this.opts.seed + shell.id * 7), roughness: 0.78, side: THREE.DoubleSide });
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
      this.root.add(mesh);
    }
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
/** A sky sphere shaded from zenith to horizon, centred on the site. Cheap enough for software GL. */
function gradientDome(mid: number, [top, middle, horizon]: [string, string, string]): THREE.Mesh {
  const gradient = document.createElement('canvas');
  gradient.width = 4;
  gradient.height = 256;
  const g = gradient.getContext('2d')!;
  const grad = g.createLinearGradient(0, 0, 0, 256);
  grad.addColorStop(0, top);
  grad.addColorStop(0.55, middle);
  grad.addColorStop(1, horizon);
  g.fillStyle = grad;
  g.fillRect(0, 0, 4, 256);
  const tex = new THREE.CanvasTexture(gradient);
  tex.colorSpace = THREE.SRGBColorSpace;
  const dome = new THREE.Mesh(
    new THREE.SphereGeometry(1500, 32, 16),
    new THREE.MeshBasicMaterial({ map: tex, side: THREE.BackSide, fog: false, depthWrite: false }),
  );
  dome.position.set(mid, 0, mid);
  return dome;
}

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
    parts.push(g);
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
