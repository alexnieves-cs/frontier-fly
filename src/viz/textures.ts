import * as THREE from 'three';

function canvasTexture(draw: (ctx: CanvasRenderingContext2D, size: number) => void, size = 256): THREE.CanvasTexture {
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext('2d');
  if (!ctx) throw new Error('2d canvas unavailable');
  draw(ctx, size);
  const tex = new THREE.CanvasTexture(canvas);
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.RepeatWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

export interface SurfaceTextures {
  wall: THREE.CanvasTexture;
  wallRough: THREE.CanvasTexture;
  floor: THREE.CanvasTexture;
  floorRough: THREE.CanvasTexture;
  concrete: THREE.CanvasTexture;
  metal: THREE.CanvasTexture;
}

export function makeTextures(seed: number): SurfaceTextures {
  const wall = canvasTexture((ctx, s) => {
    ctx.fillStyle = '#e4e0d8';
    ctx.fillRect(0, 0, s, s);
    for (let i = 0; i < 1800; i++) {
      const x = (seed * 17 + i * 53) % s;
      const y = (seed * 29 + i * 97) % s;
      const shade = 210 + ((i * 13) % 30);
      ctx.fillStyle = `rgba(${shade},${shade - 4},${shade - 10},0.18)`;
      ctx.fillRect(x, y, 2, 2);
    }
    ctx.fillStyle = '#c8bba8';
    ctx.fillRect(0, s - 28, s, 28);
    ctx.fillStyle = '#8d7b68';
    ctx.fillRect(0, s - 30, s, 3);
  });
  wall.repeat.set(1.5, 1.2);
  const wallRough = wall.clone();
  wallRough.colorSpace = THREE.NoColorSpace;

  const floor = canvasTexture((ctx, s) => {
    ctx.fillStyle = '#8a643c';
    ctx.fillRect(0, 0, s, s);
    const plank = 32;
    for (let y = 0; y < s; y += plank) {
      ctx.fillStyle = y % (plank * 2) === 0 ? '#9a7348' : '#7c5a36';
      ctx.fillRect(0, y, s, plank - 2);
      ctx.strokeStyle = 'rgba(40,24,10,0.35)';
      ctx.strokeRect(0, y, s, plank - 2);
      for (let k = 0; k < 8; k++) {
        const x = ((seed + y) * 13 + k * 47) % s;
        ctx.strokeStyle = 'rgba(60,36,16,0.25)';
        ctx.beginPath();
        ctx.moveTo(x, y + 4);
        ctx.lineTo(x + 10, y + plank - 6);
        ctx.stroke();
      }
    }
  });
  floor.repeat.set(2, 2);
  const floorRough = floor.clone();
  floorRough.colorSpace = THREE.NoColorSpace;

  const concrete = canvasTexture((ctx, s) => {
    ctx.fillStyle = '#6e7378';
    ctx.fillRect(0, 0, s, s);
    for (let i = 0; i < 2000; i++) {
      ctx.fillStyle = `rgba(255,255,255,${0.02 + (i % 5) * 0.01})`;
      ctx.fillRect((i * 19) % s, (i * 47) % s, 3, 2);
    }
  });
  const metal = canvasTexture((ctx, s) => {
    const g = ctx.createLinearGradient(0, 0, s, s);
    g.addColorStop(0, '#8d97a3');
    g.addColorStop(0.5, '#d5dde6');
    g.addColorStop(1, '#6e7884');
    ctx.fillStyle = g;
    ctx.fillRect(0, 0, s, s);
    ctx.strokeStyle = 'rgba(30,36,44,0.35)';
    for (let y = 8; y < s; y += 16) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(s, y);
      ctx.stroke();
    }
  });
  return { wall, wallRough, floor, floorRough, concrete, metal };
}

export interface PbrMaps {
  color: THREE.Texture;
  rough: THREE.Texture;
  normal: THREE.Texture;
}

function loadMap(loader: THREE.TextureLoader, url: string, srgb: boolean): Promise<THREE.Texture> {
  return new Promise((resolve, reject) => {
    loader.load(
      url,
      (tex) => {
        tex.wrapS = THREE.RepeatWrapping;
        tex.wrapT = THREE.RepeatWrapping;
        tex.colorSpace = srgb ? THREE.SRGBColorSpace : THREE.NoColorSpace;
        tex.anisotropy = 8;
        resolve(tex);
      },
      undefined,
      () => reject(new Error(`texture failed: ${url}`)),
    );
  });
}

type PbrSet = Record<'wall' | 'floor' | 'concrete' | 'door', PbrMaps>;

/** Sets requested ahead of the scene that needs them, by base URL; each is handed out once. */
const prefetched = new Map<string, Promise<PbrSet>>();

/** Start downloading a set now, for a later loadPbrSet with the same base (a page busy with other work first). */
export function prefetchPbrSet(base: string): void {
  if (prefetched.has(base)) return;
  const set = fetchPbrSet(base);
  set.catch(() => {});
  prefetched.set(base, set);
}

/** CC0 ambientCG maps. Callers keep the canvas textures if a file is missing. */
export function loadPbrSet(base: string): Promise<PbrSet> {
  const early = prefetched.get(base);
  if (!early) return fetchPbrSet(base);
  prefetched.delete(base);
  return early;
}

async function fetchPbrSet(base: string): Promise<PbrSet> {
  const loader = new THREE.TextureLoader();
  const trio = async (name: string): Promise<PbrMaps> => {
    const [color, rough, normal] = await Promise.all([
      loadMap(loader, `${base}${name}_color.jpg`, true),
      loadMap(loader, `${base}${name}_rough.jpg`, false),
      loadMap(loader, `${base}${name}_normal.jpg`, false),
    ]);
    return { color, rough, normal };
  };
  const [wall, floor, concrete, door] = await Promise.all([
    trio('plaster'),
    trio('floor'),
    trio('concrete'),
    trio('door'),
  ]);
  return { wall, floor, concrete, door };
}
