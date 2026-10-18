import { TERRAIN_N, WORLD_SIZE } from './constants';
import { fbm, lerp, smoothstep } from './math';
import type { Clearing, Terrain } from './types';

export function createTerrain(seed: number, size = WORLD_SIZE, resolution = TERRAIN_N): Terrain {
  const heights = new Float32Array(resolution * resolution);
  const cell = size / (resolution - 1);
  for (let j = 0; j < resolution; j++) {
    for (let i = 0; i < resolution; i++) {
      const x = i * cell;
      const y = j * cell;
      const hills = fbm(x * 0.028, y * 0.028, seed);
      const slopes = fbm(x * 0.07, y * 0.07, seed + 19);
      const ridge = fbm(x * 0.014 + 4, y * 0.02, seed + 53);
      heights[j * resolution + i] = 1.4 + hills * 6.2 + slopes * 1.35 + ridge * 1.1;
    }
  }
  return { size, resolution, heights };
}

export function applyClearings(terrain: Terrain, clearings: readonly Clearing[]): void {
  const { resolution: n, size, heights } = terrain;
  const cell = size / (n - 1);
  for (let j = 0; j < n; j++) {
    for (let i = 0; i < n; i++) {
      const x = i * cell;
      const y = j * cell;
      const index = j * n + i;
      let height = heights[index]!;
      for (const clearing of clearings) {
        const dist = Math.hypot(x - clearing.x, y - clearing.y);
        if (dist >= clearing.outer) continue;
        const w = dist <= clearing.inner ? 1 : 1 - smoothstep(clearing.inner, clearing.outer, dist);
        height = lerp(height, clearing.height, w);
      }
      heights[index] = height;
    }
  }
}

/** Drop the pad a few centimeters under a building so the floor slab is the surface. */
export function dipFootprint(
  terrain: Terrain,
  x0: number,
  y0: number,
  x1: number,
  y1: number,
  height: number,
): void {
  const { resolution: n, size, heights } = terrain;
  const cell = size / (n - 1);
  for (let j = 0; j < n; j++) {
    const y = j * cell;
    if (y < y0 - cell || y > y1 + cell) continue;
    for (let i = 0; i < n; i++) {
      const x = i * cell;
      if (x < x0 || x > x1 || y < y0 || y > y1) continue;
      heights[j * n + i] = height - 0.18;
    }
  }
}

export function sampleHeight(terrain: Terrain, x: number, y: number): number {
  const n = terrain.resolution;
  const u = (Math.max(0, Math.min(terrain.size, x)) / terrain.size) * (n - 1);
  const v = (Math.max(0, Math.min(terrain.size, y)) / terrain.size) * (n - 1);
  const i = Math.min(n - 2, Math.floor(u));
  const j = Math.min(n - 2, Math.floor(v));
  const tx = u - i;
  const ty = v - j;
  const h00 = terrain.heights[j * n + i]!;
  const h10 = terrain.heights[j * n + i + 1]!;
  const h01 = terrain.heights[(j + 1) * n + i]!;
  const h11 = terrain.heights[(j + 1) * n + i + 1]!;
  return lerp(lerp(h00, h10, tx), lerp(h01, h11, tx), ty);
}

export function slopeAt(terrain: Terrain, x: number, y: number): number {
  const hx = sampleHeight(terrain, x + 0.5, y) - sampleHeight(terrain, x - 0.5, y);
  const hy = sampleHeight(terrain, x, y + 0.5) - sampleHeight(terrain, x, y - 0.5);
  return Math.hypot(hx, hy);
}

export function terrainMesh(terrain: Terrain): { vertices: Float32Array; indices: Uint32Array } {
  const n = terrain.resolution;
  const cell = terrain.size / (n - 1);
  const vertices = new Float32Array(n * n * 3);
  for (let j = 0; j < n; j++) {
    for (let i = 0; i < n; i++) {
      const k = (j * n + i) * 3;
      vertices[k] = i * cell;
      vertices[k + 1] = j * cell;
      vertices[k + 2] = terrain.heights[j * n + i]!;
    }
  }
  const quads = (n - 1) * (n - 1);
  const indices = new Uint32Array(quads * 6);
  let t = 0;
  for (let j = 0; j < n - 1; j++) {
    for (let i = 0; i < n - 1; i++) {
      const v00 = j * n + i;
      const v10 = v00 + 1;
      const v01 = v00 + n;
      const v11 = v01 + 1;
      indices[t++] = v00;
      indices[t++] = v10;
      indices[t++] = v11;
      indices[t++] = v00;
      indices[t++] = v11;
      indices[t++] = v01;
    }
  }
  return { vertices, indices };
}
