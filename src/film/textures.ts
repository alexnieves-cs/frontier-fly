import * as THREE from 'three';
import { RNG } from '../shared/rng';

/** Procedural canvas textures for the film. Original, MIT. */

function canvas(size: number): [HTMLCanvasElement, CanvasRenderingContext2D] {
  const c = document.createElement('canvas');
  c.width = size;
  c.height = size;
  return [c, c.getContext('2d')!];
}

function finish(c: HTMLCanvasElement, srgb = true): THREE.CanvasTexture {
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.RepeatWrapping;
  tex.anisotropy = 4;
  if (srgb) tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

/** Painted horizontal lap siding, one texture repeat per 1.2 m. */
export function sidingTexture(seed: number, base = '#c9bfae'): THREE.CanvasTexture {
  const rng = new RNG(seed, 'siding');
  const [c, g] = canvas(256);
  g.fillStyle = base;
  g.fillRect(0, 0, 256, 256);
  const boards = 8;
  const h = 256 / boards;
  for (let i = 0; i < boards; i++) {
    const y = i * h;
    const shade = rng.range(-10, 10);
    g.fillStyle = `rgba(${shade > 0 ? 255 : 0},${shade > 0 ? 255 : 0},${shade > 0 ? 255 : 0},${Math.abs(shade) / 160})`;
    g.fillRect(0, y, 256, h);
    const grad = g.createLinearGradient(0, y, 0, y + h);
    grad.addColorStop(0, 'rgba(255,255,255,0.10)');
    grad.addColorStop(0.75, 'rgba(0,0,0,0.0)');
    grad.addColorStop(0.92, 'rgba(0,0,0,0.18)');
    grad.addColorStop(1, 'rgba(0,0,0,0.42)');
    g.fillStyle = grad;
    g.fillRect(0, y, 256, h);
  }
  for (let i = 0; i < 1400; i++) {
    g.fillStyle = `rgba(60,50,40,${rng.range(0.02, 0.07)})`;
    g.fillRect(rng.range(0, 256), rng.range(0, 256), rng.range(4, 40), 1);
  }
  return finish(c);
}

/** Asphalt shingles, staggered rows. */
export function shingleTexture(seed: number, base = '#4a4f57'): THREE.CanvasTexture {
  const rng = new RNG(seed, 'shingle');
  const [c, g] = canvas(256);
  g.fillStyle = base;
  g.fillRect(0, 0, 256, 256);
  const rows = 10;
  const h = 256 / rows;
  for (let r = 0; r < rows; r++) {
    const offset = (r % 2) * 16;
    for (let x = -32 + offset; x < 256; x += 32) {
      const l = rng.range(-18, 14);
      g.fillStyle = `rgb(${74 + l},${79 + l},${87 + l})`;
      g.fillRect(x + 1, r * h + 1, 30, h - 2);
      g.fillStyle = 'rgba(0,0,0,0.35)';
      g.fillRect(x + 1, r * h + h - 3, 30, 2);
    }
  }
  for (let i = 0; i < 3000; i++) {
    g.fillStyle = `rgba(${rng.bool(0.5) ? '255,255,255' : '0,0,0'},${rng.range(0.03, 0.09)})`;
    g.fillRect(rng.range(0, 256), rng.range(0, 256), 1, 1);
  }
  return finish(c);
}

/** Low-contrast detail noise for the terrain, multiplied over vertex colors. */
export function groundDetailTexture(seed: number): THREE.CanvasTexture {
  const rng = new RNG(seed, 'ground');
  const [c, g] = canvas(512);
  g.fillStyle = '#b8b8b8';
  g.fillRect(0, 0, 512, 512);
  for (let i = 0; i < 9000; i++) {
    const v = rng.range(120, 230) | 0;
    g.fillStyle = `rgba(${v},${v},${v},${rng.range(0.15, 0.4)})`;
    const r = rng.range(0.6, 3.2);
    g.beginPath();
    g.arc(rng.range(0, 512), rng.range(0, 512), r, 0, Math.PI * 2);
    g.fill();
  }
  for (let i = 0; i < 2600; i++) {
    const v = rng.range(70, 140) | 0;
    g.strokeStyle = `rgba(${v},${v},${v},${rng.range(0.2, 0.45)})`;
    g.lineWidth = rng.range(0.6, 1.4);
    const x = rng.range(0, 512);
    const y = rng.range(0, 512);
    g.beginPath();
    g.moveTo(x, y);
    g.lineTo(x + rng.range(-3, 3), y - rng.range(3, 9));
    g.stroke();
  }
  return finish(c);
}

/** Soft round sprite for nav lights, glints and stars. */
export function glowTexture(): THREE.CanvasTexture {
  const [c, g] = canvas(64);
  const grad = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  grad.addColorStop(0, 'rgba(255,255,255,1)');
  grad.addColorStop(0.25, 'rgba(255,255,255,0.65)');
  grad.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 64);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

/** Vertical fade used by the geofence curtain: opaque at the ground, clear at the top. */
export function curtainTexture(): THREE.CanvasTexture {
  // alphaMap reads the green channel, so this is drawn in greys on black, not in alpha.
  const [c, g] = canvas(64);
  const grad = g.createLinearGradient(0, 64, 0, 0);
  grad.addColorStop(0, 'rgb(230,230,230)');
  grad.addColorStop(0.12, 'rgb(90,90,90)');
  grad.addColorStop(0.6, 'rgb(18,18,18)');
  grad.addColorStop(1, 'rgb(0,0,0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 64);
  g.globalCompositeOperation = 'lighter';
  for (let x = 0; x < 64; x += 8) {
    g.fillStyle = 'rgb(40,40,40)';
    g.fillRect(x, 0, 1, 64);
  }
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = THREE.RepeatWrapping;
  return tex;
}

/** Diagonal hazard hatching for no-fly volumes. */
export function hatchTexture(): THREE.CanvasTexture {
  const [c, g] = canvas(64);
  g.clearRect(0, 0, 64, 64);
  g.strokeStyle = 'rgba(255,255,255,1)';
  g.lineWidth = 6;
  for (let i = -64; i < 128; i += 16) {
    g.beginPath();
    g.moveTo(i, 64);
    g.lineTo(i + 64, 0);
    g.stroke();
  }
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.RepeatWrapping;
  return tex;
}

/**
 * Scanned-ground grid for the coverage overlay: a faint fill with brighter 4 m
 * cell lines, tiled across the outdoor area so swept ground reads as mapped.
 */
export function scanGrid(areaM: number, cellM = 2.5): THREE.CanvasTexture {
  const [c, g] = canvas(64);
  g.fillStyle = 'rgba(255,255,255,0.22)';
  g.fillRect(0, 0, 64, 64);
  g.fillStyle = 'rgba(255,255,255,0.9)';
  g.fillRect(0, 0, 64, 2);
  g.fillRect(0, 0, 2, 64);
  const tex = finish(c);
  tex.repeat.set(areaM / cellM, areaM / cellM);
  return tex;
}
