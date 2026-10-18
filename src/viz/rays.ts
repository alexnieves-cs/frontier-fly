import * as THREE from 'three';
import { LIDAR_H, LIDAR_MAX, LIDAR_TILT } from '../shared/constants';

export function makeLidarLines(): THREE.LineSegments {
  const geom = new THREE.BufferGeometry();
  const positions = new Float32Array((LIDAR_H + LIDAR_TILT) * 2 * 3);
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  const mat = new THREE.LineBasicMaterial({ color: '#3ee0ff', transparent: true, opacity: 0.85 });
  const lines = new THREE.LineSegments(geom, mat);
  lines.frustumCulled = false;
  return lines;
}

export function updateLidarLines(
  lines: THREE.LineSegments,
  origin: THREE.Vector3,
  yaw: number,
  ranges01: ArrayLike<number> | null,
): void {
  const attr = lines.geometry.getAttribute('position') as THREE.BufferAttribute;
  const pos = attr.array as Float32Array;
  lines.visible = ranges01 != null;
  if (!ranges01) return;
  const write = (index: number, dx: number, dy: number, dz: number, range01: number) => {
    const dist = Math.max(0.05, range01) * LIDAR_MAX;
    const base = index * 6;
    pos[base] = origin.x;
    pos[base + 1] = origin.y;
    pos[base + 2] = origin.z;
    pos[base + 3] = origin.x + dx * dist;
    pos[base + 4] = origin.y + dz * dist;
    pos[base + 5] = origin.z + dy * dist;
  };
  for (let i = 0; i < LIDAR_H; i++) {
    const ang = yaw + (i / LIDAR_H) * Math.PI * 2;
    write(i, Math.cos(ang), Math.sin(ang), 0, ranges01[i] ?? 1);
  }
  for (let i = 0; i < LIDAR_TILT; i++) {
    const ang = yaw + (i / LIDAR_TILT) * Math.PI * 2;
    const elev = i % 2 === 0 ? 0.45 : -0.45;
    const ch = Math.cos(elev);
    write(LIDAR_H + i, Math.cos(ang) * ch, Math.sin(ang) * ch, Math.sin(elev), ranges01[LIDAR_H + i] ?? 1);
  }
  attr.needsUpdate = true;
}
