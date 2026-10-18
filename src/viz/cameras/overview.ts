import * as THREE from 'three';

export function placeOverview(
  camera: THREE.PerspectiveCamera,
  target: THREE.Vector3,
  theta: number,
  phi: number,
  radius: number,
): void {
  const cp = Math.cos(phi);
  camera.position.set(
    target.x + Math.cos(theta) * cp * radius,
    target.y + Math.sin(phi) * radius,
    target.z + Math.sin(theta) * cp * radius,
  );
  camera.up.set(0, 1, 0);
  camera.lookAt(target);
}
