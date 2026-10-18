import * as THREE from 'three';

/** Drone-mounted camera, 110° FOV, nose along +X in sim, tilted down about 18°. */
export function placeFpv(
  camera: THREE.PerspectiveCamera,
  pos: THREE.Vector3,
  yaw: number,
  yawRate: number,
): void {
  const noseX = Math.cos(yaw);
  const noseY = Math.sin(yaw);
  camera.fov = 110;
  camera.position.set(pos.x + noseX * 0.1, pos.y + 0.02, pos.z + noseY * 0.1);
  camera.up.set(-noseY * yawRate * 0.03, 1, noseX * yawRate * 0.03);
  camera.lookAt(pos.x + noseX * 3, pos.y - 0.55, pos.z + noseY * 3);
  camera.updateProjectionMatrix();
}
