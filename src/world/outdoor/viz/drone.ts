import * as THREE from 'three';

export interface OutdoorDrone {
  group: THREE.Group;
  rotors: THREE.Mesh[];
}

/** Small airframe. Isolated from the indoor rig so the outdoor page does not share that file. */
export function makeOutdoorDrone(): OutdoorDrone {
  const group = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.18, 0.05, 0.18),
    new THREE.MeshStandardMaterial({ color: '#1b2430', roughness: 0.42, metalness: 0.55 }),
  );
  body.castShadow = true;
  group.add(body);
  const nose = new THREE.Mesh(
    new THREE.BoxGeometry(0.07, 0.03, 0.04),
    new THREE.MeshStandardMaterial({ color: '#d6ff4a', roughness: 0.4 }),
  );
  nose.position.set(0.11, 0, 0);
  group.add(nose);
  const rotors: THREE.Mesh[] = [];
  const rotorMat = new THREE.MeshStandardMaterial({ color: '#c5ced8', roughness: 0.35, metalness: 0.35 });
  for (const [x, z] of [
    [0.1, -0.1],
    [-0.1, -0.1],
    [-0.1, 0.1],
    [0.1, 0.1],
  ] as const) {
    const arm = new THREE.Mesh(
      new THREE.BoxGeometry(0.12, 0.012, 0.02),
      new THREE.MeshStandardMaterial({ color: '#2c3644', roughness: 0.5, metalness: 0.4 }),
    );
    arm.position.set(x * 0.55, 0.012, z * 0.55);
    arm.lookAt(x, 0.012, z);
    group.add(arm);
    const rotor = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.008, 12), rotorMat);
    rotor.position.set(x, 0.03, z);
    group.add(rotor);
    rotors.push(rotor);
  }
  return { group, rotors };
}
