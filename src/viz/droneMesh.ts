import * as THREE from 'three';

export interface DroneRig {
  group: THREE.Group;
  rotors: THREE.Mesh[];
  body: THREE.Mesh;
}

export function buildDrone(): DroneRig {
  const group = new THREE.Group();
  const bodyMat = new THREE.MeshStandardMaterial({ color: '#1c2430', roughness: 0.4, metalness: 0.6 });
  const armMat = new THREE.MeshStandardMaterial({ color: '#2a3342', roughness: 0.45, metalness: 0.5 });
  const rotorMat = new THREE.MeshStandardMaterial({
    color: '#9aa4b2',
    roughness: 0.35,
    metalness: 0.4,
    transparent: true,
    opacity: 0.85,
  });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.045, 0.16), bodyMat);
  body.castShadow = true;
  group.add(body);
  const nose = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.03, 0.04), new THREE.MeshStandardMaterial({ color: '#d7ff4a', roughness: 0.4 }));
  nose.position.set(0.1, 0, 0);
  group.add(nose);
  const led = new THREE.Mesh(
    new THREE.SphereGeometry(0.015, 12, 8),
    new THREE.MeshStandardMaterial({ color: '#39f3c4', emissive: '#14c8a0', emissiveIntensity: 2 }),
  );
  led.position.set(-0.06, 0.03, 0);
  group.add(led);
  const rotors: THREE.Mesh[] = [];
  const offsets: [number, number][] = [
    [0.09, -0.09],
    [-0.09, -0.09],
    [-0.09, 0.09],
    [0.09, 0.09],
  ];
  for (const [x, z] of offsets) {
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.012, 0.018), armMat);
    arm.position.set(x * 0.55, 0.01, z * 0.55);
    arm.lookAt(x, 0.01, z);
    arm.castShadow = true;
    group.add(arm);
    const rotor = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.006, 16), rotorMat);
    rotor.position.set(x, 0.028, z);
    rotor.castShadow = true;
    group.add(rotor);
    rotors.push(rotor);
  }
  return { group, rotors, body };
}
