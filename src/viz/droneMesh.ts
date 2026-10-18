import * as THREE from 'three';

export interface DroneRig {
  group: THREE.Group;
  rotors: THREE.Mesh[];
  body: THREE.Mesh;
  wash: THREE.Points;
  updateWash: (time: number, thrust: number) => void;
}

/** Original quadrotor mesh, about 0.3 m across. Prop wash is a local particle system. */
export function buildDrone(): DroneRig {
  const group = new THREE.Group();
  const carbon = new THREE.MeshStandardMaterial({ color: '#1a1e24', roughness: 0.45, metalness: 0.55 });
  const plastic = new THREE.MeshStandardMaterial({ color: '#2c343f', roughness: 0.4, metalness: 0.35 });
  const alloy = new THREE.MeshStandardMaterial({ color: '#c5ced6', roughness: 0.28, metalness: 0.85 });
  const accent = new THREE.MeshStandardMaterial({ color: '#d6ff4a', roughness: 0.35, metalness: 0.1 });
  const lens = new THREE.MeshStandardMaterial({ color: '#0b1c24', roughness: 0.05, metalness: 0.2, emissive: '#14323a', emissiveIntensity: 0.4 });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.032, 0.09), carbon);
  body.castShadow = true;
  group.add(body);
  const deck = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.012, 0.05), plastic);
  deck.position.y = 0.02;
  group.add(deck);
  const cam = new THREE.Mesh(new THREE.BoxGeometry(0.028, 0.02, 0.02), plastic);
  cam.position.set(0.07, -0.01, 0);
  group.add(cam);
  const glass = new THREE.Mesh(new THREE.SphereGeometry(0.008, 12, 8), lens);
  glass.position.set(0.084, -0.01, 0);
  group.add(glass);
  const nose = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.012, 0.02), accent);
  nose.position.set(0.05, 0.018, 0);
  group.add(nose);
  const skidMat = alloy;
  for (const z of [-0.04, 0.04]) {
    const skid = new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, 0.16, 8), skidMat);
    skid.rotation.z = Math.PI / 2;
    skid.position.set(0, -0.04, z);
    group.add(skid);
    for (const x of [-0.05, 0.05]) {
      const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.035, 6), skidMat);
      leg.position.set(x, -0.022, z);
      group.add(leg);
    }
  }
  const rotors: THREE.Mesh[] = [];
  const offsets: [number, number][] = [
    [0.09, -0.09],
    [-0.09, -0.09],
    [-0.09, 0.09],
    [0.09, 0.09],
  ];
  offsets.forEach(([x, z], index) => {
    const arm = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.008, 0.13, 8), carbon);
    arm.rotation.z = Math.PI / 2;
    arm.rotation.y = Math.atan2(z, x);
    arm.position.set(x * 0.48, 0.012, z * 0.48);
    arm.castShadow = true;
    group.add(arm);
    const bell = new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.018, 0.016, 16), alloy);
    bell.position.set(x, 0.02, z);
    bell.castShadow = true;
    group.add(bell);
    const prop = new THREE.Mesh(
      new THREE.BoxGeometry(0.11, 0.004, 0.018),
      new THREE.MeshStandardMaterial({
        color: index % 2 === 0 ? '#9aa3ad' : '#d0d5db',
        roughness: 0.35,
        metalness: 0.25,
        transparent: true,
        opacity: 0.75,
      }),
    );
    prop.position.set(x, 0.032, z);
    group.add(prop);
    rotors.push(prop);
  });

  const count = 64;
  const positions = new Float32Array(count * 3);
  const washGeo = new THREE.BufferGeometry();
  washGeo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  const wash = new THREE.Points(
    washGeo,
    new THREE.PointsMaterial({ color: '#d5e6f2', size: 0.035, transparent: true, opacity: 0.28, depthWrite: false }),
  );
  group.add(wash);
  const updateWash = (time: number, thrust: number) => {
    const attr = washGeo.getAttribute('position') as THREE.BufferAttribute;
    const spread = 0.04 + thrust * 0.00015;
    for (let i = 0; i < count; i++) {
      const rotor = offsets[i % 4]!;
      const life = (time * (0.8 + thrust * 0.002) + i * 0.17) % 1;
      const ang = life * 9 + i;
      attr.setXYZ(
        i,
        rotor[0] + Math.cos(ang) * spread * (0.4 + life),
        0.02 - life * (0.18 + thrust * 0.0002),
        rotor[1] + Math.sin(ang) * spread * (0.4 + life),
      );
    }
    attr.needsUpdate = true;
    (wash.material as THREE.PointsMaterial).opacity = Math.min(0.45, 0.08 + thrust * 0.0004);
  };
  return { group, rotors, body, wash, updateWash };
}
