import type { Aabb, PlacedTarget, ResolvedTarget, Vec3 } from './types';

export function followPath(path: Vec3[], speed: number, time: number): { feet: Vec3; yaw: number } {
  const lengths: number[] = [];
  let total = 0;
  for (let i = 0; i < path.length; i++) {
    const a = path[i]!;
    const b = path[(i + 1) % path.length]!;
    const len = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    lengths.push(len);
    total += len;
  }
  if (total < 1e-4) return { feet: path[0]!, yaw: 0 };
  let dist = ((time * Math.max(0, speed)) % total + total) % total;
  for (let i = 0; i < path.length; i++) {
    const len = lengths[i]!;
    const a = path[i]!;
    const b = path[(i + 1) % path.length]!;
    if (dist <= len || i === path.length - 1) {
      const t = len < 1e-6 ? 0 : dist / len;
      const feet: Vec3 = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
      const yaw = Math.atan2(b[1] - a[1], b[0] - a[0]);
      return { feet, yaw };
    }
    dist -= len;
  }
  return { feet: path[0]!, yaw: 0 };
}

export function bodyAabb(feet: Vec3, yaw: number, hx: number, hy: number, height: number): Aabb {
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const lx of [-hx, hx]) {
    for (const ly of [-hy, hy]) {
      const x = feet[0] + c * lx - s * ly;
      const y = feet[1] + s * lx + c * ly;
      if (x < minX) minX = x;
      if (y < minY) minY = y;
      if (x > maxX) maxX = x;
      if (y > maxY) maxY = y;
    }
  }
  return { min: [minX, minY, feet[2]], max: [maxX, maxY, feet[2] + height] };
}

export function centerOf(aabb: Aabb): Vec3 {
  return [
    (aabb.min[0] + aabb.max[0]) * 0.5,
    (aabb.min[1] + aabb.max[1]) * 0.5,
    (aabb.min[2] + aabb.max[2]) * 0.5,
  ];
}

export function resolveTarget(target: PlacedTarget, time: number): ResolvedTarget {
  let feet = target.feet;
  let yaw = target.yaw;
  let speed = 0;
  if (target.path && target.path.length >= 2 && target.state === 'walking') {
    const followed = followPath(target.path, target.speed, time);
    feet = followed.feet;
    yaw = followed.yaw;
    speed = target.speed;
  }
  const aabb = bodyAabb(feet, yaw, target.hx, target.hy, target.height);
  return {
    id: target.id,
    classId: target.classId,
    category: target.category,
    state: target.state,
    feet,
    center: centerOf(aabb),
    yaw,
    aabb,
    speed,
    floor: target.floor,
    roomIndex: target.roomIndex,
    occluders: target.occluders,
    looksLike: target.looksLike,
  };
}

export function resolveScene(targets: PlacedTarget[], time: number): ResolvedTarget[] {
  return targets.map((target) => resolveTarget(target, time));
}
