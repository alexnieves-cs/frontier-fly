import { DECOY_IDS, OBJECT_IDS, type DecoyId, type ObjectId, type PersonState, type TargetClass } from './types';

export { DECOY_IDS, OBJECT_IDS };

export interface BodySize {
  hx: number;
  hy: number;
  height: number;
  speed: number;
}

export const PERSON_SIZE: Record<PersonState, BodySize> = {
  standing: { hx: 0.22, hy: 0.16, height: 1.75, speed: 0 },
  sitting: { hx: 0.26, hy: 0.28, height: 1.25, speed: 0 },
  lying: { hx: 0.85, hy: 0.22, height: 0.38, speed: 0 },
  walking: { hx: 0.22, hy: 0.16, height: 1.75, speed: 1.15 },
  hiding: { hx: 0.24, hy: 0.2, height: 1.15, speed: 0 },
};

export const OBJECT_SIZE: Record<ObjectId, BodySize> = {
  backpack: { hx: 0.18, hy: 0.12, height: 0.46, speed: 0 },
  phone: { hx: 0.04, hy: 0.015, height: 0.15, speed: 0 },
  toolbox: { hx: 0.23, hy: 0.12, height: 0.28, speed: 0 },
  medkit: { hx: 0.14, hy: 0.07, height: 0.22, speed: 0 },
  bottle: { hx: 0.05, hy: 0.05, height: 0.28, speed: 0 },
  radio: { hx: 0.04, hy: 0.025, height: 0.16, speed: 0 },
  helmet: { hx: 0.13, hy: 0.15, height: 0.2, speed: 0 },
  blanket: { hx: 0.4, hy: 0.28, height: 0.08, speed: 0 },
};

export const DECOY_SIZE: Record<DecoyId, BodySize> = {
  mannequin: { hx: 0.22, hy: 0.16, height: 1.75, speed: 0 },
  coat: { hx: 0.22, hy: 0.08, height: 1.15, speed: 0 },
  decoy_pack: { hx: 0.18, hy: 0.13, height: 0.42, speed: 0 },
  decoy_case: { hx: 0.22, hy: 0.14, height: 0.26, speed: 0 },
};

export const DECOY_LOOKS_LIKE: Record<DecoyId, TargetClass> = {
  mannequin: 'person',
  coat: 'person',
  decoy_pack: 'backpack',
  decoy_case: 'toolbox',
};

export function sizeOf(classId: TargetClass, state: PersonState | 'static'): BodySize {
  if (classId === 'person') return PERSON_SIZE[state === 'static' ? 'standing' : state];
  if (classId in OBJECT_SIZE) return OBJECT_SIZE[classId as ObjectId];
  return DECOY_SIZE[classId as DecoyId];
}
