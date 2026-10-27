/**
 * Two on-screen thumbsticks for touch screens, producing the same 4-axis action as the keyboard
 * (sim/manual.ts actionFromKeys): [forward, left, up, yaw right], each in [-1, 1].
 * Left stick: forward/back and sideways. Right stick: up/down and turn.
 */
import './touchSticks.css';

export interface TouchSticks {
  /** Adds the sticks' deflection to `action` in place, clamped to [-1, 1]. */
  apply(action: Float32Array): Float32Array;
  readonly element: HTMLElement;
}

interface Stick {
  x: number;
  y: number;
}

export function createTouchSticks(): TouchSticks {
  const element = document.createElement('div');
  element.className = 'touch-sticks';
  element.setAttribute('aria-hidden', 'true');
  const left = makeStick(element, 'left', 'Move');
  const right = makeStick(element, 'right', 'Climb · turn');
  return {
    element,
    apply(action) {
      action[0] = clamp(action[0]! - left.y);
      action[1] = clamp(action[1]! - left.x);
      action[2] = clamp(action[2]! - right.y);
      action[3] = clamp(action[3]! + right.x);
      return action;
    },
  };
}

function makeStick(parent: HTMLElement, side: 'left' | 'right', label: string): Stick {
  const state: Stick = { x: 0, y: 0 };
  const pad = document.createElement('div');
  pad.className = `touch-stick touch-stick-${side}`;
  const knob = document.createElement('span');
  knob.className = 'touch-knob';
  const caption = document.createElement('span');
  caption.className = 'touch-label';
  caption.textContent = label;
  pad.append(knob, caption);
  parent.appendChild(pad);
  let pointer: number | null = null;
  const move = (event: PointerEvent) => {
    const r = pad.getBoundingClientRect();
    const radius = r.width / 2;
    let dx = (event.clientX - (r.left + radius)) / radius;
    let dy = (event.clientY - (r.top + radius)) / radius;
    const len = Math.hypot(dx, dy);
    if (len > 1) {
      dx /= len;
      dy /= len;
    }
    // A small dead zone so a resting thumb does not drift the drone.
    state.x = Math.abs(dx) < 0.12 ? 0 : dx;
    state.y = Math.abs(dy) < 0.12 ? 0 : dy;
    knob.style.transform = `translate(${dx * radius * 0.55}px, ${dy * radius * 0.55}px)`;
  };
  const release = () => {
    pointer = null;
    state.x = 0;
    state.y = 0;
    knob.style.transform = '';
    pad.classList.remove('active');
  };
  pad.addEventListener('pointerdown', (event) => {
    pointer = event.pointerId;
    pad.setPointerCapture(event.pointerId);
    pad.classList.add('active');
    move(event);
    event.preventDefault();
  });
  pad.addEventListener('pointermove', (event) => {
    if (event.pointerId === pointer) move(event);
  });
  pad.addEventListener('pointerup', release);
  pad.addEventListener('pointercancel', release);
  return state;
}

function clamp(v: number): number {
  return Math.max(-1, Math.min(1, v));
}
