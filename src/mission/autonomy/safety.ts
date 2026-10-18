/* eslint-disable @typescript-eslint/no-unused-vars */
import type { Mission } from '../types';
import type { MissionWorld, SafetyAction, SafetyEvent, SafetyInput, SafetyMonitor } from './contracts';

// STARTER: never intervenes. The safety/report workstream replaces it.
export function createSafetyMonitor(_mission: Mission, _world: MissionWorld): SafetyMonitor {
  const events: SafetyEvent[] = [];
  return {
    check(_input: SafetyInput): SafetyAction {
      return { kind: 'none', reason: '' };
    },
    events() {
      return events;
    },
  };
}
