/* eslint-disable @typescript-eslint/no-unused-vars */
import type { BatteryModel, V3 } from './contracts';

// STARTER: constant hover power. The safety/report workstream replaces it.
export function createBattery(capacityWh = 60): BatteryModel {
  let usedWh = 0;
  const hoverW = 180;
  const cruise = 5;
  return {
    capacityWh,
    powerFor(_vel: V3, _accel: V3) {
      return hoverW;
    },
    draw(powerW, dtSec) {
      usedWh += (powerW * dtSec) / 3600;
    },
    remainingPct() {
      return Math.max(0, 100 - (usedWh / capacityWh) * 100);
    },
    usedPct() {
      return (usedWh / capacityWh) * 100;
    },
    costPct(distanceM, climbM) {
      const sec = distanceM / cruise + Math.abs(climbM) / 1.5;
      return ((hoverW * sec) / 3600 / capacityWh) * 100;
    },
  };
}
