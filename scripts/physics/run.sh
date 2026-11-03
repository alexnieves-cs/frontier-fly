#!/usr/bin/env bash
# Fly every physics-validation maneuver on PX4 SITL and on frontier-fly's own controller.
#
#   PX4_BUILD=/path/to/PX4-Autopilot/build/px4_sitl_default \
#     scripts/physics/run.sh OUT_DIR [sih|bridge|native|all] [ID...]
#
# sih:    PX4's controllers on PX4's own simulator (SIH, airframe 10040), with SIH's
#         mass, inertia, thrust and drag set to frontier-fly's mission quad.
# bridge: PX4's controllers on frontier-fly's own rigid body (QuadrotorDynamics),
#         through the lockstep bridge (scripts/px4/sim-node.ts --plant frontier).
# native: frontier-fly's own flight controller on the same rigid body, no PX4.
#
# Each PX4 flight gets a fresh PX4 in an empty working directory.
set -uo pipefail

OUT_DIR=${1:?output directory}
MODE=${2:-all}
shift 2 2>/dev/null || shift $#
ONLY=("$@")
SPEED=${PX4_SIM_SPEED_FACTOR:-1}
PY=${PYTHON:-python3}
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
VN="$ROOT/node_modules/.bin/vite-node"

mkdir -p "$OUT_DIR"
OUT_DIR=$(cd "$OUT_DIR" && pwd)
SPEC="$OUT_DIR/maneuvers.json"
"$VN" "$HERE/maneuvers.ts" -- --out "$SPEC" || exit 1
status=0

ids=$(node -e "const s=require('$SPEC');for(const m of s.maneuvers)console.log(m.id+' '+m.sims.join(','))")
while read -r id sims; do
  if [ ${#ONLY[@]} -gt 0 ] && [[ ! " ${ONLY[*]} " =~ " $id " ]]; then continue; fi
  for sim in sih bridge native; do
    [ "$MODE" != all ] && [ "$MODE" != "$sim" ] && continue
    [[ ",$sims," != *",$sim,"* ]] && continue
    echo "=== $id on $sim (x$SPEED)"
    if [ "$sim" = native ]; then
      "$VN" "$HERE/native.ts" -- --id "$id" --out "$OUT_DIR/$id.native.json" || status=1
      continue
    fi
    : "${PX4_BUILD:?set PX4_BUILD to the px4_sitl_default build directory}"
    work=$(mktemp -d)
    rm -f "$OUT_DIR/$id.$sim.plant.json" "$OUT_DIR/$id.$sim.plant.json.t0"
    sim_pid=""
    if [ "$sim" = bridge ]; then
      rig=$(node -e "const s=require('$SPEC');const m=s.maneuvers.find(x=>x.id==='$id');console.log(JSON.stringify({wind:m.wind,soc:m.soc,aero:m.aero}))")
      "$VN" "$ROOT/scripts/px4/sim-node.ts" -- --speed "$SPEED" --plant frontier --rig "$rig" \
        --log "$OUT_DIR/$id.$sim.plant.json" > "$OUT_DIR/$id.$sim.sim.log" 2>&1 &
      sim_pid=$!
      autostart=10016
    else
      autostart=10040
    fi
    (
      cd "$work" &&
        PX4_SYS_AUTOSTART=$autostart PX4_SIM_SPEED_FACTOR=$SPEED \
          PX4_HOME_LAT=473977420 PX4_HOME_LON=85455940 \
          exec "$PX4_BUILD/bin/px4" -d "$PX4_BUILD/etc" > "$OUT_DIR/$id.$sim.px4.log" 2>&1
    ) &
    px4_pid=$!
    if ! "$PY" "$HERE/fly_px4.py" "$SPEC" "$id" --sim "$sim" --speed "$SPEED" \
      --marker "$OUT_DIR/$id.$sim.plant.json.t0" --out "$OUT_DIR/$id.$sim.json"; then
      status=1
      grep -E "WARN|ERROR" "$OUT_DIR/$id.$sim.px4.log" | tail -20
    fi
    kill "$px4_pid" 2>/dev/null
    wait "$px4_pid" 2>/dev/null
    if [ -n "$sim_pid" ]; then
      sleep 1
      kill "$sim_pid" 2>/dev/null
      wait "$sim_pid" 2>/dev/null
    fi
    rm -rf "$work"
  done
done <<< "$ids"
exit $status
