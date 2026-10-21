#!/usr/bin/env bash
# Fly every exported .plan in a directory on a fresh PX4 SITL and collect the reports.
#
#   PX4_BUILD=/path/to/PX4-Autopilot/build/px4_sitl_default \
#     scripts/px4/run-check.sh PLAN_DIR OUT_DIR [sih|bridge]
#
# sih:    PX4's built-in simulator (SIH, airframe 10040) flies the vehicle.
# bridge: PX4 waits for an external simulator on TCP 4560 (airframe 10016,
#         generic quad) and frontier-fly's own physics flies it, headless,
#         through scripts/px4/sim-node.ts.
#
# Each plan gets a new PX4 instance in an empty working directory, so no
# parameter or mission state carries over between plans.
set -uo pipefail

PLAN_DIR=${1:?plan directory}
OUT_DIR=${2:?output directory}
MODE=${3:-sih}
SPEED=${PX4_SIM_SPEED_FACTOR:-1}
: "${PX4_BUILD:?set PX4_BUILD to the px4_sitl_default build directory}"
PY=${PYTHON:-python3}
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)

mkdir -p "$OUT_DIR"
OUT_DIR=$(cd "$OUT_DIR" && pwd)
status=0

for plan in "$PLAN_DIR"/*.plan; do
  id=$(basename "$plan" .plan)
  work=$(mktemp -d)
  echo "=== $id ($MODE, speed x$SPEED)"
  sim_pid=""
  if [ "$MODE" = bridge ]; then
    "$ROOT/node_modules/.bin/vite-node" "$HERE/sim-node.ts" -- --speed "$SPEED" --plan "$plan" \
      --truth "$OUT_DIR/$id.truth.json" > "$OUT_DIR/$id.sim.log" 2>&1 &
    sim_pid=$!
    autostart=10016
  else
    autostart=10040
  fi
  # SIH starts at SIH_LOC_LAT0/LON0, which PX4 1.15 stores as integer 1e-7 degrees:
  # the PX4 SITL home (47.397742, 8.545594), where the export's "PX4 SITL home" preset pins the pad.
  (
    cd "$work" &&
      PX4_SYS_AUTOSTART=$autostart PX4_SIM_SPEED_FACTOR=$SPEED \
        PX4_HOME_LAT=473977420 PX4_HOME_LON=85455940 \
        exec "$PX4_BUILD/bin/px4" -d "$PX4_BUILD/etc" > "$OUT_DIR/$id.px4.log" 2>&1
  ) &
  px4_pid=$!
  if ! "$PY" "$HERE/flight_check.py" "$plan" --meta "${plan%.plan}.meta.json" --speed "$SPEED" \
    --out "$OUT_DIR/$id.json"; then
    status=1
    echo "--- PX4 log, warnings and errors"
    grep -E "WARN|ERROR" "$OUT_DIR/$id.px4.log" | tail -40
  fi
  kill "$px4_pid" 2>/dev/null
  [ -n "$sim_pid" ] && kill "$sim_pid" 2>/dev/null
  wait "$px4_pid" 2>/dev/null
  [ -n "$sim_pid" ] && wait "$sim_pid" 2>/dev/null
  rm -rf "$work"
done
exit $status
