#!/usr/bin/env bash
# PX4 SITL flies the browser page: Chromium opens px4.html, the bridge relays
# PX4's simulator link to it, and the flight check uploads and flies a plan.
#
#   PX4_BUILD=/path/to/PX4-Autopilot/build/px4_sitl_default \
#     scripts/px4/run-browser-check.sh PLAN.plan OUT_DIR
#
# The plan must come from the same woods seed the page loads (PX4_SEED,
# default 4242), so the page's terrain is the plan's terrain.
# Extra Playwright environment (browser path, CI=1) passes through.
set -uo pipefail

PLAN=${1:?plan file}
OUT_DIR=${2:?output directory}
: "${PX4_BUILD:?set PX4_BUILD to the px4_sitl_default build directory}"
PY=${PYTHON:-python3}
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
mkdir -p "$OUT_DIR" "$ROOT/test-results/px4"
OUT_DIR=$(cd "$OUT_DIR" && pwd)
id=$(basename "$PLAN" .plan)
work=$(mktemp -d)
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null; done
  wait 2>/dev/null
  rm -rf "$work"
}
trap cleanup EXIT

node "$HERE/bridge.mjs" > "$OUT_DIR/$id.bridge.log" 2>&1 &
pids+=($!)

# Playwright empties its output folder when it starts, and the default one,
# test-results/, may hold this run's plans and logs, so give it its own.
(cd "$ROOT" && PX4_LIVE=1 PX4_CHECK_DONE="$work/done" npx playwright test tests/e2e/px4.spec.ts -g "flies the browser" --project=chromium --no-deps \
  --output "$OUT_DIR/playwright") \
  > "$OUT_DIR/$id.playwright.log" 2>&1 &
pw=$!

# PX4 waits for the simulator to accept; start it once the page holds the bridge.
for _ in $(seq 1 360); do
  grep -q "page connected" "$OUT_DIR/$id.bridge.log" && break
  if ! kill -0 "$pw" 2>/dev/null; then
    echo "Playwright exited before the page connected:"
    tail -40 "$OUT_DIR/$id.playwright.log"
    exit 1
  fi
  sleep 1
done
grep -q "page connected" "$OUT_DIR/$id.bridge.log" || { echo "the page never reached the bridge"; exit 1; }

(cd "$work" && PX4_SYS_AUTOSTART=10016 exec "$PX4_BUILD/bin/px4" -d "$PX4_BUILD/etc" > "$OUT_DIR/$id.px4.log" 2>&1) &
pids+=($!)

status=0
"$PY" "$HERE/flight_check.py" "$PLAN" --meta "${PLAN%.plan}.meta.json" --ready-timeout 240 \
  --out "$OUT_DIR/$id.json" || status=1
touch "$work/done"
wait "$pw" || status=1
tail -15 "$OUT_DIR/$id.playwright.log"
exit $status
