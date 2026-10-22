#!/usr/bin/env bash
# Fetches the Intel Research Lab (Seattle) laser log and its benchmark relations
# from the University of Freiburg SLAM benchmark, checks them, and writes the
# compact copies vendored in data/intel-lab/:
#   intel.flaser.gz  the FLASER lines of intel.clf, unchanged (front laser + odometry)
#   intel.relations  the manually verified relations, unchanged
# Licenses (stated on the benchmark's datasets page): the Radish log, CC BY 1.0;
# the relations, CC BY 3.0. See data/intel-lab/README.md and CREDITS.md.
#
#   bash scripts/loc/fetch-intel.sh [out-dir]
set -euo pipefail
out=${1:-data/intel-lab}
base=http://ais.informatik.uni-freiburg.de/slamevaluation/datasets
tmp=$(mktemp -d)
curl -fsSL -o "$tmp/intel.clf" "$base/intel.clf"
curl -fsSL -o "$tmp/intel.relations" "$base/intel.relations"
( cd "$tmp" && sha256sum -c - ) <<'SUMS'
8db0ad6231b48bad4118e6a2dc78e83213b2c306be3b4ed80a90969891abfe7b  intel.clf
330378f153eeae652d6756c1746f335801f66c3e097d90611548e6c4f593c359  intel.relations
SUMS
mkdir -p "$out"
grep '^FLASER ' "$tmp/intel.clf" | gzip -9n > "$out/intel.flaser.gz"
cp "$tmp/intel.relations" "$out/intel.relations"
rm -rf "$tmp"
ls -l "$out"
