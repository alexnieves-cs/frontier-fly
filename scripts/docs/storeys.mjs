// Multi-floor round 2 (docs/storeys-study.json, scripts/study/storeys-study.ts): the tables of
// docs/MULTI_FLOOR.md, the REPORT.md note and the GUIDE limits line.

const pct = (v) => `${(v * 100).toFixed(1)}%`;
const ci = (c) => `${(c[0] * 100).toFixed(1)}–${(c[1] * 100).toFixed(1)}`;
const NAMES = {
  planner: 'Sensor-only planner',
  'planner-view': 'Planner, view first',
  boustrophedon: 'Boustrophedon (BA*)',
  nbvp: 'NBVP (Bircher et al. 2016)',
  yamauchi: 'Nearest frontier (Yamauchi 1997)',
};
const SETS = { generated: 'generated multi-floor buildings', homes: 'real multi-storey homes' };

function find(cells, pilot, budget, set) {
  return cells.find((c) => c.pilot === pilot && c.budget === budget && c.set === set);
}

function pilotsOf(study) {
  return [...new Set(study.cells.map((c) => c.pilot))];
}

/** One table per building set: flown and seen coverage under each budget, plus what the battery flight cost. */
export function storeysTables(study) {
  if (!study) return '_Not measured yet._';
  const out = [];
  for (const set of ['generated', 'homes']) {
    const any = study.cells.find((c) => c.set === set);
    if (!any) continue;
    out.push(`**${any.flights} ${SETS[set]}**`, '');
    out.push('| Pilot | 120 s: flown | 120 s: seen | 120 s per floor: flown | 120 s per floor: seen | Battery: flown | Battery: seen (95% CI) | Battery: minutes, share of pack | Every floor reached (battery) | Flights with a contact (battery) |');
    out.push('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |');
    for (const pilot of pilotsOf(study)) {
      const b = find(study.cells, pilot, 'bench', set);
      const p = find(study.cells, pilot, 'per-floor', set);
      const y = find(study.cells, pilot, 'battery', set);
      if (!b || !p || !y) continue;
      out.push(
        `| ${NAMES[pilot] ?? pilot} | ${pct(b.coverage)} | ${pct(b.footprint)} | ${pct(p.coverage)} | ${pct(p.footprint)} | ${pct(y.coverage)} | ${pct(y.footprint)} (${ci(y.footprintCi)}) | ${(y.seconds / 60).toFixed(1)} min, ${y.batteryPct.toFixed(0)}% | ${y.allFloorsReached} of ${y.flights} | ${y.flightsWithCollision} |`,
      );
    }
    out.push('');
  }
  return out.join('\n').trim();
}

/** Main's code on the same buildings: what the shaft search and the faster climb changed. */
export function storeysControlTable(study) {
  if (!study?.control) return '_No paired control in this run._';
  const out = ['| Pilot | Buildings | 120 s: seen, main → this | 120 s: every floor reached, main → this | Battery: seen, main → this | Battery: every floor reached, main → this |', '| --- | --- | --- | --- | --- | --- |'];
  for (const set of ['generated', 'homes']) {
    for (const pilot of pilotsOf(study)) {
      const nb = find(study.cells, pilot, 'bench', set);
      const mb = find(study.control.cells, pilot, 'bench', set);
      const ny = find(study.cells, pilot, 'battery', set);
      const my = find(study.control.cells, pilot, 'battery', set);
      if (!nb || !mb || !ny || !my) continue;
      out.push(
        `| ${NAMES[pilot] ?? pilot} | ${set === 'generated' ? 'Generated' : 'Real homes'} | ${pct(mb.footprint)} → ${pct(nb.footprint)} | ${mb.allFloorsReached} → ${nb.allFloorsReached} of ${nb.flights} | ${pct(my.footprint)} → ${pct(ny.footprint)} | ${my.allFloorsReached} → ${ny.allFloorsReached} of ${ny.flights} |`,
      );
    }
  }
  return out.join('\n');
}

/** The REPORT.md bullet under "Climbing and map prediction". */
export function storeysNote(study) {
  if (!study) return '';
  const g = (pilot, budget) => find(study.cells, pilot, budget, 'generated');
  const h = (pilot, budget) => find(study.cells, pilot, budget, 'homes');
  const pb = g('planner', 'battery');
  const hb = h('planner', 'battery');
  const vb = g('planner-view', 'bench');
  const hv = h('planner-view', 'bench');
  if (!pb || !hb || !vb || !hv) return '';
  return `- **On a real battery the multi-floor number is not 40%.** [MULTI_FLOOR.md](MULTI_FLOOR.md) flies ${pb.flights} fresh generated multi-floor buildings and ${hb.flights} real two- and three-storey homes from Swiss Dwellings (their own staircases as shafts) under three budgets. Given the airframe's own pack (explore until 70% of the usable energy is spent) the planner flies ${pct(pb.coverage)} of the generated buildings and ${pct(hb.coverage)} of the real homes (the env stops a flight at 95%), reaching every floor in ${pb.allFloorsReached} of ${pb.flights} and ${hb.allFloorsReached} of ${hb.flights}, in ${(pb.seconds / 60).toFixed(1)} and ${(hb.seconds / 60).toFixed(1)} minutes on ${pb.batteryPct.toFixed(0)}% and ${hb.batteryPct.toFixed(0)}% of the pack. Inside the 120 s budget the view-first planner already brings ${pct(vb.footprint)} and ${pct(hv.footprint)} of them within 3 m of its sensor. The climber now searches for stairs when a floor is done, which mostly helps the published methods: they map a floor from its doorways and used to finish it without ever flying under the shaft.`;
}

/** The GUIDE "Honest limits" sentence. */
export function storeysLimit(study) {
  if (!study) return '';
  const pb = find(study.cells, 'planner', 'battery', 'generated');
  const hb = find(study.cells, 'planner', 'battery', 'homes');
  if (!pb || !hb) return '';
  return `- That multi-floor number is the 120 s budget, not the pilot: on the airframe's battery the planner flies ${pct(pb.coverage)} of fresh multi-floor buildings and ${pct(hb.coverage)} of ${hb.flights} real two- and three-storey homes, every floor reached, on about ${Math.round((pb.batteryPct + hb.batteryPct) / 2)}% of the pack ([MULTI_FLOOR.md](MULTI_FLOOR.md)). The real homes' stairs are flown as open shafts: the stair flight itself is not modelled.`;
}
