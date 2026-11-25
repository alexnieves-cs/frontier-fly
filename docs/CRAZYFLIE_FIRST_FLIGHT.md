# Crazyflie first flight: kit and checklist

This is the hands-on page for the first real flight of frontier-fly's Crazyflie stack. [CRAZYFLIE.md](CRAZYFLIE.md) has the plan, the simulation results and the flight ladder; this page is the kit, what each part is for, and a checklist to print and tick in the room.

Nothing here has flown on hardware yet. The software side (the link check, the flight script, the flight log and the comparison report) is tested against the simulator and a simulated radio in CI.

## Kit

Prices and where to buy are in [CRAZYFLIE.md, "What to buy"](CRAZYFLIE.md#what-to-buy) (retail listings found on 2026-11-02). Bitcraze's STEM ranging bundle has the first four rows.

| Part | What it does here | First needed at |
| --- | --- | --- |
| Crazyflie 2.1+ (brushed) | The drone. Its own firmware stabilises it and flies the velocity setpoints `fly.py` sends at 20 Hz. The brushed version is lighter and hits a wall with less energy than the Brushless. | step 1 |
| Flow deck v2 | A downward camera and range sensor: the drone's height and its x, y position estimate. Without it the Crazyflie cannot hold a position indoors. Its drift is what `compare.py --landed` measures. | step 1 |
| Multi-ranger deck | Five laser range sensors (front, back, left, right, up) out to about 4 m. The explorer maps the room with the four horizontal ones, and the guard uses them to stop the drone flying into what it can see. | step 1 |
| Crazyradio 2.0 | The USB radio on the laptop. Telemetry comes down at 20 Hz and commands go up. `bringup.py scan` finds it and `bringup.py link` measures it. | step 1 |
| 2 to 3 spare 250 mAh 1S LiPo batteries and a 1S charger | About 5 minutes in the air per battery with both decks. Fly one ladder step per battery. `fly.py` will not take off under 3.9 V at rest, and the guard lands at 3.4 V under load. | step 2 |
| Spare propellers and motor mounts | They break on a touch. A chipped prop shakes the drone and spoils the flow estimate, so swap it. | step 2 |
| Prop guards (optional) | Less damage from a touch. Check the take-off weight stays under the 42 g limit with both decks. | step 2 |
| Safety glasses | Props at eye height when you crouch to place or pick up the drone. | step 2 |
| A fireproof surface or LiPo bag for charging | Where the batteries charge. | before step 2 |
| Tape measure and masking tape | Measure the room for the arena file, mark the take-off spot and heading, and mark where it lands. | before step 0 |
| Laptop | Node 22, Python 3.10 or newer and a free USB port, with `npm ci` and `pip install -r scripts/crazyflie/requirements.txt` (cflib 0.1.34) done. | step 0 |

## Checklist

Print this. One step of the ladder per battery, and only move up when the step below passed its rule in [CRAZYFLIE.md, "Flight ladder"](CRAZYFLIE.md#flight-ladder).

### Before the day

- [ ] Firmware updated to the latest release with `cfclient`, and the radio URI written down (for example `radio://0/80/2M/E7E7E7E7E7`).
- [ ] On Linux, the Crazyradio udev rules are installed.
- [ ] The room is measured and written as an arena file, and `npm run crazyflie:arena -- my-room.json` says ok.
- [ ] The rehearsal passes in that file: `python scripts/crazyflie/fly.py --arena my-room.json --mode explore --sim`, then the same with `--mock-radio drop=0.05,latency=0.02`.
- [ ] Batteries charged on the fireproof surface, with someone watching.

### Step 1: bring-up, props off

- [ ] **Props off.** Battery in, drone on the floor at the take-off mark, nose along the arena file's heading.
- [ ] `python scripts/crazyflie/bringup.py scan` lists the drone's URI.
- [ ] `python scripts/crazyflie/bringup.py link --uri <yours>` ends with `LINK OK`. It listens for 20 s and never starts the motors. It needs both decks, fresh telemetry on 98% of samples, no gap over 0.25 s and, when cflib reports them, a round trip under 50 ms and a link quality of 90 or more.
- [ ] `python scripts/crazyflie/fly.py --arena my-room.json --mode check --uri <yours>`: both decks found; each ranger reads what the tape says to within 5 cm; turning the drone left by hand makes the yaw go up; pushing it forward moves x and y the way the arena file says.
- [ ] Props on, each one matched to its motor arm as in the Bitcraze assembly guide, pressed fully on and not chipped.

### Before every flight (steps 2 to 6)

- [ ] Nobody else in the room, pets out, door closed, nothing fragile inside the arena.
- [ ] The floor has texture under the whole flight area, and the light is even with no sun patches.
- [ ] Safety glasses on.
- [ ] A fresh battery, at least 3.9 V at rest.
- [ ] The drone on the take-off mark, nose along the arena heading.
- [ ] You stand outside the arena with the laptop, a finger on Ctrl-C, before pressing Enter.

In hover, yaw and square, `fly.py` flies under tighter limits than explore: 0.3 m/s at most, height setpoints up to 0.6 m, a landing if it measures over 0.9 m, the motors cut over 1.5 m, and 40 s (hover, yaw) or 60 s (square) in the air.

### During a flight

- **Ctrl-C once: it lands. Ctrl-C twice: the motors cut** and it drops from where it is. From 0.4 m that is fine.
- **Never catch it or reach for it.** Let it land, or cut it.
- If it drifts toward you or out of the arena, press Ctrl-C, twice if it is close.
- If it flips, the guard cuts the motors. If it hits something and the props keep turning, press Ctrl-C twice. Wait until the props have stopped before you touch it, then unplug the battery.
- After any surprise, stop for that battery. Do not retry straight away.

### After every flight

- [ ] Unplug the battery. If it is puffed or hot, retire it: do not charge it or fly it again.
- [ ] Mark where it landed with tape and measure x and y from the arena's corner, in the arena file's frame.
- [ ] `python scripts/crazyflie/compare.py flights/<the log>.jsonl --landed X,Y` and read "Where they disagree" in the report it writes next to the log.
- [ ] Write anything odd (a noise, a wobble, a beam that read wrong) in a note next to the log.
- [ ] Move up a step only when this step's pass rule held.

### Battery care

- Charge only with a 1S charger made for these cells, on the fireproof surface, never unattended and never overnight.
- Never charge or fly a battery that is puffed, dented, cut or hot.
- Store them at room temperature. For more than a few days, leave them about half charged.
- Take dead batteries to a battery recycling point, not the bin.
