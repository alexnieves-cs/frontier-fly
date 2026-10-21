# Mission export to QGroundControl

The [mission editor](https://alexnieves-cs.github.io/frontier-fly/mission.html) can save the planned flight in the formats real drone toolchains read, so a mission drawn in the browser can be opened in [QGroundControl](https://qgroundcontrol.com), checked against a PX4 or ArduPilot software-in-the-loop simulator, or uploaded by a script. Nothing here talks to a vehicle: the page only writes files.

Open **Export to QGroundControl** at the bottom of the right panel. While it is open, the map shows the exported route as a dashed line.

## What goes in the file

| Part of the mission | Exported as |
|---|---|
| Takeoff pad (the start marker) | `plannedHomePosition`, and a `MAV_CMD_NAV_TAKEOFF` to the cruise height |
| Cruise speed | `MAV_CMD_DO_CHANGE_SPEED` (ground speed): 5.5 m/s for the woods, the sim vehicle's outdoor cap; 3 m/s for the flat practice site |
| Area sweep | One `MAV_CMD_NAV_WAYPOINT` per corner of the lawnmower plan |
| Video (optional) | `MAV_CMD_VIDEO_START_CAPTURE` after takeoff, `MAV_CMD_VIDEO_STOP_CAPTURE` before the return |
| Return | `MAV_CMD_NAV_RETURN_TO_LAUNCH`, or a waypoint over the pad and `MAV_CMD_NAV_LAND` |
| Geofence | An inclusion fence polygon |
| No-fly zones | One exclusion fence polygon each |

On the generated woods (the default page) the waypoints come from the autonomy stack's own coverage planner, `planCoverage` in `src/mission/autonomy/planner/coverage.ts`: east-west lanes spaced from the camera footprint, connectors routed around no-fly zones, and heights that follow the terrain. On the flat practice site (`?site=mock`) they come from the editor's sweep preview. Densely sampled stretches are thinned with a 3D Douglas-Peucker pass (0.5 m tolerance), so a straight lane is two waypoints instead of a dozen.

### What is not in the file

Only the pre-planned part of a mission can be a waypoint list. The rest is decided in flight and stays in the sim:

- the extra looks that confirm a contact from several bearings,
- choosing an opening, entering a building, and the GPS-denied indoor search,
- the adaptive probability-map search, which replans after every look.

## Formats

| Format | File | Read by |
|---|---|---|
| QGroundControl plan | `*.plan` | QGroundControl (mission, fence and rally in one file) |
| QGC WPL 110 | `*.waypoints` | QGroundControl, Mission Planner, MAVProxy. Line 0 is the home position, as ArduPilot expects. Mission only. |
| MAVLink mission items | `*.mavlink.json` | Your own pymavlink or MAVSDK script. `MISSION_ITEM_INT` fields for the mission, fence (`MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION` / `_EXCLUSION`, vertex count in `param1`) and rally lists. `x` and `y` are degrees times 1e7; `null` params mean NaN. With ArduPilot selected, home is mission item 0. |

## Where on Earth

The sim's frame is local meters (x east, y north) with no latitude or longitude. The export pins the takeoff pad to a point you choose and converts every point through WGS84 ECEF, so the shape is exact across the site (the tests round-trip to well under a millimeter):

- **PX4 SITL home** (47.397742, 8.545594, 488 m), the default spawn of `make px4_sitl`.
- **ArduPilot SITL home** (-35.363261, 149.165230, 584 m), the default `CMAC` location of `sim_vehicle.py`.
- **Custom**: any latitude, longitude and pad altitude.

With a SITL preset, the plan opens right next to the simulated vehicle.

## Altitudes

- **Relative to home** (default, `MAV_FRAME_GLOBAL_RELATIVE_ALT`): every height is measured from the pad. This flies the sim's 3D profile, hills included, over flat ground at the chosen location.
- **Above terrain** (`MAV_FRAME_GLOBAL_TERRAIN_ALT`): every height is the sim's clearance above the ground under that point. The vehicle needs terrain data, and the real terrain at the chosen home is not the sim's terrain, so the page warns when you pick it.

## Try it with PX4 SITL

1. Run PX4's simulator (`make px4_sitl gz_x500`) and open QGroundControl. It connects on its own.
2. In the mission editor, open **Export to QGroundControl**, keep **PX4**, **PX4 SITL home** and **.plan**, and press **Download**.
3. In QGroundControl, go to **Plan**, then **File > Open**, pick the file, and **Upload**. The fence and the mission upload together.
4. Arm and start the mission from the Fly view.

For ArduPilot, pick **ArduPilot** and **ArduPilot SITL home**, run `sim_vehicle.py -v ArduCopter`, and load either the `.plan` in QGroundControl or the `.waypoints` file in Mission Planner or MAVProxy (`wp load file.waypoints`).

This is for simulators and practice fields. Fly a real aircraft only where and how your local rules allow.

## Import

**Import .plan fence** reads a `.plan` back into the editor: the first inclusion polygon becomes the geofence, every exclusion polygon a no-fly zone, and fence circles become 16-sided polygons. The plan's home lands on the current start marker, so an exported file re-imports onto the same coordinates. Mission items are not imported, because the sim plans its own route. A file is refused with a reason when QGroundControl itself would refuse it, when it has no inclusion fence, when a fence lies more than 20 km from home, or when the result breaks a mission rule (for example, a no-fly zone outside the geofence).

## How it is checked

QGroundControl publishes the `.plan` format as [prose](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/file_formats/plan.html), not as a JSON Schema, so this repository writes one: [`src/mission/export/qgc-plan.schema.json`](../src/mission/export/qgc-plan.schema.json) (draft 2020-12). It encodes the published format and the checks QGroundControl's loaders run when a file is opened (`PlanMasterController`, `MissionController`, `MissionItem`, `SimpleMissionItem`, `GeoFenceController`, `QGCFencePolygon`, `QGCMapCircle`, `RallyPointController`), including the two older mission-item layouts QGroundControl still converts on load.

`tests/unit/qgcExport.test.ts` then checks, with [Ajv](https://ajv.js.org) in strict mode:

- the schema accepts four of QGroundControl's own test plans (vendored in `tests/fixtures/qgc/`, Apache-2.0), so it is not shaped around this project's output;
- every export variant (PX4 and ArduPilot, both altitude references, return or land, with and without video, and a home at 69.6 N on the antimeridian) for three generated woods and the practice site passes;
- 23 broken plans (a wrong file type, a six-value `params`, `Altitude` without `AltitudeMode`, a fence polygon at version 2, and so on) fail both the schema and the import's own loader checks;
- every waypoint converts back to its sim point within 2 cm, its height matches the chosen reference, and the route stays inside the geofence, outside every no-fly zone and inside the altitude band after thinning;
- the WPL text parses with the same rules as QGroundControl's text loader, which also reads a Mission Planner file from QGroundControl's tests, and the WPL and MAVLink lists match the plan item for item;
- export then import gives back the same geofence and no-fly zones within 2 cm.

`tests/e2e/export.spec.ts` downloads each format from the page in Chromium, validates the `.plan` against the schema, and imports it back.

## Code

| File | Role |
|---|---|
| `src/mission/export/route.ts` | Planner output to labelled, thinned legs (`routeFromCoverage`, `routeFromPreview`, and `routeFromWaypoints` for any other planner) |
| `src/mission/export/geo.ts` | Local east-north-up meters to WGS84 and back, through ECEF |
| `src/mission/export/flightPlan.ts` | The format-neutral flight plan: items, fences, summary and warnings |
| `src/mission/export/formats.ts` | `.plan`, WPL 110 and MAVLink writers |
| `src/mission/export/importPlan.ts` | `.plan` reader with QGroundControl's loader checks |
| `src/mission/export/mavlink.ts` | The MAVLink enum values used |
| `src/ui/mission/exportPanel.ts` | The editor panel, download and import |
