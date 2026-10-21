# QGroundControl test fixtures

Copied unchanged from [mavlink/qgroundcontrol](https://github.com/mavlink/qgroundcontrol)
at commit `6133ce6d26832d3efdb81e9a7c324ec82898f42d` (2026-10-19). QGroundControl is
dual-licensed Apache-2.0 / GPL-3.0; these files are used under Apache-2.0.

| File | Upstream path |
|---|---|
| `PX4SITL-PX4-MultiRotor.plan` | `test/plans/PX4SITL-PX4-MultiRotor.plan` |
| `ArduPilotSITL-ArduCopter.plan` | `test/plans/ArduPilotSITL-ArduCopter.plan` |
| `TerrainTest-ArduPlane.plan` | `test/plans/TerrainTest-ArduPlane.plan` |
| `SectionTest.plan` | `test/MissionManager/SectionTest.plan` |
| `MissionPlanner.waypoints` | `test/MissionManager/MissionPlanner.waypoints` |

`tests/unit/qgcExport.test.ts` checks that the plan schema in
`src/mission/export/qgc-plan.schema.json` accepts every one of QGroundControl's own
plans, and that the WPL parser reads its Mission Planner waypoint file, so the
schema and parser are not just shaped around this project's own output.
