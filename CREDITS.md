# Asset credits

Frontier-fly ships only assets and data with a clear free license. Geometry written for this repository is original and covered by the project [MIT license](LICENSE). Files larger than 5 MB are not committed. The maps below are the 1K JPG downloads, except the outdoor ground and bark maps, which are 512×512 excerpts of the same CC0 sources.

| Asset | Use | Source | License |
| --- | --- | --- | --- |
| `kloofendal_43d_clear_puresky` 1K HDR | Day sky and image-based lighting | [Poly Haven](https://polyhaven.com/a/kloofendal_43d_clear_puresky) | [CC0](https://creativecommons.org/publicdomain/zero/1.0/) |
| PaintedPlaster017 1K | Interior walls and ceiling | [ambientCG](https://ambientcg.com/view?id=PaintedPlaster017) | [CC0](https://docs.ambientcg.com/license/) |
| WoodFloor051 1K | Wood floors | [ambientCG](https://ambientcg.com/view?id=WoodFloor051) | [CC0](https://docs.ambientcg.com/license/) |
| Concrete034 1K | Concrete and pots | [ambientCG](https://ambientcg.com/view?id=Concrete034) | [CC0](https://docs.ambientcg.com/license/) |
| Wood051 1K | Doors, frames, tables, shelves | [ambientCG](https://ambientcg.com/view?id=Wood051) | [CC0](https://docs.ambientcg.com/license/) |
| Ground037 512 | Outdoor terrain color | [ambientCG](https://ambientcg.com/view?id=Ground037) | [CC0](https://docs.ambientcg.com/license/) |
| Bark001 512 | Outdoor tree bark | [ambientCG](https://ambientcg.com/view?id=Bark001) | [CC0](https://docs.ambientcg.com/license/) |
| Quadrotor mesh, furniture meshes, propeller-wash particles | Drone and room props | Original, this repository | MIT |
| Outdoor terrain, tree, and shell meshes | Forest and building exteriors | Original, this repository | MIT |
| Three.js post passes (SSAO, SMAA, TAA, bloom, afterimage) | High-preset image effects and FPV trail | [three.js addons](https://github.com/mrdoob/three.js) `0.170.0` | MIT |
| `public/coi-serviceworker.js` v0.1.7 | Cross-origin isolation on GitHub Pages so `SharedArrayBuffer` works | [gzuidhof/coi-serviceworker](https://github.com/gzuidhof/coi-serviceworker) | MIT (header kept in the file) |
| Five QGroundControl test plans and one Mission Planner waypoint file | Test fixtures for the `.plan` schema and the WPL parser (`tests/fixtures/qgc/`) | [mavlink/qgroundcontrol](https://github.com/mavlink/qgroundcontrol) `test/plans`, `test/MissionManager` | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0) (QGC is dual Apache-2.0 / GPL-3.0) |
| 80 apartment floor plans (rooms, walls, doors, windows), rotated, translated and rounded to 1 cm | Real-plan benchmark and `sim.html?plan=` (`public/plans/swiss-dwellings.json`) | [Swiss Dwellings v3.0.0](https://zenodo.org/records/7788422), Archilyse AG, doi:10.5281/zenodo.7788422 | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) (attribution and changes recorded in the file and [docs/REAL_PLANS.md](docs/REAL_PLANS.md)) |
| Intel Research Lab (Seattle) laser log: the FLASER lines of `intel.clf`, unchanged; and its 2,984 benchmark relations, unchanged | Real-data check of the indoor pose estimator (`data/intel-lab/`, `npm run study:intel`) | Log recorded by Dirk Hähnel, published on [Radish](https://dspace.mit.edu/handle/1721.1/62287); `intel.clf` and `intel.relations` from the [Freiburg SLAM benchmark](http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php) (Kümmerle et al. 2009) | Log: [CC BY 1.0](http://creativecommons.org/licenses/by/1.0/) as stated by the benchmark (CC0 on the Radish record); relations: [CC BY 3.0](http://creativecommons.org/licenses/by/3.0/) (see [data/intel-lab/README.md](data/intel-lab/README.md)) |
| World Magnetic Model lookup tables (WMM-2020, 10° grid) | Earth's magnetic field for the simulated compass in the PX4 bridge (`src/px4/wmmTables.ts`) | [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) `v1.15.4`, `src/lib/world_magnetic_model/geo_magnetic_tables.hpp` | [BSD-3-Clause](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE) (notice kept in the file) |
| Procedural canvas textures | Fallback if a map fails to load | Original, this repository | MIT |
| KayKit Rig_Medium mannequin and clips | Simulated people | [Kay Lousberg](https://www.kaylousberg.com), KayKit Character Animations 1.1 | [CC0](https://creativecommons.org/publicdomain/zero/1.0/) |
| Backpack, phone, toolbox, medkit, bottle, radio, helmet, blanket, coat, hiding crate | Search objects and decoys | Original procedural meshes, this repository | MIT |

The site also bundles open-source libraries: three.js (MIT), Rapier (`@dimforge/rapier3d-compat`, Apache-2.0), TensorFlow.js (Apache-2.0) and uPlot (MIT), plus their dependencies. The build writes each one's license to `THIRD_PARTY_LICENSES.txt` at the site root.

No asset in the repository has an unclear or non-commercial license. The floor plans (CC BY 4.0) and the Intel Research Lab log and relations (CC BY 1.0 and 3.0) require attribution; the credits above, in the plan data file and in `data/intel-lab/README.md` satisfy it. Poly Haven and ambientCG downloads used here are CC0, so attribution is not required; the links above are the record of where each file came from. CC0 also does not require credit for KayKit; the credit is here anyway.

## Simulated people

The humanoid is packed into `public/assets/targets/humanoid.glb`. The license text shipped with the file is `public/assets/targets/LICENSE-KayKit.txt`.

The drone camera (`src/vision`) draws people from `public/assets/targets/humanoid-poses.bin`, which `scripts/vision/bake-humanoid.mjs` derives from that same CC0 mesh and clips: ten posed, simplified copies of the mannequin. The camera detector in `public/models/vision-detector/` was trained in this repository on frames rendered from those poses and the procedural scenes; no outside dataset or pretrained weights were used.

Clips included: `Idle_A` (standing), `Walking_A` (walking), `Sit_Chair_Idle` (sitting), `Sit_Floor_Idle` (hiding / low), `Lie_Idle` (lying, used for an injured or resting person). Hit, death, and other combat clips from that pack are not included.

The clips and mannequin were taken from the CC0 redistribution at https://github.com/NoLimitNexus/game-models (`library/animations/Rig_Medium_*.glb` and `tools/reference/Mannequin_Medium.glb`), which credits Kay Lousberg for the skeleton, mesh, and animations. This repository does not include that project's original body kit.

Joint velocities in `src/targets/skeleton.ts` are computed from an analytic pose, not by sampling the KayKit clips. The pose names match the Rig_Medium joints so a later binding can share them. The hook is reserved for a drone avoidance response: the drone dodges. It does not pursue or strike.

To refresh the indoor textures, download the same 1K JPG zips from those pages and keep only the Color, Roughness, and NormalGL images, renamed to `public/textures/{plaster,floor,concrete,door}_{color,rough,normal}.jpg`, then re-encode them as progressive JPEG (normal maps at quality 95 with full-resolution chroma, the rest at quality 90). The downloads are saved at quality 100, which tripled the page weight for no visible difference. The sky file is `public/hdri/kloofendal_43d_clear_puresky_1k.hdr`. The outdoor maps are `public/outdoor/textures/ground-color.jpg` and `bark-color.jpg`.
