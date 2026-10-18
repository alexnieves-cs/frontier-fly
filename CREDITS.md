# Asset credits

Frontier-fly ships only assets with a clear free license. Geometry written for this repository is original and covered by the project [MIT license](LICENSE). Files larger than 5 MB are not committed. The maps below are the 1K JPG downloads.

| Asset | Use | Source | License |
| --- | --- | --- | --- |
| `kloofendal_43d_clear_puresky` 1K HDR | Day sky and image-based lighting | [Poly Haven](https://polyhaven.com/a/kloofendal_43d_clear_puresky) | [CC0](https://creativecommons.org/publicdomain/zero/1.0/) |
| PaintedPlaster017 1K | Interior walls and ceiling | [ambientCG](https://ambientcg.com/view?id=PaintedPlaster017) | [CC0](https://docs.ambientcg.com/license/) |
| WoodFloor051 1K | Wood floors | [ambientCG](https://ambientcg.com/view?id=WoodFloor051) | [CC0](https://docs.ambientcg.com/license/) |
| Concrete034 1K | Concrete and pots | [ambientCG](https://ambientcg.com/view?id=Concrete034) | [CC0](https://docs.ambientcg.com/license/) |
| Wood051 1K | Doors, frames, tables, shelves | [ambientCG](https://ambientcg.com/view?id=Wood051) | [CC0](https://docs.ambientcg.com/license/) |
| Quadrotor mesh, furniture meshes, propeller-wash particles | Drone and room props | Original, this repository | MIT |
| Three.js post passes (SSAO, SMAA, TAA, bloom, afterimage) | High-preset image effects and FPV trail | [three.js addons](https://github.com/mrdoob/three.js) `0.170.0` | MIT |
| Procedural canvas textures | Fallback if a map fails to load | Original, this repository | MIT |

No asset in the repository has an unclear or non-commercial license. Poly Haven and ambientCG downloads used here are CC0, so attribution is not required; the links above are the record of where each file came from.

To refresh the textures, download the same 1K JPG zips from those pages and keep only the Color, Roughness, and NormalGL images, renamed to `public/textures/{plaster,floor,concrete,door}_{color,rough,normal}.jpg`. The sky file is `public/hdri/kloofendal_43d_clear_puresky_1k.hdr`.
