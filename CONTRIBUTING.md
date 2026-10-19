# Contributing

Thanks for taking a look. frontier-fly is a browser drone simulator for civilian search, inspection, and confirm-clear missions. Bug reports, benchmark results from other machines, new building or site generators, and policy improvements are all welcome.

## Scope

This project is civilian and defensive only. Contributions that add pursuit, targeting, weapons, payload release, surveillance of real people, or evasion of real-world detection will be closed. Avoidance behavior means the drone moves out of the way. It does not track or engage.

## Setup

Node 22 and npm.

```bash
npm ci
npm run dev      # http://127.0.0.1:3000
```

The dev and preview servers send COOP/COEP headers so `SharedArrayBuffer` works. The production build adds `coi-serviceworker.js` for GitHub Pages, which cannot send headers.

## Before you open a pull request

```bash
npm run lint
npm run typecheck
npm test
npm run build
npm run check:pages   # loads every built page without COOP/COEP headers, like GitHub Pages
```

`npm run test:e2e` runs headless Chrome checks and rewrites the screenshots in `docs/media/`. CI runs it on every pull request.

If your change affects the policy, the simulator's speed, or what a page renders, refresh the measured numbers and commit the outputs:

```bash
npm run bench    # docs/benchmarks.json
npm run media    # docs/media/*.mp4, *.gif, media.json (needs ffmpeg)
npm run docs     # docs/ARCHITECTURE.md, docs/BENCHMARKS.md, README tables
```

Never edit the generated tables by hand. Edit `scripts/docs/*.template.md` for prose. Report numbers you measured, including the ones that miss a target.

## Code layout

- `src/gen`, `src/sim`, `src/rl`, `src/shared`, and `src/world/outdoor` stay headless. ESLint blocks Three.js imports and DOM globals there.
- `src/shared` is a frozen contract. Propose changes in [CONTRACT_REQUESTS.md](CONTRACT_REQUESTS.md).
- The versioned observation (v2, 1088 floats) in [`src/sim/OBS_LAYOUT.md`](src/sim/OBS_LAYOUT.md) is tied to the shipped checkpoint. Changing it needs a new observation version and a retrained checkpoint.
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) has the module graph.

## Assets

Only add assets with a clear free license (CC0, MIT, or similar) and record each one in [CREDITS.md](CREDITS.md) with its source. Keep each file under 5 MB. Larger videos go on a GitHub Release, linked from the README.

## Commits and pull requests

Keep pull requests focused. Describe what a user would see before and after the change, and include the numbers if you touched performance or coverage.

By contributing you agree your work is licensed under the [MIT license](LICENSE).
