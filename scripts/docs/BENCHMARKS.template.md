# Benchmarks

Every number on this page comes from a script in this repository. `npm run bench` writes `docs/benchmarks.json`, `npm run media` writes `docs/media/media.json`, and `npm run docs` renders both into this page and the README. Nothing is typed in by hand. When a target is missed, the table says so.

{{PROVENANCE}}

## Against the project goals

{{TARGET_TABLE}}

## Coverage

{{PROTOCOL}}

The **browser pilot** is what the live demo flies when you pick **Policy**: observations go through the checkpoint's running normalization (clipped to ±5), or straight to the network when that normalization is the identity, as it is for the shipped checkpoint. **Raw observations** always feeds the network unnormalized, the way `npm run eval:coverage` does. Both rows are kept so a checkpoint with a real normalizer shows any gap between the two. The seeds and step budgets are the fixed protocol in `src/eval/coverage.ts`. The **lidar heuristic** is the scripted pilot the screenshot URLs use. It is a floor for comparison, not a learned policy.

{{COVERAGE_TABLE}}

<details>
<summary>Per-seed results</summary>

{{PER_SEED_TABLE}}

</details>

## Simulator throughput

Single-thread numbers are Node on the machine above. The worker rows use `node:worker_threads`, two envs per worker, the same rollout code the browser workers run.

{{THROUGHPUT_TABLE}}

## Rendering

{{RENDER_NOTE}}

{{RENDER_TABLE}}

## Missions

{{MISSION_TABLE}}

### On fresh seeds

{{MISSION_BENCH_NOTE}}

{{MISSION_BENCH_TABLE}}

## Reproduce

```bash
npm ci
npm run bench   # coverage, throughput, and the fresh-seed missions, a few minutes on 4 cores
npm run media   # headless video capture and frame rates (needs ffmpeg)
npm run docs    # re-render this page and the README tables
```

Seeds can be overridden with `BENCH_EASY_SEEDS=1,2,3` and `BENCH_HARD_SEEDS=...`, and the checkpoint with `BENCH_CHECKPOINT=path.json`.
