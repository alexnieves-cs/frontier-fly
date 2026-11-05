# Archived results

Raw results that the current docs no longer lead with but still cite, kept so every number in the
history sections of the docs can be traced back to the flights behind it. Current results stay in
`docs/` next to the pages that read them.

| Folder | What | Cited by |
| --- | --- | --- |
| `fault-bench/` | Fault bench runs 1 to 7 (`fault-bench-runN.json`) and the previous-code controls of runs 3 to 7 (`-runN-control.json`). Run 8, the current one, is `docs/fault-bench.json` with its control `docs/fault-bench-control.json`. | `docs/SAFETY_CASE.md` (runs table, written by `npm run docs`) |

Nothing here is regenerated. A new archived run goes in a folder named after its bench, and the
table above gets a row.
