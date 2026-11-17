# FrontierBench

FrontierBench scores an exploration pilot on seven fixed tasks: six sets of generated buildings and one set of
real apartments. The standings, the task list and the rules are on the [leaderboard](../docs/LEADERBOARD.md).

| Folder | What |
| --- | --- |
| [entries/](entries) | Submitted pilots, one folder each. [entries/example](entries/example) is a worked entry to copy |
| [results/](results) | Scores for the built-in baselines (`npm run bench:board`). `team/` holds the team track, `v2/` the previous version of the benchmark |

To enter, copy `entries/example`, tune on the dev split and score the test split once. The steps are under
[Submit an entry](../docs/LEADERBOARD.md#submit-an-entry). CI re-scores every entry from its code on each pull
request that touches it.
