# Completion performance evidence audit

Audited 2026-10-18. This is an independent, read-only audit of the retained
measurements and shipped planning code, using the Impeccable performance
workflow. It is not another CPU profile, a full UI/accessibility audit, a GPU
rerun, or a declaration that the still-running release CI/full-pipe run passed.
No production code or historical report was changed for this audit.

## Verdict

The bounded performance claims are supported: the measured G1 planning workload
took **20.23% less aggregate time**, and the recorded desktop camera-visible
sample had a substantially shorter frame-interval tail. The data does **not**
support a general FPS increase, a 20% reduction in every planning job, faster
physics, physical-phone performance, or universal elimination of stutter.

All 17 source hashes in [the combined report](planning-combined-final.json)
match the current files. All source hashes recorded by the seven optimized
browser JSON records match the current files as well. These are filesystem
cohort checks; the browser records' entry URLs additionally identify the
`20261018-perf-2` cohort. The collector does not independently hash every module
inside the browser's executed module cache.

## Planning: recomputed from all eight paired rows

The report uses four genuine G1 pipe592 snapshots, each tested twice with the
execution order reversed on the second trial. The measured variants call the
actual shipping parser/helper and bounded cache, not an experimental adapter.

| Input | Original total | Optimized total | Time reduction |
| --- | ---: | ---: | ---: |
| pipe-yellow-1 | 13,449.04 ms | 11,167.32 ms | 16.97% |
| pipe-red-1 | 31,574.68 ms | 21,844.46 ms | 30.82% |
| pipe-yellow-2 | 11,446.04 ms | 10,885.95 ms | 4.89% |
| pipe-red-2 | 13,626.24 ms | 12,020.89 ms | 11.78% |
| **Sum** | **70,096.00 ms** | **55,918.63 ms** | **20.22565%** |

The aggregate is `1 - sum(optimized)/sum(original)`, not an average of rounded
percentages. The corresponding time ratio is **1.25354×**. Individual trial
reductions range from **0.81% to 34.57%**; two trials per input are not a
confidence interval or broad device distribution.

For every pair the report and test enforce the same ordered safety-check
request/result stream, final plan and command payload, and controller-reported
planning step count. The paired plan hashes also match the captured original
snapshot payloads. The test checks the captured physical and derived arrays
remain unchanged during planning. This is not a proof that every internal
MuJoCo field or every possible future trajectory was exercised.

The additional four service rows reuse one isolated world, match all four
original plan hashes, and preserve the captured physical state. They exercise
the actual worker-service code through a file-loading factory in Node, **not a
separate browser Worker or `worker_threads` timing run**. Actual thread
isolation/cancellation has separate Worker-contract evidence.

The direct timing window excludes model/tree construction and final assertion
work, but includes the identical trace-encoding instrumentation on both sides.
The parser improvement being measured is repeated unchanged **joint
assignment** during planning, not a claimed URDF download/load-time gain.
The two optimizations are combined here, so this report does not partition the
20.23% into independent, additive parser/cache contributions.

The cache is installed only for G1 private planning jobs and disposed in the
service's `finally`. The report records 512 peak entries, at most 1,466,338
encoded-accounting bytes, and zero retained entries/bytes after disposal. The
2 MiB limit is an **encoded-text accounting bound**, not a measured JavaScript
heap/RSS bound. The parser's unchanged-joint option remains false by default;
the worker factory alone opts in. Neither the 1 ms physical clock nor planner
precision, collision gates, search ordering or budgets was weakened.

## Browser: what the retained records actually cover

All compared records use this desktop's visible in-app browser, night theme,
DPR 1 and three selected robots. Desktop size is 1280×720; the mobile-size case
is a 390×844 viewport on the same desktop. The sessions are warm-cache samples,
not cold-load, throttled low-end-device or network benchmarks.

| Workload | Before | After | Supported reading |
| --- | --- | --- | --- |
| Desktop, cameras visible, completed sort | 6.026 s, 606 frames, 51 feeds, recorded P95 24.7 ms | 6.000 s, 720 frames, 52 feeds, recorded P95 9.5 ms | Short-sample tail improvement, **61.54% lower recorded P95 interval** |
| Desktop idle, three 4 s samples | P50 8.3 ms; P95 9.1/9.1/9.1 ms | P50 8.3 ms; P95 9.7/9.8/9.9 ms | No demonstrated general idle-rendering speedup |
| Desktop, 12 s ordinary sort run | 1,432 frames; recorded P95 9.1 ms; RTF 0.89543 | 1,436 frames; recorded P95 9.6 ms; RTF 0.90363 | Already near a 120 Hz cap; planning-wait-inclusive sample, not faster physics |
| Mobile-size desktop idle | 6.000 s, 674 frames, no feeds; recorded P95 16.1 ms | 6.001 s, 720 frames, no feeds; recorded P95 9.8 ms | One emulated-viewport pair only, not physical-phone evidence |

The camera workload's feed rate is comparable: **8.46 to 8.67 presentations/s**;
the improved tail was not obtained by stopping feed updates. Its count/duration
ratio is about 100.56 to 119.99 rAF callbacks/s, but that is a workload-specific,
single-session observation, not a general FPS claim. Ordinary run ratios are
119.31 and 119.62 callbacks/s; the idle median interval stays 8.3 ms.

The camera records agree on selection, task, completed state, theme, size and
DPR. They do not preserve focused robot/camera identity or all scene/object
poses, so they are a comparable workload observation, not an exactly replayed
pixel-identical before/after app scene. No browser performance pair is an
end-to-end completion-time benchmark for the full 72-pipe task.

### Measurement limitations

- The JSON records retain counts and summary quantiles, **not the individual
  frame-interval samples**. Count/duration ratios and percentage arithmetic were
  independently recomputed; the recorded P95 values cannot be recomputed from
  these files alone. The collector saves supplied browser measurements rather
  than implementing their rAF/observer producer itself.
- `runtime.averageWorkMs` is the app's ending rolling window (up to 120 frames),
  not the mean for the complete 4/6/12 s sample. It must not be presented as the
  whole-window main-thread cost. Cumulative runtime counters from different
  browser sessions are likewise not directly comparable; use recorded deltas.
- All records contain empty long-task lists, but no retained observer-support
  flag, INP, input-event latency trace, Web Vitals or per-frame GPU timing. Do
  not translate those empty arrays into proof of zero interaction delay.
- The mobile-sized sample has **zero feed presentations**. It cannot establish
  faster mobile camera readback, touch handling, or physical-phone performance.
- Sampling was sequential, short and not randomized. Browser/GPU/OS version,
  hardware/thermal state and a global process-load trace are not preserved in
  these JSON records. There is no CPU-affinity or global-OS isolation proof.

## CPU isolation and chronology

The known validation timestamps and team scheduling are consistent with no
overlapping heavy local benchmark during the frame-timing windows. Before
browser samples ran at approximately 02:37–02:40 UTC, after the earlier heavy
exploratory profiles. The combined planning benchmark ran from 03:05:04.789 to
approximately 03:08:20.600 UTC. The optimized frame samples ran around
03:09:52–03:12:53 UTC; the full Worker contracts and physical regressions started
after 03:14 UTC. This supports the intended within-team scheduling separation,
not forensic proof that every background OS process was idle.

The fixed-scene GPU pixel check at 03:07:33 UTC overlaps the planning benchmark,
but it is a correctness check, **not one of the frame-performance samples**.
It reports zero byte differences at exposures 0.6, 1.15 and 2, immediate renderer
state restoration, retention of newer main-view state and GL error zero. It
does not certify all robot-scene pixels, contact behavior or FPS.

## Reproducibility and source binding

The gzip fixture was **stream-decompressed directly into SHA-256**. No raw JSON
was written or rewritten by this audit, and no expensive planning/profile was
rerun. A raw fixture already existed in the working directory before the check.

| Check | Result |
| --- | --- |
| Compressed archive size | 24,928,891 bytes |
| Decompressed byte count | 81,581,729 bytes |
| Decompressed SHA-256 | `eb613c832587d06a5d5083112dfcf476cc1823f426cfb1b7a884b49ed4414be8` |
| Combined `inputHash` | Exact match to the decompressed stream |
| Current combined source bindings | 17/17 match |
| Optimized browser source bindings | All seven JSON records match |

Current critical source identities, matching the combined paired report:

```text
planning-worker-core.mjs  eb853f820c0d784630404df2ec89eef550b3de657ab968ea152d4322f91f5faf
planning-robot.mjs        40a8e554b38f39f4d328b932d895028c789c0b3fb2e4adc7b98ff32906ec9d70
planning-check-cache.mjs  8a83abae4e6c7be94be51ce43210fbbe034fc82950c4ddafdefc9477b76c7938
```

Audited combined report SHA-256:
`4b15498e4e2a54e9b58501b6a2def8454f1029238ff568f574106267c70953f4`.
Archive SHA-256:
`8459ef7f8d11f684c1976454106fdb261aa415500ba9734e839adb97c9f99eb9`.

The archive therefore supplies the exact benchmark input identified by the
report, rather than an unlinked regenerated layout. The historical hashes were
not rebound. The unpublished motor-view experiments remain outside `dist/` and
contribute no claimed shipping speedup.

## Safe completion wording

“On the measured G1 pipe-planning snapshots, aggregate planning time decreased
by about 20%. With camera feeds visible, this desktop's short sample improved
from a recorded 24.7 ms to 9.5 ms P95 frame interval. Physics and grasp safety
gates were retained. Mobile coverage is desktop viewport emulation.”

Full physical success and deployment are separate source-bound release checks.
They subsequently passed: the local final acceptance covers all 72 pipes,
and [CI execution](ci-execution-audit.json) confirms the complete suite and
successful deployment for `ee380fb`. The [served-file check](deployment-final.json)
matches nine key resources to the tested bytes. These later release results do
not broaden the timing claims or browser/hardware scope of this audit.
