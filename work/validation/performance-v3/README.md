# Planning performance v3

Source cohort: `20261018-perf-3`. This is a local, unpushed implementation on
top of `3949cdc`. The [v2 release](../performance-v2/README.md), its CI record
and historical hashes remain unchanged; they do not certify this version.

## Changes and measured scope

- The coordinator polls every pending Worker each frame, so another station's
  failure or deadline no longer waits behind the first station's search.
  Cooperative fallback still advances at most one candidate per frame, fairly.
- A Worker session registers its immutable model once. Later plans send the
  full dynamic state, not another copy of both XML strings. A replacement Worker
  must register again; protocol revisions and stale results fail closed.
- Only the immutable, mesh-free Worker tree reuses known-current FK matrices
  and name lookups. Original R1/G1/C5 solver files are unchanged. Budget checks
  now run inside IK and path checks, with exception-safe restoration.

The [paired Node benchmark](planning-ik-paired.json) uses four genuine G1
pipe-592 snapshots, each in both execution orders: **70.68 s → 46.89 s** in
aggregate, **33.66% less planning time**. All eight pairs retain exactly the
same candidate counts, full plan/command, ordered safety-check requests/results
and physical/derived arrays. Equal tracing overhead is included; fresh model
initialization is excluded. This is not a universal speedup, browser FPS result
or physical execution test.

The [protocol test](worker-protocol-final.json) reconstructs four real jobs
across a Node Worker boundary: first serialized message 20,067,954 bytes,
subsequent messages 23,967 / 24,112 / 24,117 bytes. Four-message total falls
74.91%. These are JSON wire-size measurements, not network traffic or a promise
about structured-clone latency. The first model compilation cost remains.

The [native browser sample](browser/desktop-run-night.json) covers 72.81 seconds
of three-robot pipe execution, including 51 planning-state samples: 8,664 frame
intervals, P95 9.9 ms, maximum 108 ms, three Long Tasks. Each station had
classified two pipes before stop. Orbit control changed the camera, and it
remained at the user-chosen view. Separately, a fresh planning batch was visibly
cancelled and reset restored all stations. These are warm-cache observations
on this desktop; they do not replace FULL72 or certify every device.
The camera thumbnails were outside the scrolled control panel's visible area
(`feedRenderCount: 0`); this is not a camera-visible GPU/readback benchmark.

Shared planning barriers and the common **1 ms physics clock** remain. Robots
still wait together while any selected station plans. Original 10 s / G1 pipe
30 s compute budgets, 90 s transport watchdog, candidate order, collision
sampling, contact/lift/release/support and precision gates are unchanged.
Cooperative checkpoints are not hard-real-time preemption.

## Validation

Complete local regression and positive/negative acceptance passed on the current
sources. The physical runs use real compiled models and original success gates:

| Evidence | Result |
| --- | --- |
| [Ordinary grasping](sort-physical-compiled-model-final.json) | 8 scenarios, 12 real grasps; 50.094 s |
| [Stacking](stack-physical-compiled-model-final.json) | 2 scenarios, 3 stacks, 9 real grasps |
| [FULL pipe classification](pipes-physical-compiled-model-final.json) | 8 scenarios, 72 pipes, 300 layouts; 1,053.716 s |
| [Worker integration](contracts-final.json) | 25 contracts |
| [Framing](framing-final.json) | 113 checks, 98 projection cases |
| [Multi acceptance](multi-acceptance-final.json) and [negative cases](multi-verifier-final.json) | PASS; 92 verifier checks |
| [Pipe acceptance](pipes-acceptance-final.json) and [negative cases](pipe-verifier-final.json) | PASS; 87 verifier checks |
| [Final byte audit](final-hash-audit.json) | 29 reports, 1,122 source bindings and 44 report bindings; no mismatch |

All eight original model test commands, 15 theme checks, 13 camera checks,
static UI, Pages and release-cache checks also passed. Physical and contract
reports are source-frozen; the acceptance verifiers bind them to current bytes.
The final audit counts repeated/end-source bindings, not unique files.
Original standalone acceptance was rechecked successfully against its preserved
evidence; those standalone physical runs were not repeated. See the saved
[standalone command results](standalone-current-check.json).

The paired IK measurement above covers a bounded planning segment. The complete
FULL pipe suite took 1,053.716 s; it does not establish a whole-suite speedup.
This version remains local and unpushed, without a new CI or deployment result.

The required commands write fresh reports here without relabeling older evidence:

```sh
npm test
npm run test:pages
npm run test:acceptance
node work/verify-acceptance.mjs
node work/verify-g1-acceptance.mjs
node work/verify-c5-acceptance.mjs
node work/audit-performance-v3.mjs
```

The release requires 14 polling, 22 protocol and 16 IK/budget contracts, plus
the source-bound eight-pair benchmark. The verifiers reject missing/duplicate
cases, stale sources, old cohorts and mismatched paired safety/physics evidence.
Quick pipe tests cannot substitute for the complete physical run.

To repeat the optional paired benchmark:

```sh
node work/test-planning-ik-performance-v3.mjs --report=work/validation/performance-v3/planning-ik-paired-rerun.json
```

Its preserved input is `../performance-v2/g1-pipe592-snapshots.json.gz`, SHA-256
`8459ef7f8d11f684c1976454106fdb261aa415500ba9734e839adb97c9f99eb9`.
The script reads the gzip directly and validates frozen reference sources.
CI consumes the saved, source-bound benchmark evidence; it does not repeat CPU
timings. CI reruns the full physics and positive/negative acceptance suites.

See the [finish review](finish-review.md) for browser scope and limitations.
