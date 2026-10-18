# Completion audit: performance release `ee380fb`

Audit time: 2026-10-18, approximately 03:30 UTC. Baseline: `8ee1531`.
Published source revision: `ee380fb973fc02eddfcd1edd953c17af74140df4`.
Release cohort: `20261018-perf-2`.

This is an independent source/evidence audit, not another physical or browser
run. No production code, numerical gates, saved measurements, or report hashes
were changed by this audit. The initial audit preceded FULL72 and final local
acceptance. The 03:43 UTC append below confirms those local gates now passed;
GitHub validation/deployment remains **pending**, not inferred from local success.

## What actually changed

The complete production diff contains 11 files. Shipping work is confined to:

- `app.mjs`: submit camera work through a single-in-flight queue; invalidate
  old presentations on focus/arm/participants/start/stop/reset/task/theme/page
  visibility changes; add read-only readback diagnostics and recoverable feed
  error captions. Existing task, pointer, camera-fit and physical scheduling
  behavior is retained.
- `visuals/camera-feeds.mjs` and new `visuals/camera-feed-queue.mjs`: use the
  bundled Three asynchronous PBO readback; restore renderer state in the same
  synchronous turn; keep the readback buffer owned until completion. Late
  results cannot overwrite a newer main-view renderer state or obsolete feed.
- `planning-worker-core.mjs`, `planning-robot.mjs` and new
  `planning-check-cache.mjs`: exact unchanged-joint reuse only in the private
  Worker parser, and exact collision-result reuse only in one restored G1
  planning job. First joint assignments still compute the original transform;
  `Object.is` distinguishes signed zero. Cache installation follows restoration
  and every completion/error path disposes it in `finally`.
- `index.html` and the Worker/runtime import cohort: advance document and
  import-map-free Worker URLs consistently; preserve old document aliases.

The experimental motor-data-view adapters are not present in `dist/` or imported
by the shipping application. Their work snapshots are not evidence of a shipped
motor optimization.

## Measured hotspot removal, not renamed tests

| Workload | Evidence and result | Boundary |
| --- | --- | --- |
| Four genuine G1 pipe-592 snapshots, two alternating paired trials each | [planning-combined-final.json](planning-combined-final.json): 70.096 s original versus 55.919 s optimized, 20.23% less elapsed time. Per-item mean reductions: 16.97%, 30.82%, 4.89%, 11.78%. | Bounded Node timings include exact trace encoding; not a universal speedup or browser FPS result. |
| Redundant collision checks | The paired report compares **every** safety-check request and returned result, complete plans/commands, candidate counts 69/137/201/267, and all authoritative physical/derived arrays. Every comparison is exact. Cache peaks at 512 entries; its approximately 1.47 MB recorded text accounting is not a JavaScript heap measurement. | The cache does not remove path samples, change predicates, approximate floats, or store executable motion. |
| Main view with camera feeds visible | [baseline-desktop-cameras.json](browser/baseline-desktop-cameras.json) versus [optimized-desktop-cameras.json](browser/optimized-desktop-cameras.json): frame-interval P95 24.7 ms to 9.5 ms; 51 versus 52 presented feeds in approximately six seconds. Physical real-time factors are approximately 1.006 and 1.000; neither sample gains smoothness by dropping physical work. | One warm-cache desktop/device sample. Idle/run rendering was already near 120 Hz; there is no general FPS claim. |
| Actual display readback equivalence | [optimized-desktop-ready.json](browser/optimized-desktop-ready.json): real browser WebGL2 sync/async output differs by zero bytes at exposures 0.6, 1.15 and 2.0; immediate state restoration and preservation of newer state both pass, with GL error zero. | Fixed-scene GPU evidence, not every robot frame, full pipe completion, or every GPU. |

The combined paired report's 17 current source bindings were independently
recomputed without mismatch. The preserved `.json.gz` input decompresses to the
exact original `inputHash`; no historical input or measurement was rebound.

## Numerical, visual and interaction preservation

Git object comparison confirms the entire `dist/r1`, `dist/g1`, `dist/c5`,
`dist/vendor` and `dist/Rebot_Arm_AGV` trees are identical between the two
revisions. All URDF/MJCF models, meshes, original motor formulas and source
limits therefore retain their exact bytes.

The following multi-robot files are also identical: coordinator, planning,
pipe-controller, pipe-layout, pipe-model, R1 safe dynamics, planning serialization,
styles, workcell, frame scheduler, scene environment and theme controller.
The application frame loop, `fitView` and visible `updateUI` are individually
byte-identical. HTML outside the import map and release query strings is
identical. In particular:

- Physical stepping remains 1 ms for all ready worlds, including unselected
  worlds; shared planning barriers and cancellation ownership are unchanged.
- Original 3 mm IK position, 0.035 rad orientation, 0.30 rad branch-jump,
  0.045 rad swept-joint and 2 mm opening-envelope checks remain unchanged.
  Ordinary/G1-pipe planning compute budgets remain 10 s/30 s. Some equivalence
  fixtures intentionally allow longer comparison time; the physical release
  runners separately reject production-budget overrides.
- No grip welds, kinematic carries, object teleportation, synthetic contacts,
  weakened collision geometry or relaxed force/lift/release/support criteria
  were introduced. The cache reads private scratch-check results only, clones
  stored/returned values, and permanently bypasses after `qpos`/`ctrl` or epoch
  identity drift. Unusual inputs bypass rather than alias ordinary input keys.
- Main renderer resolution/pixel ratio/shadows, meshes, lighting, scene themes,
  and 240×180 feed resolution are not reduced. The same HalfFloat HDR scene
  pass, explicit ACES/sRGB RGBA8 display pass, shader chunks, exposure and row
  flip remain. Missing async APIs retain the declared synchronous fallback.
- Robot selection, simultaneous starts, object editing, orbit/zoom controls,
  explicit panorama/focus actions, scene switching and concise Chinese planning
  text retain their established behavior. The visible UI still does **not**
  display current search-item indices, candidate counts or waiting times.
  Existing actual classification completion counts remain visible.

## CI coverage of the new mechanisms

The current workflow executes `npm run test:pages`, then `npm test`, then the
positive/negative source-bound acceptance chain before its dependent Pages job.
The new core mechanisms are covered concretely, not merely by a green step name:

- Async pass contracts execute production code with deferred readback promises:
  immediate restoration, supplied-buffer identity, later-main-state preservation,
  pending sync/async reentry and disposal rejection, draw/submit/reject recovery,
  synchronous fallback, shader identity and resource reuse/owned disposal.
- Queue contracts execute the production queue: overlap prevention, ownership
  until read/presentation completion, generation and station/controller/arm/theme/
  visibility staleness, synchronous/asynchronous errors, reentrant idle lifetime,
  and zero unhandled rejections. Responsiveness tests additionally execute the
  actual app queue callbacks and `updateCameras`, including row flip and recovery.
- Cache contracts exercise exact ordered keys/signed zero/missing arguments,
  unsafe results, clone isolation, `qpos`/`ctrl` and identity invalidation,
  hidden/accessor/symbol/custom-array bypass, LRU/text accounting and disposal.
- The 25 Worker contracts include the explicit unchanged-joint optimization
  equivalence check: every world matrix and quaternion matches the default
  parser exactly, repeated assignments really skip work only in the Worker,
  signed zero is preserved, and nonfinite values still fail. Actual worker
  plan/command equality, moving snapshots, cross-seed compiled-model ordering,
  cancellation, unreachable/reset recovery and host heartbeat are also tested.
- Physical runners use the actual shipped Worker parser factory and service,
  not a cooperative fixture pretending to exercise the new optimization. Their
  manifests require the new cache source. Final verifiers require the named
  optimization contract; a negative fixture deletes it and must be rejected.
- Pages/cache checks include the new queue in the document graph and the cache
  in the separate import-map-free Worker graph. Unversioned Worker entry/import
  and inherited-import-map fixtures fail. The new cohort retains native-page
  cohorts and old document aliases. Fresh reports are preserved as the
  `multi-performance-v2-evidence` artifact; historical reports are not rebound.

## Completion checklist and remaining gates

| Gate | Audit-time status | Concrete evidence |
| --- | --- | --- |
| Optimize measured planning hotspot with exact safety/plan equivalence | Passed within the measured scope | Eight paired rows plus four actual private-service jobs in `planning-combined-final.json`. |
| Remove camera GPU-wait pressure without reducing visual quality | Passed within measured browser/fixture scope | Browser camera samples and zero-byte GPU comparison; async 12, queue 16, legacy shader/color 13 contracts. |
| Preserve source models, motor physics, 1 ms stepping, sampling and budgets | Passed source audit; current ordinary/stack regression passed | Identical trees/files; [sort](sort-physical-compiled-model-final.json) 8 cases/12 grasps and [stack](stack-physical-compiled-model-final.json) 2 cases/3 stacks/9 grasps, both real-worker and source-frozen. |
| Preserve multi-robot UI, camera ownership, scenes and hidden search statistics | Passed bounded contracts/source audit | UI 29, responsiveness 20, pipe UI 46, theme 15, coordinator 20; unchanged fit/frame/UI/scene implementation. Native visual scope is separately recorded in `finish-review.md`. |
| Publish coherent current document/Worker resources | Passed static/cache simulation | Pages 17 / 47 modules; cache 13 / 6 graphs / 10 negative fixtures. |
| Complete current FULL72 pipe manipulation and retain every original safety gate | **Passed locally; CI remains separate** | [Fresh full report](pipes-physical-compiled-model-final.json): `mode=full`, 8/8 cases, 72/72 pipes, 300 layouts, 970.3098 s, source-frozen, real Worker, no production-budget override. |
| Rebuild framing and execute complete positive/negative acceptance on fresh physics | **Passed locally** | [Framing](framing-final.json): 113/113 checks and 98 projections; [multi negatives](multi-verifier-final.json): 38; [pipe negatives](pipe-verifier-final.json): 42; [multi](multi-acceptance-final.json) and [pipe](pipes-acceptance-final.json) positive acceptance both pass with empty errors. |
| Complete GitHub validation, deploy and verify served revision | **Passed** | [Run 39093502625](https://github.com/wander521/r1-simulator/actions/runs/39093502625), exact head `ee380fb`, completed successfully at 03:54:31 UTC. [Actual CI execution](ci-execution-audit.json) confirms the full suite and acceptance; [served-file audit](deployment-final.json) matches nine key online resources byte-for-byte. |

At the initial audit, nine selected current paired/contract/physical/publication
reports were audited against 350 recorded source bindings with zero mismatches.
The subsequent local completion evidence is recorded below; it does not turn
unfinished GitHub validation or deployment into a success.

## Local final-acceptance append — 2026-10-18 03:43 UTC

Read directly from the newly written reports, without rerunning tests or editing
their contents:

- [FULL72](pipes-physical-compiled-model-final.json) passes all eight cases and
  72 physical classifications, plus 300 layouts, in 970.3098 s. It records
  `clockStepSeconds=0.001`, `worker=true`, `cooperative=false`,
  `productionBudgetOverride=false`, and the original 10 s/30 s planning budgets.
- [Models](models-isolated-final.json) records exit code zero for all eight
  unchanged original model test scripts executed from an archive of the exact
  published commit in an isolated directory. Its retained historical R1-camera
  and C5-visual report hashes are unchanged; new model results are saved here.
- [Framing](framing-final.json) passes 113/113 checks with 98 projections and
  source freeze. Themes, legacy camera-color contracts and publication/cache
  contracts also pass in the final sequence, rather than reusing their names as
  a substitute for current evidence.
- [Multi positive acceptance](multi-acceptance-final.json) confirms 12 ordinary
  physical grasps, three stacks/nine stack grasps, 12 async-camera contracts,
  16 queue contracts, and 19 cache contracts, with `pass=true` and `errors=[]`.
  [Pipe positive acceptance](pipes-acceptance-final.json) independently verifies
  all 72 classifications and 300 layouts with `pass=true` and `errors=[]`.
- The actual source-bound negative verifier reports pass 38 multi cases and
  42 pipe cases, including rejection when the new Worker-only unchanged-joint
  equivalence contract is absent.
- [Final hash audit](final-hash-audit.json) identifies the exact source commit
  `ee380fb973fc02eddfcd1edd953c17af74140df4`: 18 reports, 791 source bindings,
  14 report bindings, with every `stale` array empty and overall `pass=true`.

Thus the local source, model, physical, visual-contract, framing and positive/
negative acceptance gates are complete. The CI run and served Pages revision
were still distinct external gates at that local audit time. Both subsequently
passed as recorded below.

## Completed release verification

The GitHub run for the exact source commit `ee380fb` finished successfully at
2026-10-18 03:54:31 UTC. Both validation and dependent Pages deployment passed.
The [CI execution audit](ci-execution-audit.json) reads actual completed logs:
72 classified pipes, 12 ordinary grasps, three stacks/nine grasped items, and
source-bound positive/negative acceptance all passed. The
[served-file audit](deployment-final.json) records HTTP 200 and exact local/online
SHA-256 equality for the entry HTML and eight critical runtime, Worker, cache
and camera modules. This confirms publication of the tested code; local native
browser/GPU checks retain their separate scope in `finish-review.md`.

Unverified beyond the requested local-browser scope: physical-phone touch and
low-end hardware, all browser/GPU combinations, cold-load/network performance,
hardware-robot calibration and cross-station collisions between separate
physics worlds. Remaining G1 IK searches can still take seconds; this release
does not promise instant planning or reachability of arbitrary distant layouts.
