# Performance validation and experiments

Release cohort: `20261018-perf-2`. This directory supersedes the historical
`background-planning/` evidence for the current multi-robot source. Those older
reports retain their original hashes and are not relabeled as current results.

## Measured shipping changes

- The G1 Worker memoizes only identical collision-check arguments within one
  immutable planning job. Its LRU is limited to 512 results and a 2 MiB
  **encoded-text accounting** budget (not a measured JavaScript heap limit).
  Model/data identity or physical-state drift permanently bypasses the memo;
  disposal restores the original method and releases all entries.
- Only the private Worker URDF parser skips repeated, exactly unchanged joint
  assignments. First assignments still compute the original transform, and
  `Object.is` keeps signed-zero distinctions. Live and independent parsers retain
  their previous default behavior.
- Camera feeds use the bundled renderer's asynchronous PBO readback, with the
  original HDR/ACES/sRGB display path. One outstanding read owns its buffer;
  focus, arm, theme, task/reset and visibility changes invalidate old frames.
  Renderers without the asynchronous API retain the synchronous fallback.

The [combined paired benchmark](planning-combined-final.json) runs four actual
G1 pipe snapshots twice: 70.10 s versus 55.92 s in aggregate (20.23% less time).
Every tested safety-check input/result, candidate count, full plan/command and
physical/derived state compares exactly. These are bounded Node measurements,
not a universal speedup, reduced search count, changed physics, or browser FPS.

Actual in-app-browser samples at 1280×720 show camera-visible frame interval P95
changing from 24.7 ms to 9.5 ms. Ordinary idle/run rendering was already near the
display's 120 Hz limit; the measurements do not support claiming a general FPS
increase. See `browser/baseline-*` and `browser/optimized-*` for workload, frame
intervals, long tasks and source hashes. Samples used this desktop's warm cache;
390×844 is viewport emulation, not a physical-phone or cold-load benchmark.
`browser/optimized-desktop-ready.json` separately records actual WebGL2 fixed-
scene synchronous/async pixel equality at three exposures and renderer-state
preservation. It is not robot-contact or FPS evidence.

The current 25 Worker contracts, 12 ordinary physical grasps, three physical
stacks (nine grasps), and UI/resource/camera contracts passed locally. Full
72-pipe acceptance is a separate long-running check, not implied by those
contracts. GitHub Pages runs the complete suite and its source-bound verifiers
before deployment, and preserves the reports as the
`multi-performance-v2-evidence` workflow artifact. Check that run's result before
claiming the complete current release has passed.

## Release regression

```sh
npm test
npm run test:pages
npm run test:acceptance
```

These commands write fresh evidence here. `test:pipes:quick` cannot substitute
for the full 72-pipe regression. Physics stays at 1 ms, with original controller
budgets, contact/lift/release/support requirements and collision/precision gates.

For the optional large planning benchmark, first expand the preserved input:

```sh
gzip -dk work/validation/performance-v2/g1-pipe592-snapshots.json.gz
node work/test-planning-performance-v2.mjs
```

The 78 MiB raw JSON is generated/ignored; its lossless gzip archive is included.
Regenerating snapshots with `node work/benchmark-planning-hotspots.mjs
--snapshots-only` uses that checkout's sources and is not the same as rebinding
historical measurements to a later implementation.

Shipping changes in this round are the measured worker planning cache, the
worker-only unchanged-joint parser fast path, and asynchronous camera readback
with a stale-frame-safe queue. Their final acceptance reports must match the
published source cohort. Browser/GPU observations are separate from Node
contract fixtures and mesh-free CPU measurements.

## Unpublished motor-view experiments

`optimized-dynamics-proxy-initial.mjs` and
`optimized-dynamics-own-getter.mjs` are **experimental snapshots, not released
runtime modules**. Nothing in `dist/` imports them. The own-getter adapter was
moved out of `dist/multi/` before release acceptance; its bytes were preserved.

The paired ordinary/pipe/stop/reset experiments preserved every tested 1 ms
physical and motor-command state. Their timing gain was modest and did not
benefit C5 consistently, so neither adapter is enabled in the published app.
These experiments do not support a browser FPS or universal speedup claim.

Historical JSON reports retain the source hashes that were actually measured.
They are not rebound to later source edits, renamed snapshots, or the final
published source cohort. The initial Proxy report's original module bytes are
preserved in `optimized-dynamics-proxy-initial.mjs`; the own-getter bytes are
preserved in `optimized-dynamics-own-getter.mjs`.

To repeat an experiment against the current checkout:

```sh
node work/test-multi-runtime-perf-v2.mjs
node work/test-dynamics-adapter-perf-v2.mjs
node work/profile-multi-runtime-perf-v2.mjs
```

New default outputs go under `rerun/`, leaving the recorded historical reports
untouched. These CPU-heavy commands are development experiments, not an extra
release requirement. The adapter's lightweight invalidation-only checks are:

```sh
node work/test-dynamics-adapter-perf-v2.mjs --contracts-only
```

The profile driver also accepts `--from-profile=/absolute/runtime.cpuprofile`
and `--benchmark-report=work/validation/performance-v2/profile-experiment-run.json`
to classify a retained raw trace against its matching historical benchmark.
