# Collaboration loading optimization · 2026-10-26

Scope: `/collab/` presentation loading. Physics geometry, controller parameters,
station poses and task semantics retain their source definitions.

## Changes

- A small visual configuration imports materials and robot profiles without
  importing the simulation and planning runtime into the main entry graph.
- `npm run build:collab:visuals` creates deterministic gzip transport copies and
  a source-hash manifest. All 72 STL assets total 51,821,798 bytes originally and
  20,496,031 bytes compressed: 60.45% fewer asset bytes. The active scene loads
  70 STL assets; its compressed responses total 20,444,281 bytes. Two collision
  assets in the manifest are not requested by the visual loader.
- Two short-lived workers fetch, decompress and parse STL files using the same
  Three.js STL parser. Transferable attribute arrays avoid copying triangle
  buffers to the UI thread. Original assets are the fallback when the manifest,
  compressed response or gzip decompression is unavailable.
- A view-wide cache normalizes URDF paths and shares repeated Dex1 geometry
  across robots. Geometry belongs to the view's disposal registry; workers are
  terminated after loading and pending work is rejected on disposal/failure.
- Preload the two entry modules; keep the existing full release-version chain.
  Read-only load timings distinguish scene, visual models and physics readiness.

## Measurements and limits

Desktop Chromium, fresh browser contexts, localhost, same machine:

| Run | Measured ready time |
| --- | ---: |
| Before, first observation of ready | 3,365 ms |
| After, first observation of ready | 2,837 ms |
| After, internal ready mark, subsequent cold context | 2,435 ms |
| After, reload in that context, internal ready mark | 2,135 ms |
| Final version, fresh context while IAB was also open | 3,280 ms |

The first two rows share the same observation method. Cold observations vary
roughly 2.4–3.3s with concurrent browser/CPU activity; the network byte reduction
is the stable result, not a guaranteed percentage improvement in total ready time.
These are local samples,
not a statistical/device-wide guarantee. The marks show physics initialization
is now the remaining critical path (about 2.4s), with visual models ready in
about 0.9s. Page Resource Timing excludes dedicated-worker mesh payloads; do not
describe its smaller page-byte value as total network traffic. Context response
events separately confirmed 70 compressed STL responses and zero raw STL on a
normal load. A page-CDP 10Mbps baseline was collected, but is not used to claim
a slow-network before/after result because requests moved into workers.

## Verification

- All 72 compressed files gunzip to byte-identical original STL data, with
  SHA-256 checks. Lightweight configuration matches runtime configuration for
  robot IDs, labels, root heights, URDF URLs and material functions.
- Normal cold load, warm reload and forced missing-manifest fallback all reach
  ready with three robots and both rack support meshes. Fallback requested 70
  original STLs and completed successfully.
- Forced original-asset failure shows the retry button with the failed resource
  named and no uncaught page errors. Late progress messages no longer overwrite
  this error. Import maps precede module preloads. The decoder's STL import is
  versioned too: the existing IAB had cached an older bare-import parser and
  initially failed worker startup; after the version correction, the same IAB
  reaches ready with all stations waiting and the start button enabled.
  Actual browser proof: `output/playwright/load-optimized-user-browser.png`.
- Desktop 1440×900 and mobile 390×844 captures inspected:
  `output/playwright/load-optimized.png`, `load-optimized-mobile.png`.
  No overflow, changed appearance or initialization error was observed.
- A 15-second browser start/orbit/stop smoke run remained running with no station
  errors and an unchanged user camera. This is not a full five-minute physical
  grasp/endurance run.
- Release-chain test, syntax checks, lossless verification and diff whitespace
  checks pass. The design detector returned the six existing advisories only.

Regenerate gzip files and manifest together whenever source STL assets change.
Content-hash filenames are safe for static GitHub Pages hosting; files are
manually decompressed and do not require special server compression headers.
