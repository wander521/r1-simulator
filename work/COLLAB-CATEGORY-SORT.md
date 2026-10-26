# 四类分拣调整 · 2026-10-26

Scope: `/collab/` only. Keep the shared physical world, independent station
scheduling, preparation postures and existing transport aisle.

- C5 routes by immutable material category, not incoming bin number: red
  cylinder, yellow cylinder, blue box and green box each have their own region.
  Four finite regions contain two distinct contact-supported positions each.
  Assignment is independent of admission/processing order and reserves unique
  positions for all eight original parts. No spawning, stacking or attachment.
- Colored region outlines and Chinese labels use the existing physical part
  palette. Cabinet signage uses 96px canvas glyphs instead of 48px on the same
  physical plane (exactly twice the previous glyph size).
- The rack backing ends at 1.04m, just above the 1.00m upper shelf; mesh and
  collision geometry consume the same definition in `cell-fixtures.mjs`.
- Simulation target rate is 2.4×, a 20% increase over 2×. The physics timestep
  stays 1ms; controller forces, friction and movement profiles are unchanged.
  The admission window is still 300 real seconds, and catch-up stays bounded.
  Hardware workload may limit achieved rate.

## Evidence and limits

The first development integration found the old per-bin placement index still
used in inventory verification. A correctly placed blue box was rejected.
Preserved evidence: `work/validation/collab/category-sort-physical.json`.
The fix checks the immutable category, region and sub-position, including
previously placed parts; it does not relax position or contact thresholds.

Desktop (1440×900) and mobile (390×844) captures:
`output/playwright/collab-sort-desktop.png` and `collab-sort-mobile.png`.
Mobile document width equals viewport width. The only initial console network
error is the incumbent missing favicon; no application exception was observed
in the idle inspection. Layout, typography and material treatment preserve the
incumbent industrial scene (Impeccable refinement, not a visual redesign).

The design detector returned six advisory findings only: existing progress
radius, table border and the four requested semantic part colors. Global design
sidecar drift and unset buildPath remain untouched and outside this request.

## Final verification

- `npm run test:collab:bins`: 57/57 pass, including category mapping, duplicate
  position rejection, contact-backed category verification, rack geometry and
  the 2.4× fixed-step / real-clock contract.
- The second accelerated integration reached 6/8 sorted parts without station
  faults, but hit its 480-second diagnostic deadline while sharing CPU with the
  browser test. `work/validation/collab/category-sort-physical-final.json` is an
  **incomplete timeout**, not a passing full run.
- The real browser completed the full workflow: 300.0186s admission, 575.0s
  from first running sample to completion including drain, all 8 original parts
  in their four correct regions (two each), 4 empty bins returned and all three
  robots plus R1 base home. All 20 physical manipulations carry dual-contact,
  lift, support and inventory evidence. Maximum final placement error: 3.73mm.
  No runtime/station errors, automatic camera movement or long tasks; p95 frame
  interval 9.1ms. The 2.4× target was CPU-limited during parallel diagnostics,
  not a measured 2.4× guarantee on this device.
- Browser acceptance: **15/15 pass**, saved in
  `output/playwright/collab-sort-browser-checks.json`; full measured data in
  `output/playwright/collab-sort-browser.json`; completion capture in
  `output/playwright/collab-sort-complete.png`.

This is one full new-configuration browser run, not the interrupted historical
20-round soak and not proof of zero future faults or hardware safety. No GitHub
push was performed for this change.
