# Scoped design documentation validation

Recorded on 2026-10-16 for release `20261016-lab-2`.

`DESIGN.md` merges the finished `/multi/` laboratory system into the prior record. It explicitly retains the separate appearance of `/Rebot_Arm_AGV/`, `/r1/`, `/g1/` and `/c5/`; it does not direct those routes to adopt the new palette. The companion `.impeccable/design.json` uses schemaVersion 2 and extends rather than duplicates frontmatter primitives.

## Extraction

The final built sources were inspected: multi HTML, stylesheet, shared workcell, theme controller, scene environment and camera-feed display pass. The incumbent reference stylesheet and R1/G1/C5 overrides were checked to preserve source-backed legacy values. Approved direction and scope came from `PRODUCT.md` and `work/MULTI-SURFACE-BRIEF.md`. Existing desktop and mobile captures were inspected for vocabulary and scope; the finish-review verdict remains in `finish-review.md`.

| Recorded primitive or pattern | Count |
| --- | ---: |
| Source-matched interface colors | 46 |
| Typography roles | 7 |
| Corner radii | 9 |
| Spacing steps | 8 |
| Component token variants | 10 |
| Self-contained component snippets | 10 |

The source declares 19 CSS custom properties per multi theme. Equivalent focus/selection primitives and shared danger color are consolidated in the frontmatter; HUD shadows move to extensions. Scene-material values, chart palettes, renderer configuration, persistence and responsive breakpoints also live in extensions rather than unsupported frontmatter groups.

The ten snippets comprise seven multi patterns and three legacy patterns. The prior legacy joint chip and target slider are retained together; legacy container and camera examples are retained together. Every class is `ds-` prefixed. Every used CSS variable is locally declared or has a fallback. Each multi snippet includes locally scoped night and hangar values and dark native-control color scheme, so it does not require app CSS or a framework runtime. Static examples are identified as samples; blank camera canvases do not invent live imagery. Synthetic eight-step tonal ramps are preview metadata, not new shipping palette primitives.

## Checks performed

- Parsed frontmatter as YAML using Ruby's standard YAML parser and parsed the sidecar as JSON.
- Checked the allowed frontmatter groups and eight component-property names, resolved every component token reference, and rejected primitive-to-primitive references.
- Checked schemaVersion 2, component kinds/count, component references, `ds-` class scope, local-or-fallback CSS variables, eight-step color metadata and the canonical eight Markdown sections in order.
- Matched all 46 color values to the appropriate multi or legacy stylesheet source.
- Recomputed all seven frozen multi production-source SHA-256 values from `browser-final.json`; all matched.
- Confirmed that only the two design records and this validation note were edited by the documentation handoff.

No official external DESIGN.md/schema JSON-Schema linter was available, so these are parser and schema-contract checks, not a claim that an external linter ran. No detector, application, browser task, physical test or hardware check was rerun for this documentation pass.

## Limits retained

The legacy record previously treated control containers and buttons as one radius and used approximate camera/task snippet spacing. The merged record distinguishes actual legacy button/container radii and uses the current source spacing; independent application files were not changed. Other incumbent stylesheet one-offs remain outside the reusable extraction. Numerical simulation reports do not establish visual, touch, optical or hardware approval. Native observations, the single in-progress night-to-hangar switch, fallback-path limitations and historical physical reports retain the limits stated in `browser-final.json` and `finish-review.md`.
