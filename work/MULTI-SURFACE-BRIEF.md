# Multi-robot laboratory redesign

Mode: Operate. Code-led replacement of the /multi/ visual world only.

## Confirmed brief

On 2026-10-16 the owner rejected the dark reference-like presentation and asked
for a refined, futuristic opening, visible robot top lighting, and two switchable
scenes. After inspecting the first build the owner approved 深空指挥舱 but rejected
the overly white 极昼实验室. The current pair is 深空指挥舱 (default) + 轨道机库,
both in the approved lit graphite-metal family. Environment, floor, light and
interface change together. Robot placement, physical behavior and task progress
must remain unchanged; individual routes retain the old appearance.

## Direction contract

THESIS: A working robot demonstration presented as a lit orbital robot laboratory.
The robots, not neon chrome or invented telemetry, own the screen.

OWN-WORLD: 深空指挥舱 uses graphite-blue structure, an observatory aperture,
neutral-white overhead pools, restrained amber edge light and cool mint controls.
轨道机库 inherits this readable depth, with blue-gray bulkheads, structural bay
doors, warmer work lights and champagne controls, not white walls or black voids.
Real robot finishes and red/blue/yellow object semantics stay intact.

STORY: Choose participants and a task, start together, inspect the real work and
camera/force feedback. Switch laboratory lighting at any point without restarting,
moving, hiding or replacing the robots or changing the current view.

FIRST VIEWPORT: The shared 3D work area fills the left, with three full robots and
grounded tables. A two-choice scene control is always visible in its top toolbar.
A precise 340px right panel leads with participant rows and the common task/start;
progress and real observation follow. At <=900px the 58svh scene stays above
the same ordered controls, and toolbar choices remain reachable.

FORM: User-pinned dual laboratory direction; concept calibration 4ea0c164 assigned
grounded candidate 7 (observatory lightwell). Familiar native web controls remain.
Signature: one light-field relay changes the actual overhead illumination,
architectural set, floor and interface in place, with no decorative perpetual motion.

FINISH: unreviewed and undocumented is unfinished; this build ends with the finish review, the verdict, DESIGN.md, and every shipping raster carrying its provenance

## Grounding and calibration

Seven considered families, ordered before committing the approved direction:
optical cleanroom (silver-white planes), aerospace exhibition (neutral-lit metal),
science-museum graphics (clear functional coding), technical monograph (rigorous
readable hierarchy), precision titanium object packaging (controlled joins),
photographic cyclorama (honest key/fill/rim exposure), observatory lightwell
(architectural aperture expressed as two lit orbital environments). The seventh carries
the approved two environments without borrowing a fictional control interface.

The calibration catalog was available after one permitted GET retry. The streaming
wall, lexicon, warm consumer-card surface, catalog sleeve, garden map and darkroom
record were declined as replacement worlds: hiding controls/art in focus, dense
print apparatus, an action buried in cards, decoded labels on black, competing
object-color regions, or hatch overlays respectively weaken this product's opening
and its clearly labeled task compared with the pinned laboratory.

Raised disciplines retained, not motifs: streaming focus prioritization;
lexicon hierarchy; consumer touch targets; sleeve restraint; map semantic-color
separation; darkroom exposure consistency. No catalog palette, layout costume,
label encoding, artwork or commercial claims are adopted. The existing approved
scope/scene answer is the direction decision, not a second unanswered style round.

## Boundaries and implementation

No runtime/controller/URDF/MJCF, source optical mounts, original robot materials,
task selection or scheduling changes. New visual modules live in multi/visuals/,
outside the existing physical-source manifest. Stage geometry is scene-owned,
not a station child or a draggable target. Table dimensions, support heights,
1.6m station spacing and real idle/motion framing policies remain.

Switch in-place, preserve camera/orbit/task identity, catch unavailable/invalid
storage, support keyboard/focus and reduced motion, allocate resources once and
dispose only environment-owned resources. Camera previews need the same explicit
ACES/sRGB display treatment as the main view, not clipped linear pixel copies.

Reuse the 9c34842 source-bound physical reports unchanged if their complete
physical dependency hashes still match. Re-run new display/theme, UI, framing,
path/cache and native two-scenes × desktop/phone checks. Keep source review,
pixel review and hardware safety claims separate.

## Scope of durable design

At finish replace/merge DESIGN.md and its sidecar with the implemented multi theme
tokens and components, explicitly retaining the legacy style scope for independent
R1/G1/C5/reference routes. No shipping image asset is needed: robot assets remain
original, and the new set/lighting are real Three.js geometry and procedural maps.

## Later scoped runtime refinement · 2026-10-16

The owner subsequently requested fixes for three-robot lag, automatic camera jumps
and unreachable-object hangs. This authorizes the multi runtime/planning wrapper
changes documented in `work/validation/performance/README.md`, superseding the
redesign-only scheduling restriction above for this separate fix. The approved
two visual worlds, independent robot pages, models and physical precision gates
remain unchanged. Start/reset/task/resize preserve user camera position; initial
load, participant changes and explicit view controls may refit it. Cooperative
planning is visible and cancellable between complete candidates; no promise of
frame-sized planning or arbitrary reachability is introduced. Current physical
evidence was rerun into new paths; retained older reports remain historical.
