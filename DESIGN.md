---
name: Robot manipulation simulator
description: Lit orbital laboratories for the multi-robot route, with independent reference workcells preserved.
colors:
  multi-night-accent: "#82eade"
  multi-night-selected-field: "#2a4956"
  multi-night-selected-ink: "#b8fff3"
  multi-night-action: "#9ae9df"
  multi-night-action-ink: "#12363d"
  multi-night-action-hover: "#b9f7ed"
  multi-hangar-accent: "#f3ca86"
  multi-hangar-selected-field: "#4c4c46"
  multi-hangar-selected-ink: "#ffdfaa"
  multi-hangar-action: "#f1cc91"
  multi-hangar-action-ink: "#302d25"
  multi-hangar-action-hover: "#ffe0af"
  multi-night-success: "#98eac8"
  multi-hangar-success: "#9de3c0"
  multi-danger: "#ffb5bf"
  multi-object-red: "#e93647"
  multi-object-blue: "#218ae9"
  multi-object-yellow: "#f0c945"
  multi-night-page: "#152431"
  multi-night-panel: "#1b2e3d"
  multi-night-surface: "#233c4d"
  multi-night-field: "#142a3b"
  multi-night-ink: "#edf5fa"
  multi-night-muted: "#b3c8d6"
  multi-night-edge: "#3d5669"
  multi-night-status: "#233f4c"
  multi-night-hud: "#233b4cf5"
  multi-hangar-page: "#222c35"
  multi-hangar-panel: "#273541"
  multi-hangar-surface: "#304451"
  multi-hangar-field: "#1d2d39"
  multi-hangar-ink: "#f0f4f6"
  multi-hangar-muted: "#bac9d1"
  multi-hangar-edge: "#52626c"
  multi-hangar-status: "#34434c"
  multi-hangar-hud: "#2e404ef5"
  legacy-background: "#01030d"
  legacy-text: "#eaf7ff"
  legacy-focus: "#8ae8ff"
  legacy-selected: "#46d9f6"
  legacy-selected-ink: "#e6f9ff"
  legacy-selected-field: "#104f71a3"
  legacy-field: "#051324"
  legacy-camera: "#010713"
  legacy-joint: "#030b1de0"
  legacy-muted: "#a7c8d4"
typography:
  multi-body:
    fontFamily: '-apple-system, BlinkMacSystemFont, "PingFang SC", "Microsoft YaHei", sans-serif'
    fontSize: "14px"
    lineHeight: 1.55
  multi-headline:
    fontSize: "22px"
    fontWeight: 700
    lineHeight: 1.3
    letterSpacing: "-.025em"
  multi-title:
    fontSize: "15px"
    fontWeight: 650
    lineHeight: 1.4
  multi-label:
    fontSize: "12px"
  multi-status:
    fontSize: "13px"
    lineHeight: 1.6
  legacy-body:
    fontFamily: "SF Pro Text, PingFang SC, Hiragino Sans, Noto Sans JP, Microsoft YaHei, sans-serif"
  legacy-headline:
    fontSize: "20px"
rounded:
  multi-control: "8px"
  multi-hud: "10px"
  multi-segment: "7px"
  multi-chart: "6px"
  multi-badge: "5px"
  legacy-control: "12px"
  legacy-button: "8px"
  legacy-camera: "9px"
  legacy-chip: "10px"
spacing:
  compact: "6px"
  control-gap: "8px"
  field-inset: "10px"
  control-inset: "12px"
  toolbar-inset: "16px"
  section-gap: "18px"
  panel-inset: "20px"
  panel-bottom: "28px"
components:
  multi-night-primary:
    backgroundColor: "{colors.multi-night-action}"
    textColor: "{colors.multi-night-action-ink}"
    rounded: "{rounded.multi-control}"
    padding: "9px 12px"
  multi-night-primary-hover:
    backgroundColor: "{colors.multi-night-action-hover}"
  multi-hangar-primary:
    backgroundColor: "{colors.multi-hangar-action}"
    textColor: "{colors.multi-hangar-action-ink}"
    rounded: "{rounded.multi-control}"
    padding: "9px 12px"
  multi-hangar-primary-hover:
    backgroundColor: "{colors.multi-hangar-action-hover}"
  multi-night-secondary:
    backgroundColor: "{colors.multi-night-surface}"
    textColor: "{colors.multi-night-ink}"
    rounded: "{rounded.multi-control}"
    padding: "9px 12px"
  multi-hangar-secondary:
    backgroundColor: "{colors.multi-hangar-surface}"
    textColor: "{colors.multi-hangar-ink}"
    rounded: "{rounded.multi-control}"
    padding: "9px 12px"
  multi-night-selected:
    backgroundColor: "{colors.multi-night-selected-field}"
    textColor: "{colors.multi-night-selected-ink}"
    rounded: "{rounded.multi-control}"
  multi-hangar-selected:
    backgroundColor: "{colors.multi-hangar-selected-field}"
    textColor: "{colors.multi-hangar-selected-ink}"
    rounded: "{rounded.multi-control}"
  legacy-selected-task:
    backgroundColor: "{colors.legacy-selected-field}"
    textColor: "{colors.legacy-selected-ink}"
    rounded: "{rounded.legacy-button}"
    padding: "6px 3px"
  legacy-camera-frame:
    backgroundColor: "{colors.legacy-camera}"
    textColor: "{colors.legacy-text}"
    rounded: "{rounded.legacy-camera}"
---

# Design System: Robot manipulation simulator

## Overview

**Creative North Star: "Lit orbital robot laboratory"**

The multi-robot route (`/multi/`) presents real robot finishes and working manipulation inside a grounded, illuminated orbital laboratory. 深空指挥舱 is the default: graphite-blue structure, a static observatory aperture, neutral-white overhead pools and mint controls. 轨道机库 is its companion: blue-steel bay doors, structural braces, warmer work-light details and champagne controls. Both preserve the approved depth and clear shell lighting; the owner rejected the overly white 极昼实验室.

This is a scoped merge of the built multi-robot design and the incumbent system. `/Rebot_Arm_AGV/`, `/r1/`, `/g1/` and `/c5/` retain the supplied dark-blue/cyan reference appearance. Tokens prefixed `multi-` apply only to `/multi/`; `legacy-` tokens preserve the independent reference workcells. Object swatches are shared by the two multi scenes and describe task choices. The route-specific composition and direction decision live in `work/MULTI-SURFACE-BRIEF.md`.

**Key Characteristics:**

- Clearly lit robot shells against graphite and steel-blue architecture.
- Compact, Chinese-labeled native controls with visible selected and focus states.
- One in-place scene choice changes the set, floor, light and interface together.
- Independent reference workcells keep their existing visual identity.

**The Route Scope Rule.** Apply each token family only to its named routes; this document does not recolor the independent workcells.

## Colors

The two `/multi/` palettes use restrained action color over layered metal neutrals. The frontmatter is the normative interface palette; the sidecar records scene-material and rendering settings separately.

### Primary

- **Deep-space mint** (`multi-night-accent`, `multi-night-action` and their selected/hover partners) marks selection, focus and the common start action in 深空指挥舱. Focus and selection intentionally share one accent primitive.
- **Hangar champagne** (`multi-hangar-accent`, `multi-hangar-action` and their partners) performs the same roles in 轨道机库. It is a scene treatment, not an additional task category.
- **Reference cyan** (`legacy-selected`, `legacy-focus`) remains the selection and keyboard-focus vocabulary of the independent routes.

### Secondary

- **Task red, blue and yellow** (`multi-object-red`, `multi-object-blue`, `multi-object-yellow`) identify the labeled object choices. Preserve their association with physical objects and matching placement frames across both scenes.
- **Success and recovery** use the theme's success token and shared `multi-danger`, always with written status. Do not substitute these for task-object colors.

### Neutral

- Each multi theme supplies page, panel, surface, field, ink, muted, edge, status and nearly opaque HUD roles. Fields recede inside the panel; selected fields raise a labeled option without an extra decorative glow.
- The legacy background, text, field, camera and joint tokens retain their dark-blue reference roles. Preserve the existing gradient containers and outlines rather than translating them to multi-panel colors.

**The Object Meaning Rule.** Theme changes must preserve red/blue/yellow object meaning and written labels. Scene accents never replace robot finishes or task colors.

## Typography

The multi interface uses its Chinese-capable system stack from `multi-body`. There is no separate display face. Its hierarchy is functional: headline (`multi-headline`) for the panel title, title (`multi-title`) for sections and legends, body (`multi-body`) for controls and names, label (`multi-label`) for supporting copy, and status (`multi-status`) for live progress messages. Robot names use medium-to-semibold weight; controls use weight 550 and the start action uses weight 700.

The independent routes retain `legacy-body` and `legacy-headline`. Their dense camera and telemetry labels are generally smaller than the multi interface; route-specific overrides remain authoritative. Do not impose the multi type scale on them.

**The Measurement Rule.** Use tabular numerals for counts, progress, angles and measured output. Keep source units and state honest: radians/degrees, N, N·m and mm stay attached to the corresponding values; unavailable measurements display a placeholder rather than fabricated telemetry.

## Layout

On `/multi/`, the scene and a fixed control column (340px) form a full-height grid (100svh, minimum 480px). The panel scrolls independently and uses top/side/bottom padding (23px / 20px / 28px). Sections are separated by thin rules and spacing (18px), not nested ornamental cards. The two camera feeds share a two-column grid with an 8px gap; metrics use three columns.

At widths up to 900px, the scene sits above the ordered controls with height 58svh and minimum height 340px. Panel sections center within a maximum width of 620px. The scene toolbar uses two rows until 1280px; at that wider breakpoint the scene choices and view actions share its top row. On small screens the scene choices span their own row. Keep these choices reachable and keep the entire robots, grounded tables and active work area readable under the existing framing policy.

The independent reference layout retains a flexible scene and a 310px panel. R1 stacks at 800px; G1 and C5 stack at 900px with their existing 58svh scene treatment. Preserve their camera-layout controls and joint annotations. A breakpoint here documents the source behavior; it is not permission to change another route.

## Elevation & Depth

Multi controls use tonal layers and edge contrast. Scene toolbar surfaces and station tags use a restrained HUD shadow; ordinary panel sections remain flat. Shadow values and state-transition timing are stored in the sidecar. Transitions run only when reduced motion is not requested; the set and stars do not add decorative perpetual motion.

The 3D space uses actual overhead spotlights, a shadow-casting key, fill/rim lighting, a receiving floor and procedural image-based reflections. ACES filmic tone mapping and sRGB output apply to the main view and its camera-feed display treatment. Deep-space retains its approved parameters; hangar changes its architecture, floor material, table tones and warmer rim/reflections within the same readable family. These are software rendering settings, not optical or hardware calibration.

Legacy workcells retain their dark layered gradient panels, outlined feeds, reference reflections and joint annotation depth. Selection pulses follow the incumbent reduced-motion treatment. No new raster imagery is required by the laboratory set: its architecture, floor seams, stars and reflection maps are code-generated Three.js resources.

**The Shell Legibility Rule.** Reveal original robot materials with light and reflection; do not repaint the robot with an emissive theme color.

## Shapes

Multi buttons, selected participant rows, status blocks and camera frames share `multi-control`. The scene switcher groups its two choices inside `multi-hud`, with `multi-segment` corners on each choice; compact charts and station tags use `multi-chart`. Use thin borders and quiet geometry rather than ornamental frames around every section.

Legacy containers retain `legacy-control`, buttons retain `legacy-button`, feed frames retain `legacy-camera`, and joint annotation chips retain `legacy-chip`. Physical placement zones remain four thin bars around an empty center. Original robot meshes and table dimensions remain the source of their silhouettes.

## Components

### Actions and scene choices

The multi start action is the strongest filled control, with minimum height 44px; secondary actions use a bordered surface and minimum height 40px. Hover uses the selected-field/edge pair, active secondary actions return to the field tone, and disabled controls reduce opacity to .45 with a disabled cursor. Text-only actions remain visibly labeled. The scene choices are native buttons in a labeled group, with `aria-pressed` exposing the active scene and a 36px minimum height inside their compact HUD.

**The In-Place Scene Rule.** Scene choice is presentation only: it changes the environment, floor, lighting, interface and chart palette without resetting an executing task, moving the robots or replacing the current camera view. Remember a valid choice when browser storage is available; unavailable or invalid storage falls back to 深空指挥舱.

### Inputs and participants

Use real labeled selects and checkboxes. Selects have field backgrounds, minimum height 42px and bounded width; checkbox rows expose participant names, descriptions and real load state. A checked participant row uses the selected-field tone. Object choices pair their swatch and written name and expose `aria-pressed`; the yellow cylinder uses a circular swatch. Keep task selection separate from the robot currently being observed.

Interactive multi buttons, selects, links, inputs and the main canvas use a visible 2px focus outline with 3px offset. Preserve native keyboard behavior, disabled state and the existing live status semantics. Do not replace controls with unlabeled decorative geometry.

### Progress and observation

Each robot has a written progress row, a real progress element and an explanatory note; success/failure changes the output color as well as its text. Feed frames have written camera captions above 4:3 canvases. Force and torque plots report the actual simulation history; missing values remain placeholders. The sidecar's camera examples are explicitly static frame samples, not invented live images.

### Independent workcell controls

Retain reference task buttons, loading/retry feedback, camera visibility/model/layout controls and checked joint target sliders. Joint annotations expand to bounded target sliders rather than direct writes to actual joint state. R1 and C5 retain their six-axis UI; G1 retains its seven-axis UI. Keep mm gripper travel separate from angles, reject colliding motions without changing the last safe target, and preserve both gripper openings when switching arms. The sidecar retains scoped reference task, joint and camera/container snippets for these routes.

## Do's and Don'ts

### Do

- Do apply the two orbital laboratory palettes only to `/multi/` and preserve the independent reference routes.
- Do keep neutral overhead lighting, readable original robot finishes and grounded worktables in both multi scenes.
- Do pair task colors, status colors and scene selection with written labels and exposed native states.
- Do preserve keyboard focus, native form behavior, reduced-motion handling and reachable scene choices at phone widths.
- Do keep blank-space camera gestures separate from object editing and scene switching separate from task state.
- Do keep measurements tied to real simulation output and distinguish software evidence from hardware behavior.

### Don't

- Don't reintroduce the rejected white 极昼实验室 as the companion to 深空指挥舱.
- Don't recolor robot meshes, task objects or independent workcells with the scene accent.
- Don't add invented telemetry, fake live-feed imagery or decorative perpetual motion.
- Don't reset tasks, alter robot placement or replace the current view when switching scenes.
- Don't infer visual, touch, optical or real-robot approval from numerical simulation tests.
- Don't approximate supplied robot silhouettes or replace G1 Dex1 with photographed five-finger hands.
