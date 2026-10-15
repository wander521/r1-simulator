# AUBO C5 manipulation extension

Mode: Operate. Extend the existing reference workcell, without changing its visual identity or the retained R1/G1 pages.

The user requests a new `dev_aobo_c5` branch, an independently located genuine AUBO C5 URDF, full functional tests and a GitHub push. The user explicitly permits the existing Unitree Dex1 gripper if the official C5 is bare-flange. Preserve the exact original C5 model and source provenance, generate the mounted C5/Dex1 assembly separately, and distinguish the simulated adapter from a verified physical installation.

## Direction contract

THESIS: Replace the actual mechanism with one six-axis C5 and a contact-driven Dex1, not a reskinned humanoid or an i5 mislabeled C5.

OWN-WORLD: Inherit the reference dark-blue workcell, cyan Chinese controls, readable system type, red/blue/yellow task objects, flashing selection and matching placement frames.

STORY: Select and reposition a task object, orbit the scene, run physical sort/stack, inspect two camera feeds and measured contact/joint telemetry, then stop or reset safely.

FIRST VIEWPORT: A mounted C5 and grounded worktable lead the scene; the existing task panel remains at right. Six real joint controls and one gripper opening replace G1's seven-axis and left/right controls. Mobile retains the stacked scene/panel layout.

FORM: Precisely scoped, code-led robot extension of the incumbent design; no identity exploration or concept seed is required. Preserve working gestures and collision-checked manual control.

FINISH: unreviewed and undocumented is unfinished; this build ends with the finish review, the verdict, DESIGN.md, and every shipping raster carrying its provenance.

## Ordinary-extension documentation check

The C5 surface inherits the incumbent system rather than establishing a new visual identity. It loads the existing reference stylesheet and reuses the established workcell lighting, reflections, background, floor, and Dex1 optical mount. Its route-scoped CSS preserves the cyan controls, system typography, tabular measurements, 900px stacked-layout breakpoint, 58svh mobile scene, focus treatment, and reduced-motion behavior. The rendered desktop and 390px mobile checks showed the grounded robot/table, both camera feeds and six-axis controls without horizontal overflow.

Robot-specific changes are task truth, not system changes: the original C5 3DS silver/gray/blue face colors remain intact; the attached Dex1 is black; a grounded cylindrical support replaces humanoid feet; six arm axes and one opening replace the seven-axis/left-right interface. No G1 humanoid paint or posture copy carries into C5. Existing `DESIGN.md` and `.impeccable/design.json` remain unchanged. Their G1-specific seven-axis/dual-arm examples are pre-existing surface descriptions, not a request to rewrite the global identity in this extension. No new shipping raster was created; the C5 model and material provenance is recorded separately in the asset manifest.

Rendered verification used the native browser screenshot interface; captures were displayed inline, but that interface exposed no file-path option, so no on-disk screenshot artifact is claimed. The browser interaction evidence and exact source hashes are in `work/validation/c5/browser-final.json`. Touch input, actual-robot calibration and contact-driven task success require their own evidence and are not inferred from this documentation check.
