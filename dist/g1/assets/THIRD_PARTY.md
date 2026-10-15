# Unitree G1 / Dex1-1 model sources

This directory vendors the **unaltered** URDF requested by the user and only its
38 unique mesh dependencies. No prebuilt MJCF, policy weights, DDS runtime,
dataset or alternative robot/hand variant is imported.

- Requested file: <https://huggingface.co/lerobot/unitree-g1-mujoco/resolve/main/assets/g1_29dof_with_dex1_1.urdf>
- Pinned Hugging Face repository revision: `68459ed68f6f68e1f661091dfcb6ebce44681aec`.
- Pinned URDF: <https://huggingface.co/lerobot/unitree-g1-mujoco/resolve/68459ed68f6f68e1f661091dfcb6ebce44681aec/assets/g1_29dof_with_dex1_1.urdf>
- URDF original size: 39,614 bytes; SHA256: `0a25ea0ed77740ca66af74a9f3c2dae649324a369351bf2f6d5cab3555b5f059`.
- Per-file source URL, byte size and SHA256: [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json).
- Reproducible importer/verifier: `work/import-g1-assets.mjs` at repository root.
  Run `node work/import-g1-assets.mjs --verify` to check the vendored bytes.

## Attribution and license evidence

Unitree robot descriptions and mesh assets originate from
[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros).
That upstream repository's root license is **BSD-3-Clause**, retained verbatim
as [licenses/Unitree-BSD-3-Clause.txt](licenses/Unitree-BSD-3-Clause.txt).
The copied license is pinned to upstream commit
`5994d4faef0a9cadd3287f8de0199a67eeb2a259`:
<https://github.com/unitreerobotics/unitree_ros/blob/5994d4faef0a9cadd3287f8de0199a67eeb2a259/LICENSE>.
The upstream G1 description README identifies the Dex1 model as a 29-body-joint
robot with two independently driven finger slides per hand:
<https://github.com/unitreerobotics/unitree_ros/blob/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/g1_description/README.md>.

The body description is also distributed by the official
[unitreerobotics/avp_teleoperate](https://github.com/unitreerobotics/avp_teleoperate)
repository. Its `assets/g1/g1_body29_hand14.urdf` is byte-identical to the HF
body source: 51,870 bytes, SHA256
`8bbf006633fc50b616f665c7a970780cc296577a0adfd7d28b049e751c238735`.
The verified official file is pinned to commit
`817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b`:
<https://github.com/unitreerobotics/avp_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/assets/g1/g1_body29_hand14.urdf>.
AVP's Unitree 2025 Apache-2.0 copyright/license declaration is retained verbatim
as [licenses/Unitree-AVP-Apache-Notice.txt](licenses/Unitree-AVP-Apache-Notice.txt),
with the official [Apache-2.0 full text](licenses/Apache-2.0.txt). Preserve both
this declaration and the original Unitree BSD notice; do not label all source
model files as exclusively Apache-2.0 or exclusively BSD.

### Portable-URDF provenance check

The pinned HF snapshot has no top-level LICENSE/model-card license field.
To distinguish a missing wrapper license from unknown model content, the exact
portable assembly was independently reproduced in memory:

1. Parse HF `assets/g1_body29_hand14.urdf`, verified byte-identical to the official
   AVP file above, and remove the 16 old hand links and their 16 joints.
2. Append the six Dex1 links and six joints from HF's reference mode-15 URDF.
   After resolving global material colors and mesh-path basenames, those 12
   elements are semantically identical to the official BSD model
   [g1_29dof_mode_15_with_dex1_1.urdf](https://github.com/unitreerobotics/unitree_ros/blob/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/g1_description/g1_29dof_mode_15_with_dex1_1.urdf).
   The complete reference mode-15 URDF also matches the official one
   semantically; its differences are comments/formatting/material placement.
3. Rename the root robot, set portable compiler paths, rewrite mesh filenames
   to the two relative directories, and serialize with two-space indentation.

The result is **byte-identical** to the user-requested 39,614-byte URDF, including
the SHA256 recorded above. All 37 retained body links and 36 body joints match
the official AVP source without numerical edits. All six gripper links and six
gripper joints match the official Unitree Dex1 source. No HF-authored inertia,
geometry, frame, limit, or joint-tree content was found in this portable step;
it is a mechanical combination/path adaptation of the identified Unitree models.
This is evidence for upstream-derived model attribution, not a new license grant.

No HIW-500 Python/model-builder code, control policy, approximate camera
implementation, or prebuilt MJCF is vendored here. Consequently a HIW code
license is not presented as the license of these Unitree model assets. The HF
mirror/source credit is preserved by this document and each asset's manifest
URL. No unresolved substantive model-content provenance gap was identified by
the checks above.

## Model identity and measured geometry

The exact URDF contains 43 links and 42 joints: 9 fixed joints, **29 revolute body
joints plus 4 prismatic Dex1 finger joints**. The body is two six-joint legs,
three waist joints and two seven-joint arms. “29DOF” is not the total joint count
with these grippers attached.

All mesh paths in the URDF are relative to this directory (`meshes/` or
`dex1_1/meshes/`). STL geometry is retained without scaling or recoloring; the
URDF supplies visual RGBA materials. Original mass, COM, full inertia tensors,
axes, joint frames and limits remain in the unmodified source URDF.

The similarly named prebuilt HF `g1_29dof_with_grippers.xml` is **not** a strict
conversion of this portable URDF: its waist/shoulder offsets, torso/waist
inertias and body mesh revisions differ; finger lower limits are trimmed to
−0.023m rather than the source URDF's −0.020m. A generated project MJCF must use
the requested URDF as its source rather than silently mixing these variants.

Dex1's local +X points toward the fingertips and the closing direction is local
±Y. Its base is fixed 0.0415m forward of `*_wrist_yaw_link`; the repository's
grasp site is at `[0.12,0,0]` in `*_dex1_base_link` (equivalently
`[0.1615,0,0]` in the wrist frame). Both hand sides use finger1 axis `[0,-1,0]`
and finger2 axis `[0,1,0]`.

The closest opposing collision-mesh points at zero finger coordinates are
`y=±0.02293841727m`. The inner faces are canted by approximately 0.5°;
at the TCP's x=0.12m their opposing-face gap is approximately 46.279mm.
With equal finger positions q, the minimum whole-mesh gap is approximately
`0.04587683454 + 2*q` metres. Therefore the source range
`[-0.020,0.0245]m` corresponds to approximately 5.877–94.877mm opening, rather
than zero-to-49mm. Each source finger effort limit is 20N, velocity limit
0.2m/s. Do not reuse the R1 zero-offset opening conversion or six-axis mapping.

At all-zero joints with pelvis origin at zero, the left foot STL minimum world
Z is `−0.7922728974794127m`; the source foot-contact spheres imply
`−0.7918637524222113m`. Their difference is approximately 0.409mm. A pelvis
height `0.792273m` aligns the visual soles with the ground; the zero-joint
shoulder pitch origins are then approximately `1.084053m` above ground.
These measurements do not imply a walking/balance controller or a free-base
standing validation.

The supplied `dex1_col_1/2.stl` meshes have 1516/1456 triangles and
760/730 exact unique vertices. Each contains one edge shared by four faces;
they are not strictly watertight manifold convex solids. Their finite signed
tetrahedral volumes are approximately `1.1525273e-5` and `1.1300548e-5m³`.
MuJoCo's standard mesh contact uses a convex hull, which fills concave recesses;
loading these original collision meshes is not a claim of exact non-convex
triangle contact. The original opposing canted grip surfaces are retained.
