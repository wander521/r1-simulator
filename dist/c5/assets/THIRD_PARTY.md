# AUBO C5 source assets

The original C5 arm files are imported verbatim from the manufacturer's public
repository, [AuboRobot/aubo_description](https://github.com/AuboRobot/aubo_description),
at commit [`47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1`](https://github.com/AuboRobot/aubo_description/tree/47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1).
`SOURCE_MANIFEST.json` records every upstream path, Git blob identity, byte length,
and SHA-256 digest. `source/package.xml` and `source/README.md` preserve the
upstream package declaration and documentation.

## License evidence and limitation

Upstream `package.xml` version 1.3.3 declares `<license>BSD</license>` and credits
Allen Liu (`liug@our-robotics.com`) as author/maintainer. At the pinned commit,
the source tree provides no `LICENSE`, `COPYING`, or `NOTICE` file, and these C5
URDF and mesh files have no model-specific copyright/license header. Upstream
does not identify which BSD variant applies. This project preserves the exact
declaration; it does not invent a copyright notice, select an unsupported BSD
variant, claim a warranty, or suggest manufacturer endorsement.

The manufacturer's older, separate `AuboRobot/aubo_robot` repository supplies
BSD-3-Clause terms, but the C5 resource set is not present there. That separate
license must not be represented as demonstrated licensing evidence for these
particular C5 files. Redistribution should retain all supplied source and
license evidence; users requiring a complete SPDX-specific or commercial
license assurance should obtain clarification from the upstream rights holder.

## Model selection and deliberate repairs

- `aubo_C5.urdf` is the exact original, not the i5 URDF and not renamed i5 mesh
  geometry. The imported collision/visual files come from `meshes/aubo_C5`.
- The original C5 URDF references `.DAE` visual filenames that do not exist in
  the pinned C5 mesh folder. The corresponding official visual files are `.3ds`.
  The original URDF is retained unchanged; any corrected mesh paths or converted
  render assets belong only to a separately labelled, derived integration.
- The seven official visual `.3ds` files use millimeter-sized vertex coordinates
  (for example, link 2 spans -59 to 467 on its long X axis), despite containing a
  master-scale value of 1. The source collision STLs use meters. Render conversion
  must multiply visual vertices by 0.001 without rescaling the URDF joint origins,
  inertial data, or collision meshes. All seven 3DS local-coordinate matrices are
  identity. Together they contain 14,862 vertices and 15,469 visual triangles.
- Preserve per-face material groups: source diffuse RGB values include light
  silver `(217,216,233)`, gray `(127,127,127)`, and blue `(24,76,198)`, with
  `(185,185,185)` on the base and flange. Material indices are not consistent
  across links. The source 3DS files do not reference external texture files.
- `meshes/aubo_C5_30` is an upstream symlink to `aubo_i5`. It is deliberately not
  used as the C5 display model. `aubo_C5_39` is a separately named CANLite version;
  its visual assets are not silently substituted either.
- The arm has six revolute joints and an unactuated fixed world joint. The
  official model ends at `wrist3_Link`; it contains no gripper, camera, fingers,
  or calibrated tool center point. Added tooling is project-specific simulation
  equipment, not a claim about the manufacturer's standard C5 product.
- Preserve exact upstream joint origins/axes/limits, mass, center of mass, and
  inertia in any generated dynamics model. Upstream extended `property` fields
  are not assumed to be calibrated MuJoCo motor parameters.

This is a web simulation, not a controller or an approved real-machine safety
model. The source URDF is nominal and not calibrated to a particular physical
serial number.
