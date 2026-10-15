# G1 photo-guided appearance and front camera

The two user images are the visual authority for this change. The simulator's existing workcell and UI stay in place; the supplied G1 URDF/STL and Dex1 grippers stay intact.

| Visible part | Finish |
| --- | --- |
| Torso, shoulder caps, elbow covers, outer hip covers | Silver white, moderately reflective |
| Arm cylinders, wrists, thighs, knees, shins, ankle mechanisms | Satin silver gray |
| Head crown and rear cover | Dark metallic gray |
| Neck, pelvis, hip motors, waist | Graphite |
| Front oval face and sensor windows | Dark matte face with black, low-roughness windows |
| Feet | Black with a rougher finish |
| Original Dex1 base and both sliding fingers | Black; no replacement with the five-finger hands shown in the color reference |

The source head is one connected 18,654-triangle mesh. Its link-local coordinates span X=-66.36..73.98 mm, Y=-77.77..77.82 mm, Z=324.80..530.57 mm, where +X faces forward. The neck occupies its lower region. The face mask follows an oval centered at Z=451 mm, with 64 mm lateral and 69 mm vertical radii; a forward-X test excludes the rear crown. Material groups preserve the original vertex positions and normals. Resulting head surface area: crown 36.65%, neck 39.19%, face 18.56%, sensor windows 5.59%. Every finish has zero emission.

The camera now follows the photo's small black single-lens form, on the front of the palm's +Z dorsal side. Its optical center is (+60,0,+60) mm in `dex1_base_link`, looking at TCP (+120,0,0) mm. The URDF already advances this base by 41.5 mm from the wrist; the visual mount does not add that offset twice. The camera body is 28×32×16 mm. The visible lens center and the live RGB camera coincide. Both arms use the same local mounting frame and rotate with their grippers.

The virtual vertical field of view is 76 degrees to include the close Dex1 fingertips and the approach target. This is a simulator framing choice, not a claimed hardware calibration. The camera and bracket stay at least 11.4 mm above the fingers' complete sliding sweep.

Run `node work/check-g1-camera-materials.mjs` for reproducible source-geometry checks. It checks all 39 visual instances, preserves their source bytes/position/normal hashes, verifies both default main-camera frustums, and projects 54 wrist-camera groups over both arms, three openings, fingertips, TCP and the three target shapes at contact and 130 mm approach.

`photo-camera-material-check.json` is the current evidence and supersedes the earlier R1-derived wrist-camera projection files. The check is geometric only: CUA still reports `Browser is not available: iab`. Rendered appearance, shadows, occlusion, highlights and touch operation have not been newly verified.
