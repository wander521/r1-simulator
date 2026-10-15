# G1 standing manipulation

Mode: Operate. This is a robot/model replacement within the existing simulator, not a visual redesign.

The user's requirements are a preserved `dev_r1` branch and a new `unitree_g1` branch using the supplied G1 29-body-DOF + Dex1 URDF. Feet stay in place, legs and waist pitch/roll retain their standing configuration, and there is no crouch or walking. The user explicitly allows horizontal waist yaw to align the active arm before reaching. Raise and position the worktable to make the existing desktop tasks reachable.

The user's recording rejected upside-down, twisted-arm IK and arm–torso penetration. New motion must use the G1 native upright gripper frame, natural shoulder/elbow posture, complete original upper-body collision geometry, and swept checks for preparation, grasp, return, result actions and manual controls. The two supplied photos supersede the old R1 camera placement and unmodified gray materials: use a single camera on the front palm back (+X, +Z), silver-white/silver-gray shell, dark head/waist/hip mechanism and black feet/Dex1. Preserve the Dex1 model, not the reference photo's five-finger hands.

## Direction contract

THESIS: Keep the reference simulator's working scene and interactions, replacing the actual joint tree, grippers, control and telemetry rather than merely changing its visible shell.

OWN-WORLD: Inherit the existing dark blue workcell, cyan controls, Chinese UI, readable system type, task-colored red/blue/yellow shapes and placement frames. Do not introduce a new palette or page composition.

STORY: Select a hand and object, edit the object's pose, orbit the view, run physical grasp/sort/stack, inspect live vision and measured forces/joint tracking, then stop or reset safely.

FIRST VIEWPORT: The G1 and grounded, raised table occupy the scene at left; the existing control panel remains at right. Both feet, the active gripper, objects and primary task button remain legible. Mobile keeps the existing stacked scene/panel layout.

FORM: Precisely scoped extension of the incumbent simulator; code-led, no concept seed required. The signature interaction remains measured two-sided Dex1 contact followed by actual physical lift, transfer and release.

FINISH: unreviewed and undocumented is unfinished; this build ends with the finish review, the verdict, DESIGN.md, and every shipping raster carrying its provenance.
