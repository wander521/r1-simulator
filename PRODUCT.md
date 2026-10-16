# Product

<!-- impeccable:product-schema 1 -->

## Platform

web

## Users

The project owner and people opening the shared robot demonstration in a browser.

## Product Purpose

Demonstrate physical manipulation by R1, Unitree G1 and AUBO C5, individually or
concurrently, in one interactive scene. The owner wants the opening experience to
feel polished, futuristic and clearly distinct from the original reference demo.

## Operating Context

The main multi-robot route is `/multi/`. People select participants, choose sorting
or stacking, start the task, inspect live cameras and measured feedback, and may
stop, reset, move task objects or orbit the view. Local previews use port 4173;
the existing repository also publishes through GitHub Pages.

## Capabilities and Constraints

- Preserve robot models, station placement, joint/IK controllers, physical limits,
  real grasp/release checks and task progress during this visual redesign.
- Two complete environment/interface themes are approved: 深空指挥舱 and 轨道机库.
  After inspecting the first build the owner approved 深空指挥舱, rejected the
  overly white 极昼实验室, and requested the second scene in the same darker family.
  Switching changes lighting, environment, floor and interface together without
  resetting robots or an executing task.
- Redesign only the multi-robot route. Existing R1/G1/C5 individual routes and the
  reference route retain their appearance and behavior.
- Controls and task state remain readable in Chinese and usable at phone widths.
- This is browser simulation, not hardware control or safety certification.

## Evidence on Hand

Original URDF/MJCF assets, supplied robot photographs, current rendered screenshots,
and source-bound numerical grasping, framing, interaction and publication tests.
Historical and current evidence remain distinguished; no commercial or hardware
performance claims are authorized.

## Product Principles

- Make the robots and their real work the visual centerpiece.
- Light robot shells clearly; do not sacrifice legibility for atmosphere.
- Preserve working manipulation and separate view gestures from object editing.
- Theme switching must be cosmetic and keep the live simulation intact.
