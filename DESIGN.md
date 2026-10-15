---
name: R1 and G1 manipulation simulator
description: Preserve the supplied reference workcell while adapting the robot and measured controls.
colors:
  background: "#01030d"
  text: "#eaf7ff"
  focus: "#8ae8ff"
  selected: "#46d9f6"
  field: "#051324"
typography:
  body:
    fontFamily: "SF Pro Text, PingFang SC, Hiragino Sans, Noto Sans JP, Microsoft YaHei, sans-serif"
  headline:
    fontSize: "20px"
rounded:
  control: "12px"
  camera: "9px"
---

# Design System: Manipulation simulator

## Overview

This records the incumbent implementation, not a redesign. The user requested the Rebot_Arm_AGV workcell's effects and controls: dark blue scene, cyan interface, task-colored objects and matching placement frames. G1 extends that visual world; R1 remains unchanged. The surface is a working simulation, not a marketing page.

## Colors

Use the established background, pale text and cyan selection/focus vocabulary. Red, blue and yellow identify physical task objects and placement destinations, with written labels alongside color. Robot finishes follow the supplied photographs: silver-white covers, silver-gray limbs, dark head/waist/hip mechanisms and black feet/Dex1. Do not replace physical material color with emissive lighting.

## Typography

Retain the reference's Chinese-capable system font stack. Measurements use tabular numerals. Seven arm axes use radians/degrees and N·m; finger travel uses mm and force uses N. Keep recovery messages in Chinese and avoid presenting modeled estimates as sensor measurements.

## Layout

Desktop inherits a flexible scene and a 310px control panel. At 900px and below G1 stacks the scene above the controls; the scene occupies 58svh. Keep the feet, grounded table and active gripper visible. These layout rules are source-derived; the new G1 render and touch layout still require visual confirmation.

## Elevation & Depth

Preserve the reference's dark layered panels, outlined camera feeds, measured joint annotations and workcell reflections. Object selection pulses as in the reference, with reduced-motion support. Lighting should reveal silver shells against the dark background without clipping all surface detail.

## Shapes

Retain gently rounded controls and camera frames. Placement zones are four thin bars around an empty center, not solid targets. Do not change the supplied robot mesh silhouette to approximate a photograph.

## Components

Task controls select a side/object, run sorting or stacking, stop and reset. Joint chips expand to real seven-axis sliders; opening has its own mm range. Sliders and gripper buttons reject colliding motions without changing the last safe target. Switching arms preserves both gripper openings. Camera controls retain parallel/stacked layouts, feed visibility and model visibility. Loading reports real mesh/runtime progress; failure messages explain how to retry.

## Do's and Don'ts

- Do keep blank-space camera gestures separate from object editing.
- Do preserve the reference scene, Chinese copy, colored objects and frames.
- Do treat supplied robot color and camera photos as reference, not hardware calibration.
- Don't claim visual, touch or real-robot approval from numerical simulation tests.
- Don't alter R1 when extending G1, or replace Dex1 with the photographed five-finger hands.
