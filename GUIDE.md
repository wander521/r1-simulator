**English** · [简体中文](README.zh-CN.md)

# Orbit Manipulation Lab

Three robots. Independent workbenches. One shared clock.

A browser-based robotics playground for **Galaxea R1, Unitree G1 and AUBO C5**. Arrange objects, choose one or more robots, then run contact-based grasping, stacking and pipe sorting side by side. **MuJoCo** computes the dynamics and contacts; **Three.js** renders the workcell. Expensive path searches run in background **Web Workers**.

**[Open the lab](https://wander521.github.io/r1-simulator/)** · [Run locally](#run-locally) · [Architecture](#architecture) · [Verification](#verification-and-evidence)

![R1, G1 and C5 at separate workbenches in the deep-space laboratory, with robot selection and pipe-sorting controls](work/validation/background-planning/screenshots/desktop-ready.jpg)

The screenshot is an actual browser capture from the current background-planning release. The application controls are currently in Chinese; this repository provides English and Chinese documentation. **Orbit Manipulation Lab** is the display name; the repository and Pages URL remain `r1-simulator`.

## Explore the lab

- **Select one, two or three robots.** A common scheduler advances independent physical worlds with a **1 ms timestep**. Every robot handles objects on its own table; this is concurrent manipulation, not three robots carrying one shared object.
- **Run three task families.** Return objects to matching color areas, stack blue → red → yellow, or sort four red and four yellow pipes per workbench. R1 and G1 alternate arms for pipe sorting; C5 uses its single arm.
- **Edit the physical scene.** Select and move objects, rotate them, inspect joint targets and actual motion, and switch the observed robot without changing who participates in a task.
- **Inspect execution.** View gripper-camera images, contact forces, actuator torques and tracking errors. Completion is checked against physical contact, lifting, release, support and stability rather than a scripted success label.
- **Keep the camera responsive during planning.** Each robot searches candidate paths in a separate Worker. Stop and reset terminate pending work; stale results are rejected. The physical worlds wait together at planning barriers.
- **Switch environments.** Deep-space Command Cabin and Orbital Hangar change the scenery, lighting and interface palette while preserving robot state, objects, camera and task. The preference is remembered when browser storage is available.

<details>
<summary>Background planning and the mobile interface</summary>

![The desktop lab while a robot computes a path in the background](work/validation/background-planning/screenshots/desktop-planning.jpg)
<p><img src="work/validation/background-planning/screenshots/mobile-planning.jpg" width="300" alt="The lab at a 390-pixel mobile viewport during background planning"></p>

These are recorded desktop/mobile browser observations, not a claim of physical-device or all-browser certification. See the [background-planning review](work/validation/background-planning/finish-review.md).

</details>

## Robots and physical models

| Robot | Role in the lab | Important boundary |
| --- | --- | --- |
| **Galaxea R1** | Dual-arm manipulation with URDF-derived joints, inertias and grippers; native wrist-camera frames | Fixed-base manipulation; grasp contacts use finger pads, not a complete full-body self-collision model |
| **Unitree G1 + Dex1** | Seven-axis arms and two parallel grippers; 33 actuated joints in the assembled model | Feet/pelvis are fixed; walking and free-floating balance are outside scope |
| **AUBO C5 + Dex1** | Six-axis industrial arm with two driven fingers; independent original C5 geometry | The gripper mount is a simulation adapter, not a claim of a factory assembly or hardware calibration |

The runtime uses motor-driven joints and free task objects. Objects are not welded to grippers, carried by hidden supports or teleported during transport. Planning copies predict candidate paths; they do not overwrite the actual object's state. Reachability and collision coverage remain robot-specific.

## Run locally

You need **Python 3** to serve the included static files. **Node.js 22** is the recommended environment for npm commands and matches CI. The browser needs WebGL2, WebAssembly and module Workers. Runtime libraries and robot assets are included in `dist/`; no model download from Hugging Face or external inference service is needed.

```sh
git clone https://github.com/wander521/r1-simulator.git
cd r1-simulator
npm start
```

Open **http://127.0.0.1:4173/multi/**. No `npm install` or application build is required to run the lab.

Without npm, serve the same files directly:

```sh
python3 -m http.server 4173 --bind 127.0.0.1 --directory dist
```

Serve over HTTP rather than opening HTML from disk. If the port is occupied, change it in the Python command.

| Entry | Purpose |
| --- | --- |
| `/multi/` | Current multi-robot lab; the root redirects here |
| `/r1/` | Independent R1 controls |
| `/g1/` | Independent G1 controls |
| `/c5/` | Independent C5 controls |
| `/Rebot_Arm_AGV/?variant=arm` | Retained standard-arm reference |

## Controls and task flow

1. Select the participating robots in the sidebar, then choose color sorting, stacking or pipe sorting.
2. Arrange reachable objects before execution. Left-drag a physical object to move it; right-drag or Shift-drag to rotate it. WASD moves the selected object.
3. Start the task. The selected robots pass the common preparation barrier, then execute against their own physical worlds. An invalid participant prevents a partial start.
4. Use **Stop all** to cancel pending work. Gravity remains active. Use reset or rerandomize to begin a fresh pipe-sorting batch.

Drag empty space to orbit the camera, right-drag empty space to pan, and scroll to zoom. **Panorama** and **Focus current** explicitly reframe the scene. Merely changing the observed camera/plot does not change task participation. Participants are locked while a task runs.

## Architecture

```text
Selection, object edits, task controls
                  ↓
Shared coordinator / planning barrier
        ┌─────────┼─────────┐
        R1        G1        C5
        ↓         ↓         ↓
Per-robot Worker: candidate paths, IK, collision search
                  ↓
Validated paths and control commands only
                  ↓
Independent MuJoCo worlds / shared 1 ms scheduler
                  ↓
Actual body state, contacts and joint measurements
                  ↓
Three.js workcell, wrist cameras and live plots
```

The main thread still handles real physics, safety checks and limited placement-feedback IK. Worker snapshots preserve the compiled model and body/joint ordering. Generation IDs, job IDs and state checks reject stale responses after object edits, stop, reset or task changes. There is no silent fallback to expensive main-thread search if a Worker fails.

Planning can still take seconds. All physical worlds wait at the common barrier; camera responsiveness does not mean robots continue moving during planning. On overloaded hardware, simulation can run slower than wall time without enlarging the physics timestep.

| Path | Responsibility |
| --- | --- |
| `dist/multi/` | Coordinator, shared lab UI, scenes and background-planning transport |
| `dist/r1/`, `dist/g1/`, `dist/c5/` | Maintained robot pages, controllers, kinematics, dynamic scenes and assets |
| `dist/vendor/` | Vendored rendering/runtime dependencies |
| `dist/Rebot_Arm_AGV/` | Retained reference and MuJoCo runtime resources |
| `work/` | Model conversion, regression tools, acceptance reports and evidence |
| `.github/workflows/` | Validation and GitHub Pages deployment |

`dist/` contains maintained application code and must remain tracked; it is not a disposable generated folder in this repository.

## Verification and evidence

```sh
npm test                       # Multi-robot, task, model, framing and visual contracts
npm run test:pages             # Static paths and release-cache consistency
npm run test:acceptance        # Current source/evidence bindings and negative cases
npm run test:background-planning
```

Focused commands include `test:r1`, `test:g1`, `test:c5`, `test:pipes` and `test:framing`. **`test:pipes:quick` is a development aid and does not replace the FULL suite.** Full physical tests can take many minutes.

The recorded **2026-10-16 background-planning release** passed 24 Worker contracts, 12 ordinary grasps, three three-layer stacks and a FULL pipe suite with **72/72 pipes across eight scenarios**. These are existing source-bound simulation reports, not new tests run for this documentation refresh, guarantees for arbitrary layouts, or hardware safety certification.

| Read next | Scope |
| --- | --- |
| [Background planning](work/validation/background-planning/README.md) | Current reports, compiled-model ordering, cancellation and failure cases |
| [Multi-robot acceptance](work/MULTI-ACCEPTANCE.md) | Participation, concurrent task semantics and evidence rules |
| [Pipe sorting](work/PIPE-ACCEPTANCE.md) | Layout constraints, alternating arms and physical completion |
| [Scenes](work/LAB-SCENES-ACCEPTANCE.md) | Lighting, theme isolation and browser observations |
| [Pose and vision](work/POSE-AND-VISION-ACCEPTANCE.md) | Current standby poses, native camera frames and collision limits |
| [Complete implementation history](TECHNICAL_NOTES.zh-CN.md) | Preserved Chinese technical and branch records; historical states are not current instructions |

Detailed engineering reports are currently in Chinese. The two READMEs cover the same current feature set, architecture, setup and limitations.

## Deploy and branch layout

`main` is the current multi-robot application. GitHub Actions validates the site before publishing **only `dist/`** to the existing Pages URL. Configure Pages Source as **GitHub Actions** when deploying a new copy; see [.github/workflows/deploy-pages.yml](.github/workflows/deploy-pages.yml). Historical independent development branches are `dev_r1`, `unitree_g1` and `dev_aobo_c5`; their old default routes and release notes should not be mistaken for current `main` behavior.

## Boundaries and credits

- This is a browser simulation and engineering playground, not a learned end-to-end manipulation policy, production robot controller or real-machine safety validation.
- Object placement is constrained by reachability, gripper clearance and the tested workspaces. Not every dragged or random position is solvable.
- R1's contact coverage is narrower than G1's; original visual meshes do not establish complete continuous self-collision checking.
- The retained standard-arm reference comes from [yang-ci/Rebot_Arm_AGV](https://yang-ci.github.io/Rebot_Arm_AGV/?variant=arm). Robot models and runtime dependencies retain their respective attribution and license terms.
- See the [G1 asset provenance](dist/g1/assets/THIRD_PARTY.md) and [C5 asset provenance](dist/c5/assets/THIRD_PARTY.md). The C5 upstream snapshot declares BSD but does not supply a complete license text or explicit variant; that gap is recorded, not filled by an unrelated model's license. No blanket license grant is asserted for all repository assets.

For a bug report, include the robot, task, object layout or seed, browser and steps to reproduce. Physical-behavior changes should include regression evidence.
