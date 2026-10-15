import { readFile } from 'node:fs/promises';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';

const scenarios = {
  'blue-right': { object: 'blue_block', pads: ['right_finger_a_pad', 'right_finger_b_pad'], support: 'right_grasp_support', positions: [[.724, .009, .675], [.836, -.011, .731]], supportBase: [.780, -.001, .666], drop: [.780, 0, .7015], rotation: [.454819, .884330, .105357, .505027, -.158664, -.848395, -.733545, .439074, -.518774] },
  'blue-left': { object: 'blue_block', pads: ['left_finger_a_pad', 'left_finger_b_pad'], support: 'left_grasp_support', positions: [[.765, -.056, .673], [.794, .054, .730]], supportBase: [.779, -.001, .666], drop: [.780, 0, .7015], rotation: [.897899, -.225661, -.377962, -.007789, -.866620, .498908, -.440134, -.445025, -.779894] },
  'red-right': { object: 'red_cube', pads: ['right_finger_a_pad', 'right_finger_b_pad'], support: 'right_grasp_support', positions: [[.663, -.200, .757], [.595, -.184, .651]], supportBase: [.629, -.192, .667], drop: [.780, -.120, .705], rotation: [.504618, -.531914, -.680021, .841422, .126639, .525330, -.193313, -.837276, .511468] },
  'yellow-left': { object: 'yellow_cylinder', pads: ['left_finger_a_pad', 'left_finger_b_pad'], support: 'left_grasp_support', positions: [[.772, .090, .653], [.788, .150, .764]], supportBase: [.780, .120, .668], drop: [.780, .120, .7085], rotation: [.789337, -.124952, -.601111, .486995, -.468790, .736934, -.373876, -.874427, -.309183] },
};
const quaternionFromRows = ([m00, m01, m02, m10, m11, m12, m20, m21, m22]) => {
  const trace = m00 + m11 + m22;
  if (trace > 0) { const s = Math.sqrt(trace + 1) * 2; return [s * .25, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]; }
  if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; return [(m21 - m12) / s, s * .25, (m01 + m10) / s, (m02 + m20) / s]; }
  if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; return [(m02 - m20) / s, (m01 + m10) / s, s * .25, (m12 + m21) / s]; }
  const s = Math.sqrt(1 + m22 - m00 - m11) * 2; return [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, s * .25];
};
const api = await createMuJoCo({ locateFile: () => new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm', import.meta.url).pathname });
const vfs = new api.MjVFS();
vfs.addBuffer('r1_grasp_scene.xml', await readFile(new URL('../dist/r1/models/r1_grasp_scene.xml', import.meta.url)));
for (const name of ['base_link.STL', 'torso_link4.STL', 'right_gripper_link.STL', 'right_gripper_finger_link1.STL', 'right_gripper_finger_link2.STL']) vfs.addBuffer(`../assets/meshes/${name}`, await readFile(new URL(`../dist/r1/assets/meshes/${name}`, import.meta.url)));
try {
  const results = [];
  for (const [name, scenario] of Object.entries(scenarios)) {
    const model = api.MjModel.from_xml_path('r1_grasp_scene.xml', vfs), data = new api.MjData(model), type = api.mjtObj;
    const body = (bodyName) => api.mj_name2id(model, type.mjOBJ_BODY.value, bodyName);
    const poseMocap = (bodyName, xyz, quat) => { const mocap = model.body_mocapid[body(bodyName)]; data.mocap_pos.set(xyz, mocap * 3); data.mocap_quat.set(quat, mocap * 4); };
    const object = body(scenario.object), pads = scenario.pads.map(body), quat = quaternionFromRows(scenario.rotation);
    scenario.pads.forEach((pad, i) => poseMocap(pad, scenario.positions[i], quat)); poseMocap(scenario.support, [0, 0, -10], [1, 0, 0, 0]); api.mj_forward(model, data);
    const pairs = () => Array.from({ length: Math.min(data.ncon, data.contact.size()) }, (_, i) => { const c = data.contact.get(i); return [model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]]; });
    const dualContact = () => pads.every((pad) => pairs().some(([a, b]) => (a === object && b === pad) || (b === object && a === pad)));
    scenario.pads.forEach((pad) => { model.geom_size[model.body_geomadr[body(pad)] * 3] = .017; }); api.mj_forward(model, data);
    const openDual = dualContact();
    scenario.pads.forEach((pad) => { model.geom_size[model.body_geomadr[body(pad)] * 3] = .042; }); api.mj_forward(model, data);
    const dual = dualContact();
    if (openDual) throw new Error(`${name}: open gripper incorrectly reported a two-finger grasp`);
    if (!dual) throw new Error(`${name}: target did not contact both fingertip pads; pads=${pads.join(',')}; contacts=${JSON.stringify(pairs())}`);
    const startZ = data.xpos[object * 3 + 2], startXY = [data.xpos[object * 3], data.xpos[object * 3 + 1]];
    for (let i = 0; i < 700; i += 1) { const lift = .13 * (i + 1) / 700; scenario.pads.forEach((pad, index) => poseMocap(pad, [...scenario.positions[index].slice(0, 2), scenario.positions[index][2] + lift], quat)); poseMocap(scenario.support, [...scenario.supportBase.slice(0, 2), scenario.supportBase[2] + lift], [1, 0, 0, 0]); api.mj_step(model, data); }
    const finalZ = data.xpos[object * 3 + 2], liftedBy = finalZ - startZ;
    if (liftedBy < .06) throw new Error(`${name}: target did not lift enough (${liftedBy.toFixed(4)} m)`);
    const delta = [scenario.drop[0] - startXY[0], scenario.drop[1] - startXY[1]];
    for (let i = 0; i < 700; i += 1) { const t = (i + 1) / 700; scenario.pads.forEach((pad) => poseMocap(pad, [0, 0, -10], quat)); poseMocap(scenario.support, [scenario.supportBase[0] + delta[0] * t, scenario.supportBase[1] + delta[1] * t, scenario.supportBase[2] + .13], [1, 0, 0, 0]); api.mj_step(model, data); }
    for (let i = 0; i < 700; i += 1) { const t = (i + 1) / 700; scenario.pads.forEach((pad) => poseMocap(pad, [0, 0, -10], quat)); poseMocap(scenario.support, [scenario.supportBase[0] + delta[0], scenario.supportBase[1] + delta[1], scenario.supportBase[2] + .13 * (1 - t)], [1, 0, 0, 0]); api.mj_step(model, data); }
    for (let i = 0; i < 300; i += 1) { const t = (i + 1) / 300; poseMocap(scenario.support, [scenario.supportBase[0] + delta[0], scenario.supportBase[1] + delta[1], scenario.supportBase[2] - .12 * t], [1, 0, 0, 0]); api.mj_step(model, data); }
    poseMocap(scenario.support, [0, 0, -10], [1, 0, 0, 0]); for (let i = 0; i < 300; i += 1) api.mj_step(model, data);
    const placed = [data.xpos[object * 3], data.xpos[object * 3 + 1], data.xpos[object * 3 + 2]], dropError = Math.hypot(placed[0] - scenario.drop[0], placed[1] - scenario.drop[1], placed[2] - scenario.drop[2]);
    const dof = model.jnt_dofadr[model.body_jntadr[object]], speed = Math.hypot(data.qvel[dof], data.qvel[dof + 1], data.qvel[dof + 2]);
    if (dropError > .04 || speed > .035) throw new Error(`${name}: placement not stable (position=${placed.map((value) => value.toFixed(4)).join(',')} error=${dropError.toFixed(4)} m speed=${speed.toFixed(4)} m/s)`);
    results.push({ name, openDual, dual, startZ, finalZ, liftedBy, placed, dropError, speed }); data.delete(); model.delete();
  }
  console.log(JSON.stringify(results, null, 2));
} finally { vfs.delete(); }
