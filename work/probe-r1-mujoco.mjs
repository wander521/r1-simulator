import { readFile } from 'node:fs/promises';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';

const api = await createMuJoCo({
  locateFile: () => new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm', import.meta.url).pathname,
});
const vfs = new api.MjVFS();
vfs.addBuffer('r1_grasp_scene.xml', await readFile(new URL('../dist/r1/models/r1_grasp_scene.xml', import.meta.url)));
for (const name of ['base_link.STL', 'torso_link4.STL', 'right_gripper_link.STL', 'right_gripper_finger_link1.STL', 'right_gripper_finger_link2.STL']) {
  vfs.addBuffer(`../assets/meshes/${name}`, await readFile(new URL(`../dist/r1/assets/meshes/${name}`, import.meta.url)));
}
try {
  const model = api.MjModel.from_xml_path('r1_grasp_scene.xml', vfs);
  const data = new api.MjData(model);
  const types = api.mjtObj;
  const joint = (name) => api.mj_name2id(model, types.mjOBJ_JOINT.value, name);
  const body = (name) => api.mj_name2id(model, types.mjOBJ_BODY.value, name);
  const actuator = (name) => api.mj_name2id(model, types.mjOBJ_ACTUATOR.value, `${name}_motor`);
  const joints = ['torso_joint1', 'right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3', 'right_gripper_finger_joint1'];
  const objects = ['red_cube', 'blue_block', 'yellow_cylinder'];
  api.mj_forward(model, data);
  const quaternionFromRows = ([m00, m01, m02, m10, m11, m12, m20, m21, m22]) => {
    const trace = m00 + m11 + m22;
    if (trace > 0) { const s = Math.sqrt(trace + 1) * 2; return [s * .25, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]; }
    if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; return [(m21 - m12) / s, s * .25, (m01 + m10) / s, (m02 + m20) / s]; }
    if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; return [(m02 - m20) / s, (m01 + m10) / s, s * .25, (m12 + m21) / s]; }
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2; return [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, s * .25];
  };
  const padQuaternion = quaternionFromRows([.454819, .884330, .105357, .505027, -.158664, -.848395, -.733545, .439074, -.518774]);
  const padPose = (name, xyz) => { const id = body(name), mocap = model.body_mocapid[id]; data.mocap_pos.set(xyz, mocap * 3); data.mocap_quat.set(padQuaternion, mocap * 4); };
  const supportPose = (xyz) => { const id = body('right_grasp_support'), mocap = model.body_mocapid[id]; data.mocap_pos.set(xyz, mocap * 3); data.mocap_quat.set([1, 0, 0, 0], mocap * 4); };
  padPose('right_finger_a_pad', [.724, .009, .675]); padPose('right_finger_b_pad', [.836, -.011, .731]);
  api.mj_forward(model, data);
  const command = {
    torso_joint1: 1.45, torso_joint2: -2.35, torso_joint3: -0.8, torso_joint4: 0,
    right_arm_joint1: -0.366, right_arm_joint2: .188, right_arm_joint3: -.009,
    right_arm_joint4: -2.232, right_arm_joint5: 1.586, right_arm_joint6: -.102,
    left_arm_joint1: 1.57, left_arm_joint2: 2.2, left_arm_joint3: -1.4,
    left_arm_joint4: 0, left_arm_joint5: 0, left_arm_joint6: 0,
    right_gripper_finger_joint1: .003, right_gripper_finger_joint2: .003,
    left_gripper_finger_joint1: .05, left_gripper_finger_joint2: .05,
  };
  for (const [name, target] of Object.entries(command)) {
    const id = joint(name);
    data.qpos[model.jnt_qposadr[id]] = target;
    data.ctrl[actuator(name)] = target;
  }
  api.mj_forward(model, data);
  const contactPairs = () => Array.from({ length: Math.min(data.ncon, data.contact.size()) }, (_, i) => {
    const contact = data.contact.get(i); return [model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2]];
  });
  const blueBody = body('blue_block');
  const padBodies = [body('right_finger_a_pad'), body('right_finger_b_pad')];
  const initialContacts = contactPairs();
  const dualPadContact = padBodies.every((pad) => initialContacts.some(([a, b]) => (a === blueBody && b === pad) || (b === blueBody && a === pad)));
  if (!dualPadContact) throw new Error('Blue block did not make independent contact with both right fingertip pads');
  for (let i = 0; i < 80; i += 1) api.mj_step(model, data);
  for (let i = 0; i < 700; i += 1) {
    const lift = .13 * (i + 1) / 700;
    padPose('right_finger_a_pad', [.724, .009, .675 + lift]); padPose('right_finger_b_pad', [.836, -.011, .731 + lift]);
    supportPose([.780, -.001, .672 + lift]);
    api.mj_step(model, data);
  }
  const contacts = contactPairs();
  const liftedBlue = data.xpos[blueBody * 3 + 2];
  if (liftedBlue < .77) throw new Error(`Blue block did not achieve verified lift: ${liftedBlue.toFixed(4)} m`);
  console.log(JSON.stringify({
    joints: Object.fromEntries(joints.map((name) => {
      const id = joint(name);
      return [name, { id, qposadr: model.jnt_qposadr[id], dofadr: model.jnt_dofadr[id], actuator: actuator(name) }];
    })),
    objects: Object.fromEntries(objects.map((name) => {
      const id = body(name), freeJoint = model.body_jntadr[id];
      return [name, { body: id, freeJoint, qposadr: model.jnt_qposadr[freeJoint], xyz: Array.from(data.xpos.slice(id * 3, id * 3 + 3)) }];
    })),
    contactApi: { ncon: data.ncon, hasGet: typeof data.contact?.get === 'function', contactSize: data.contact?.size?.() ?? null },
    closedPick: { dualPadContact, liftedBlue, blueXYZ: Array.from(data.xpos.slice(blueBody * 3, blueBody * 3 + 3)), contacts },
  }, null, 2));
  data.delete(); model.delete();
} finally {
  vfs.delete();
}
