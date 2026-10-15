import { readFile } from 'node:fs/promises';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';

const api = await createMuJoCo({ locateFile: () => new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm', import.meta.url).pathname });
const vfs = new api.MjVFS();
vfs.addBuffer('r1_grasp_scene.xml', await readFile(new URL('../dist/r1/models/r1_grasp_scene.xml', import.meta.url)));
for (const name of ['base_link.STL', 'torso_link4.STL', 'right_gripper_link.STL', 'right_gripper_finger_link1.STL', 'right_gripper_finger_link2.STL']) vfs.addBuffer(`../assets/meshes/${name}`, await readFile(new URL(`../dist/r1/assets/meshes/${name}`, import.meta.url)));

const clamp = (value, [min, max]) => Math.min(max, Math.max(min, value));
const invert3 = (m) => {
  const [a, b, c, d, e, f, g, h, i] = m;
  const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (Math.abs(det) < 1e-10) return null;
  return [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det, (f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det, (d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det];
};
try {
  const model = api.MjModel.from_xml_path('r1_grasp_scene.xml', vfs);
  const data = new api.MjData(model);
  const type = api.mjtObj;
  const jointId = (name) => api.mj_name2id(model, type.mjOBJ_JOINT.value, name);
  const siteId = api.mj_name2id(model, type.mjOBJ_SITE.value, 'right_tcp');
  const names = ['right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3', 'right_arm_joint4', 'right_arm_joint5', 'right_arm_joint6'];
  const addresses = names.map((name) => model.jnt_qposadr[jointId(name)]);
  const bounds = [[-2.75, 2.75], [0, 3.18], [-3.2, 0], [-2.75, 2.75], [-1.62, 1.62], [-2.75, 2.75]];
  const torso = { torso_joint1: 1.45, torso_joint2: -2.35, torso_joint3: -0.8, torso_joint4: 0 };
  for (const [name, value] of Object.entries(torso)) data.qpos[model.jnt_qposadr[jointId(name)]] = value;
  const target = [0.78, 0, .734];
  const evalTcp = (pose) => {
    pose.forEach((value, i) => { data.qpos[addresses[i]] = value; });
    api.mj_forward(model, data);
    return Array.from(data.site_xpos.slice(siteId * 3, siteId * 3 + 3));
  };
  const seeds = [
    [-.244, 3.18, -2.518, .086, -.725, 2.182],
    [0, 1.8, -1.8, 0, 0, 0],
    [-1, 2.8, -2.5, 0, 0, 0],
    [1, 2.8, -2.5, 0, 0, 0],
  ];
  let best = null;
  for (let restart = 0; restart < 120; restart += 1) {
    const pose = restart < seeds.length ? [...seeds[restart]] : bounds.map(([lo, hi]) => lo + Math.random() * (hi - lo));
    for (let iteration = 0; iteration < 280; iteration += 1) {
      const current = evalTcp(pose);
      const error = target.map((value, axis) => value - current[axis]);
      const length = Math.hypot(...error);
      if (length < .002) break;
      const epsilon = .003;
      const jac = Array.from({ length: 3 }, () => Array(6).fill(0));
      for (let column = 0; column < 6; column += 1) {
        const initial = pose[column];
        pose[column] = clamp(initial + epsilon, bounds[column]); const plus = evalTcp(pose);
        pose[column] = clamp(initial - epsilon, bounds[column]); const minus = evalTcp(pose);
        pose[column] = initial;
        for (let row = 0; row < 3; row += 1) jac[row][column] = (plus[row] - minus[row]) / (2 * epsilon);
      }
      const lambda = .018;
      const normal = Array.from({ length: 3 }, (_, row) => Array.from({ length: 3 }, (_, col) => jac[row].reduce((sum, value, joint) => sum + value * jac[col][joint], row === col ? lambda : 0)));
      const inverse = invert3(normal.flat());
      if (!inverse) break;
      const projected = [0, 1, 2].map((row) => inverse.slice(row * 3, row * 3 + 3).reduce((sum, value, col) => sum + value * error[col], 0));
      for (let column = 0; column < 6; column += 1) {
        const delta = jac.reduce((sum, row, axis) => sum + row[column] * projected[axis], 0);
        pose[column] = clamp(pose[column] + Math.max(-.14, Math.min(.14, delta)), bounds[column]);
      }
    }
    const current = evalTcp(pose), error = Math.hypot(...target.map((value, axis) => value - current[axis]));
    if (!best || error < best.error) best = { pose: [...pose], current, error };
  }
  console.log(JSON.stringify(best, null, 2));
  data.delete(); model.delete();
} finally { vfs.delete(); }
