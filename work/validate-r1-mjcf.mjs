import { readFile } from 'node:fs/promises';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';

const root = new URL('../', import.meta.url);
const api = await createMuJoCo({
  locateFile: () => new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm', import.meta.url).pathname,
});
const vfs = new api.MjVFS();
const xmlName = 'r1_grasp_scene.xml';
const xml = await readFile(new URL('../dist/r1/models/r1_grasp_scene.xml', import.meta.url));
vfs.addBuffer(xmlName, xml);
for (const name of ['base_link.STL', 'torso_link4.STL', 'right_gripper_link.STL', 'right_gripper_finger_link1.STL', 'right_gripper_finger_link2.STL']) {
  vfs.addBuffer(`../assets/meshes/${name}`, await readFile(new URL(`../dist/r1/assets/meshes/${name}`, import.meta.url)));
}
try {
  const model = api.MjModel.from_xml_path(xmlName, vfs);
  const data = new api.MjData(model);
  api.mj_forward(model, data);
  console.log(JSON.stringify({ nq: model.nq, nv: model.nv, nu: model.nu, nbody: model.nbody, ngeom: model.ngeom, bodies: ['red_cube', 'blue_block', 'yellow_cylinder'].map((body) => api.mj_name2id(model, api.mjtObj.mjOBJ_BODY.value, body)) }));
  data.delete(); model.delete();
} finally {
  vfs.delete();
}
