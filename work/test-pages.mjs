import assert from 'node:assert/strict';
import {readFile, stat} from 'node:fs/promises';

const root = new URL('../dist/', import.meta.url);
const local = new URL('http://127.0.0.1:4173/');
const hosted = new URL('https://wander521.github.io/r1-simulator/');
const files = [
  'g1/index.html', 'g1/g1-app.mjs', 'g1/g1-workcell.mjs',
  'g1/g1-dynamics.mjs', 'g1/g1-joint-ui.mjs', 'g1/g1-materials.mjs', 'g1/styles.css',
  'g1/grasp-controller.mjs', 'g1/grasp-kinematics.mjs', 'g1/g1-profile.mjs',
  'g1/assets/g1_29dof_with_dex1_1.urdf', 'g1/models/g1-dynamic-scene.xml',
  'index.html', 'r1/index.html', 'r1/r1-app.mjs', 'r1/r1-workcell.mjs',
  'r1/r1-dynamics.mjs', 'r1/r1-joint-ui.mjs', 'r1/r1-joint-ui.css',
  'r1/r1-materials.mjs', 'r1/grasp-controller.mjs', 'r1/grasp-kinematics.mjs',
  'r1/mujoco-runtime.mjs', 'r1/assets/r1.urdf', 'r1/models/r1-dynamic-scene.xml',
  'vendor/three.module.min.js', 'vendor/URDFLoader.js', 'vendor/OrbitControls.js',
  'Rebot_Arm_AGV/index.html', 'Rebot_Arm_AGV/assets/index-BXF1-FJz.js',
  'Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',
];
for (const file of files) assert((await stat(new URL(file, root))).size > 0, `${file} is missing or empty`);

const home = await readFile(new URL('index.html', root), 'utf8');
assert(home.includes('url=./g1/') && home.includes("location.replace('./g1/')"));
const publishedHome=await readFile(new URL('pages-preview-index.html',import.meta.url),'utf8');
assert(publishedHome.includes('url=./r1/')&&publishedHome.includes("location.replace('./r1/')"),'G1 preview must retain the existing R1 root entry');
for (const base of [local, hosted]) {
  assert.equal(new URL('./r1/', base).pathname, `${base.pathname}r1/`);
  for (const [path, from] of [
    ['../vendor/three.module.min.js', 'g1/g1-app.mjs'],
    ['./assets/g1_29dof_with_dex1_1.urdf', 'g1/'],
    ['./models/g1-dynamic-scene.xml', 'g1/'],
    ['../r1/mujoco-runtime.mjs', 'g1/g1-app.mjs'],
    ['../vendor/three.module.min.js', 'r1/r1-app.mjs'],
    ['./assets/r1.urdf', 'r1/'],
    ['./models/r1-dynamic-scene.xml', 'r1/'],
    ['../Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm', 'r1/r1-app.mjs'],
  ]) {
    const url = new URL(path, new URL(from, base));
    assert(url.pathname.startsWith(base.pathname), `Asset escaped the project: ${url}`);
    await stat(new URL(url.pathname.slice(base.pathname.length), root));
  }
}

const urdf = await readFile(new URL('r1/assets/r1.urdf', root), 'utf8');
const meshes = [...urdf.matchAll(/filename="package:\/\/r1_v2_1_0\/([^\"]+)"/g)].map(m => m[1]);
assert(meshes.length > 20, 'Expected the complete R1 mesh set');
for (const mesh of meshes) await stat(new URL(`r1/assets/${mesh}`, root));
const g1URDF=await readFile(new URL('g1/assets/g1_29dof_with_dex1_1.urdf',root),'utf8');
const g1Meshes=[...new Set([...g1URDF.matchAll(/filename="([^"]+)"/g)].map(m=>m[1]))];
assert.equal(g1Meshes.length,38,'Expected all unique G1/Dex1 mesh assets');
for(const mesh of g1Meshes)await stat(new URL(`g1/assets/${mesh}`,root));

// Shared memory would require headers that GitHub Pages cannot configure.
// Compile the deployed binary to ensure it is valid; the application passes
// downloaded bytes directly and does not depend on streaming MIME handling.
const binary = await readFile(new URL('Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm', root));
assert(WebAssembly.validate(binary), 'Published MuJoCo binary must be valid WASM');
console.log(JSON.stringify({pass: true, checkedFiles: files.length, meshReferences: meshes.length, wasmBytes: binary.length, localBase: local.href, hostedBase: hosted.href}));
