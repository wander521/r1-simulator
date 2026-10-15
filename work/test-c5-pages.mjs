import assert from 'node:assert/strict';
import {readFile,stat} from 'node:fs/promises';
const root=new URL('../dist/',import.meta.url);
const files=['index.html','c5/index.html','c5/c5-app.mjs','c5/c5-profile.mjs','c5/c5-dynamics.mjs','c5/c5-3ds.mjs','c5/c5-joint-ui.mjs','c5/c5-materials.mjs','c5/c5-workcell.mjs','c5/grasp-controller.mjs','c5/grasp-kinematics.mjs','c5/styles.css','c5/models/c5-dynamic-scene.xml','c5/assets/aubo_C5.urdf','c5/assets/c5-dex1.urdf','r1/index.html','g1/index.html','g1/g1-dynamics.mjs','g1/g1-workcell.mjs','r1/mujoco-runtime.mjs','r1/r1-area-light-textures.mjs','vendor/URDFLoader.js','vendor/URDFClasses.js','vendor/STLLoader.js','vendor/OrbitControls.js','vendor/three.module.min.js','Rebot_Arm_AGV/assets/index-B74gtmCa.css','Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm'];
for(const file of files)assert((await stat(new URL(file,root))).size>0,'Missing asset: '+file);
const home=await readFile(new URL('index.html',root),'utf8');assert(home.includes('url=./c5/')&&home.includes("location.replace('./c5/')"));
const urdf=await readFile(new URL('c5/assets/c5-dex1.urdf',root),'utf8');
const meshPaths=[...new Set([...urdf.matchAll(/filename="([^"]+)"/g)].map(m=>m[1]))];
assert.equal(meshPaths.length,19,'Seven C5 visual+collision files and five Dex1 source mesh files');
for(const base of ['http://127.0.0.1:4173/','https://wander521.github.io/r1-simulator/']){
  for(const path of meshPaths){
    assert(!path.startsWith('package:'),'Derived robot must use portable relative paths');
    const url=new URL(path,new URL('c5/assets/',base));assert(url.pathname.startsWith(new URL(base).pathname),'Asset escaped project path');
    await stat(new URL(url.pathname.slice(new URL(base).pathname.length),root));
  }
  const entry=new URL('c5/',base);assert(entry.pathname.endsWith('/c5/'));
}
const index=await readFile(new URL('c5/index.html',root),'utf8');
assert(index.includes('lang="zh-CN"')&&index.includes('c5-app.mjs'));
assert(!index.includes('model-variant'),'Single C5 must not expose left/right arm selection');
const wasm=await readFile(new URL('Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root));assert(WebAssembly.validate(wasm));
console.log(JSON.stringify({pass:true,files:files.length,meshFiles:meshPaths.length,wasmBytes:wasm.length,projectBase:'https://wander521.github.io/r1-simulator/'}));
