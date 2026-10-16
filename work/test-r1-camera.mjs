import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import {createWristVision,R1_WRIST_OPTICS,WORKCELL_REFERENCE} from '../dist/r1/r1-workcell.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {loadTestRobot} from './r1-test-robot.mjs';
import {REST,REST_TORSO,SPREAD,CROUCH} from '../dist/r1/grasp-controller.mjs';

const root=new URL('../',import.meta.url),paths=['work/test-r1-camera.mjs','work/r1-test-robot.mjs',
  'dist/r1/r1-workcell.mjs','dist/r1/r1-app.mjs','dist/multi/robot-runtime.mjs',
  'dist/r1/grasp-controller.mjs','dist/r1/assets/r1.urdf','dist/vendor/three.module.min.js',
  'dist/vendor/three.core.min.js',...['left','right'].map(side=>`dist/r1/assets/meshes/${side}_realsense_link.STL`)];
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const hashes=async()=>Object.fromEntries(await Promise.all(paths.map(async path=>[path,sha(await readFile(new URL(path,root)))])));
const sourceHashes=await hashes(),checks=[],worldObservations=[];
function check(name,fn){const detail=fn();checks.push({name,pass:true,...(detail===undefined?{}:{detail})});}
function near(a,b,label,tolerance=1e-10){assert(a.distanceTo(b)<tolerance,`${label}: ${a.toArray()} != ${b.toArray()}`);}
function originalSTL(bytes){
  const triangles=bytes.readUInt32LE(80);assert.equal(bytes.byteLength,84+triangles*50);
  const positions=new Float32Array(triangles*9);
  for(let face=0;face<triangles;face++)for(let component=0;component<9;component++)positions[face*9+component]=bytes.readFloatLE(84+face*50+12+component*4);
  assert(positions.every(Number.isFinite));
  return{triangles,geometry:new THREE.BufferGeometry().setAttribute('position',new THREE.Float32BufferAttribute(positions,3))};
}
const scene=new THREE.Scene(),station=new THREE.Group(),robot=await loadTestRobot();scene.add(station);station.add(robot);
const meshes=[],assetStats=[],disposedAssets=[];
for(const side of ['left','right']){
  const link=robot.getObjectByName(`${side}_realsense_link`),bytes=await readFile(new URL(`dist/r1/assets/meshes/${side}_realsense_link.STL`,root));
  assert(link);const parsed=originalSTL(bytes),material=new THREE.MeshBasicMaterial(),mesh=new THREE.Mesh(parsed.geometry,material);link.add(mesh);meshes.push(mesh);
  parsed.geometry.addEventListener('dispose',()=>disposedAssets.push(`${side}/geometry`));material.addEventListener('dispose',()=>disposedAssets.push(`${side}/material`));
  parsed.geometry.computeBoundingBox();const bounds=parsed.geometry.boundingBox;
  assert(bounds.containsPoint(new THREE.Vector3(.0325,0,.001)),'Geometry-inferred optical centre must stay inside the original camera body bounds');
  assetStats.push({side,triangles:parsed.triangles,min:bounds.min.toArray(),max:bounds.max.toArray(),sha256:sha(bytes)});
}
// Nondefault layer/visibility states prove that the view does not take ownership
// of source assets or blindly reset their original renderer configuration.
meshes[1].layers.enable(3);meshes[1].visible=false;
const camera=new THREE.PerspectiveCamera(65,4/3,.01,10);camera.layers.enable(4);
const originalCameraMask=camera.layers.mask,originalNodes=new Map();
for(const side of ['left','right'])robot.getObjectByName(`${side}_realsense_link`).traverse(node=>originalNodes.set(node,{mask:node.layers.mask,visible:node.visible}));
const transforms=()=>{const rows=[];robot.traverse(node=>rows.push({id:node.id,parent:node.parent?.id??null,children:node.children.map(child=>child.id),position:node.position.toArray(),quaternion:node.quaternion.toArray(),scale:node.scale.toArray()}));return rows;};
const beforeFactory=transforms(),sceneChildren=scene.children.map(node=>node.id);let addedMeshes=0;
class CountedMesh extends THREE.Mesh{constructor(...args){super(...args);addedMeshes++;}}
const vision=createWristVision({scene,THREE:{...THREE,Mesh:CountedMesh},camera});
check('Factory adds no duplicate camera geometry or scene nodes',()=>{
  assert.equal(addedMeshes,0);assert.deepEqual(scene.children.map(node=>node.id),sceneChildren);assert.deepEqual(transforms(),beforeFactory);
  assert(vision.model.isGroup);assert.equal(vision.model.children.length,0);assert.equal(vision.model.parent,null);
});
check('Optical metadata is explicit geometry inference, not real calibration',()=>{
  assert.deepEqual(R1_WRIST_OPTICS.lensLocal,[.0325,0,.001]);assert.deepEqual(R1_WRIST_OPTICS.forwardLocal,[0,0,1]);assert.deepEqual(R1_WRIST_OPTICS.upLocal,[0,-1,0]);
  assert.equal(R1_WRIST_OPTICS.calibrated,false);assert.match(R1_WRIST_OPTICS.provenance,/Geometry-based.*not calibrated/);assert(Object.isFrozen(R1_WRIST_OPTICS)&&Object.isFrozen(R1_WRIST_OPTICS.lensLocal));
});
check('Unbound hidden state is retained when actual source links first bind',()=>{
  vision.setVisible(false);assert(vision.update(robot.getObjectByName('right_realsense_link')));
  for(const side of ['left','right'])assert.equal(robot.getObjectByName(`${side}_realsense_link`).visible,false);
  vision.setVisible(true);assert.equal(vision.visible,true);
});
const poses={idle:{...REST_TORSO,...REST.left,...REST.right},spread:{...REST_TORSO,...SPREAD.left,...SPREAD.right},crouch:{...CROUCH,...SPREAD.left,...SPREAD.right}};
for(const[name,pose]of Object.entries(poses))for(const side of ['left','right'])check(`${name}/${side}: source optical origin and axes follow real joint frames`,()=>{
  robot.setJointValues(pose);scene.updateMatrixWorld(true);const before=transforms(),link=robot.getObjectByName(`${side}_realsense_link`);
  const expectedPosition=link.localToWorld(new THREE.Vector3(.0325,0,.001)),expectedDirection=new THREE.Vector3(0,0,1).transformDirection(link.matrixWorld),expectedUp=new THREE.Vector3(0,-1,0).transformDirection(link.matrixWorld);
  assert(vision.update(link));assert.equal(vision.model,link);near(camera.position,expectedPosition,'Source lens world centre');near(camera.getWorldDirection(new THREE.Vector3()),expectedDirection,'Source optical +Z');near(camera.up,expectedUp,'Source optical up -Y');
  const ahead=expectedPosition.clone().addScaledVector(expectedDirection,.30).project(camera);assert(Math.abs(ahead.x)<1e-10&&Math.abs(ahead.y)<1e-10&&Math.abs(ahead.z)<1);
  assert.deepEqual(transforms(),before,'Camera update must not change any source or fixed joint transform');worldObservations.push({pose:name,side,position:camera.position.toArray(),direction:expectedDirection.toArray(),up:expectedUp.toArray()});
});
check('Translated and rotated station frames affect only the rendered camera world transform',()=>{
  robot.setJointValues(poses.idle);station.position.set(0,0,0);scene.updateMatrixWorld(true);const link=robot.getObjectByName('right_realsense_link');assert(vision.update(link));
  const basePosition=camera.position.clone(),baseDirection=camera.getWorldDirection(new THREE.Vector3()),offset=new THREE.Vector3(0,0,1.6);station.position.copy(offset);scene.updateMatrixWorld(true);assert(vision.update(link));
  near(camera.position,basePosition.clone().add(offset),'Station translation');near(camera.getWorldDirection(new THREE.Vector3()),baseDirection,'Translation preserves optical direction');
  station.rotation.y=.37;scene.updateMatrixWorld(true);assert(vision.update(link));near(camera.position,basePosition.clone().applyAxisAngle(new THREE.Vector3(0,1,0),.37).add(offset),'Station rotation');
  station.rotation.y=0;station.position.set(0,0,0);scene.updateMatrixWorld(true);
});
check('Both original camera subtrees share the model toggle without changing posture',()=>{
  const before=transforms();vision.setVisible(false);for(const side of ['left','right'])assert.equal(robot.getObjectByName(`${side}_realsense_link`).visible,false);
  vision.setVisible(true);for(const side of ['left','right'])assert.equal(robot.getObjectByName(`${side}_realsense_link`).visible,true);
  assert.equal(meshes[1].visible,false,'Originally hidden source submesh must stay hidden');assert.deepEqual(transforms(),before);
});
check('Original camera surfaces are visible in main layers but excluded from their own feed',()=>{
  for(const[node]of originalNodes){assert.equal(node.layers.mask,2);assert.equal(camera.layers.test(node.layers),false);}
  const main=new THREE.PerspectiveCamera();main.layers.enable(1);assert(meshes.every(mesh=>main.layers.test(mesh.layers)));
  assert.equal(camera.fov,WORKCELL_REFERENCE.wristFov);assert.equal(camera.near,.008);assert.equal(camera.far,10);
});
check('Wrong or missing source frame is rejected instead of silently using gripper coordinates',()=>{
  const position=camera.position.clone(),current=vision.model;assert.equal(vision.update(null),false);assert.equal(vision.update(robot.getObjectByName('right_gripper_link')),false);near(camera.position,position,'Rejected binding');assert.equal(vision.model,current);
});
check('Multi runtime separates R1 optical frames from gripper TCP; other robots retain their existing frame',()=>{
  for(const side of ['left','right']){assert.equal(ROBOT_CONFIGS.r1.cameraFrame(side),`${side}_realsense_link`);assert.equal(ROBOT_CONFIGS.r1.gripperLink(side),`${side}_gripper_link`);}
  for(const id of ['g1','c5'])assert.equal(ROBOT_CONFIGS[id].cameraFrame,undefined);
});
const app=(await readFile(new URL('dist/r1/r1-app.mjs',root),'utf8')),runtime=(await readFile(new URL('dist/multi/robot-runtime.mjs',root),'utf8'));
check('Single and multi entry points route the source Realsense link to camera updates',()=>{
  assert(app.includes('const link=robot.getObjectByName(`${arm}_realsense_link`)'));assert(app.includes('wristVision.update(link)'));
  assert(runtime.includes('(config.cameraFrame??config.gripperLink)(controller.arm)'));
});
check('Dispose restores source state and never removes or disposes URDF assets',()=>{
  const before=transforms();vision.setVisible(false);vision.dispose();vision.dispose();assert.deepEqual(transforms(),before);
  for(const[node,original]of originalNodes){assert.equal(node.layers.mask,original.mask);assert.equal(node.visible,original.visible);}
  assert.equal(camera.layers.mask,originalCameraMask);assert.equal(disposedAssets.length,0);assert.equal(vision.model.children.length,0);assert.equal(vision.update(robot.getObjectByName('left_realsense_link')),false);
});
const sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(await hashes());assert(sourceFrozen,'R1 camera source changed during verification');
const report={pass:true,cases:checks.length,checks,assetStats,worldObservations,optics:R1_WRIST_OPTICS,sourceHashes,sourceFrozen,
  scope:'Original R1 source-joint frames and unchanged camera STL vertices; mathematical optical transforms, asset ownership and renderer-layer isolation. No rendered pixels, physical camera calibration, intrinsics, occlusion or hardware safety claim.'};
await mkdir(new URL('work/validation/r1-camera/',root),{recursive:true});await writeFile(new URL('work/validation/r1-camera/final.json',root),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass:true,cases:checks.length,sourceFrozen,sourceCameraTriangles:assetStats.map(item=>item.triangles)}));
