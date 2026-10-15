import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import {loadG1TestRobot} from './g1-test-robot.mjs';
import {REST,REST_TORSO} from '../dist/g1/grasp-controller.mjs';
import {createWristVision,G1_WRIST_CAMERA} from '../dist/g1/g1-workcell.mjs';
import {applyG1Materials,G1_FINISHES,G1_HEAD_MASK_PROFILE} from '../dist/g1/g1-materials.mjs';
import * as profile from '../dist/g1/g1-profile.mjs';

const root=new URL('../dist/g1/assets/',import.meta.url),urdf=fs.readFileSync(new URL('g1_29dof_with_dex1_1.urdf',root),'utf8'),robot=await loadG1TestRobot();
const hash=value=>crypto.createHash('sha256').update(value).digest('hex');
const attr=(source,tag,key,fallback='')=>source.match(new RegExp(`<${tag}\\b[^>]*\\b${key}="([^"]+)"`))?.[1]??fallback;
const visuals=[];
// Source link tags contain only name. Requiring the non-self-closing opening
// prevents an empty IMU link from swallowing the next real visual link.
for(const [,name,body]of urdf.matchAll(/<link\s+name="([^"]+)"\s*>([\s\S]*?)<\/link>/g)){
  const visual=body.match(/<visual\b[^>]*>([\s\S]*?)<\/visual>/)?.[1],filename=attr(visual||'','mesh','filename');if(!filename)continue;
  const file=new URL(filename,root),bytes=fs.readFileSync(file),n=bytes.readUInt32LE(80),positions=new Float32Array(n*9),normals=new Float32Array(n*9);
  for(let i=0;i<n;i++)for(let j=0;j<3;j++)for(let k=0;k<3;k++){positions[i*9+j*3+k]=bytes.readFloatLE(84+i*50+12+j*12+k*4);normals[i*9+j*3+k]=bytes.readFloatLE(84+i*50+k*4);}
  const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));geometry.setAttribute('normal',new THREE.BufferAttribute(normals,3));
  const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial());mesh.position.fromArray(attr(visual,'origin','xyz','0 0 0').split(/\s+/).map(Number));mesh.rotation.set(...attr(visual,'origin','rpy','0 0 0').split(/\s+/).map(Number),'ZYX');robot.links[name].add(mesh);
  visuals.push({name,filename,file,mesh,fileHash:hash(bytes),positionHash:hash(new Uint8Array(positions.buffer)),normalHash:hash(new Uint8Array(normals.buffer))});
}
robot.setJointValues({...REST_TORSO,...REST.left,...REST.right});robot.updateMatrixWorld(true);
const materialReport=applyG1Materials(robot),unchanged=visuals.map(v=>({name:v.name,sourceBytes:v.fileHash===hash(fs.readFileSync(v.file)),positions:v.positionHash===hash(new Uint8Array(v.mesh.geometry.attributes.position.array.buffer)),normals:v.normalHash===hash(new Uint8Array(v.mesh.geometry.attributes.normal.array.buffer))}));
const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(65,4/3,.01,10),vision=createWristVision({scene,THREE,camera}),wrist=[];
const project=(name,points)=>{
  const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];let clipped=0;
  for(const point of points){const p=point.clone().project(camera);for(let k=0;k<3;k++){min[k]=Math.min(min[k],p.getComponent(k));max[k]=Math.max(max[k],p.getComponent(k));}if(Math.abs(p.x)>1||Math.abs(p.y)>1||Math.abs(p.z)>1)clipped++;}
  return {name,points:points.length,clipped,minNDC:min,maxNDC:max};
};
for(const side of ['right','left'])for(const q of [profile.GRIPPER_Q_MIN,0,profile.GRIPPER_Q_MAX]){
  robot.setJointValues(Object.fromEntries(profile.fingerJointNames(side).map(name=>[name,q])));robot.updateMatrixWorld(true);const link=robot.links[profile.gripperLinkName(side)];vision.update(link);
  const groups=[project('TCP',[link.localToWorld(new THREE.Vector3(profile.GRIPPER_TCP_X,0,0))])];
  for(const name of profile.fingerBodyNames(side)){
    const visual=visuals.find(v=>v.name===name),position=visual.mesh.geometry.attributes.position,points=[];
    for(let i=0;i<position.count;i++){const point=new THREE.Vector3().fromBufferAttribute(position,i);if(point.x>=profile.GRIPPER_TCP_X-.020)points.push(visual.mesh.localToWorld(point));}
    groups.push(project(name+' distal20mm',points));
  }
  for(const depth of [0,.13])for(const [id,hx,hy,hz]of [['red',.0225,.0225,.0225],['blue',.026,.016,.019],['yellow',.02,.02,.026]]){
    const corners=[];for(const x of [-hx,hx])for(const y of [-hy,hy])for(const z of [-hz,hz])corners.push(link.localToWorld(new THREE.Vector3(profile.GRIPPER_TCP_X+x,y,z-depth)));
    groups.push(project(`${id} at ${depth?'130mm approach':'contact'}`,corners));
  }
  const visibleLens=vision.model.getObjectByName('g1-front-camera-lens'),visibleCenter=visibleLens.getWorldPosition(new THREE.Vector3());
  const modelBounds=new THREE.Box3();vision.model.updateMatrixWorld(true);const inverse=new THREE.Matrix4().copy(link.matrixWorld).invert();
  vision.model.traverse(mesh=>{if(!mesh.isMesh)return;const position=mesh.geometry.attributes.position,matrix=new THREE.Matrix4().multiplyMatrices(inverse,mesh.matrixWorld);for(let i=0;i<position.count;i++)modelBounds.expandByPoint(new THREE.Vector3().fromBufferAttribute(position,i).applyMatrix4(matrix));});
  wrist.push({side,q,cameraLocal:link.worldToLocal(camera.position.clone()).toArray(),fov:camera.fov,groups,lensAndCameraSeparation:visibleCenter.distanceTo(camera.position),assemblyQuaternionNorm:vision.model.getObjectByName('g1-front-camera-assembly').quaternion.length(),mountBounds:{min:modelBounds.min.toArray(),max:modelBounds.max.toArray()}});
}
const fingerMaxZ=Math.max(...visuals.filter(v=>v.name.includes('_dex1_finger_link_')).map(v=>{v.mesh.geometry.computeBoundingBox();return v.mesh.geometry.boundingBox.max.z;}));
const mountMinZ=Math.min(...wrist.map(v=>v.mountBounds.min[2]));
const allPoints=[];for(const v of visuals){const positions=v.mesh.geometry.attributes.position;for(let i=0;i<positions.count;i++)allPoints.push(v.mesh.localToWorld(new THREE.Vector3().fromBufferAttribute(positions,i)));}
const mainCamera=new THREE.PerspectiveCamera(39,1,.02,15);mainCamera.position.set(2.2,1.6,2.6);mainCamera.lookAt(.22,.85,0);mainCamera.updateMatrixWorld(true);
const mainFrustums=[{window:[1280,720],scene:[970,720]},{window:[390,844],scene:[390,844*.58]}].map(view=>{mainCamera.aspect=view.scene[0]/view.scene[1];mainCamera.fov=mainCamera.aspect<1?54:39;mainCamera.updateProjectionMatrix();let clipped=0;for(const point of allPoints){const p=point.clone().project(mainCamera);if(Math.abs(p.x)>1||Math.abs(p.y)>1||Math.abs(p.z)>1)clipped++;}return {...view,vertices:allPoints.length,clipped};});
const result={note:'Static geometry/material checks only. The new +Z single-lens photo-guided mount supersedes earlier R1-derived wrist-camera projection artifacts. No rendered-image, occlusion, reflection or touch claim.',cameraReference:'codex-clipboard-b580ef28-743c-4683-bde9-c6cb5e142c6c.jpg',materialReference:'codex-clipboard-19ee4959-8e0b-4e75-81b1-c8e74cc5b223.png',cameraConfiguration:G1_WRIST_CAMERA,visualMeshes:visuals.length,uniqueMeshFiles:new Set(visuals.map(v=>v.filename)).size,materialReport,finishes:G1_FINISHES,headProfile:G1_HEAD_MASK_PROFILE,geometryUnchanged:unchanged,mainFrustums,wristChecks:wrist,fullFingerSweepMaxZ:fingerMaxZ,mountMinZ,mountToFingerSweepZClearance:mountMinZ-fingerMaxZ,pass:materialReport.unmappedLinks.length===0&&unchanged.every(v=>v.sourceBytes&&v.positions&&v.normals)&&mainFrustums.every(v=>v.clipped===0)&&wrist.every(v=>v.groups.every(g=>g.clipped===0)&&v.lensAndCameraSeparation<1e-9)&&mountMinZ>fingerMaxZ};
const sourceFiles=['work/check-g1-camera-materials.mjs','work/g1-test-robot.mjs','dist/g1/index.html','dist/g1/styles.css','dist/g1/g1-app.mjs','dist/g1/g1-workcell.mjs','dist/g1/g1-materials.mjs','dist/g1/g1-joint-ui.mjs','dist/g1/grasp-controller.mjs','dist/g1/grasp-kinematics.mjs','dist/g1/g1-profile.mjs','dist/g1/assets/g1_29dof_with_dex1_1.urdf',...new Set(visuals.map(v=>'dist/g1/assets/'+v.filename))];
result.sourceHashes=Object.fromEntries(sourceFiles.map(file=>[file,hash(fs.readFileSync(new URL('../'+file,import.meta.url)))]));
console.log(JSON.stringify(result,null,2));vision.dispose();
if(!result.pass)process.exitCode=1;
