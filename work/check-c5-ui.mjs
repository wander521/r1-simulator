import fs from 'node:fs';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import * as THREE from '../dist/vendor/three.module.min.js';
import {createWorkcell,createWristVision,C5_PLINTH} from '../dist/c5/c5-workcell.mjs';
import {applyC5Materials} from '../dist/c5/c5-materials.mjs';
import * as profile from '../dist/c5/c5-profile.mjs';

const file=path=>fs.readFileSync(new URL('../'+path,import.meta.url));
const source=path=>file(path).toString('utf8');
const app=source('dist/c5/c5-app.mjs'),html=source('dist/c5/index.html'),css=source('dist/c5/styles.css'),joint=source('dist/c5/c5-joint-ui.mjs');
const checks=[];
function check(name,fn){fn();checks.push(name);}
check('Single six-axis route has no humanoid controls or copy',()=>{
  assert(!/model-variant|双臂|七轴|腰部|下蹲|站立|宇树 G1/.test(html+app));
  assert(html.includes('遨博 C5'));assert(html.includes('Dex1'));
  assert(joint.includes('const gripper=i===6'));assert(joint.includes('Array.from({length:6}'));
  assert(joint.includes('values.length===6'));assert.equal(profile.armJointNames().length,6);
});
check('Solid objects own editing; blank space remains available to orbit',()=>{
  assert(app.includes('intersectObjects(targets.map(t=>t.mesh),false)'));
  assert(app.includes('if(!hit)return'));assert(app.includes('orbit.enabled=false'));assert(app.includes('orbit.enabled=true'));
  for(const name of ['pointercancel','lostpointercapture','blur','KeyW','KeyA','KeyS','KeyD'])assert(app.includes(name));
  assert(app.includes("mode:event.button===2||event.shiftKey?'rotate':'move'"));
});
check('Reference objects, pulse and matching empty placement frames are retained',()=>{
  for(const id of ['red','blue','yellow'])assert(app.includes(`id:'${id}'`));
  assert(app.includes('new THREE.CylinderGeometry(.02,.02,.052,32)'));
  assert(app.includes('[[0,-.04,.08,.006],[0,.04,.08,.006],[-.04,0,.006,.08],[.04,0,.006,.08]]'));
  assert(app.includes('reducedMotion?.5:.5+.5*Math.sin(now*.006)'));
});
check('Manual gripper and joint motions go through swept collision validation',()=>{
  assert.equal((app.match(/setManualOpening\(/g)||[]).length,3);
  assert.equal((app.match(/applyManualPose\(/g)||[]).length,2);
  assert(!/controller\.opening\s*=/.test(app));
});
check('Two real camera renderers and measured force/joint charts remain connected',()=>{
  for(const id of ['overhead-camera-canvas','wrist-camera-canvas','torque-chart','error-chart'])assert(html.includes(id));
  assert(app.includes('feeds.forEach((r,i)=>r.render(scene,cameras[i]))'));
  assert(app.includes('controller.dynamics.telemetry(arm)'));assert(app.includes('forceHistory.push(m.forces)'));
  assert(joint.includes('六关节实际电机力矩'));assert(joint.includes('六关节目标减实际角度'));
});
check('Responsive layout, keyboard focus and reduced motion preserve the incumbent system',()=>{
  assert(css.includes('@media(max-width:900px)'));assert(css.includes('height:58svh'));
  assert(css.includes('focus-visible'));assert(css.includes('prefers-reduced-motion:reduce'));
  assert(html.includes('lang="zh-CN"'));assert(html.includes('aria-live="polite"'));
});
check('Official source visuals use native 3DS loader, not collision STL silhouettes',()=>{
  assert(app.includes("import {loadC5Mesh} from './c5-3ds.mjs"));
  assert(app.includes('loadC5Mesh(path,manager,done):defaultMeshLoader(path,manager,done)'));
});
const scene=new THREE.Scene(),workcell=createWorkcell(scene,{THREE,TABLE_HEIGHT:profile.TABLE_HEIGHT,tableCenter:profile.TABLE_CENTER_X,tableHalfX:profile.TABLE_HALF_X,tableHalfZ:profile.TABLE_HALF_Z});
const bounds=new THREE.Box3().setFromObject(workcell.plinth),table=new THREE.Box3().setFromObject(workcell.table);
check('Robot plinth and table both meet the floor and remain separate',()=>{
  assert.equal(C5_PLINTH.height,profile.ROOT_HEIGHT);assert.equal(C5_PLINTH.radius,profile.PLINTH_RADIUS);
  assert(Math.abs(bounds.min.y)<1e-7);assert(Math.abs(bounds.max.y-profile.ROOT_HEIGHT)<1e-7);
  assert(Math.abs(table.min.y)<1e-7);assert(Math.abs(table.max.y-profile.TABLE_HEIGHT)<1e-7);
  assert(table.min.x-bounds.max.x>.09);
});
const camera=new THREE.PerspectiveCamera(65,4/3,.01,10),vision=createWristVision({scene,THREE,camera});
const link=new THREE.Group();link.name=profile.gripperLinkName();link.position.set(.4,.9,.1);link.rotation.set(-.2,.4,.1);scene.add(link);scene.updateMatrixWorld(true);vision.update(link);
const lens=vision.model.getObjectByName('c5-front-camera-lens').getWorldPosition(new THREE.Vector3()),tcp=link.localToWorld(new THREE.Vector3(profile.GRIPPER_TCP_X,0,0));
check('Single front Dex1 lens shares the actual optical center and sees TCP',()=>{
  assert(lens.distanceTo(camera.position)<1e-9);
  const local=link.worldToLocal(camera.position.clone());assert(Math.abs(local.x-.060)<1e-9);assert(Math.abs(local.z-.060)<1e-9);
  const projected=tcp.clone().project(camera);assert(Math.abs(projected.x)<1e-8&&Math.abs(projected.y)<1e-8&&Math.abs(projected.z)<1);
  assert.equal(vision.model.getObjectByName('c5-front-camera-lens').layers.mask,2);
  assert.equal(camera.layers.mask&2,0);assert.equal(camera.fov,76);
});
const paintRobot=new THREE.Group(),body=Object.assign(new THREE.Group(),{name:'upperArm_Link',isURDFLink:true}),finger=Object.assign(new THREE.Group(),{name:'left_dex1_base_link',isURDFLink:true});
const sourceMaterial=new THREE.MeshStandardMaterial({color:0x184cc6}),shell=new THREE.Mesh(new THREE.BoxGeometry(.1,.1,.1),sourceMaterial),dex=new THREE.Mesh(new THREE.BoxGeometry(.1,.1,.1));body.add(shell);finger.add(dex);paintRobot.add(body,finger);const painted=applyC5Materials(paintRobot);
check('Official C5 source material colors are not overwritten with G1 finishes',()=>{
  assert.equal(shell.material,sourceMaterial);assert.equal(painted.sourceMaterials,1);assert.equal(dex.userData.c5Finish,'gripper');
});
const paths=['dist/c5/index.html','dist/c5/c5-app.mjs','dist/c5/c5-joint-ui.mjs','dist/c5/c5-workcell.mjs','dist/c5/styles.css','dist/c5/c5-materials.mjs','dist/c5/c5-profile.mjs','work/check-c5-ui.mjs'];
const result={scope:'Static UI wiring, geometry and camera checks; browser rendering and pointer gestures are a separate acceptance check.',checks,pass:true,plinth:{min:bounds.min.toArray(),max:bounds.max.toArray()},table:{min:table.min.toArray(),max:table.max.toArray()},sourceHashes:Object.fromEntries(paths.map(path=>[path,crypto.createHash('sha256').update(file(path)).digest('hex')]))};
console.log(JSON.stringify(result,null,2));vision.dispose();workcell.dispose();
