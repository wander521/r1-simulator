import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {G1Dynamics} from '../dist/g1/g1-dynamics.mjs';
import {ROOT_HEIGHT,STAND_POSE,armJointNames,fingerJointNames,GRIPPER_Q_MAX,qToOpening,openingToQ,GRIPPER_GAP_AT_ZERO,GRIPPER_TCP_X,GRIPPER_WRIST_TCP_X} from '../dist/g1/g1-profile.mjs';
import {loadG1TestRobot} from './g1-test-robot.mjs';
import {convertG1URDF,sourceFootSole,readG1CollisionMeshes} from './build-g1-dynamics.mjs';
const source=await readFile(new URL('../dist/g1/assets/g1_29dof_with_dex1_1.urdf',import.meta.url),'utf8'),xml=await readFile(new URL('../dist/g1/models/g1-dynamic-scene.xml',import.meta.url),'utf8');
const fingerSTLs=await Promise.all([1,2].map(i=>readFile(new URL(`../dist/g1/assets/dex1_1/meshes/dex1_col_${i}.stl`,import.meta.url)))),collisionMeshes=await readG1CollisionMeshes(source);assert.equal(convertG1URDF(source,{fingerSTLs,collisionMeshes}),xml,'Generated artifact must match original source inputs');
const api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname}),vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model);vfs.delete();
const robot=await loadG1TestRobot(),dynamic=new G1Dynamics(api,model,data,robot),id=(type,name)=>api.mj_name2id(model,api.mjtObj[type].value,name);
assert.equal(model.nu,33);assert.equal(dynamic.map.size,33);assert.equal(dynamic.telemetry().joints.length,7);assert.equal(dynamic.telemetry().allJoints.length,33);assert.equal(model.nmocap,0);assert.equal(model.neq,0);
assert(Math.abs(sourceFootSole(source).rootHeight-ROOT_HEIGHT)<1e-12);assert(Math.abs(qToOpening(0)-GRIPPER_GAP_AT_ZERO)<1e-12);assert(Math.abs(openingToQ(qToOpening(.01))-.01)<1e-12);assert.equal(GRIPPER_TCP_X+.0415,GRIPPER_WRIST_TCP_X);
const attr=(s,tag,key)=>Number(s.match(new RegExp(`<${tag}\\b[^>]*\\b${key}="([^"]+)"`))?.[1]);
let massError=0,inertiaError=0,comError=0;
for(const [,name,body]of source.matchAll(/<link\s+name="([^"]+)"(?![^>]*\/>)[^>]*>([\s\S]*?)<\/link>/g)){
  const inertial=body.match(/<inertial>([\s\S]*?)<\/inertial>/)?.[1];if(!inertial)continue;
  const b=id('mjOBJ_BODY',name);assert(b>=0,name);massError=Math.max(massError,Math.abs(model.body_mass[b]-attr(inertial,'mass','value')));
  const p=inertial.match(/<origin[^>]*xyz="([^"]+)"/)[1].split(/\s+/).map(Number);for(let i=0;i<3;i++)comError=Math.max(comError,Math.abs(model.body_ipos[b*3+i]-p[i]));
  const q=new THREE.Quaternion(model.body_iquat[b*4+1],model.body_iquat[b*4+2],model.body_iquat[b*4+3],model.body_iquat[b*4]),r=new THREE.Matrix4().makeRotationFromQuaternion(q),diagonal=new THREE.Matrix4().makeScale(...model.body_inertia.subarray(b*3,b*3+3)),tensor=r.clone().multiply(diagonal).multiply(r.clone().transpose()).elements;
  for(const [key,i]of [['ixx',0],['iyy',5],['izz',10],['ixy',4],['ixz',8],['iyz',9]])inertiaError=Math.max(inertiaError,Math.abs(tensor[i]-attr(inertial,'inertia',key)));
}
assert(massError<1e-12);assert(comError<1e-12);assert(inertiaError<2e-8);
let fkPositionError=0,fkAngleError=0;
for(let trial=0;trial<8;trial++){
  const pose=Object.fromEntries([...dynamic.map].map(([name,j])=>[name,Math.max(j.lower,Math.min(j.upper,Math.sin(trial*1.1+j.q)*(j.finger?.015:.3)))]));dynamic.reset(pose);
  for(const name of Object.keys(robot.links)){
    const b=id('mjOBJ_BODY',name);assert(b>=0,`Missing ${name}`);const link=robot.links[name],p=link.getWorldPosition(new THREE.Vector3()),q=link.getWorldQuaternion(new THREE.Quaternion()),actualP=new THREE.Vector3(data.xpos[b*3],data.xpos[b*3+2],-data.xpos[b*3+1]),mq=new THREE.Quaternion(data.xquat[b*4+1],data.xquat[b*4+2],data.xquat[b*4+3],data.xquat[b*4]),actualQ=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2).multiply(mq);
    fkPositionError=Math.max(fkPositionError,p.distanceTo(actualP));fkAngleError=Math.max(fkAngleError,q.angleTo(actualQ));
  }
}
assert(fkPositionError<1e-10);assert(fkAngleError<1e-6);
const idle={...STAND_POSE};for(const side of ['left','right']){armJointNames(side).forEach(name=>idle[name]=0);idle[`${side}_elbow_joint`]=1.45;fingerJointNames(side).forEach(name=>idle[name]=GRIPPER_Q_MAX);}
dynamic.reset(idle);const initialFeet=['left','right'].map(side=>robot.links[`${side}_ankle_roll_link`].getWorldPosition(new THREE.Vector3()));let maxStaticError=0,maxFootDrift=0,maxTorque=0;
for(let step=0;step<6000;step++){
  dynamic.beforeStep();api.mj_step(model,data);dynamic.afterStep();
  if(step%100===0){const t=dynamic.telemetry();for(const j of t.allJoints){assert(Number.isFinite(j.actual)&&Number.isFinite(j.torque)&&Number.isFinite(j.error));assert(Math.abs(j.torque)<=j.effortLimit+1e-8);if(j.unit==='rad')maxStaticError=Math.max(maxStaticError,Math.abs(j.error));maxTorque=Math.max(maxTorque,Math.abs(j.torque));}for(const [i,side]of ['left','right'].entries())maxFootDrift=Math.max(maxFootDrift,robot.links[`${side}_ankle_roll_link`].getWorldPosition(new THREE.Vector3()).distanceTo(initialFeet[i]));}
}
assert(maxStaticError<.001);assert(maxFootDrift<.0002);
for(const side of ['left','right']){const q=robot.links[`${side}_ankle_roll_link`].getWorldQuaternion(new THREE.Quaternion()),up=new THREE.Vector3(0,0,1).applyQuaternion(q);assert(up.y>.9999,'Standing feet stay horizontal');}
let maxTrackingError=0;
for(let step=0;step<4000;step++){
  const t=step*.001,s=.5-.5*Math.cos(t*Math.PI/2),pose={...idle,right_shoulder_pitch_joint:-.2*s,left_shoulder_pitch_joint:-.2*s,right_elbow_joint:1.45-.2*s,left_elbow_joint:1.45-.2*s};dynamic.setTargets(pose);dynamic.beforeStep();api.mj_step(model,data);dynamic.afterStep();if(step%100===0)for(const j of dynamic.telemetry().joints)maxTrackingError=Math.max(maxTrackingError,Math.abs(j.error));
}
assert(maxTrackingError<.003);
const before={qpos:[...data.qpos],qvel:[...data.qvel],ctrl:[...data.ctrl],time:data.time,targets:{...dynamic.targets}};
const neutral=dynamic.checkPose(idle,{opening:qToOpening(GRIPPER_Q_MAX)});assert(neutral.safe,`Neutral pose collides: ${JSON.stringify(neutral)}`);
const blocked=dynamic.checkPose({...idle,right_shoulder_roll_joint:.6},{opening:qToOpening(GRIPPER_Q_MAX)});assert(!blocked.safe&&blocked.contacts.some(c=>c.kind==='robot-self'&&c.dist<0),'Whole upper arm must collide with the real torso hull');
assert.deepEqual([...data.qpos],before.qpos);assert.deepEqual([...data.qvel],before.qvel);assert.deepEqual([...data.ctrl],before.ctrl);assert.equal(data.time,before.time);assert.deepEqual(dynamic.targets,before.targets);
const blue=id('mjOBJ_BODY','blue_block'),blueQ=model.jnt_qposadr[model.body_jntadr[blue]],link=robot.links.right_dex1_base_link;
const editBlue=p=>{data.qpos.set([p.x,-p.z,p.y,1,0,0,0],blueQ);api.mj_forward(model,data);};
editBlue(link.localToWorld(new THREE.Vector3(.12,0,0)));
const gripped=dynamic.checkPose(idle,{opening:{right:.020}}),allowed=dynamic.checkPose(idle,{opening:{right:.020},ignoreObjectBodies:['blue_block']});
assert(!gripped.safe&&gripped.contacts.some(c=>c.body1.includes('finger')||c.body2.includes('finger')));assert(allowed.safe,`Allowed fingertip grasp should be ignored: ${JSON.stringify(allowed)}`);
editBlue(link.localToWorld(new THREE.Vector3(.04,0,0)));const palmBlocked=dynamic.checkPose(idle,{opening:{right:.020},ignoreObjectBodies:[blue]});assert(!palmBlocked.safe&&palmBlocked.contacts.some(c=>[c.body1,c.body2].includes('right_dex1_base_link')),'Ignoring the selected object must never allow palm/arm penetration');
data.qpos.set(before.qpos);data.qvel.set(before.qvel);api.mj_forward(model,data);
const predictionBefore={qpos:[...data.qpos],qvel:[...data.qvel],ctrl:[...data.ctrl],xpos:[...data.xpos],xquat:[...data.xquat],time:data.time,targets:{...dynamic.targets}};
const predictedPosition=[1.2,1.4,.6],predictedQuaternion=new THREE.Quaternion().setFromEuler(new THREE.Euler(.23,-.42,.31));
const prediction=dynamic.checkPose(idle,{objectPoses:[{body:'blue_block',position:predictedPosition,quaternion:predictedQuaternion.toArray()}]});assert(prediction.safe);
const actualPrediction=new THREE.Vector3(dynamic.scratch.xpos[blue*3],dynamic.scratch.xpos[blue*3+2],-dynamic.scratch.xpos[blue*3+1]),predictionPositionError=actualPrediction.distanceTo(new THREE.Vector3(...predictedPosition));
const sq=dynamic.scratch.xquat.subarray(blue*4,blue*4+4),basis=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),Math.PI/2),predictedActualQ=basis.clone().invert().multiply(new THREE.Quaternion(sq[1],sq[2],sq[3],sq[0])).multiply(basis),predictionRotationError=predictedActualQ.angleTo(predictedQuaternion);assert(predictionPositionError<1e-12&&predictionRotationError<1e-7);
const torsoPoint=robot.links.torso_link.localToWorld(new THREE.Vector3(.02,0,.15)),carriedIntoTorso=dynamic.checkPose(idle,{ignoreObjectBodies:[blue],objectPoses:[{body:blue,position:torsoPoint.toArray(),quaternion:[0,0,0,1]}]});assert(!carriedIntoTorso.safe&&carriedIntoTorso.contacts.some(c=>[c.body1,c.body2].includes('torso_link')&&[c.body1,c.body2].includes('blue_block')),'Predicted selected task object must collide with torso even if fingertip contact is ignored');
const invalidCases=[{body:'does_not_exist',position:[0,0,0],quaternion:[0,0,0,1]},{body:'task_table',position:[0,0,0],quaternion:[0,0,0,1]},{body:'pelvis',position:[0,0,0],quaternion:[0,0,0,1]},{body:'blue_block',position:[NaN,0,0],quaternion:[0,0,0,1]},{body:'blue_block',position:[0,0,0],quaternion:[0,0,0,0]},{body:'blue_block',position:[0,0,0],quaternion:[0,0,1]}];
for(const value of invalidCases){const result=dynamic.checkPose(idle,{objectPoses:[value]});assert(!result.safe&&result.limitViolations.some(v=>v.type==='invalid-object'));}
const invalidList=dynamic.checkPose(idle,{objectPoses:null});assert(!invalidList.safe&&invalidList.limitViolations.some(v=>v.type==='invalid-object'));
assert.deepEqual([...data.qpos],predictionBefore.qpos);assert.deepEqual([...data.qvel],predictionBefore.qvel);assert.deepEqual([...data.ctrl],predictionBefore.ctrl);assert.deepEqual([...data.xpos],predictionBefore.xpos);assert.deepEqual([...data.xquat],predictionBefore.xquat);assert.equal(data.time,predictionBefore.time);assert.deepEqual(dynamic.targets,predictionBefore.targets);
const sourceHashes=Object.fromEntries(await Promise.all(['work/build-g1-dynamics.mjs','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs','dist/g1/models/g1-dynamic-scene.xml','dist/g1/assets/g1_29dof_with_dex1_1.urdf'].map(async path=>[path,createHash('sha256').update(await readFile(new URL('../'+path,import.meta.url))).digest('hex')])));for(const [file,buffer]of collisionMeshes)sourceHashes['dist/g1/assets/'+file]=createHash('sha256').update(buffer).digest('hex');
console.log(JSON.stringify({nq:model.nq,nv:model.nv,nu:model.nu,links:Object.keys(robot.links).length,rootHeight:ROOT_HEIGHT,massError,comError,inertiaError,fkPositionError,fkAngleError,maxStaticError,maxFootDrift,maxTorque,maxTrackingError,collisionChecks:{neutralSafe:neutral.safe,blockedContacts:blocked.contacts,selectedFingerIgnoreSafe:allowed.safe,selectedPalmStillBlocked:!palmBlocked.safe,actualStateUnchanged:true},objectPoseChecks:{predictionPositionError,predictionRotationError,selectedObjectTorsoBlocked:!carriedIntoTorso.safe,invalidCasesRejected:invalidCases.length+1,actualStateUnchanged:true},actualLegs:dynamic.telemetry().legs.map(j=>({name:j.name,actual:j.actual,target:j.target})),sourceHashes}));
dynamic.dispose();data.delete();model.delete();
