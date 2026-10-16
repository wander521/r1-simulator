import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {C5Dynamics} from '../dist/c5/c5-dynamics.mjs';
import {ROOT_HEIGHT,TABLE_HEIGHT,PLINTH_RADIUS,PLINTH_HEIGHT,HOME_POSE,armJointNames,fingerJointNames,GRIPPER_Q_MIN,GRIPPER_Q_MAX,qToOpening,GRIPPER_GAP_AT_ZERO,GRIPPER_TCP_X,FLANGE_TCP_LOCAL} from '../dist/c5/c5-profile.mjs';
import {loadC5TestRobot} from './c5-test-robot.mjs';
import {deriveC5Dex1,readC5CollisionMeshes,convertC5URDF,parseURDF,attr} from './build-c5-dynamics.mjs';

const url=path=>new URL('../'+path,import.meta.url),hash=buffer=>createHash('sha256').update(buffer).digest('hex');
const [c5,g1,source,xml]=await Promise.all(['dist/c5/assets/aubo_C5.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf','dist/c5/models/c5-dynamic-scene.xml'].map(path=>readFile(url(path),'utf8')));
assert.equal(hash(c5),'c87a1b853791f91a7908d77e05bbf5f87496b02016ea1040916e7cad5c1c70df','Must be the audited official C5, not i5/_30');
assert.equal(deriveC5Dex1(c5,g1),source,'Derived assembly must regenerate from audited sources');
const collisionMeshes=await readC5CollisionMeshes(source);assert.equal(convertC5URDF(source,{collisionMeshes}),xml,'Compiled scene must regenerate exactly');
const official=parseURDF(c5.replace(/\r\n/g,'\n')),derived=parseURDF(source);assert.equal(derived.root,'base_link');assert.equal(derived.links.size,10);assert.equal(derived.joints.filter(j=>j.type==='revolute').length,6);assert.equal(derived.joints.filter(j=>j.type==='prismatic').length,2);
for(const name of armJointNames())assert.equal(derived.joints.find(j=>j.name===name).body,official.joints.find(j=>j.name===name).body,'Original movable joint metadata must be text-identical');
for(const name of ['base_link','shoulder_Link','upperArm_Link','foreArm_Link','wrist1_Link','wrist2_Link','wrist3_Link'])assert.equal(derived.links.get(name).body.match(/<inertial>[\s\S]*?<\/inertial>/)[0],official.links.get(name).body.match(/<inertial>[\s\S]*?<\/inertial>/)[0]);
const api=await createMuJoCo({locateFile:()=>url('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm').pathname}),vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model);vfs.delete();
const robot=await loadC5TestRobot(),dynamic=new C5Dynamics(api,model,data,robot),id=(type,name)=>api.mj_name2id(model,api.mjtObj[type].value,name),basis=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),Math.PI/2);
assert.equal(model.nu,8);assert.equal(dynamic.map.size,8);assert.equal(dynamic.telemetry().joints.length,6);assert.equal(dynamic.telemetry().allJoints.length,8);assert.equal(dynamic.telemetry().torso.length,0);assert.equal(dynamic.telemetry().legs.length,0);assert.equal(model.nmocap,0);assert.equal(model.neq,0);
assert.equal(ROOT_HEIGHT,TABLE_HEIGHT);assert.equal(PLINTH_HEIGHT,ROOT_HEIGHT);assert.equal(PLINTH_RADIUS,.12);assert(Math.abs(qToOpening(0)-GRIPPER_GAP_AT_ZERO)<1e-12);
let massError=0,comError=0,inertiaError=0;
const number=(s,tag,key)=>Number(attr(s,tag,key));
for(const {name,body}of derived.links.values()){
  const inertial=body.match(/<inertial>([\s\S]*?)<\/inertial>/)?.[1];if(!inertial)continue;const b=id('mjOBJ_BODY',name);assert(b>=0);
  massError=Math.max(massError,Math.abs(model.body_mass[b]-number(inertial,'mass','value')));
  const p=attr(inertial,'origin','xyz').trim().split(/\s+/).map(Number);for(let i=0;i<3;i++)comError=Math.max(comError,Math.abs(model.body_ipos[b*3+i]-p[i]));
  const q=new THREE.Quaternion(model.body_iquat[b*4+1],model.body_iquat[b*4+2],model.body_iquat[b*4+3],model.body_iquat[b*4]),r=new THREE.Matrix4().makeRotationFromQuaternion(q),diagonal=new THREE.Matrix4().makeScale(...model.body_inertia.subarray(b*3,b*3+3)),tensor=r.clone().multiply(diagonal).multiply(r.clone().transpose()).elements;
  for(const [key,i]of [['ixx',0],['iyy',5],['izz',10],['ixy',4],['ixz',8],['iyz',9]])inertiaError=Math.max(inertiaError,Math.abs(tensor[i]-number(inertial,'inertia',key)));
}
assert(massError<1e-12&&comError<1e-12&&inertiaError<2e-8);
for(const name of armJointNames()){
  const j=dynamic.map.get(name),original=official.joints.find(row=>row.name===name);assert.equal(j.lower,number(original.body,'limit','lower'));assert.equal(j.upper,number(original.body,'limit','upper'));assert.equal(j.effort,number(original.body,'limit','effort'));assert.equal(j.velocityLimit,number(original.body,'limit','velocity'));assert.equal(model.dof_armature[j.d],0,'No undocumented C5 rotor inertia is invented');assert.equal(model.dof_damping[j.d],number(original.body,'property','damping'));
}
for(const name of fingerJointNames()){const j=dynamic.map.get(name);assert.equal(j.lower,GRIPPER_Q_MIN);assert.equal(j.upper,GRIPPER_Q_MAX);assert.equal(j.effort,20);assert.equal(j.kp,1000);assert.equal(j.kv,20);assert.equal(model.jnt_solref[j.joint*2],.002);}
let fkPositionError=0,fkAngleError=0;
for(let trial=0;trial<12;trial++){
  const pose=Object.fromEntries([...dynamic.map].map(([name,j])=>[name,Math.max(j.lower,Math.min(j.upper,Math.sin(trial*1.3+j.q)*(j.finger?.014:.7)))]));dynamic.reset(pose);
  for(const [name,link]of Object.entries(robot.links)){
    const b=id('mjOBJ_BODY',name),p=link.getWorldPosition(new THREE.Vector3()),q=link.getWorldQuaternion(new THREE.Quaternion()),actualP=new THREE.Vector3(data.xpos[b*3],data.xpos[b*3+2],-data.xpos[b*3+1]),mq=new THREE.Quaternion(data.xquat[b*4+1],data.xquat[b*4+2],data.xquat[b*4+3],data.xquat[b*4]),actualQ=basis.clone().invert().multiply(mq);fkPositionError=Math.max(fkPositionError,p.distanceTo(actualP));fkAngleError=Math.max(fkAngleError,q.angleTo(actualQ));
  }
}
assert(fkPositionError<1e-10&&fkAngleError<1e-6);
const idle={...HOME_POSE,...Object.fromEntries(fingerJointNames().map(name=>[name,GRIPPER_Q_MAX]))};dynamic.reset(idle);
const flange=robot.links.wrist3_Link,gripper=robot.links.left_dex1_base_link,flangeTCP=flange.localToWorld(new THREE.Vector3(...FLANGE_TCP_LOCAL)),gripperTCP=gripper.localToWorld(new THREE.Vector3(GRIPPER_TCP_X,0,0));assert(flangeTCP.distanceTo(gripperTCP)<1e-12,'Adapter must align native Dex1 X to flange Z');
const base=id('mjOBJ_BODY','base_link'),basePosition=[...data.xpos.subarray(base*3,base*3+3)];assert(Math.abs(basePosition[2]-ROOT_HEIGHT)<1e-12);
const neutral=dynamic.checkPose(idle,{opening:qToOpening(GRIPPER_Q_MAX)});assert(neutral.safe,JSON.stringify(neutral));
const shoulderPoint=robot.joints.upperArm_joint.getWorldPosition(new THREE.Vector3()),elbowPoint=robot.joints.foreArm_joint.getWorldPosition(new THREE.Vector3()),wristPoint=robot.joints.wrist1_joint.getWorldPosition(new THREE.Vector3());
const upperDirection=elbowPoint.clone().sub(shoulderPoint).normalize(),foreDirection=wristPoint.clone().sub(elbowPoint).normalize(),toolDirection=new THREE.Vector3(1,0,0).applyQuaternion(gripper.getWorldQuaternion(new THREE.Quaternion()));
assert(upperDirection.y>Math.cos(10*Math.PI/180)&&upperDirection.x<0,'Photo standby upper arm must remain near upright, leaning slightly away from the front table');
assert(foreDirection.x>.85&&foreDirection.y>.25&&foreDirection.y<.55,'Photo standby elbow must bend the forearm forward and slightly up, not remain fully extended');
assert(toolDirection.distanceTo(new THREE.Vector3(0,-1,0))<1e-10,'Photo standby Dex1 tool-forward axis must point vertically down');
assert(gripperTCP.x>.25&&gripperTCP.x<.65&&gripperTCP.y-TABLE_HEIGHT>.30,'Photo standby TCP must remain in front and safely above the tabletop');
const homeGeometry={pose:{...HOME_POSE},shoulder:shoulderPoint.toArray(),elbow:elbowPoint.toArray(),wrist:wristPoint.toArray(),tcp:gripperTCP.toArray(),upperDirection:upperDirection.toArray(),foreDirection:foreDirection.toArray(),toolDirection:toolDirection.toArray(),collisionSafe:neutral.safe};
let maxStaticError=0,maxTrackingError=0,maxTorque=0,maxFingerLimitViolation=0,maxFingerSpeed=0,actualMotorWork=false;
const inspect=()=>{for(const j of dynamic.telemetry().allJoints){assert(Number.isFinite(j.actual)&&Number.isFinite(j.torque)&&Number.isFinite(j.error));assert(Math.abs(j.torque)<=j.effortLimit+1e-8);assert(j.actual>=j.lower-(j.unit==='m'?.0002:.001)&&j.actual<=j.upper+(j.unit==='m'?.0002:.001));assert(Math.abs(j.velocity)<=j.velocityLimit+.01,`${j.name} actual velocity exceeds source limit + .01 solver allowance`);maxTorque=Math.max(maxTorque,Math.abs(j.torque));if(j.unit==='m'){assert(Math.abs(j.targetVelocity)<=j.velocityLimit+1e-10,'Finger motor target must ramp at original 0.2m/s');maxFingerLimitViolation=Math.max(maxFingerLimitViolation,j.lower-j.actual,j.actual-j.upper);maxFingerSpeed=Math.max(maxFingerSpeed,Math.abs(j.velocity));}actualMotorWork ||= Math.abs(j.torque)>.01&&Math.abs(j.velocity)>.01;}assert.deepEqual([...data.xpos.subarray(base*3,base*3+3)],basePosition);};
for(let step=0;step<6000;step++){dynamic.beforeStep();api.mj_step(model,data);dynamic.afterStep();inspect();for(const j of dynamic.telemetry().joints)maxStaticError=Math.max(maxStaticError,Math.abs(j.error));}
assert(maxStaticError<.001);
for(let step=0;step<4000;step++){
  // Exercise the same smooth increments about the configured standby pose.
  // Assigning old zero-centred targets here would introduce a 1.57 rad step
  // at sample zero after HOME became the reference-photo bent-arm posture.
  const s=(1-Math.cos(step*.001*Math.PI/2))/2,pose={...idle};armJointNames().forEach((name,i)=>pose[name]=HOME_POSE[name]+[.15,.18,.13,.12,-.1,.1][i]*s);dynamic.setTargets(pose);dynamic.beforeStep();api.mj_step(model,data);dynamic.afterStep();inspect();for(const j of dynamic.telemetry().joints)maxTrackingError=Math.max(maxTrackingError,Math.abs(j.error));
}
assert(maxTrackingError<.003);assert(actualMotorWork,'Actual qfrc_actuator and qvel must exhibit motor work during tracking');
dynamic.reset(idle);
for(let step=0;step<6000;step++){
  const target=Math.floor(step/500)%2===0?GRIPPER_Q_MIN:GRIPPER_Q_MAX;dynamic.setTargets(Object.fromEntries(fingerJointNames().map(name=>[name,target])));dynamic.beforeStep();api.mj_step(model,data);dynamic.afterStep();inspect();
}
// A one-shot API target must persist even when no controller rewrites ctrl;
// also cover the real browser loader, which does not populate limit.velocity.
dynamic.reset(idle);dynamic.setTargets(Object.fromEntries(fingerJointNames().map(name=>[name,GRIPPER_Q_MIN])));
for(let step=0;step<600;step++){dynamic.beforeStep();api.mj_step(model,data);dynamic.afterStep();inspect();}
assert(dynamic.telemetry().fingers.every(j=>j.target===GRIPPER_Q_MIN));
const loaderRobot=await loadC5TestRobot();for(const joint of Object.values(loaderRobot.joints))delete joint.limit.velocity;
const loaderData=new api.MjData(model),loaderDynamic=new C5Dynamics(api,model,loaderData,loaderRobot);assert(loaderDynamic.telemetry().allJoints.every(j=>Number.isFinite(j.velocityLimit)&&j.velocityLimit>0));loaderDynamic.dispose();loaderData.delete();
dynamic.reset(idle);dynamic.setTargets(Object.fromEntries(fingerJointNames().map(name=>[name,GRIPPER_Q_MIN])));dynamic.beforeStep();
const resetTargets=Object.fromEntries(dynamic.telemetry().fingers.map(j=>[j.name,j.target]));for(const name of fingerJointNames())data.ctrl[dynamic.map.get(name).actuator]=GRIPPER_Q_MIN;
dynamic.reset();for(const j of dynamic.telemetry().fingers){assert.equal(j.target,resetTargets[j.name]);assert.equal(j.desiredTarget,resetTargets[j.name]);assert.equal(j.actual,resetTargets[j.name]);assert.equal(j.targetVelocity,0);}
dynamic.reset(idle);
const snapshot=()=>({qpos:[...data.qpos],qvel:[...data.qvel],ctrl:[...data.ctrl],xpos:[...data.xpos],xquat:[...data.xquat],time:data.time,targets:{...dynamic.targets}}),before=snapshot();
let blockedSelf,blockedEnvironment;
for(const upper of [-2.8,-2.1,-1.4,.7,1.4,2.1,2.8])for(const elbow of [-2.7,-1.8,-.9,0,.9,1.8,2.7]){
  const pose={...idle,upperArm_joint:upper,foreArm_joint:elbow},check=dynamic.checkPose(pose,{opening:.07});if(!blockedSelf&&check.contacts.some(c=>c.kind==='robot-self'&&c.dist<-.001))blockedSelf={pose,contacts:check.contacts.filter(c=>c.kind==='robot-self')};if(!blockedEnvironment&&check.contacts.some(c=>c.kind==='robot-environment'&&c.dist<-.001))blockedEnvironment={pose,contacts:check.contacts.filter(c=>c.kind==='robot-environment')};
}
assert(blockedSelf,'Non-adjacent C5 shell self-collision must be detectable');assert(blockedEnvironment,'C5 shell against table/plinth/floor must be detectable');assert.deepEqual(snapshot(),before,'Planning probes must never mutate actual state');
const blue=id('mjOBJ_BODY','blue_block'),objectPose=p=>[{body:blue,position:p.toArray(),quaternion:[0,0,0,1]}];
const graspPoint=gripper.localToWorld(new THREE.Vector3(.12,0,0)),fingerContact=dynamic.checkPose(idle,{opening:.020,objectPoses:objectPose(graspPoint)}),fingerAllowed=dynamic.checkPose(idle,{opening:.020,objectPoses:objectPose(graspPoint),ignoreObjectBodies:[blue]});assert(!fingerContact.safe&&fingerContact.contacts.some(c=>[c.body1,c.body2].some(n=>n.includes('finger'))));assert(fingerAllowed.safe,JSON.stringify(fingerAllowed));
const palmPoint=gripper.localToWorld(new THREE.Vector3(.04,0,0)),palmBlocked=dynamic.checkPose(idle,{opening:.020,objectPoses:objectPose(palmPoint),ignoreObjectBodies:[blue]});assert(!palmBlocked.safe&&palmBlocked.contacts.some(c=>[c.body1,c.body2].includes('left_dex1_base_link')),'Ignoring selected fingertips must not disable palm collision');
const armPoint=robot.links.upperArm_Link.localToWorld(new THREE.Vector3(.204,0,0)),armBlocked=dynamic.checkPose(idle,{objectPoses:objectPose(armPoint),ignoreObjectBodies:[blue]});assert(!armBlocked.safe&&armBlocked.contacts.some(c=>[c.body1,c.body2].includes('upperArm_Link')));
const predictedPosition=[1.3,1.4,.7],predictedQuaternion=new THREE.Quaternion().setFromEuler(new THREE.Euler(.23,-.42,.31)),prediction=dynamic.checkPose(idle,{objectPoses:[{body:'blue_block',position:predictedPosition,quaternion:predictedQuaternion.toArray()}]});assert(prediction.safe);
const actualPrediction=new THREE.Vector3(dynamic.scratch.xpos[blue*3],dynamic.scratch.xpos[blue*3+2],-dynamic.scratch.xpos[blue*3+1]),predictionPositionError=actualPrediction.distanceTo(new THREE.Vector3(...predictedPosition)),sq=dynamic.scratch.xquat.subarray(blue*4,blue*4+4),predictedActualQ=basis.clone().invert().multiply(new THREE.Quaternion(sq[1],sq[2],sq[3],sq[0])).multiply(basis),predictionRotationError=predictedActualQ.angleTo(predictedQuaternion);assert(predictionPositionError<1e-12&&predictionRotationError<1e-7);
const invalidCases=[{body:'missing',position:[0,0,0],quaternion:[0,0,0,1]},{body:'task_table',position:[0,0,0],quaternion:[0,0,0,1]},{body:'base_link',position:[0,0,0],quaternion:[0,0,0,1]},{body:'blue_block',position:[NaN,0,0],quaternion:[0,0,0,1]},{body:'blue_block',position:[0,0,0],quaternion:[0,0,0,0]}];for(const p of invalidCases){const check=dynamic.checkPose(idle,{objectPoses:[p]});assert(!check.safe&&check.limitViolations.length>0);}assert.deepEqual(snapshot(),before);
const paths=['work/build-c5-dynamics.mjs','work/c5-test-robot.mjs','work/test-c5-dynamics.mjs','dist/c5/c5-profile.mjs','dist/c5/c5-dynamics.mjs','dist/c5/assets/aubo_C5.urdf','dist/c5/assets/c5-dex1.urdf','dist/c5/models/c5-dynamic-scene.xml','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs','dist/g1/assets/g1_29dof_with_dex1_1.urdf'],sourceHashes=Object.fromEntries(await Promise.all(paths.map(async path=>[path,hash(await readFile(url(path)))])));for(const [file,buffer]of collisionMeshes)sourceHashes['dist/c5/assets/'+file]=hash(buffer);
console.log(JSON.stringify({nu:model.nu,nq:model.nq,nv:model.nv,links:derived.links.size,rootHeight:ROOT_HEIGHT,massError,comError,inertiaError,fkPositionError,fkAngleError,homeGeometry,maxStaticError,maxTrackingError,maxTorque,maxFingerLimitViolation,maxFingerSpeedStress:maxFingerSpeed,actualMotorWork,collisionChecks:{neutralSafe:neutral.safe,blockedSelf,blockedEnvironment,fingerAllowed:fingerAllowed.safe,palmBlocked:!palmBlocked.safe,carriedObjectArmBlocked:!armBlocked.safe,actualStateUnchanged:true},objectPoseChecks:{predictionPositionError,predictionRotationError,invalidCasesRejected:invalidCases.length,actualStateUnchanged:true},sourceHashes}));
dynamic.dispose();data.delete();model.delete();
