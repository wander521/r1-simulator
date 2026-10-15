import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {R1Dynamics} from '../dist/r1/r1-dynamics.mjs';
import {loadTestRobot} from './r1-test-robot.mjs';
const api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
const xml=await readFile(new URL('../dist/r1/models/r1-dynamic-scene.xml',import.meta.url),'utf8');
const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(xml,vfs),data=new api.MjData(model);vfs.delete();
const robot=await loadTestRobot(),dynamic=new R1Dynamics(api,model,data,robot);
const idle={torso_joint1:0,torso_joint2:.06,torso_joint3:0,torso_joint4:0};
for(const arm of ['right','left']){
  const s=arm==='right'?1:-1;
  [-1.57*s,2.2,-1.4,0,0,0].forEach((v,i)=>idle[`${arm}_arm_joint${i+1}`]=v);
  [1,2].forEach(i=>idle[`${arm}_gripper_finger_joint${i}`]=.05);
}
const id=(type,name)=>api.mj_name2id(model,api.mjtObj[type].value,name);
const physicalNames=name=>name.replace(/_gripper_finger_link(\d)/,'_pad$1');
const source=await readFile(new URL('../dist/r1/assets/r1.urdf',import.meta.url),'utf8');
const attribute=(s,tag,key)=>Number(s.match(new RegExp(`<${tag}\\b[^>]*\\b${key}="([^"]+)"`))?.[1]);
let massError=0,inertiaError=0;
for(const [,name,body] of source.matchAll(/<link\s+name="([^"]+)"[^>]*>([\s\S]*?)<\/link>/g)){
  const b=id('mjOBJ_BODY',physicalNames(name));massError=Math.max(massError,Math.abs(model.body_mass[b]-attribute(body,'mass','value')));
  const q=new THREE.Quaternion(model.body_iquat[b*4+1],model.body_iquat[b*4+2],model.body_iquat[b*4+3],model.body_iquat[b*4]);
  const rotation=new THREE.Matrix4().makeRotationFromQuaternion(q),diagonal=new THREE.Matrix4().makeScale(...model.body_inertia.subarray(b*3,b*3+3));
  const tensor=rotation.clone().multiply(diagonal).multiply(rotation.clone().transpose()).elements;
  for(const [key,index] of [['ixx',0],['iyy',5],['izz',10],['ixy',4],['ixz',8],['iyz',9]])inertiaError=Math.max(inertiaError,Math.abs(tensor[index]-attribute(body,'inertia',key)));
}
// MuJoCo diagonalizes full inertia tensors with a finite eigensolver
// tolerance; compare the compiled reconstructed tensor at that precision.
assert(massError<1e-12,`URDF masses ${massError}`);assert(inertiaError<2e-8,`URDF inertia tensors ${inertiaError}`);
let fkPositionError=0,fkAngleError=0;
for(let trial=0;trial<8;trial++){
  const pose={...idle};for(const [name,j] of dynamic.map)if(!name.includes('finger'))pose[name]=Math.max(j.lower,Math.min(j.upper,(Math.sin(trial*1.7+j.q)*.4)+(idle[name]??0)));
  dynamic.reset(pose);
  for(const name of ['torso_link1','torso_link2','torso_link3','torso_link4','zed_link','right_arm_link2','left_arm_link2','right_gripper_link','left_gripper_link','right_gripper_finger_link1','left_gripper_finger_link2']){
    const b=id('mjOBJ_BODY',physicalNames(name)),link=robot.getObjectByName(name),p=link.getWorldPosition(new THREE.Vector3()),q=link.getWorldQuaternion(new THREE.Quaternion());
    const actualP=new THREE.Vector3(data.xpos[b*3],data.xpos[b*3+2],-data.xpos[b*3+1]);
    const mq=new THREE.Quaternion(data.xquat[b*4+1],data.xquat[b*4+2],data.xquat[b*4+3],data.xquat[b*4]);
    const actualQ=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2).multiply(mq);
    fkPositionError=Math.max(fkPositionError,p.distanceTo(actualP));fkAngleError=Math.max(fkAngleError,q.angleTo(actualQ));
  }
}
assert(fkPositionError<1e-10,`FK positions ${fkPositionError}`);assert(fkAngleError<1e-6,`FK angles ${fkAngleError}`);
dynamic.reset(idle);
for(const arm of ['right','left'])for(const i of [1,2])data.ctrl[id('mjOBJ_ACTUATOR',`${arm}_motor${i}`)]=.05;
let maxStaticError=0,maxMovingError=0,maxTorque=0;
for(let step=0;step<6000;step++){
  const t=step*.001,target={...idle};
  if(t>1&&t<5){const a=(t-1)*Math.PI/2,envelope=.5-.5*Math.cos(a);for(const arm of ['right','left'])target[`${arm}_arm_joint1`]+=.15*envelope;target.torso_joint2+=.06*envelope;}
  dynamic.setTargets(target);dynamic.beforeStep();api.mj_step(model,data);dynamic.afterStep();
  const telemetry=dynamic.telemetry();
  for(const j of [...telemetry.joints,...telemetry.torso]){
    assert(Number.isFinite(j.actual)&&Number.isFinite(j.torque));assert(Math.abs(j.torque)<=j.effortLimit+1e-8);
    if(t<1)maxStaticError=Math.max(maxStaticError,Math.abs(j.error));else if(t<5)maxMovingError=Math.max(maxMovingError,Math.abs(j.error));
    maxTorque=Math.max(maxTorque,Math.abs(j.torque));
  }
}
console.log(JSON.stringify({nq:model.nq,nv:model.nv,nu:model.nu,articulatedJoints:dynamic.map.size,massError,inertiaError,fkPositionError,fkAngleError,maxStaticError,maxMovingError,maxTorque,final:dynamic.telemetry().rms}));
assert(maxStaticError<1e-5,`Static gravity compensation error: ${maxStaticError}`);
assert(maxMovingError<.003,`Smooth trajectory tracking: ${maxMovingError}`);
// The existing visible preparation/crouch pose is substantially larger than
// the small tracking probe above; exercise it with original effort limits.
const spread={torso_joint1:0,torso_joint2:.06,torso_joint3:0,torso_joint4:0},crouch={torso_joint1:1.45,torso_joint2:-2.35,torso_joint3:-.8,torso_joint4:.3};
for(const arm of ['right','left']){const s=arm==='right'?1:-1;[.64*s,1.04,-1.35,.1*s,.3*s,.02*s].forEach((v,i)=>spread[`${arm}_arm_joint${i+1}`]=v);}
Object.assign(crouch,spread,{torso_joint1:1.45,torso_joint2:-2.35,torso_joint3:-.8,torso_joint4:.3});
dynamic.reset(idle);let crouchError=0,crouchFinalError=0,crouchTorque=0;
const segments=[[idle,spread,1.4],[spread,crouch,1.5],[crouch,crouch,1],[crouch,idle,2.6],[idle,idle,.5]];
for(const [from,to,duration] of segments)for(let step=0;step<duration/.001;step++){
  const t=Math.min(1,(step+1)*.001/duration),s=t*t*(3-2*t),pose={};for(const name of Object.keys(to))pose[name]=(from[name]??0)+((to[name]??0)-(from[name]??0))*s;
  dynamic.setTargets(pose);dynamic.beforeStep();api.mj_step(model,data);dynamic.afterStep();
  const telemetry=dynamic.telemetry();for(const j of [...telemetry.joints,...telemetry.torso]){crouchError=Math.max(crouchError,Math.abs(j.error));crouchTorque=Math.max(crouchTorque,Math.abs(j.torque));}crouchFinalError=telemetry.rms;
}
console.log(JSON.stringify({crouchError,crouchTorque,crouchFinalError}));
assert(crouchError<.01,`Full preparation/crouch tracking ${crouchError}`);assert(crouchFinalError<1e-4);
data.delete();model.delete();
