import * as THREE from '../vendor/three.module.min.js';

export const VERSION='20261023-collab-1';
export const IDS=Object.freeze(['r1','g1','c5']);
export const LABELS=Object.freeze({r1:'星海图 R1',g1:'宇树 G1',c5:'遨博 C5'});
export const KIND_LABELS=Object.freeze({valve:'阀体',pin:'销轴',sleeve:'轴套'});
export const toThree=p=>[p[0],p[2],-p[1]];
const basis=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),Math.PI/2);
const inverse=basis.clone().invert();
export function bodyPose(data,body){
  return {position:toThree(data.xpos.subarray(body*3,body*3+3)),
    quaternion:inverse.clone().multiply(new THREE.Quaternion(data.xquat[body*4+1],data.xquat[body*4+2],data.xquat[body*4+3],data.xquat[body*4])).multiply(basis).toArray()};
}
export function robotFrame(meta,workbench){
  return {prefix:meta.prefix,position:toThree(meta.origin),yaw:meta.yaw??0,mountName:meta.mountName,mobile:meta.mobile,mobility:meta.mobility,floorGeom:'cell_floor',
    entryShoulderRoll:meta.entryShoulderRoll,readyShoulderPitch:meta.readyShoulderPitch,operationReturn:meta.operationReturn,binFlow:meta.binFlow,
    aliases:{task_table:workbench.body,floor:'cell_floor',robot_plinth:'c5_pedestal_body'}};
}
export const CONTROL_FIELDS=Object.freeze(['state','running','time','elapsed','tick','arm','selected','openings','opening','pose','evidence','history','task','itemIndex',
  'stackEvidence','stackStableSeconds','stackResultVerified','reaction','reactionFinished','observerYaw','reducedMotion','crouch','from','phaseGoal','continuing','manualMotion','placeLowering','placePose','sharedAtReady']);
export const MOTOR_FIELDS=Object.freeze(['targets','previous','targetVelocity','fingerDesiredTargets','fingerRampedTargets']);
export function plainFields(object,fields){return Object.fromEntries(fields.filter(key=>object[key]!==undefined).map(key=>[key,structuredClone(object[key]) ]));}
export function physicalSnapshot(data){return {time:data.time,qpos:Array.from(data.qpos),qvel:Array.from(data.qvel),ctrl:Array.from(data.ctrl)};}
export function restorePrivateSnapshot(api,model,data,snapshot){
  if(!Number.isFinite(snapshot?.time))throw new Error('无效的共享物理快照');
  for(const name of ['qpos','qvel','ctrl']){
    if(snapshot[name]?.length!==data[name].length||!snapshot[name].every(Number.isFinite))throw new Error(`共享模型快照维度不符：${name}`);
    data[name].set(snapshot[name]);
  }
  data.time=snapshot.time;api.mj_forward(model,data);
}
