import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {GraspController,REST} from '../dist/g1/grasp-controller.mjs';
import {G1Dynamics} from '../dist/g1/g1-dynamics.mjs';
import {tcpPose} from '../dist/g1/grasp-kinematics.mjs';
import {STAND_POSE,TABLE_HEIGHT,OBJECT_START_X,OBJECT_DROP_X,OBJECT_LATERAL_OFFSET,armJointNames,fingerJointNames,openingToQ,qToOpening,GRIPPER_Q_MIN,GRIPPER_Q_MAX} from '../dist/g1/g1-profile.mjs';
import {loadG1TestRobot} from './g1-test-robot.mjs';

const root=new URL('../',import.meta.url),OPEN=qToOpening(GRIPPER_Q_MAX),CLOSED=qToOpening(GRIPPER_Q_MIN);
const targets=[{id:'red',body:'red_cube',h:.0225,z:OBJECT_LATERAL_OFFSET},{id:'blue',body:'blue_block',h:.019,z:0},{id:'yellow',body:'yellow_cylinder',h:.026,z:-OBJECT_LATERAL_OFFSET}].map(t=>({...t,start:new THREE.Vector3(OBJECT_START_X,TABLE_HEIGHT+t.h,t.z),drop:new THREE.Vector3(OBJECT_DROP_X,TABLE_HEIGHT+t.h,t.z),rotation:new THREE.Quaternion()}));
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const vfs=new api.MjVFS(),model=api.MjModel.from_xml_string(await readFile(new URL('dist/g1/models/g1-dynamic-scene.xml',root),'utf8'),vfs),data=new api.MjData(model),robot=await loadG1TestRobot();vfs.delete();
const dynamics=new G1Dynamics(api,model,data,robot),c=new GraspController(api,model,data,robot,targets,dynamics),results=[];
// This fixture is an independently found source-mesh counterexample: the
// closed gripper clears the table, but opening it causes an 11.79 mm collision.
const tablePose={...STAND_POSE,...REST.left,...REST.right,...Object.fromEntries(armJointNames('right').map((name,i)=>[name,[.4488732945173979,-.5865767343435436,-.0199489851482213,-.03498584872577337,.07753818593919282,-.35161968008615074,-.934218670707196][i]]))};
const gaps=()=>({left:c.openings.left,right:c.openings.right});
const actual=()=>({qpos:[...data.qpos],qvel:[...data.qvel],ctrl:[...data.ctrl],xpos:[...data.xpos],xquat:[...data.xquat],time:data.time});
const command=()=>({pose:{...c.pose},openings:gaps(),targets:{...dynamics.targets}});
function unchanged(before,label){assert.deepEqual(actual(),before.actual,`${label}: real simulation state must not be rewritten`);assert.deepEqual(command(),before.command,`${label}: rejected operation must preserve all motion commands`);}
const snapshot=()=>({actual:actual(),command:command()});
function tableFixture(){
  c.reset();assert(c.selectArm('right'));assert(c.setManualOpening(CLOSED));
  c.pose={...tablePose};
  // Explicit initial-state setup only. No test operation clamps actual qpos;
  // the production adapter and mj_step own the state after this reset.
  dynamics.reset({...tablePose,...Object.fromEntries(['left','right'].flatMap(side=>fingerJointNames(side).map(name=>[name,side==='right'?GRIPPER_Q_MIN:GRIPPER_Q_MAX])))});
  assert.deepEqual(gaps(),{left:OPEN,right:CLOSED});
}
function record(name,detail={}){results.push({name,pass:true,...detail});}
try{
  for(const name of ['selectArm','setManualOpening'])assert.equal(typeof c[name],'function',`Missing checked controller API ${name}`);
  assert(c.openings&&Number.isFinite(c.openings.left)&&Number.isFinite(c.openings.right),'Each arm needs an independent commanded opening');

  tableFixture();
  const closed=dynamics.checkPose(tablePose,{opening:gaps()}),opened=dynamics.checkPose(tablePose,{opening:{left:OPEN,right:OPEN}});
  assert(closed.safe,'Counterexample must initially be collision-free');
  const collisions=opened.contacts.filter(contact=>contact.dist<0&&[contact.body1,contact.body2].includes('task_table')&&[contact.body1,contact.body2].includes('right_dex1_finger_link_2'));
  assert(!opened.safe&&collisions.length,'Counterexample must detect finger/table contact');
  assert(Math.min(...collisions.map(contact=>contact.dist))<-.011,'Counterexample must reproduce more than 11 mm penetration');
  record('source-mesh-opening-counterexample',{closedSafe:closed.safe,openSafe:opened.safe,penetration:Math.max(...collisions.map(contact=>-contact.dist))});

  let before=snapshot();
  assert.equal(c.setManualOpening(OPEN),false,'Manual expansion into the table must be rejected');unchanged(before,'manual opening rejection');
  assert.match(c.message,/拒绝|干涉|碰撞|安全/,'Rejected manual motion must explain the problem');
  record('manual-opening-rejected-without-state-or-command-write');

  tableFixture();before=snapshot();
  assert(c.selectArm('left'));assert.equal(c.arm,'left');unchanged(before,'switch to left arm');
  assert.equal(c.opening,OPEN,'Active opening readout must reflect the selected arm');
  assert(c.selectArm('right'));assert.equal(c.opening,CLOSED);unchanged(before,'switch back to right arm');
  assert(c.selectArm('left'));c.step();
  for(const side of ['left','right'])for(const finger of c.fingers[side])assert.equal(data.ctrl[finger.actuator],openingToQ(side==='right'?CLOSED:OPEN),'Physics control must preserve the inactive arm opening');
  assert.deepEqual(gaps(),before.command.openings);
  assert(dynamics.checkPose(tablePose,{opening:gaps()}).safe);
  record('arm-switch-keeps-both-openings-through-a-physics-step');

  tableFixture();before=snapshot();
  assert.equal(c.start('right','blue'),false,'Automatic preparation must reject unsafe initial expansion');
  assert.equal(c.running,false);assert.equal(c.state,'failed');unchanged(before,'failed preparation');
  record('failed-automatic-preparation-preserves-safe-gaps');

  tableFixture();c.running=true;c.state='transfer';before=snapshot();c.stop();
  assert.equal(c.running,false);assert.equal(c.state,'stopped');unchanged(before,'unsafe stop expansion');
  record('stop-keeps-safe-gap-when-release-would-hit-table');

  tableFixture();c.running=true;c.state='approach';c.task='putaway';before=snapshot();c.fail('opening-safety failure fixture');
  assert.equal(c.running,false);assert.equal(c.state,'failed');assert.match(c.message,/opening-safety failure fixture/);unchanged(before,'unsafe failure expansion');
  record('failure-keeps-safe-gap-when-release-would-hit-table');

  tableFixture();c.running=true;c.reaction='happy';c.reactionCenter={...tablePose};c.reactionSteps=[{opening:OPEN,hold:.4,timeout:1}];c.reactionIndex=0;before=snapshot();c.transition('happy');
  assert.equal(c.running,false,'Rejected reaction opening must stop the motion');assert.equal(c.state,'failed');unchanged(before,'unsafe reaction expansion');
  record('reaction-opening-is-checked-before-phase-start');

  c.reset();assert(c.selectArm('right'));before=actual();
  assert(c.setManualOpening(CLOSED),'Normal idle closure must remain available');assert.equal(c.openings.right,CLOSED);assert.equal(c.openings.left,OPEN);assert.deepEqual(actual(),before,'Setting an accepted target must not teleport the actual fingers');
  for(let n=0;n<1500;n++)c.step();
  assert(c.fingers.right.every(finger=>Math.abs(data.qpos[finger.q]-GRIPPER_Q_MIN)<.001),'Real motor-driven fingers must close');
  before=actual();assert(c.setManualOpening(OPEN),'Normal idle opening must remain available');assert.deepEqual(actual(),before,'Opening target must not teleport actual fingers');
  for(let n=0;n<1500;n++)c.step();
  assert(c.fingers.right.every(finger=>Math.abs(data.qpos[finger.q]-GRIPPER_Q_MAX)<.001),'Real motor-driven fingers must open');
  record('idle-open-close-is-accepted-and-physically-driven');

  c.reset();const targetPose=tcpPose(robot,'right');assert(c.setObject('blue',targetPose.position));
  assert(c.setManualOpening(CLOSED),'Intended selected-object fingertip contact must remain legal');
  record('selected-object-fingertip-contact-remains-legal');

  const sourceHashes=Object.fromEntries(await Promise.all(['work/test-g1-opening-safety.mjs','dist/g1/grasp-controller.mjs','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs','dist/g1/models/g1-dynamic-scene.xml'].map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));
  console.log(JSON.stringify({pass:true,cases:results.length,results,sourceHashes}));
}finally{dynamics.dispose();data.delete();model.delete();}
