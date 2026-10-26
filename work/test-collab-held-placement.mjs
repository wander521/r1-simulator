import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import * as THREE from '../dist/vendor/three.module.min.js';
import {createPlanningWorkerRobot} from '../dist/multi/planning-worker-core.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {CROUCH,SPREAD} from '../dist/r1/grasp-controller.mjs';
import {solvePose,tcpPose,naturalArmLimits} from '../dist/r1/grasp-kinematics.mjs';
import {buildHeldPlacementPlan} from '../dist/collab/held-placement.mjs';

const source=await readFile(new URL('../dist/r1/assets/r1.urdf',import.meta.url),'utf8');
function fixture(){
  const robot=createPlanningWorkerRobot(source,{rootHeight:ROBOT_CONFIGS.r1.rootHeight});
  const base={...CROUCH,torso_joint4:.3,...SPREAD.left,...SPREAD.right};
  const orientation=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),.35).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),-Math.PI/2)).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),Math.PI/9));
  robot.setJointValues(base);robot.updateMatrixWorld(true);
  const names=Array.from({length:6},(_,i)=>`right_arm_joint${i+1}`),seed=Object.fromEntries(names.map((n,i)=>[n,[-.3,2.7,-1.7,-1.8,-.8,.9][i]]));
  const result=solvePose(robot,'right',new THREE.Vector3(.73,.816,.10),orientation,seed,350,{limits:naturalArmLimits('right')});
  assert.ok(result.positionError<.003&&result.angleError<.035);
  const actualPose={...base,...result.pose};robot.setJointValues(actualPose);robot.updateMatrixWorld(true);
  const tcp=tcpPose(robot,'right'),objectPose={position:tcp.position.clone().add(new THREE.Vector3(.002,-.003,.001)),quaternion:new THREE.Quaternion().setFromEuler(new THREE.Euler(.02,.4,-.01))};
  return {controller:{robot,arm:'right',planData:{naturalSelection:{wristPitchLimit:1.35}}},actualPose,objectPose,drop:objectPose.position.clone().add(new THREE.Vector3(0,-.08,0)),clearance:.08};
}
test('loaded placement solves measured offset and levels the same carrier without moving torso',()=>{
  const input=fixture(),plan=buildHeldPlacementPlan(input),g=plan.graspTransform;
  assert.ok(g.position.length()>.001,'fixture must have a nonzero grasp offset');
  const before=JSON.stringify(input.actualPose);
  for(const phase of ['transfer','place','retreat'])for(const pose of plan[phase]){
    for(const [n,v]of Object.entries(input.actualPose))if(!n.startsWith('right_arm_'))assert.equal(pose[n],v);
    for(const [n,bound]of Object.entries(naturalArmLimits('right')))assert.ok(pose[n]>=bound.lower&&pose[n]<=bound.upper);
  }
  input.controller.robot.setJointValues(plan.place.at(-1));input.controller.robot.updateMatrixWorld(true);
  const tcp=tcpPose(input.controller.robot,'right'),p=tcp.position.clone().add(g.position.clone().applyQuaternion(tcp.quaternion)),q=tcp.quaternion.clone().multiply(g.quaternion);
  assert.ok(p.distanceTo(input.drop)<.003);assert.ok(q.angleTo(plan.releaseObjectQuaternion)<.035);
  assert.ok(new THREE.Vector3(0,1,0).applyQuaternion(plan.releaseObjectQuaternion).distanceTo(new THREE.Vector3(0,1,0))<1e-9);
  assert.equal(JSON.stringify(input.actualPose),before);
});
test('loaded placement rejects off-centre payloads and invalid clearance',()=>{
  const input=fixture();input.objectPose.position.x+=.10;
  assert.throws(()=>buildHeldPlacementPlan(input),/centred/);
  assert.throws(()=>buildHeldPlacementPlan({...input,clearance:NaN}),/clearance/);
});
