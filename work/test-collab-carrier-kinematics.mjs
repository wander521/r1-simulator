import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createPlanningWorkerRobot} from '../dist/multi/planning-worker-core.mjs';
import {solveCarrierPose} from '../dist/collab/carrier-kinematics.mjs';
import {STAND_POSE,armJointNames} from '../dist/g1/g1-profile.mjs';
import {tcpPose,naturalArmLimits,naturalArmPose} from '../dist/g1/grasp-kinematics.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';

const source=await readFile(new URL('../dist/g1/assets/g1_29dof_with_dex1_1.urdf',import.meta.url),'utf8');
for(const side of ['left','right'])test(`carrier IK ${side}: reachable FK target, fixed feet, original natural limits`,()=>{
  const robot=createPlanningWorkerRobot(source,{rootHeight:ROBOT_CONFIGS.g1.rootHeight}),names=armJointNames(side),s=side==='left'?1:-1;
  const seed={...STAND_POSE,...Object.fromEntries(names.map((name,i)=>[name,naturalArmPose(side)[i]]))};
  const targetPose={...seed,waist_yaw_joint:.15*s,[names[0]]:-.18,[names[2]]:.2*s,[names[3]]:-.48};
  robot.setJointValues(targetPose);robot.updateMatrixWorld(true);const target=tcpPose(robot,side);
  const result=solveCarrierPose(robot,side,target.position,target.quaternion,seed,320);
  assert.ok(result.positionError<.003,`position error ${result.positionError}`);
  assert.ok(result.angleError<.035,`orientation error ${result.angleError}`);
  for(const [i,name]of names.entries()){const [lo,hi]=naturalArmLimits(side)[i];assert.ok(result.pose[name]>=lo&&result.pose[name]<=hi);}
  assert.ok(Math.abs(result.pose.waist_yaw_joint)<=1);
  for(const [name,value]of Object.entries(STAND_POSE))if(name!=='waist_yaw_joint')assert.equal(result.pose[name],value,`${name} must not move`);
});
test('carrier IK retains natural limits when solving from a saturated waist',()=>{
  const robot=createPlanningWorkerRobot(source,{rootHeight:ROBOT_CONFIGS.g1.rootHeight}),names=armJointNames('right');
  const seed={...STAND_POSE,waist_yaw_joint:.999,...Object.fromEntries(names.map((name,i)=>[name,[-.7,-.28,-.23,1.28,.21,-.71,.29][i]]))};
  const targetPose={...seed,[names[0]]:-.83,[names[3]]:1.43,[names[5]]:-.81};
  robot.setJointValues(targetPose);robot.updateMatrixWorld(true);const target=tcpPose(robot,'right');
  const result=solveCarrierPose(robot,'right',target.position,target.quaternion,seed,320);
  assert.ok(result.positionError<.003);assert.ok(result.angleError<.035);assert.ok(Math.abs(result.pose.waist_yaw_joint)<=.999000001);
  for(const [i,name]of names.entries()){const [lo,hi]=naturalArmLimits('right')[i];assert.ok(result.pose[name]>=lo&&result.pose[name]<=hi);}
  for(const [name,value]of Object.entries(STAND_POSE))if(name!=='waist_yaw_joint')assert.equal(result.pose[name],value);
});
