import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import {parsePlanningRobot} from '../dist/multi/planning-robot.mjs';
import {enablePlanningWorkerIK} from '../dist/multi/planning-ik-context.mjs';
import {PlanningBudgetError,isPlanningBudgetError,planningBudgetCheckpoint,withPlanningBudget} from '../dist/multi/planning-budget.mjs';
import {planningControllerClass,PLANNING_SEGMENT_BUDGET_MS,G1_PIPE_PLANNING_SEGMENT_BUDGET_MS} from '../dist/multi/planning.mjs';
import {pipeControllerClass} from '../dist/multi/pipe-controller.mjs';
import * as g1Current from '../dist/g1/grasp-kinematics.mjs';
import * as r1Current from '../dist/r1/grasp-kinematics.mjs';
import * as c5Current from '../dist/c5/grasp-kinematics.mjs';
import {STAND_POSE,ROOT_HEIGHT as G1_HEIGHT,armJointNames as g1Names} from '../dist/g1/g1-profile.mjs';
import {ROOT_HEIGHT as C5_HEIGHT,HOME_POSE,armJointNames as c5Names} from '../dist/c5/c5-profile.mjs';
import {REST_TORSO,REST} from '../dist/r1/grasp-controller.mjs';

const root=new URL('../',import.meta.url),output=process.argv.find(value=>value.startsWith('--report='))?.slice(9)??'work/validation/performance-v4/planning-ik-budget-contracts.json';
const hash=value=>createHash('sha256').update(typeof value==='string'||ArrayBuffer.isView(value)?value:JSON.stringify(value)).digest('hex');
const paths=['work/test-planning-ik-budget-perf-v3.mjs','dist/multi/planning-budget.mjs','dist/multi/planning-ik-context.mjs','dist/multi/planning.mjs','dist/multi/pipe-controller.mjs','dist/multi/planning-robot.mjs',
  ...['r1','g1','c5'].flatMap(id=>[`dist/${id}/grasp-kinematics.mjs`,`dist/${id}/grasp-controller.mjs`,`work/fixtures/performance-v3/${id}-kinematics-3949cdc.txt`]),
  'dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf','dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs','dist/vendor/three.module.min.js','dist/vendor/three.core.min.js'];
const sources=async()=>Object.fromEntries(await Promise.all(paths.map(async path=>[path,hash(await readFile(new URL(path,root)))])));
const sourceHashes=await sources(),checks=[];
const check=(name,run)=>{const detail=run();checks.push({name,pass:true,...(detail?{detail}:{})});};
const absoluteSource=(source,id)=>source.replace(/\bfrom\s*(['"])([^'"]+)\1/g,(all,quote,path)=>`from ${quote}${new URL(path,new URL(`dist/${id}/grasp-kinematics.mjs`,root)).href}${quote}`);
const frozenHashes={r1:'806811345db1cdf43567542d943984c2f5a1557c537d1c65c44240612d563778',g1:'9e3ebf66e4a865a2f8e95500d4df44c80433cdd2100e28e7769873b57ea379a1',c5:'afcb166c8a3468465a0377c6cd6d841cd580acfd64c0bbb917d1cc5fbec3cc4b'};
async function baseline(id){const source=await readFile(new URL(`work/fixtures/performance-v3/${id}-kinematics-3949cdc.txt`,root),'utf8');assert.equal(hash(source),frozenHashes[id],`Frozen 3949cdc reference changed: ${id}`);return import(`data:text/javascript;base64,${Buffer.from(absoluteSource(source,id)).toString('base64')}`);}
const original={r1:await baseline('r1'),g1:await baseline('g1'),c5:await baseline('c5')};
const urdfs={r1:'dist/r1/assets/r1.urdf',g1:'dist/g1/assets/g1_29dof_with_dex1_1.urdf',c5:'dist/c5/assets/c5-dex1.urdf'};
const robots={};for(const [id,path]of Object.entries(urdfs))robots[id]=await readFile(new URL(path,root),'utf8');
const make=(id,optimized=false)=>{const robot=parsePlanningRobot(robots[id],{rootHeight:id==='g1'?G1_HEIGHT:id==='c5'?C5_HEIGHT:0,reuseUnchangedJoints:true});return optimized?enablePlanningWorkerIK(robot):robot;};
const traceRobot=robot=>{
  const poses=[],frames=[],set=robot.setJointValues,update=robot.updateMatrixWorld;
  robot.setJointValues=function(pose){poses.push(Object.entries(pose));return set.call(this,pose);};
  robot.updateMatrixWorld=function(...args){const result=update.apply(this,args);frames.push(Object.values(this.joints).flatMap(joint=>joint.matrixWorld.elements));return result;};
  return{poses,frames};
};
for(const id of ['r1','g1','c5'])check(`${id}: standalone default and Worker opt-in preserve every solver pose/FK/result against frozen 3949cdc source`,()=>{
  const modules={r1:r1Current,g1:g1Current,c5:c5Current},arms=id==='c5'?['left']:['left','right'];let comparisons=0;
  for(const arm of arms)for(const offset of [0,.06,-.04]){
    const seed=id==='g1'?{...STAND_POSE,...Object.fromEntries(g1Names(arm).map((name,index)=>[name,[0,arm==='left'?.3:-.3,.1,-.35,0,.1,0][index]]))}:id==='c5'?{...HOME_POSE}:{...REST_TORSO,...REST.left,...REST.right};
    const target=new THREE.Vector3(id==='r1'?.74:.32,.94+offset,(arm==='left'?-.1:.1)+offset),quaternion=original[id].topDownQuaternion(.15+offset);
    const runs=[];for(const mode of ['baseline','default','worker']){
      const robot=make(id,mode==='worker'),trace=traceRobot(robot);robot.setJointValues(seed);robot.updateMatrixWorld(true);
      const result=(mode==='baseline'?original[id]:modules[id]).solvePose(robot,arm,target,quaternion,seed,32);
      runs.push({mode,result,trace});
    }
    for(const row of runs.slice(1)){assert.deepEqual(row.result,runs[0].result);assert.deepEqual(row.trace,runs[0].trace);comparisons++;}
  }
  return{comparisons,frozenSourceCommit:'3949cdc3c156221f3db6268f6191b603ec546245'};
});
check('Worker node context is explicit, idempotent, non-enumerable, and does not alter default robot serialization',()=>{
  const robot=make('g1'),keys=Object.keys(robot);assert.equal(robot[Symbol.for('r1-simulator.worker-ik-context.v1')],undefined);
  assert.equal(enablePlanningWorkerIK(robot),robot);const first=robot[Symbol.for('r1-simulator.worker-ik-context.v1')];assert.equal(enablePlanningWorkerIK(robot),robot);assert.equal(robot[Symbol.for('r1-simulator.worker-ik-context.v1')],first);assert.deepEqual(Object.keys(robot),keys);
});
check('Worker FK node reads fall back correctly after a direct joint mutation and match original nonuniform-scale transforms',()=>{
  const plain=make('g1'),worker=make('g1',true);
  for(const robot of [plain,worker]){robot.scale.set(1.2,.8,1.1);robot.setJointValues(STAND_POSE);robot.updateMatrixWorld(true);}
  const read=robot=>{const link=robot.getObjectByName('right_dex1_base_link');return{position:link.getWorldPosition(new THREE.Vector3()).toArray(),quaternion:link.getWorldQuaternion(new THREE.Quaternion()).toArray(),point:link.localToWorld(new THREE.Vector3(.12,0,0)).toArray()};};
  assert.deepEqual(read(worker),read(plain));
  for(const robot of [plain,worker])robot.joints.right_shoulder_pitch_joint.setValue(-.4);
  assert.deepEqual(read(worker),read(plain));
});
check('Budget scopes restore nested callbacks and collision methods after success and typed failure',()=>{
  const robot={},dynamics={checkPose:()=>({safe:true})},originalCheck=dynamics.checkPose;let outer=0,inner=0;
  withPlanningBudget(robot,()=>outer++,()=>{
    planningBudgetCheckpoint(robot);withPlanningBudget(robot,()=>inner++,()=>dynamics.checkPose({}),dynamics);planningBudgetCheckpoint(robot);
  },dynamics);
  assert.equal(outer,5);assert.equal(inner,3);assert.equal(dynamics.checkPose,originalCheck);assert.equal(robot[Symbol.for('r1-simulator.planning-budget.v1')],undefined);
  assert.throws(()=>withPlanningBudget(robot,()=>{throw new PlanningBudgetError({budgetMs:10});},()=>{},dynamics),isPlanningBudgetError);
  assert.equal(dynamics.checkPose,originalCheck);assert.equal(robot[Symbol.for('r1-simulator.planning-budget.v1')],undefined);
});
check('Budget installation failures restore exact own descriptors, prototype methods, and previous hooks',()=>{
  const key=Symbol.for('r1-simulator.planning-budget.v1'),robot={},old=()=>{};
  Object.defineProperty(robot,key,{configurable:true,enumerable:false,writable:true,value:old});const oldDescriptor=Object.getOwnPropertyDescriptor(robot,key);
  for(const kind of ['prototype','readonly','setter-throws','getter-throws']){
    const originalCheck=()=>({safe:true}),prototype={checkPose:originalCheck},dynamics=kind==='prototype'?Object.create(prototype):{};
    if(kind==='readonly')Object.defineProperty(dynamics,'checkPose',{configurable:false,writable:false,value:originalCheck});
    if(kind==='setter-throws')Object.defineProperty(dynamics,'checkPose',{configurable:true,enumerable:true,get:()=>originalCheck,set(){throw new Error('setter failure');}});
    if(kind==='getter-throws')Object.defineProperty(dynamics,'checkPose',{configurable:true,get(){throw new Error('getter failure');}});
    const before=Object.getOwnPropertyDescriptor(dynamics,'checkPose');
    if(kind==='prototype')withPlanningBudget(robot,()=>{},()=>dynamics.checkPose(),dynamics);
    else assert.throws(()=>withPlanningBudget(robot,()=>{},()=>{},dynamics));
    assert.deepEqual(Object.getOwnPropertyDescriptor(dynamics,'checkPose'),before);assert.deepEqual(Object.getOwnPropertyDescriptor(robot,key),oldDescriptor);
  }
  const sealed={};Object.defineProperty(sealed,key,{configurable:false,value:old});assert.throws(()=>withPlanningBudget(sealed,()=>{},()=>{}));assert.equal(sealed[key],old);
});
for(const id of ['r1','g1','c5'])check(`${id}: active budget interrupts within a solver, then removes all scoped hooks`,()=>{
  const robot=make(id,true),arm=id==='c5'?'left':'right',seed=id==='g1'?{...STAND_POSE}:id==='c5'?{...HOME_POSE}:{...REST_TORSO,...REST.left,...REST.right};
  const solver={r1:r1Current,g1:g1Current,c5:c5Current}[id];let checks=0,evaluations=0;const update=robot.updateMatrixWorld;
  robot.updateMatrixWorld=function(...args){evaluations++;return update.apply(this,args);};
  assert.throws(()=>withPlanningBudget(robot,()=>{if(++checks===3)throw new PlanningBudgetError({budgetMs:10000});},()=>solver.solvePose(robot,arm,new THREE.Vector3(10,10,10),new THREE.Quaternion(),seed,400)),isPlanningBudgetError);
  assert.equal(checks,3);assert(evaluations>0&&evaluations<100,'Checkpoint should interrupt inside the 400-iteration solve');assert.equal(robot[Symbol.for('r1-simulator.planning-budget.v1')],undefined);
  return{evaluations,checkpoints:checks};
});
for(const robotId of ['r1','g1','c5'])check(`${robotId}: production candidate catches preserve budget type and restore pose without physics stepping`,()=>{
  let time=0,physicsSteps=0,candidates=0,restores=0;const previous=Object.getOwnPropertyDescriptor(globalThis,'performance');
  Object.defineProperty(globalThis,'performance',{configurable:true,value:{now:()=>time}});
  class Legacy{
    constructor(){this.robot=make(robotId,true);this.pose=robotId==='g1'?{...STAND_POSE}:robotId==='c5'?{...HOME_POSE}:{...REST_TORSO,...REST.left,...REST.right};this.history=[];this.dynamics={syncRobot:()=>restores++};this.cooperativePlanning=true;
      const position=new THREE.Vector3(robotId==='r1'?.74:robotId==='c5'?.48:.32,robotId==='r1'?.718:.94,.1);this.targets=[{id:'blue',drop:position.clone()}];this.objectPose=()=>({position:position.clone(),quaternion:new THREE.Quaternion()});}
    *planCandidates(){try{for(let i=0;i<100;i++){candidates++;time+=10001;planningBudgetCheckpoint(this.robot);yield;}}finally{this.generatorClosed=true;}}
    waistAssistAngles(){return[0];}
    placementEndpoints(target){return{start:this.objectPose().position,drop:target.drop.clone()};}
    applyPose(pose){this.pose={...pose};}step(){physicsSteps++;}
  }
  try{
    const Controller=planningControllerClass(Legacy,{robotId}),controller=new Controller(),before={...controller.pose};
    assert.equal(controller.prepareItem(robotId==='c5'?'left':'right','blue',false),true,controller.message);
    const result=controller.advancePlanning();assert.equal(result.error.type,'budget');assert.equal(result.done,true);assert.equal(result.ok,false);assert.equal(candidates,1);assert.equal(physicsSteps,0);assert(controller.generatorClosed);assert.deepEqual(controller.pose,before);assert.equal(controller.planning,false);assert.equal(controller.robot[Symbol.for('r1-simulator.planning-budget.v1')],undefined);assert(restores>0);
  }finally{Object.defineProperty(globalThis,'performance',previous);}
  assert.equal(PLANNING_SEGMENT_BUDGET_MS,10000);assert.equal(G1_PIPE_PLANNING_SEGMENT_BUDGET_MS,30000);
});
check('Budget expiry cannot be disguised by a legacy catch that wraps the typed error',()=>{
  let time=0;const previous=Object.getOwnPropertyDescriptor(globalThis,'performance');Object.defineProperty(globalThis,'performance',{configurable:true,value:{now:()=>time}});
  class Legacy{constructor(){this.robot={};this.pose={};this.dynamics={syncRobot(){}};}applyPose(pose){this.pose=pose;}}
  try{
    const Controller=planningControllerClass(Legacy,{robotId:'g1'}),controller=new Controller();
    controller._planningJob={before:{},budgetMs:10000,segmentElapsedMs:0,steps:0,iterator:(function*(){try{time=10001;planningBudgetCheckpoint(controller.robot);}catch{throw new Error('legacy opening failure');}yield;})()};
    const result=controller.advancePlanning();assert.equal(result.error.type,'budget');assert.match(result.reason,/时限/);
  }finally{Object.defineProperty(globalThis,'performance',previous);}
});
check('Budget expiry cannot be hidden by a legacy catch that returns normally',()=>{
  const robot={},error=new PlanningBudgetError({budgetMs:30000});let expired=false;
  assert.throws(()=>withPlanningBudget(robot,()=>{if(expired)throw error;},()=>{expired=true;try{planningBudgetCheckpoint(robot);}catch{}return{accepted:true};}),caught=>caught===error);
  assert.equal(robot[Symbol.for('r1-simulator.planning-budget.v1')],undefined);
});
check('G1 pipe tilt/direction candidate catches never convert a typed inner-IK budget error into another yielded candidate',()=>{
  const target={id:'pipe-yellow-1',shape:'cylinder',start:new THREE.Vector3(.32,.926,.1),drop:new THREE.Vector3(.5,.926,.055),halfX:.02,halfZ:.02,h:.026};
  class Legacy{
    constructor(){this.robot=make('g1',true);this.targets=[target];this.arm='right';this.pose={...STAND_POSE};this.bodies=new Map([[target.id,1]]);this.dynamics={checkPose:()=>({safe:true})};}
    objectPose(){return{position:target.start.clone(),quaternion:new THREE.Quaternion()};}
  }
  const Controller=pipeControllerClass(Legacy,{robotId:'g1'}),controller=new Controller(),iterator=controller.planCandidates(target,0,0,.14);let checks=0,yielded=false;
  assert.throws(()=>withPlanningBudget(controller.robot,()=>{if(++checks===3)throw new PlanningBudgetError({budgetMs:30000});},()=>{const next=iterator.next();yielded=true;return next;},controller.dynamics),isPlanningBudgetError);
  assert.equal(yielded,false);assert.equal(checks,3);assert.equal(controller.robot[Symbol.for('r1-simulator.planning-budget.v1')],undefined);
});
const endSourceHashes=await sources(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes);
assert(sourceFrozen);const report={pass:true,releaseCohort:'20261018-perf-4',total:checks.length,passed:checks.length,checks,sourceFrozen,sourceHashes,endSourceHashes,scope:'Frozen 3949cdc solvers versus default/current and explicit Worker FK scratch. Exact every commanded pose/FK/error output; all three solver budgets and cleanup with declared clocks. No MuJoCo stepping, native browser responsiveness or physical grasp claimed.'};
await mkdir(new URL('.',new URL(output,root)),{recursive:true});await writeFile(new URL(output,root),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({pass:true,total:checks.length,report:output}));
