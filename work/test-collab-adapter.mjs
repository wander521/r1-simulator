import assert from 'node:assert/strict';
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {buildCollaborationModel} from '../dist/collab/model.mjs';
import {createRigidFrame,createNamespacedAPI,createRobotAgent,validatePresetRoute,SharedCollisionChecker} from '../dist/collab/robot-adapter.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {planningControllerClass} from '../dist/multi/planning.mjs';
import {loadTestRobot as r1} from './r1-test-robot.mjs';
import {loadG1TestRobot as g1} from './g1-test-robot.mjs';
import {loadC5TestRobot as c5} from './c5-test-robot.mjs';

const root=new URL('../',import.meta.url),started=performance.now(),checks=[],check=(name,detail={})=>checks.push({name,pass:true,...detail});
const paths=['work/test-collab-adapter.mjs','dist/collab/robot-adapter.mjs','dist/collab/task-plans.mjs','dist/collab/model.mjs','dist/multi/robot-runtime.mjs','dist/multi/r1-safe-dynamics.mjs','dist/multi/planning.mjs',...['r1','g1','c5'].flatMap(id=>[`dist/${id}/grasp-controller.mjs`,`dist/${id}/grasp-kinematics.mjs`,`dist/${id}/${id}-dynamics.mjs`,`dist/${id}/models/${id}-dynamic-scene.xml`,`work/${id}-test-robot.mjs`]),'dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf'];
const hashes=async()=>Object.fromEntries(await Promise.all(paths.map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')]))),sourceHashes=await hashes();
for(let i=0;i<30;i++){
  const f=createRigidFrame({position:[i*.03,.208,-.5],yaw:i*.19}),p=new THREE.Vector3(.78,.72,i*.01),q=new THREE.Quaternion().setFromEuler(new THREE.Euler(.1,i*.03,.2));
  assert(f.toLocal(f.toWorld(p)).distanceTo(p)<1e-12);assert(f.rotationToLocal(f.rotationToWorld(q)).angleTo(q)<1e-7);
}check('Rigid frames round-trip canonical/global position and rotation', {cases:30});
assert.throws(()=>createRigidFrame({position:[NaN,0,0]}));check('Invalid frame rejected');
const fakeData={},calls=[],fakeApi={mj_name2id:(m,t,name)=>{calls.push(name);return name==='r1__torso_joint1'?7:8;},mj_id2name:()=> 'r1__torso_joint1',mj_resetData:()=>calls.push('reset')};
const ns=createNamespacedAPI(fakeApi,{model:{},liveData:fakeData,prefix:'r1__',aliases:{task_table:'tray__order_1'}});
assert.equal(ns.api.mj_name2id({},0,'torso_joint1'),7);assert.equal(calls.at(-1),'r1__torso_joint1');
ns.api.mj_name2id({},0,'task_table');assert.equal(calls.at(-1),'tray__order_1');assert.equal(ns.api.mj_id2name({},0,7),'torso_joint1');check('Namespaced API returns global IDs with explicit shared aliases');
assert.throws(()=>ns.api.mj_resetData({},fakeData));ns.withInitialization(()=>ns.api.mj_resetData({},fakeData));assert.equal(ns.stats.suppressedInitializationResets,1);assert(!calls.includes('reset'));check('Robot initialization never resets shared live data');
assert.throws(()=>ns.api.mj_step({},fakeData));ns.withControl(()=>ns.api.mj_step({},fakeData));assert.equal(ns.stats.controlStepRequests,1);check('Per-robot physics stepping is gated and never advances world');

const ids=['r1','g1','c5'],loaders={r1,g1,c5},sources=Object.fromEntries(await Promise.all(ids.map(async id=>[id,await readFile(new URL(ROBOT_CONFIGS[id].xml),'utf8')])));
const built=buildCollaborationModel(sources,{orderCount:1}),api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname}),vfs=new api.MjVFS();
let model,data;const agents=[];
try{
  model=api.MjModel.from_xml_string(built.xml,vfs);data=new api.MjData(model);api.mj_forward(model,data);
  for(const id of ids){
    const before=Array.from(data.qpos),otherIndices=agents.flatMap(agent=>[...agent.dynamics.map.values()].map(j=>j.q));
    const agent=createRobotAgent({api,model,data,robot:await loaders[id](),id,metadata:built.metadata.robots[id],targets:[],Controller:planningControllerClass(ROBOT_CONFIGS[id].Controller,{robotId:id})});
    for(const q of otherIndices)assert.equal(data.qpos[q],before[q]);
    assert.equal(agent.controller.model,model);assert.equal(agent.controller.data,data);assert.equal(agent.dynamics.data,data);assert.equal(agent.namespace.stats.suppressedInitializationResets,1);
    agents.push(agent);
  }check('All controllers/dynamics use ONE model and live MjData; later constructors preserve peer qpos',{robots:3,nq:model.nq,nv:model.nv});
  const allQ=agents.flatMap(agent=>[...agent.dynamics.map.values()].map(j=>j.q));assert.equal(new Set(allQ).size,allQ.length);check('Native joint maps resolve disjoint addresses in shared qpos');
  api.mj_forward(model,data);
  const part=built.metadata.parts[0],worldStart=new THREE.Vector3(part.start[0],part.start[2],-part.start[1]);
  for(const agent of agents){
    const drop=worldStart.clone().add(new THREE.Vector3(.03,.008,0));
    agent.configureTask({target:{...part,start:worldStart,drop},supportBody:'shared_workbench'});
    const read=agent.controller.objectPose(part.id),end=agent.controller.placementEndpoints(agent.controller.targets[0]);
    assert(agent.toWorld(read.position).distanceTo(worldStart)<1e-9);assert(agent.toWorld(end.drop).distanceTo(drop)<1e-9);
    assert(Math.abs(end.drop.y-end.start.y-.008)<1e-10);assert(agent.controller.measure().worldPosition.distanceTo(worldStart)<1e-9);
  }check('All robots read the same shared object through canonical frames and preserve real drop height');
  const state=()=>JSON.stringify({qpos:Array.from(data.qpos),qvel:Array.from(data.qvel),ctrl:Array.from(data.ctrl),time:data.time});
  for(const agent of agents){
    const before=state();assert.throws(()=>agent.controller.reset());assert.throws(()=>agent.dynamics.reset());assert.throws(()=>agent.controller.setObject(part.id,worldStart));assert.equal(state(),before);
    const result=agent.dynamics.checkPose(agent.controller.pose,{objectPoses:[{body:-1,position:[0,0,0],quaternion:[0,0,0,1]}]});assert.equal(result.safe,false);assert(result.limitViolations.some(v=>v.type==='invalid-object'));assert.equal(state(),before);
  }check('Reset/object teleport forbidden outside owner initialization; invalid mirror object rejected without live mutation');
  const start=data.time;
  for(let i=0;i<100;i++){
    const before=data.time;for(const agent of agents)agent.stepControls(.001);assert.equal(data.time,before);
    api.mj_step(model,data);for(const agent of agents)agent.afterWorldStep();assert(Math.abs(data.time-before-.001)<1e-12);
  }
  assert(Math.abs(data.time-start-.1)<1e-12);for(const agent of agents)assert.equal(agent.namespace.stats.controlStepRequests,100);
  assert([...data.qpos,...data.qvel,...data.ctrl].every(Number.isFinite));check('300 controller ticks make exactly 100 physical steps; all shared physical state remains finite',{physicalSeconds:data.time-start});
  for(const agent of agents){
    const before=state(),joint=[...agent.dynamics.map].find(([,j])=>!j.finger),result=agent.dynamics.checkPose({...agent.controller.pose,[joint[0]]:joint[1].upper+.1});
    assert.equal(result.safe,false);assert(result.limitViolations.some(v=>v.name===joint[0]));assert.equal(state(),before);
  }check('Full-model scratch safety rejects source joint-range violation without changing live state');
  const mobile=agents.find(a=>a.id==='r1'),baseX=api.mj_name2id(model,api.mjtObj.mjOBJ_JOINT.value,'r1__base_slide_x'),baseYaw=api.mj_name2id(model,api.mjtObj.mjOBJ_JOINT.value,'r1__base_yaw');
  const originalQ=Array.from(data.qpos),originalV=Array.from(data.qvel),originalCtrl=Array.from(data.ctrl),originalTime=data.time;
  try{
    const initial=mobile.syncFrame(),armState=[...mobile.dynamics.map.values()].map(j=>data.qpos[j.q]);
    // Explicit adversarial fixture placement, not a runtime navigation method.
    data.qpos[model.jnt_qposadr[baseX]]=.20;data.qpos[model.jnt_qposadr[baseYaw]]=.04;api.mj_forward(model,data);
    const before=state(),moved=mobile.syncFrame();assert(moved.version>initial.version);assert(Math.abs(moved.position[0]-.20)<1e-9);
    assert(mobile.toWorld(mobile.controller.objectPose(part.id).position).distanceTo(new THREE.Vector3(data.xpos[mobile.controller.bodies.get(part.id)*3],data.xpos[mobile.controller.bodies.get(part.id)*3+2],-data.xpos[mobile.controller.bodies.get(part.id)*3+1]))<1e-9);
    assert.equal(state(),before);assert.deepEqual([...mobile.dynamics.map.values()].map(j=>data.qpos[j.q]),armState);
    mobile.setMobilityInterlock({moving:true});assert.throws(()=>mobile.configureTask({target:{...part,start:worldStart,drop:worldStart}}));assert.throws(()=>mobile.controller.start('right',part.id));assert.throws(()=>mobile.controller.prepareItem('right',part.id,false));assert.throws(()=>mobile.controller.plan(mobile.controller.targets[0]));
    mobile.setMobilityInterlock({moving:false});mobile.configureTask({target:{...part,start:worldStart,drop:worldStart.clone().add(new THREE.Vector3(.02,.008,0))}});
    assert(mobile.toWorld(mobile.controller.targets[0].drop).distanceTo(worldStart.clone().add(new THREE.Vector3(.02,.008,0)))<1e-9);
    assert.equal(state(),before);
    check('Tracked frame follows actual translated/yawed mount, leaves arm state intact and rejects planning during navigation');
  }finally{data.qpos.set(originalQ);data.qvel.set(originalV);data.ctrl.set(originalCtrl);data.time=originalTime;api.mj_forward(model,data);}
  const mirror=new SharedCollisionChecker({api,model,data,jointMap:new Map(),robotBodyIds:[],frame:mobile.frame});
  try{
    const before=state(),body=mobile.controller.bodies.get(part.id),point=[.80,.718,.03],joints={'r1__base_slide_x':.20,'r1__base_slide_y':.02,'r1__base_yaw':.04};
    // This isolated contract checks coordinate prediction only, not navigation
    // collision clearance (the production checker retains all robot IDs).
    mirror.check({}, {baseJointPositions:joints,objectPoses:[{body,position:point,quaternion:[0,0,0,1]}]});
    const origin=built.metadata.robots.r1.origin,expectedFrame=createRigidFrame({position:[origin[0]+.20,origin[2],-(origin[1]+.02)],yaw:built.metadata.robots.r1.yaw+.04});
    const actual=new THREE.Vector3(mirror.scratch.xpos[body*3],mirror.scratch.xpos[body*3+2],-mirror.scratch.xpos[body*3+1]);
    assert(actual.distanceTo(expectedFrame.toWorld(point))<1e-10);assert(actual.distanceTo(mobile.toWorld(point))>.15);assert.equal(state(),before);
    check('Predicted held object uses candidate scratch mount transform, not the different live mount frame');
  }finally{mirror.dispose();}
  const baseJoints=Object.fromEntries(['r1__base_slide_x','r1__base_slide_y','r1__base_yaw'].map(name=>{const j=api.mj_name2id(model,api.mjtObj.mjOBJ_JOINT.value,name);return[name,data.qpos[model.jnt_qposadr[j]]];}));
  const navBefore=state(),nav=mobile.validateMobileRoute({fromMount:{jointPositions:baseJoints},toMount:{jointPositions:{...baseJoints,r1__base_slide_x:.04}}});
  assert.equal(nav.safe,true,JSON.stringify(nav));assert(nav.samples>=3);assert.equal(state(),navBefore);check('Private full-scene navigation samples actual mount poses within 2 cm / .03 rad without live writes',{samples:nav.samples});
  const blocked=mobile.validateMobileRoute({fromMount:{jointPositions:baseJoints},toMount:{jointPositions:{...baseJoints,r1__base_slide_x:.02}},selectedBody:mobile.controller.bodies.get(part.id),objectLocalPose:{position:[.80,.65,0],quaternion:[0,0,0,1]}});
  assert.equal(blocked.safe,false);assert(blocked.contacts.some(c=>c.body1==='feed_workbench'||c.body2==='feed_workbench'),JSON.stringify(blocked));assert.equal(state(),navBefore);check('Carried-object/table interference rejects mobile route; chassis/platform allowance does not hide it');
  const g=agents.find(a=>a.id==='g1'),table=api.mj_name2id(model,api.mjtObj.mjOBJ_BODY.value,'g1_workbench'),oldTablePosition=Array.from(model.body_pos.subarray(table*3,table*3+3)),savedPlan=g.controller.planData;
  try{
    // Move the complete real support as an adversarial fixture. Its unchanged
    // primitive geometry and broadphase bounds intersect the preset sweep.
    model.body_pos[table*3]-=.08;model.body_pos[table*3+1]+=.30;api.mj_forward(model,data);g.controller.planData={startYaw:0,preOpening:g.controller.opening,approach:{...g.controller.pose}};
    let rejected;try{validatePresetRoute(g.controller,['spread','raise'],{opening:g.controller.openings});}catch(error){rejected=error;}
    assert(rejected,'Preset path unexpectedly accepted a real tabletop obstacle');assert.equal(rejected.detail.stage,'preset-route');assert(['spread','raise'].includes(rejected.detail.phase));assert(rejected.detail.contacts.some(c=>c.body1==='g1_workbench'||c.body2==='g1_workbench'),JSON.stringify(rejected.detail));
    check('Real tabletop obstruction in preset spread/raise is rejected before arm execution',{phase:rejected.detail.phase,contacts:rejected.detail.contacts});
    const plan=g.controller.plan;let candidates=0;g.controller.plan=()=>{candidates++;throw new Error('Unexpected candidate search');};
    try{g.controller.cooperativePlanning=true;assert.equal(g.controller.prepareItem('left',part.id,false),false);assert.equal(candidates,0);assert.equal(g.controller.planning,false);assert.equal(g.controller.planningResult.error.type,'collision');assert.equal(g.controller.planningResult.error.detail.stage,'preset-route');check('Candidate-independent entry collision fails once before allocating or searching IK candidates',{candidates});}finally{g.controller.plan=plan;}
  }finally{model.body_pos.set(oldTablePosition,table*3);g.controller.planData=savedPlan;api.mj_forward(model,data);}
  const pair=[];
  try{
    for(const deferRobotSync of[false,true]){
      const d=new api.MjData(model),a=[];pair.push({data:d,agents:a,counts:{}});
      for(const id of ids){const agent=createRobotAgent({api,model,data:d,robot:await loaders[id](),id,metadata:built.metadata.robots[id],deferRobotSync});agent.configureTask({target:{...part,start:worldStart,drop:worldStart}});a.push(agent);
        const sync=agent.dynamics.syncRobot.bind(agent.dynamics);pair.at(-1).counts[id]=0;agent.dynamics.syncRobot=()=>{pair.at(deferRobotSync?1:0).counts[id]++;return sync();};
      }
      const controller=a.find(agent=>agent.id==='g1').controller;controller.planData={startYaw:0,preOpening:controller.opening,approach:{...controller.pose}};controller.running=true;controller.transition('spread');
    }
    for(let tick=0;tick<1600;tick++){
      for(const world of pair){for(const agent of world.agents)agent.stepControls(.001);api.mj_step(model,world.data);if((tick+1)%10===0)for(const agent of world.agents)agent.afterWorldStep();}
      for(const key of['qpos','qvel','ctrl','qfrc_actuator'])assert.deepEqual(Array.from(pair[0].data[key]),Array.from(pair[1].data[key]),`FK throttling changed ${key} at tick ${tick}`);
      for(let i=0;i<3;i++){assert.equal(pair[0].agents[i].controller.state,pair[1].agents[i].controller.state);assert.deepEqual(pair[0].agents[i].controller.history,pair[1].agents[i].controller.history);}
    }
    for(const id of ids)assert(pair[1].counts[id]<pair[0].counts[id]);
    for(const world of pair){const c=world.agents.find(a=>a.id==='g1').controller;assert(c.history.includes('spread'));assert(c.history.includes('raise'),JSON.stringify({state:c.state,message:c.message,error:c.planError}));assert.equal(c.running,true);}
    check('100 Hz FK synchronization preserves exact 1 kHz qpos/qvel/ctrl/torque and preset transition history',{ticks:1600,physicalSeconds:pair[0].data.time,baselineSyncCalls:pair[0].counts,deferredSyncCalls:pair[1].counts});
  }finally{for(const world of pair){for(const agent of world.agents)agent.dispose();world.data.delete();}}
  const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes);assert(sourceFrozen,'Adapter sources changed while testing');
  const report={pass:true,total:checks.length,passed:checks.length,wallSeconds:(performance.now()-started)/1000,scope:'Actual shared MuJoCo model and original mesh-free URDF joint trees. Frame/reset/step/scratch and paired-control contracts only; this is not a successful grasp or five-minute runtime claim.',sourceFrozen,sourceHashes,endSourceHashes,checks};
  const destination=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9);if(destination){const path=new URL(destination,root);await mkdir(new URL('./',path),{recursive:true});await writeFile(path,JSON.stringify(report,null,2)+'\n');}
  console.log(JSON.stringify(report,null,2));
}finally{for(const agent of agents)agent.dispose();data?.delete();model?.delete();vfs.delete();}
