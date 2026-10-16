import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {performance} from 'node:perf_hooks';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotRuntimeFromParts,ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {loadTestRobot as r1} from './r1-test-robot.mjs';
import {loadG1TestRobot as g1} from './g1-test-robot.mjs';
import {loadC5TestRobot as c5} from './c5-test-robot.mjs';

// Real production controllers, MuJoCo scenes and source URDF joint trees.
// These fixtures intentionally omit visual meshes: these CPU measurements do
// not measure rendering, browser frame rate, GPU work or production mesh cost.
const ids=['r1','g1','c5'],loaders={r1,g1,c5},root=new URL('../',import.meta.url);
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const pairs=[],results=[],checks=[];
const tickCount=32000;
async function create(id,options={}){
  const xml=await readFile(new URL(ROBOT_CONFIGS[id].xml),'utf8'),vfs=new api.MjVFS();
  let model,data;
  try{model=api.MjModel.from_xml_string(xml,vfs);data=new api.MjData(model);
    return createRobotRuntimeFromParts({id,api,model,data,robot:await loaders[id](),sourceXml:xml,...options});
  }catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
}
const actualPose=runtime=>[...runtime.data.qpos];
function compare(a,b,step,entry){
  assert.equal(a.data.time,b.data.time,`${entry.id}: physics clock differs at ${step}`);
  assert.equal(a.controller.state,b.controller.state,`${entry.id}: phase differs at ${step}`);
  for(const field of ['qpos','qvel']){
    const av=a.data[field],bv=b.data[field];
    assert.equal(av.length,bv.length);
    for(let i=0;i<av.length;i++){
      const delta=Math.abs(av[i]-bv[i]);
      entry[field==='qpos'?'maxQposDelta':'maxQvelDelta']=Math.max(entry[field==='qpos'?'maxQposDelta':'maxQvelDelta'],delta);
      assert.equal(av[i],bv[i],`${entry.id}: ${field}[${i}] differs at ${step}`);
    }
  }
}
function countSync(runtime,fn){
  const original=runtime.dynamics.syncRobot;let calls=0;
  runtime.dynamics.syncRobot=function(...args){calls++;return original.apply(this,args);};
  try{fn();return calls;}finally{runtime.dynamics.syncRobot=original;}
}
function checkLightweightStatus(runtime){
  const replacements=[];
  for(const [object,key]of [[runtime.controller,'measure'],[runtime.controller,'wristError'],[runtime.controller,'objectPose'],[runtime.dynamics,'telemetry']]){
    const original=object[key];replacements.push(()=>object[key]=original);
    object[key]=()=>assert.fail(`status must not invoke ${key}`);
  }
  try{
    const status=runtime.status;
    assert.equal(status.id,runtime.id);assert.equal(status.time,runtime.controller.time);assert.equal(status.physicsTime,runtime.data.time);
    assert.equal(status.running,runtime.controller.running);assert.equal(status.planning,runtime.planning);
    assert.equal(status.classification?.started,runtime.taskMode==='pipes'?runtime.controller.classification.started:undefined);
  }finally{replacements.forEach(restore=>restore());}
}
try{
  for(const id of ids){
    const baseline=await create(id),deferred=await create(id);
    // Restore exactly the legacy adapter's per-tick FK synchronization for
    // the comparator. No motor, contact, solver, target or actual state edit.
    baseline.dynamics.afterStep=()=>baseline.dynamics.syncRobot();
    for(const runtime of [baseline,deferred]){
      assert.equal(runtime.controller.cooperativePlanning,false);
      assert.equal(runtime.planning,false);assert.equal(runtime.data.time,0);
      checkLightweightStatus(runtime);
    }
    const syncs=countSync(deferred,()=>{for(let i=0;i<100;i++)deferred.step(.001);});
    assert.equal(syncs,0,`${id}: idle 1 ms physics ticks still traverse the canonical robot`);
    for(let i=0;i<100;i++)baseline.step(.001);
    compare(baseline,deferred,-1,{id,maxQposDelta:0,maxQvelDelta:0});
    assert(countSync(deferred,()=>deferred.tcpWorld())>0,`${id}: TCP must read current actual FK`);
    assert(countSync(deferred,()=>deferred.snapshot)>0,`${id}: snapshot wrist error must read current actual FK`);
    let forcedDisplayTraversals=0;const original=deferred.group.updateMatrixWorld;
    deferred.group.updateMatrixWorld=function(force){if(force)forcedDisplayTraversals++;return original.call(this,force);};
    deferred.syncVisuals();deferred.group.updateMatrixWorld=original;
    assert.equal(forcedDisplayTraversals,0,'syncVisuals must leave the display traversal to renderer/world queries');
    for(const runtime of [baseline,deferred])for(let i=0;i<200;i++)runtime.step(.001);
    const planningMs=[];
    for(const runtime of [baseline,deferred]){
      const start=performance.now();assert.equal(runtime.start({objectId:'blue',arm:runtime.config.defaultArm}),true,runtime.controller.message);
      planningMs.push(performance.now()-start);
      assert.equal(runtime.planning,false,'default public start must remain synchronous');
    }
    const entry={id,ticks:tickCount,baselineStepMs:0,deferredStepMs:0,maxQposDelta:0,maxQvelDelta:0,planningMs};
    pairs.push({baseline,deferred,entry});checks.push(`${id}: deferred idle FK, explicit TCP/snapshot sync, lightweight status, synchronous default start`);
  }
  // Alternate which version executes first to reduce ordering/JIT bias. The
  // measured duration includes production step work, not comparison assertions.
  for(let step=0;step<tickCount;step++)for(const {baseline,deferred,entry}of pairs){
    const order=step%2?[['deferredStepMs',deferred],['baselineStepMs',baseline]]:[['baselineStepMs',baseline],['deferredStepMs',deferred]];
    for(const [key,runtime]of order){const start=performance.now();runtime.step(.001);entry[key]+=performance.now()-start;}
    compare(baseline,deferred,step,entry);
    if(step%2000===0){
      // Reading diagnostics/render state during a run must not perturb either
      // world or make desired FK leak into the next physics tick.
      const before=actualPose(deferred);baseline.syncVisuals();deferred.syncVisuals();
      assert.equal(baseline.snapshot.wristError,deferred.snapshot.wristError);
      assert.deepEqual(actualPose(deferred),before);
      checkLightweightStatus(deferred);
    }
  }
  for(const {baseline,deferred,entry}of pairs){
    assert.equal(baseline.controller.state,'complete');assert.equal(deferred.controller.state,'complete');
    assert.deepEqual(deferred.controller.history,baseline.controller.history);
    assert.deepEqual(deferred.controller.evidence,baseline.controller.evidence);
    assert(deferred.controller.evidence.dual);assert(deferred.controller.evidence.lift>.035);
    results.push({...entry,reductionPercent:100*(1-entry.deferredStepMs/entry.baselineStepMs),state:deferred.controller.state,
      history:[...deferred.controller.history],evidence:{...deferred.controller.evidence}});
  }
  // Task reconfiguration creates fresh dynamics/controller instances. Verify
  // that these retain deferred FK and the lightweight status contract too.
  for(const {deferred:runtime}of pairs){
    const result=runtime.configureTask('pipes',{seed:1});assert.equal(result.ok,true,result.reason);
    assert.equal(runtime.controller.cooperativePlanning,false);
    checkLightweightStatus(runtime);
    assert.equal(countSync(runtime,()=>{for(let i=0;i<20;i++)runtime.step(.001);}),0);
    checks.push(`${runtime.id}: reconfigured pipe dynamics also defer per-tick FK`);
  }
}finally{for(const {baseline,deferred}of pairs){baseline.dispose();deferred.dispose();}}
const totals=results.reduce((sum,row)=>({baselineStepMs:sum.baselineStepMs+row.baselineStepMs,deferredStepMs:sum.deferredStepMs+row.deferredStepMs}),{baselineStepMs:0,deferredStepMs:0});
console.log(JSON.stringify({pass:true,clockStepSeconds:.001,fixtures:'Mesh-free source URDF joint trees and production MuJoCo worlds; CPU step comparison only, not rendered frame rate.',
  comparison:'All qpos/qvel coordinates compared after every 1 ms tick; identical complete blue sort history/evidence per robot. Comparator restores legacy afterStep->syncRobot only.',
  checks,results,totals:{...totals,reductionPercent:100*(1-totals.deferredStepMs/totals.baselineStepMs)}}));
