import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {performance} from 'node:perf_hooks';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {ROBOT_CONFIGS,createRobotRuntimeFromParts} from '../dist/multi/robot-runtime.mjs';
import {loadTestRobot as loadR1} from './r1-test-robot.mjs';
import {loadG1TestRobot as loadG1} from './g1-test-robot.mjs';
import {loadC5TestRobot as loadC5} from './c5-test-robot.mjs';

// Current production versus an explicitly test-only view-cache experiment.
// This does not edit shipping files, change step order, skip a world, coarsen
// 1 ms physics or treat a mesh-free fixture as a browser/GPU benchmark.
const root=new URL('../',import.meta.url),args=process.argv.slice(2);
const reportPath=args.find(arg=>arg.startsWith('--report='))?.slice(9)??'work/validation/performance-v2/rerun/runtime-before-and-experiment.json';
const ticks=Number(args.find(arg=>arg.startsWith('--ticks='))?.slice(8)??32000);
assert(Number.isSafeInteger(ticks)&&ticks>=1000);
const ids=['r1','g1','c5'],loaders={r1:loadR1,g1:loadG1,c5:loadC5};
const sourcePaths=[
  'work/test-multi-runtime-perf-v2.mjs','work/test-multi-runtime-performance.mjs',
  'dist/multi/robot-runtime.mjs','dist/multi/r1-safe-dynamics.mjs','dist/multi/pipe-controller.mjs',
  'dist/multi/planning.mjs','dist/multi/background-planning.mjs','dist/multi/planning-serialization.mjs',
  ...ids.flatMap(id=>[`work/${id}-test-robot.mjs`,`dist/${id}/grasp-controller.mjs`,`dist/${id}/grasp-kinematics.mjs`,`dist/${id}/${id}-dynamics.mjs`,`dist/${id}/models/${id}-dynamic-scene.xml`]),
  'dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf','dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs',
  'dist/r1/mujoco-runtime.mjs','dist/vendor/three.module.min.js','dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',
].sort();
const hash=buffer=>createHash('sha256').update(buffer).digest('hex');
const hashes=async()=>Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,hash(await readFile(new URL(path,root)))])));
const sourceHashes=await hashes(),startedAt=new Date().toISOString(),started=performance.now();
const api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
const runtimes=[],pairs=[],measurements=[],checks=[];let failure=null;
async function create(id){
  const sourceXml=await readFile(new URL(ROBOT_CONFIGS[id].xml),'utf8'),vfs=new api.MjVFS();let model,data;
  try{model=api.MjModel.from_xml_string(sourceXml,vfs);data=new api.MjData(model);const runtime=createRobotRuntimeFromParts({id,api,model,data,sourceXml,robot:await loaders[id]()});runtimes.push(runtime);return runtime;}
  catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
}
function countGetters(data,fn){
  const counts={},restores=[];
  for(const name of ['ctrl','qfrc_bias','qpos','qvel','qfrc_actuator','xpos','xquat']){
    let owner=data,descriptor;while(owner&&!(descriptor=Object.getOwnPropertyDescriptor(owner,name)))owner=Object.getPrototypeOf(owner);
    if(!descriptor?.get)continue;
    const own=Object.getOwnPropertyDescriptor(data,name),get=descriptor.get;counts[name]=0;
    Object.defineProperty(data,name,{configurable:true,get(){counts[name]++;return get.call(data);}});
    restores.push(()=>own?Object.defineProperty(data,name,own):delete data[name]);
  }
  try{fn();return counts;}finally{for(const restore of restores)restore();}
}
function compare(a,b){
  assert.equal(a.data.time,b.data.time);assert.equal(a.controller.state,b.controller.state);
  for(const key of ['qpos','qvel','ctrl'])assert.deepEqual([...a.data[key]],[...b.data[key]],`${a.id}: ${key} trajectory changed`);
  for(const key of ['targets','previous','targetVelocity','fingerDesiredTargets','fingerRampedTargets','pipeFingerDesired','pipeFingerRamped'])assert.deepEqual(a.dynamics[key],b.dynamics[key],`${a.id}: ${key} command state changed`);
}
function cachedBeforeStep(dynamics){
  const original=dynamics.beforeStep;
  dynamics.beforeStep=function(...args){
    const source=this.data;
    // The production beforeStep chain only reads these two MuJoCo views.
    // Keeping them within this invocation is safe across every subsequent
    // Wasm allocation, physical step and model rebuild. Object swapping adds
    // overhead compared with a future direct local-const implementation.
    this.data={ctrl:source.ctrl,qfrc_bias:source.qfrc_bias};
    try{return original.apply(this,args);}finally{this.data=source;}
  };
}
function timed(fn,iterations){const begin=performance.now();for(let n=0;n<iterations;n++)fn();return performance.now()-begin;}
try{
  for(const id of ids){
    const baseline=await create(id),candidate=await create(id);cachedBeforeStep(candidate.dynamics);
    for(let tick=0;tick<300;tick++){baseline.step(.001);candidate.step(.001);compare(baseline,candidate);}
    const originalCalls=countGetters(baseline.data,()=>baseline.dynamics.beforeStep(.001));
    const cachedCalls=countGetters(candidate.data,()=>candidate.dynamics.beforeStep(.001));compare(baseline,candidate);
    assert.equal(cachedCalls.ctrl,1);assert.equal(cachedCalls.qfrc_bias,1);
    assert.equal(baseline.start({objectId:'blue'}),true,baseline.controller.message);assert.equal(candidate.start({objectId:'blue'}),true,candidate.controller.message);compare(baseline,candidate);
    let canonicalNodes=0,displayNodes=0;baseline.robot.traverse(()=>canonicalNodes++);baseline.visualRobot.traverse(()=>displayNodes++);
    const entry={id,ticks,originalCalls,cachedCalls,canonicalNodes,displayNodes,baselineStepMs:0,candidateStepMs:0};pairs.push({baseline,candidate,entry});
  }
  for(let tick=0;tick<ticks;tick++)for(const {baseline,candidate,entry}of pairs){
    const order=tick%2?[[candidate,'candidateStepMs'],[baseline,'baselineStepMs']]:[[baseline,'baselineStepMs'],[candidate,'candidateStepMs']];
    for(const [runtime,key]of order){const before=performance.now();runtime.step(.001);entry[key]+=performance.now()-before;}
    compare(baseline,candidate);
  }
  for(const {baseline,candidate,entry}of pairs){
    assert.deepEqual(baseline.controller.history,candidate.controller.history);assert.deepEqual(baseline.controller.evidence,candidate.controller.evidence);
    if(ticks>=32000){assert.equal(baseline.controller.state,'complete');assert(baseline.controller.evidence.dual);assert(baseline.controller.evidence.lift>.035);}
    const operations={};
    for(const [name,fn,iterations]of [
      ['syncVisuals',()=>baseline.syncVisuals(),2000],['snapshot',()=>baseline.snapshot,1000],['status',()=>baseline.status,1000],
      ['telemetry',()=>baseline.dynamics.telemetry(baseline.arm),1000],['canonicalSync',()=>baseline.dynamics.syncRobot(),2000],
    ])operations[name]={iterations,totalMs:timed(fn,iterations),gettersPerCall:countGetters(baseline.data,fn)};
    checks.push(`${baseline.id}: all qpos/qvel/ctrl plus complete motor command state equal after every 1 ms tick`);
    measurements.push({...entry,reductionPercent:100*(1-entry.candidateStepMs/entry.baselineStepMs),state:baseline.controller.state,evidence:baseline.controller.evidence,operations});
  }
}catch(error){failure={message:error.message,stack:error.stack};}
finally{for(const runtime of runtimes)runtime.dispose();}
const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes);
const totals=measurements.reduce((s,row)=>({baselineStepMs:s.baselineStepMs+row.baselineStepMs,candidateStepMs:s.candidateStepMs+row.candidateStepMs}),{baselineStepMs:0,candidateStepMs:0});
const report={pass:!failure&&sourceFrozen,startedAt,finishedAt:new Date().toISOString(),wallSeconds:(performance.now()-started)/1000,
  sourceFrozen,sourceHashes,endSourceHashes,checks,measurements,totals:{...totals,reductionPercent:100*(1-totals.candidateStepMs/totals.baselineStepMs)},failure,
  experiment:'Test-only beforeStep data.ctrl/qfrc_bias view cache. Shipping production source is unmodified. No 1 ms physical state, motor command trajectory, safety gate or execution order is changed.',
  limitations:'Mesh-free original-source URDF trees and genuine production MuJoCo. Node CPU method measurements are not browser FPS, GPU, graphics-mesh, touchscreen, hardware-robot or all-device evidence. Performance.now per-tick measurement and strict assertions add test overhead outside timed production steps.'};
const destination=new URL(reportPath,root);await mkdir(new URL('.',destination),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));if(!report.pass)process.exitCode=1;
