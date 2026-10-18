import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {performance} from 'node:perf_hooks';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotRuntimeFromParts,ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {planningPreflight} from '../dist/multi/planning.mjs';
import {loadTestRobot as r1} from './r1-test-robot.mjs';
import {loadG1TestRobot as g1} from './g1-test-robot.mjs';
import {loadC5TestRobot as c5} from './c5-test-robot.mjs';

// An optional --report destination saves this actual run independently of the
// retained physical acceptance reports. Production IK, collision checks and
// source joint trees are used for every planning comparison.
const root=new URL('../',import.meta.url),reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9);
const startedAt=new Date().toISOString(),wallStart=performance.now();
const sourcePaths=[
  'work/test-planning-responsiveness.mjs','dist/multi/planning.mjs','dist/multi/robot-runtime.mjs',
  'dist/multi/coordinator.mjs','dist/multi/pipe-controller.mjs','dist/multi/pipe-layout.mjs',
  'dist/multi/pipe-model.mjs','dist/multi/r1-safe-dynamics.mjs','dist/multi/planning-budget.mjs','dist/multi/planning-ik-context.mjs',
  ...['r1','g1','c5'].flatMap(id=>[`work/${id}-test-robot.mjs`,`dist/${id}/grasp-controller.mjs`,
    `dist/${id}/grasp-kinematics.mjs`,`dist/${id}/${id}-dynamics.mjs`,`dist/${id}/models/${id}-dynamic-scene.xml`]),
  'dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf',
  'dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs','dist/vendor/three.module.min.js',
  'dist/r1/mujoco-runtime.mjs','dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',
].sort();
async function hashes(){return Object.fromEntries(await Promise.all(sourcePaths.map(async path=>
  [path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));}
const sourceHashes=await hashes();
let api,failure=null;
const checks=[],loaders={r1,g1,c5},runtimes=[];
async function create(id,cooperativePlanning){
  const xml=await readFile(new URL(ROBOT_CONFIGS[id].xml),'utf8'),vfs=new api.MjVFS();let model,data;
  try{
    model=api.MjModel.from_xml_string(xml,vfs);data=new api.MjData(model);
    const robot=await loaders[id](),runtime=createRobotRuntimeFromParts({id,api,model,data,robot,sourceXml:xml,cooperativePlanning});
    runtimes.push(runtime);return runtime;
  }catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
}
function run(name,fn){
  try{fn();checks.push({name,pass:true});}
  catch(error){checks.push({name,pass:false,error:error.message});throw error;}
}
function drain(runtime){
  const c=runtime.controller,initial=[...runtime.data.qpos],time=runtime.data.time,durations=[];
  let result,steps=0;
  while(c.planning){
    const started=performance.now();result=c.advancePlanning();durations.push(performance.now()-started);
    assert.equal(runtime.data.time,time,'Planning advanced the physical clock');
    assert.deepEqual([...runtime.data.qpos],initial,'Planning edited physical positions');
    assert(++steps<2000,'Candidate iterator did not terminate');
    runtime.syncVisuals(); // Render/snapshot FK reads between candidates are safe.
  }
  return {...result,durations};
}
try{
  api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
  for(const id of ['r1','g1','c5']){
    const sync=await create(id,false),cooperative=await create(id,true),c=cooperative.controller;
    // A larger comparison budget prevents hardware speed from changing this
    // equivalence assertion; production's ordinary default is ten seconds.
    c.planningBudgetMs=10000;
    run(`${id}: source planner and cooperative route produce identical plans`,()=>{
      assert.equal(sync.start({objectId:'blue'}),true);
      assert.equal(cooperative.start({objectId:'blue'}),true);assert(c.planning);
      const result=drain(cooperative);assert.equal(result.ok,true);
      assert.deepEqual(c.planData,sync.controller.planData);
      assert.deepEqual(c.history,sync.controller.history);assert.equal(c.state,sync.controller.state);
      checks.push({id,candidateSlices:result.steps,segmentElapsedMs:result.segmentElapsedMs,maxSliceMs:Math.max(...result.durations)});
    });
    run(`${id}: far start and destination reject before candidate search`,()=>{
      for(const kind of ['start','drop']){
        cooperative.reset();const target=cooperative.targets.find(item=>item.id==='blue');
        if(kind==='start')c.setObject('blue',new THREE.Vector3(2.5,cooperative.config.tableHeight+.019,0));
        else target.drop.x=2.5;
        const started=performance.now();assert.equal(cooperative.start({objectId:'blue'}),false);
        assert.equal(c.planning,false);assert.equal(c.planningResult.error.type,'unreachable');assert.equal(c.planData,null);
        assert(performance.now()-started<150,`${id}/${kind}: coarse preflight was not immediate`);
        target.drop.x=cooperative.config.dropX;
      }
    });
    run(`${id}: non-finite destination rejects without IK`,()=>{
      cooperative.reset();const target=cooperative.targets.find(item=>item.id==='blue');target.drop.x=NaN;
      assert.equal(cooperative.start({objectId:'blue'}),false);assert.equal(c.planError.type,'invalid-position');
      target.drop.x=cooperative.config.dropX;
    });
    run(`${id}: work budget produces an explicit failure and permits recovery`,()=>{
      cooperative.reset();c.planningBudgetMs=0;
      assert.equal(cooperative.start({objectId:'blue'}),true);
      const result=c.advancePlanning();assert.equal(result.ok,false);assert.equal(result.error.type,'budget');
      assert.equal(c.running,false);assert.equal(c.planning,false);assert.equal(c.planData,null);
      cooperative.reset();c.planningBudgetMs=10000;
      assert.equal(cooperative.start({objectId:'blue'}),true);assert.equal(drain(cooperative).ok,true);
    });
    sync.dispose();cooperative.dispose();runtimes.splice(runtimes.indexOf(sync),1);runtimes.splice(runtimes.indexOf(cooperative),1);
  }
  const pipeSync=await create('g1',false),pipeCooperative=await create('g1',true);
  run('G1 pipe tilt/direction candidates preserve the original accepted route',()=>{
    assert.equal(pipeSync.configureTask('pipes',{seed:521}).ok,true);
    assert.equal(pipeCooperative.configureTask('pipes',{seed:521}).ok,true);
    pipeCooperative.controller.planningBudgetMs=10000;
    assert.equal(pipeSync.start({task:'pipes'}),true);
    assert.equal(pipeCooperative.start({task:'pipes'}),true);
    const result=drain(pipeCooperative);assert.equal(result.ok,true);
    assert.deepEqual(pipeCooperative.controller.planData,pipeSync.controller.planData);
    assert.deepEqual(pipeCooperative.controller.history,pipeSync.controller.history);
    checks.push({id:'g1-pipe',candidateSlices:result.steps,segmentElapsedMs:result.segmentElapsedMs,maxSliceMs:Math.max(...result.durations)});
  });
  pipeSync.dispose();pipeCooperative.dispose();runtimes.splice(runtimes.indexOf(pipeSync),1);runtimes.splice(runtimes.indexOf(pipeCooperative),1);
  const runtime=await create('r1',true),c=runtime.controller;
  run('near unreachable R1 yields after one route, cancels, and restores actual FK',()=>{
    c.setObject('blue',new THREE.Vector3(1.08,runtime.config.tableHeight+.019,.30));
    assert.equal(runtime.start({objectId:'blue'}),true);const qpos=[...runtime.data.qpos],before=performance.now();
    const first=c.advancePlanning();assert.equal(first.pending,true);assert.equal(first.steps,1);
    assert(performance.now()-before<500,'One R1 route monopolized the event loop');
    runtime.syncVisuals();assert(c.cancelPlanning('测试停止'));assert.equal(c.state,'stopped');assert.equal(c.planning,false);
    assert.deepEqual([...runtime.data.qpos],qpos);assert.equal(c.planData,null);
    assert.equal(c.advancePlanning().error.type,'cancelled');
    runtime.reset();c.planningBudgetMs=10000;assert.equal(runtime.start({objectId:'blue'}),true);assert.equal(drain(runtime).ok,true);
  });
  run('R1 accumulated computation segments are bounded and event-loop suspension is excluded',()=>{
    runtime.reset();c.planningBudgetMs=1;c.setObject('blue',new THREE.Vector3(1.08,runtime.config.tableHeight+.019,.30));
    assert.equal(runtime.start({objectId:'blue'}),true);
    const qpos=[...runtime.data.qpos],time=runtime.data.time;
    const result=c.advancePlanning();assert.equal(result.pending,false);assert.equal(result.done,true);
    assert.equal(result.error.type,'budget');assert.equal(result.steps,0);assert(result.segmentElapsedMs>=1);
    assert.deepEqual([...runtime.data.qpos],qpos);assert.equal(runtime.data.time,time);
    assert.equal(c.advancePlanning().error.type,'budget');
    assert.equal(c.planData,null);assert.equal(c.running,false);
  });
  runtime.reset();c.planningBudgetMs=10000;assert.equal(runtime.start({objectId:'blue'}),true);
  const first=c.advancePlanning();assert(first.pending);
  const candidateElapsed=first.segmentElapsedMs;
  await new Promise(resolve=>setTimeout(resolve,40));
  run('scheduled waiting does not consume the computation segment allowance',()=>{
    assert.equal(c._planningJob.segmentElapsedMs,candidateElapsed);c.cancelPlanning();
  });
  run('conservative reach sphere admits source workspace corners without claiming feasibility',()=>{
    runtime.reset();const target=runtime.targets.find(item=>item.id==='blue'),original=target.drop.clone();
    for(const [x,z]of [[.60,-.25],[.60,.25],[1.05,-.25],[1.05,.25]]){
      c.setObject('blue',new THREE.Vector3(x,runtime.config.tableHeight+.019,z));
      assert.doesNotThrow(()=>planningPreflight(c,{robotId:'r1',arm:'right',target}));
    }
    target.drop.copy(original);runtime.dynamics.syncRobot();
  });
  runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);
}catch(error){failure={message:error.message,stack:error.stack};}
finally{for(const runtime of runtimes)runtime.dispose();}

const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(endSourceHashes)===JSON.stringify(sourceHashes);
const namedChecks=checks.filter(check=>typeof check.name==='string'),measurements=checks.filter(check=>typeof check.id==='string');
const pass=!failure&&sourceFrozen&&namedChecks.every(check=>check.pass===true);
const report={pass,startedAt,finishedAt:new Date().toISOString(),wallSeconds:(performance.now()-wallStart)/1000,
  checkCount:namedChecks.length,measurementCount:measurements.length,checks,failure,sourceFrozen,sourceHashes,endSourceHashes,
  scope:'Actual production MuJoCo and source URDF fixtures. Exact synchronous/cooperative plan equality, source-chain preflight, cancellation, bounded computation segments, and unchanged physical qpos/time. Slice measurements are elapsed wall durations; they do not certify browser frame rate or exact OS CPU time.',
  budgetPolicy:'Focused contract fixtures use explicit 0/1 ms abort allowances and 10000 ms equality/recovery allowances. The separate full physical regressions exercise production defaults.'};
if(reportPath){const target=new URL(reportPath,root);await mkdir(new URL('.',target),{recursive:true});await writeFile(target,JSON.stringify(report,null,2)+'\n');}
console.log(JSON.stringify(report,null,2));
if(!pass)process.exitCode=1;
