import assert from 'node:assert/strict';
import {readFile,readdir,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {MultiRobotCoordinator} from '../dist/multi/coordinator.mjs';
import {compilePlanningRuntime} from '../dist/multi/planning-worker-core.mjs';
import {ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {generatePipeLayout} from '../dist/multi/pipe-layout.mjs';
import {buildPipeScene} from '../dist/multi/pipe-model.mjs';
import {cacheRobotMeshes} from '../dist/multi/mesh-cache.mjs';
import {createShadowStateTracker} from '../dist/multi/shadow-state.mjs';

const root=new URL('../',import.meta.url),checks=[],paths=['work/test-performance-v4.mjs','dist/multi/app.mjs'];
for(const dir of ['dist/multi','dist/multi/visuals'])for(const name of await readdir(new URL(dir+'/',root)))if(name.endsWith('.mjs'))paths.push(dir+'/'+name);
for(const id of ['r1','g1','c5'])paths.push(`dist/${id}/${id}-app.mjs`,`dist/${id}/index.html`,`dist/${id}/grasp-controller.mjs`,`dist/${id}/grasp-kinematics.mjs`,`dist/${id}/${id}-dynamics.mjs`);
for(const name of ['deploy-pages','validate-g1','validate-c5'])paths.push(`.github/workflows/${name}.yml`);
const hashes=async()=>Object.fromEntries(await Promise.all([...new Set(paths)].sort().map(async p=>[p,createHash('sha256').update(await readFile(new URL(p,root))).digest('hex')])));
const sourceHashes=await hashes(),start=performance.now();
async function check(name,run){try{const detail=await run();checks.push({name,pass:true,detail});}catch(e){checks.push({name,pass:false,error:e.message,stack:e.stack});}}
function stations(){return ['r1','g1','c5'].map(id=>({id,ready:true,backgroundPlanning:true,config:{label:id},data:{time:0},controller:{running:false,state:'idle'},planning:false,
  get snapshot(){return {time:this.data.time,state:this.controller.state,running:this.controller.running};},
  start(){this.controller.running=true;this.planning=true;return true;},advancePlanning(){if(this.answer){this.planning=false;this.controller.state='approach';return this.answer;}return {pending:true};},
  step(dt){this.data.time+=dt;},stop(){this.planning=false;this.controller.running=false;this.controller.state='stopped';},reset(){this.data.time=0;this.controller.state='idle';}}));}
await check('Initial start remains atomic while later planning freezes only its own world',()=>{
  const s=stations(),c=new MultiRobotCoordinator({stations:s,independentPlanning:true});assert(c.start().preparing);
  s[0].answer={done:true,ok:true};c.advancePlanning();assert(c.blockingPlanning);assert.throws(()=>c.step());assert(s.every(s=>s.data.time===0));
  s[1].answer=s[2].answer={done:true,ok:true};c.advancePlanning();assert.equal(c.batchNumber,1);assert.equal(new Set(Object.values(c.startTimes)).size,1);
  c.step();s[1].planning=true;s[1].answer=null;assert(c.planning&&!c.blockingPlanning);
  for(let n=0;n<100;n++){c.advancePlanning();c.step();}
  assert(Math.abs(s[0].data.time-.101)<1e-12);assert.equal(s[1].data.time,.001);assert.equal(s[2].data.time,s[0].data.time);
  s[1].answer={done:true,ok:true};c.advancePlanning();c.step();assert.equal(s[1].data.time,.002);assert.equal(c.batchNumber,1);
  return {peerSteps:100,pausedTime:.001,resumedTime:.002,catchUpSteps:0};
});
await check('A later failure aborts peers and reset removes all pending ownership',()=>{
  const s=stations(),c=new MultiRobotCoordinator({stations:s,independentPlanning:true});c.start();s.forEach(s=>s.answer={done:true,ok:true});c.advancePlanning();c.step();
  s[1].planning=true;s[1].answer={done:true,ok:false,reason:'unreachable'};assert.equal(c.advancePlanning().ok,false);assert(s.every(s=>!s.planning&&!s.controller.running));
  c.reset();assert.equal(c.tick,0);assert.equal(c.pendingStart,null);assert(s.every(s=>s.data.time===0));
});
await check('A fresh task can start together after independent world clocks diverge',()=>{
  const s=stations(),c=new MultiRobotCoordinator({stations:s,independentPlanning:true});s[0].data.time=1;s[1].data.time=2;s[2].data.time=3;
  c.start();s.forEach(s=>s.answer={done:true,ok:true});c.advancePlanning();assert.deepEqual(c.startTimes,{r1:1,g1:2,c5:3});assert.equal(c.startTick,0);
});
await check('Repeated STL assets parse once with independent nodes and material ownership',async()=>{
  const manager=new THREE.LoadingManager(),callbacks=[];let loads=0;
  const loader={loadMeshCb(path,manager,done){loads++;queueMicrotask(()=>done(new THREE.Mesh(new THREE.BoxGeometry(),new THREE.MeshPhongMaterial())));}};
  const stats=cacheRobotMeshes(loader);
  const objects=await Promise.all(Array.from({length:12},()=>new Promise((resolve,reject)=>loader.loadMeshCb('finger.stl',manager,(o,e)=>e?reject(e):resolve(o)))));
  assert.equal(loads,1);assert.equal(stats.hits,11);assert.equal(new Set(objects).size,12);assert(objects.every(o=>o.geometry===objects[0].geometry));
  assert.equal(new Set(objects.map(o=>o.material)).size,12);objects[0].material.color.set('red');assert(!objects[1].material.color.equals(objects[0].material.color));
  objects[0].geometry.dispose();objects.forEach(o=>o.material.dispose());return stats;
});
await check('A failed mesh request is evicted and a retry can load successfully',async()=>{
  const manager=new THREE.LoadingManager();manager.onError=()=>{};let calls=0;
  const loader={loadMeshCb(path,m,done){calls++;queueMicrotask(()=>done(calls===1?null:new THREE.Mesh(new THREE.BoxGeometry(),new THREE.MeshPhongMaterial()),calls===1?new Error('load failed'):null));}};
  cacheRobotMeshes(loader);const load=()=>new Promise((resolve,reject)=>loader.loadMeshCb('retry.stl',manager,(o,e)=>e?reject(e):resolve(o)));
  await assert.rejects(load(),/load failed/);const mesh=await load();assert.equal(calls,2);mesh.geometry.dispose();mesh.material.dispose();
});
await check('Shadow invalidation skips unchanged state and accumulates small motion',()=>{
  const cache=createShadowStateTracker();assert(cache.changed('r1',[0,1]));assert(!cache.changed('r1',[0,1]));assert(!cache.changed('r1',[.6e-6,1]));
  assert(cache.changed('r1',[1.2e-6,1]));assert(!cache.changed('r1',[1.2e-6,1]));assert(cache.changed('r1',[1.2e-6,1,0]));
  assert.throws(()=>cache.changed('r1',[NaN]));cache.clear();assert(cache.changed('r1',[1.2e-6,1,0]));
});
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
for(const id of ['r1','g1','c5'])await check(`${id}: first pipe mirror compiles exactly once and retains original topology after randomization`,async()=>{
  const config=ROBOT_CONFIGS[id],sourceXml=await readFile(new URL(config.xml),'utf8'),source=await readFile(new URL(config.urdf),'utf8');
  const modelXml=buildPipeScene(sourceXml,generatePipeLayout(config,{seed:521}));let compiles=0;
  const counted=Object.create(api);Object.defineProperty(counted,'MjModel',{value:{from_xml_string(...args){compiles++;return api.MjModel.from_xml_string(...args);}}});
  const runtime=compilePlanningRuntime({robotId:id,sourceXml,modelXml,taskMode:'pipes',modelPipeSeed:521},{api:counted,source});
  try{assert.equal(compiles,1);assert.equal(runtime.modelXml,modelXml);assert.equal(runtime.modelPipeSeed,521);assert.equal(runtime.targets.length,8);
    const joints=Array.from(runtime.model.jnt_qposadr),bodies=runtime.targets.map(t=>[t.id,runtime.controller.bodies.get(t.id)]);
    runtime.reset({seed:592});assert.equal(compiles,1);assert.deepEqual([...runtime.model.jnt_qposadr],joints);assert.deepEqual(runtime.targets.map(t=>[t.id,runtime.controller.bodies.get(t.id)]),bodies);
    assert.equal(runtime.controller.pipeLayout.seed,592);
    const restore=[runtime.controller.measure,runtime.controller.wristError,runtime.dynamics.telemetry];
    const fail=()=>{throw new Error('A lightweight row read requested full physics telemetry');};
    runtime.controller.measure=runtime.controller.wristError=runtime.dynamics.telemetry=fail;
    try{for(let n=0;n<1000;n++){const summary=runtime.summary;assert.equal(summary.classification.completed,0);assert.equal(summary.itemIndex,0);}}
    finally{[runtime.controller.measure,runtime.controller.wristError,runtime.dynamics.telemetry]=restore;}
    return {compiles,modelSeed:runtime.modelPipeSeed,currentSeed:592,lightweightReads:1000,fullTelemetryReads:0};
  }finally{runtime.dispose();}
});
for(const id of ['r1','g1','c5'])await check(`${id}: standalone preparation uses Worker polling and telemetry is not read every frame`,async()=>{
  const source=await readFile(new URL(`dist/${id}/${id}-app.mjs`,root),'utf8');
  assert(source.includes(`createStandalonePlanningController(GraspController,{robotId:'${id}'`));
  assert(source.includes('if(wasPlanning){controller.advancePlanning();accumulator=0;lastTime=performance.now();}'));
  assert(source.includes('while(accumulator>=.001&&!controller.planning)'));assert(source.includes('controller?.disposePlanning()'));
  assert(source.includes('if(controller&&latestTelemetry&&now-lastJointUI>100)'));assert(source.includes('shadowState.changed('));
});
await check('Participant rows read only lightweight summaries, not full physics telemetry',async()=>{
  const source=await readFile(new URL('dist/multi/app.mjs',root),'utf8'),ui=source.slice(source.indexOf('function updateUI()'),source.indexOf("$('#task-mode').addEventListener"));
  assert(ui.includes('const snapshot=station.summary'));assert(!ui.includes('station.snapshot'));assert(ui.includes('if(!focus||focus.planning)return'));
  assert(source.includes('independentPlanning:true'));assert(source.includes('coordinator?.blockingPlanning'));
  const participants=source.slice(source.indexOf("$('#robot-options').addEventListener('change'"),source.indexOf("document.querySelectorAll('[data-object]')"));
  assert.equal((participants.match(/renderer\.shadowMap\.needsUpdate=true/g)??[]).length,2,'Single and select-all visibility changes must invalidate shadows even when poses are unchanged');
});
await check('Every full-regression CI job has at least sixty minutes',async()=>{
  for(const name of ['deploy-pages','validate-g1','validate-c5']){const s=await readFile(new URL(`.github/workflows/${name}.yml`,root),'utf8');assert(/validate:\s*[\s\S]*?timeout-minutes: 60/.test(s));assert(!s.includes('timeout-minutes: 15'));}
});
const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes),pass=sourceFrozen&&checks.every(c=>c.pass);
const report={pass,releaseCohort:'20261018-perf-4',total:checks.length,passed:checks.filter(c=>c.pass).length,wallSeconds:(performance.now()-start)/1000,sourceFrozen,sourceHashes,endSourceHashes,checks};
const output=new URL('work/validation/performance-v4/remaining-contracts-final.json',root);await mkdir(new URL('.',output),{recursive:true});await writeFile(output,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass,total:report.total,passed:report.passed,sourceFrozen,failures:checks.filter(c=>!c.pass)},null,2));if(!pass)process.exitCode=1;
