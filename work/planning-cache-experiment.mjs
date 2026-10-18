import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotRuntimeFromParts,ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {parsePlanningRobot} from '../dist/multi/planning-robot.mjs';
import {decodePlanningValue,restorePlanningJob,capturePlanningResult} from '../dist/multi/planning-serialization.mjs';

// Experiment only: a bounded, single immutable planning-job memo. No production
// module is patched, no candidate is skipped, and every returned safety result
// is compared against a fresh invocation of the original dynamics implementation.
const root=new URL('../',import.meta.url),args=process.argv.slice(2);
const inputPath=args.find(value=>value.startsWith('--snapshots='))?.slice(12)??'work/validation/performance-v2/g1-pipe592-snapshots.json';
const outputPath=args.find(value=>value.startsWith('--out='))?.slice(6)??'work/validation/performance-v2/planning-cache-experiment.json';
const itemCount=Number(args.find(value=>value.startsWith('--items='))?.slice(8)??4);
const trials=Number(args.find(value=>value.startsWith('--trials='))?.slice(9)??2);
const maxEntries=Number(args.find(value=>value.startsWith('--capacity='))?.slice(11)??16384);
const hash=value=>createHash('sha256').update(typeof value==='string'||ArrayBuffer.isView(value)?value:JSON.stringify(value)).digest('hex');
const sourcePaths=['work/planning-cache-experiment.mjs','dist/multi/robot-runtime.mjs','dist/multi/pipe-controller.mjs','dist/multi/planning.mjs',
  'dist/multi/planning-robot.mjs','dist/multi/planning-serialization.mjs','dist/multi/planning-worker-core.mjs',
  'dist/g1/grasp-controller.mjs','dist/g1/grasp-kinematics.mjs','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs',
  'dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/g1/models/g1-dynamic-scene.xml','dist/vendor/three.module.min.js',
  'dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm'];
const sourceHashes=Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,hash(await readFile(new URL(path,root)))])));

// Length-delimited, type-exact encoding. Preserve object entry and array order,
// missing/undefined, signed zero and nonfinite diagnostics. The cache only uses
// supported plain, finite inputs; unusual inputs still call the original method.
export function exactKey(value,{finiteOnly=false}={}){
  const seen=new Set();
  const visit=item=>{
    if(item===null)return'n';
    if(item===undefined)return'u';
    if(typeof item==='boolean')return item?'t':'f';
    if(typeof item==='string')return`s${item.length}:${item}`;
    if(typeof item==='number'){
      if(!Number.isFinite(item)&&finiteOnly)throw new TypeError('Nonfinite cache input');
      return`d${Object.is(item,-0)?'-0':String(item)};`;
    }
    if(typeof item!=='object'||seen.has(item))throw new TypeError('Unsupported cache input');
    const prototype=Object.getPrototypeOf(item),array=Array.isArray(item);
    if(!array&&prototype!==Object.prototype&&prototype!==null)throw new TypeError('Nonplain cache input');
    seen.add(item);
    const keys=Object.keys(item),result=`${array?`a${item.length}`:prototype===null?'p':'o'}${keys.length}{${keys.map(key=>{
      const descriptor=Object.getOwnPropertyDescriptor(item,key);
      if(!descriptor||!Object.hasOwn(descriptor,'value'))throw new TypeError('Accessor cache input');
      return`${visit(key)}${visit(descriptor.value)}`;
    }).join('')}}`;
    seen.delete(item);return result;
  };
  return visit(value);
}

export function installJobMemo(dynamics,{capacity=16384,trace=null,enabled=true}={}){
  assert(Number.isInteger(capacity)&&capacity>0);
  const original=dynamics.checkPose,cache=new Map();
  const metrics={requests:0,evaluations:0,hits:0,unsafeHits:0,bypasses:0,evictions:0,peakEntries:0,peakTextBytes:0,
    keyMs:0,cloneMs:0,backendMs:0,traceMs:0,stateGuardMs:0,stateInvalidations:0,cleared:false,restored:false};let cacheBytes=0,closed=false,invalidated=false;
  const model=dynamics.model,sourceData=dynamics.data,qpos=sourceData?.qpos&&Array.from(sourceData.qpos),ctrl=sourceData?.ctrl&&Array.from(sourceData.ctrl);
  const unchanged=(actual,expected)=>actual?.length===expected.length&&expected.every((value,index)=>Object.is(value,actual[index]));
  dynamics.checkPose=function(...values){
    assert(!closed,'A completed job reused its cache');metrics.requests++;
    const guardStarted=performance.now();
    if(!invalidated&&sourceData&&(this.model!==model||this.data!==sourceData||!unchanged(this.data.qpos,qpos)||!unchanged(this.data.ctrl,ctrl))){
      invalidated=true;cache.clear();cacheBytes=0;metrics.stateInvalidations++;
    }
    metrics.stateGuardMs+=performance.now()-guardStarted;
    let key=null;const keyStarted=performance.now();
    try{key=exactKey(values,{finiteOnly:true});}catch{metrics.bypasses++;}
    metrics.keyMs+=performance.now()-keyStarted;
    let result;
    if(enabled&&!invalidated&&key!==null&&cache.has(key)){
      const row=cache.get(key);cache.delete(key);cache.set(key,row);metrics.hits++;if(!row.result.safe)metrics.unsafeHits++;
      const began=performance.now();result=structuredClone(row.result);metrics.cloneMs+=performance.now()-began;
    }else{
      const began=performance.now();result=original.apply(this,values);metrics.backendMs+=performance.now()-began;metrics.evaluations++;
      if(enabled&&!invalidated&&key!==null){
        const cloneStarted=performance.now(),stored=structuredClone(result);metrics.cloneMs+=performance.now()-cloneStarted;
        const bytes=2*(key.length+exactKey(stored).length);
        while(cache.size>=capacity){const [oldKey,row]=cache.entries().next().value;cache.delete(oldKey);cacheBytes-=row.bytes;metrics.evictions++;}
        cache.set(key,{result:stored,bytes});cacheBytes+=bytes;
        metrics.peakEntries=Math.max(metrics.peakEntries,cache.size);metrics.peakTextBytes=Math.max(metrics.peakTextBytes,cacheBytes);
      }
    }
    if(trace){const began=performance.now();trace.push([key??exactKey(values),exactKey(result)]);metrics.traceMs+=performance.now()-began;}
    return result;
  };
  return{metrics,close(){cache.clear();cacheBytes=0;closed=true;dynamics.checkPose=original;metrics.cleared=cache.size===0;metrics.restored=dynamics.checkPose===original;}};
}

function unitChecks(){
  for(const [left,right]of [[0,-0],[{}, {x:undefined}],[{x:NaN},{x:null}],[[1,2],[2,1]],[{a:1,b:2},{b:2,a:1}]])assert.notEqual(exactKey(left),exactKey(right));
  assert.throws(()=>exactKey({x:NaN},{finiteOnly:true}));assert.throws(()=>exactKey({get x(){throw Error('must not run');}}));
  let calls=0;const dynamics={checkPose(pose){calls++;return{safe:pose.x===1,contacts:[{dist:pose.x}],limitViolations:[]};}},original=dynamics.checkPose;
  const memo=installJobMemo(dynamics,{capacity:2});
  try{
    dynamics.checkPose({x:1}).contacts[0].dist=90;
    assert.equal(dynamics.checkPose({x:1}).contacts[0].dist,1,'Consumer mutation poisoned stored result');
    dynamics.checkPose({x:2});assert.equal(dynamics.checkPose({x:2}).safe,false);
    dynamics.checkPose({x:3});dynamics.checkPose({x:1});assert.equal(calls,4);assert.equal(memo.metrics.peakEntries,2);assert.equal(memo.metrics.unsafeHits,1);
    assert.throws(()=>{try{throw Error('cancel/error');}finally{memo.close();}},/cancel\/error/);
  }finally{memo.close();}
  assert.equal(dynamics.checkPose,original);assert.equal(memo.metrics.cleared,true);assert.equal(memo.metrics.restored,true);
  const changed={data:{qpos:new Float64Array([0]),ctrl:new Float64Array([0])},checkPose(){return{safe:true,contacts:[{dist:this.data.ctrl[0]}]};}};
  const guarded=installJobMemo(changed,{capacity:2});
  try{assert.equal(changed.checkPose({x:1}).contacts[0].dist,0);changed.data.ctrl[0]=1;
    assert.equal(changed.checkPose({x:1}).contacts[0].dist,1,'Live motor state invalidation reused an old safety result');
    changed.data.ctrl[0]=2;assert.equal(changed.checkPose({x:1}).contacts[0].dist,2);assert.equal(guarded.metrics.stateInvalidations,1);assert.equal(guarded.metrics.hits,0);
  }finally{guarded.close();}
  return{signedZero:true,missingUndefined:true,diagnosticsExact:true,orderedArraysAndEntries:true,accessorsBypass:true,
    mutableConsumerIsolation:true,negativeResultsCached:true,boundedLRU:true,errorFinallyCleanup:true,liveStateChangeDisablesMemo:true};
}
const checks=unitChecks(),snapshots=JSON.parse(await readFile(new URL(inputPath,root),'utf8'));
for(const [path,expected]of Object.entries(snapshots.sourceHashes))if(sourceHashes[path])assert.equal(sourceHashes[path],expected,`Snapshot production source changed: ${path}`);
const startedAt=new Date().toISOString(),started=performance.now(),runtimes=[];
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const config=ROBOT_CONFIGS.g1,baseXml=await readFile(new URL(config.xml),'utf8'),urdf=await readFile(new URL(config.urdf),'utf8');
async function fixture(encoded){
  const job=decodePlanningValue(encoded),vfs=new api.MjVFS();let model,data;
  try{
    model=api.MjModel.from_xml_string(baseXml,vfs);data=new api.MjData(model);
    const runtime=createRobotRuntimeFromParts({id:'g1',api,model,data,robot:parsePlanningRobot(urdf,{rootHeight:config.rootHeight}),sourceXml:baseXml,cooperativePlanning:true});
    runtimes.push(runtime);assert.equal(runtime.configureTask('pipes',{seed:job.modelPipeSeed}).ok,true);
    if(job.pipeSeed!==job.modelPipeSeed)runtime.reset({seed:job.pipeSeed});restorePlanningJob(runtime,encoded);return{runtime,job};
  }catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
}
const state=runtime=>exactKey({qpos:[...runtime.data.qpos],qvel:[...runtime.data.qvel],ctrl:[...runtime.data.ctrl],time:runtime.data.time});
async function run(input,enabled){
  const {runtime,job}=await fixture(input.job),trace=[],before=state(runtime),controller=runtime.controller;
  assert.equal(controller.planningBudgetMs,undefined,'Experiment must not override the original controller budget');
  const memo=installJobMemo(runtime.dynamics,{capacity:maxEntries,trace,enabled});let result,wallMs;
  try{
    const began=performance.now();assert.equal(controller.prepareItem(job.request.arm,job.request.id,job.request.continuing),true,controller.message);
    do{result=controller.advancePlanning();}while(result.pending);wallMs=performance.now()-began;
    assert.equal(result.ok,true,result.reason??controller.message);assert.equal(result.steps,input.baseline.steps);
    assert.equal(state(runtime),before,'Planning changed authoritative physics');
    const payload=capturePlanningResult(controller,controller.history.length-1);
    assert.deepEqual(payload,input.baseline.payload,'Memo changed the accepted plan or resulting command state');
    return{enabled,wallMs,steps:result.steps,planHash:hash(payload),exactPlan:true,unchangedPhysics:true,trace,metrics:memo.metrics};
  }finally{
    memo.close();assert.equal(memo.metrics.cleared,true);assert.equal(memo.metrics.restored,true);
    runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);
  }
}
try{
  const rows=[];
  for(const input of snapshots.rows.slice(0,itemCount))for(let trial=0;trial<trials;trial++){
    // Reverse pair order on alternating trials to expose warmup/order effects.
    const order=trial%2?[true,false]:[false,true],runs=[];
    for(const enabled of order){runs.push(await run(input,enabled));console.error(JSON.stringify({item:input.index+1,trial:trial+1,cache:enabled,wallMs:runs.at(-1).wallMs,hits:runs.at(-1).metrics.hits}));}
    const baseline=runs.find(row=>!row.enabled),cached=runs.find(row=>row.enabled);
    assert.equal(cached.trace.length,baseline.trace.length,'Changed check request count');
    for(let index=0;index<baseline.trace.length;index++)assert.deepEqual(cached.trace[index],baseline.trace[index],`Safety request/result differed at ${index}`);
    const safetyTraceHash=hash(baseline.trace),strip=({trace,...row})=>row;
    rows.push({index:input.index,selected:input.selected,trial:trial+1,order:order.map(value=>value?'memo':'original'),
      baseline:strip(baseline),cached:strip(cached),safetyTraceHash,exactRequestOrderAndEveryResult:true,speedup:baseline.wallMs/cached.wallMs});
  }
  const after=Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,hash(await readFile(new URL(path,root)))]))),sourceFrozen=JSON.stringify(after)===JSON.stringify(sourceHashes);
  const report={pass:sourceFrozen,startedAt,wallSeconds:(performance.now()-started)/1000,sourceFrozen,sourceHashes,inputPath,inputHash:hash(await readFile(new URL(inputPath,root))),checks,maxEntries,trials,
    scope:'Experiment only, fresh private G1 planning worlds from four actual immutable snapshots. Original budgets, candidate order, all safety checks and accepted plans unchanged. No production code modified. Paired timing includes exact trace encoding in both paths and bounded memo/clone costs in the memo path; this is Node planning time, not browser FPS.',rows};
  await mkdir(new URL('.',new URL(outputPath,root)),{recursive:true});await writeFile(new URL(outputPath,root),JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify({pass:report.pass,sourceFrozen,rows:rows.length,report:outputPath}));
}finally{for(const runtime of runtimes)runtime.dispose();}
