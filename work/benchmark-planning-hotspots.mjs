import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import inspector from 'node:inspector';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotRuntimeFromParts,ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {parsePlanningRobot} from '../dist/multi/planning-robot.mjs';
import {capturePlanningJob,restorePlanningJob,encodePlanningValue,capturePlanningResult,decodePlanningValue} from '../dist/multi/planning-serialization.mjs';

// Read-only production-source profiling. Virtual data: modules add timing hooks
// to lexical solvePose imports; no production file or search policy is changed.
const root=new URL('../',import.meta.url),args=process.argv.slice(2);
const output=args.find(value=>value.startsWith('--out='))?.slice(6)??'work/validation/performance-v2/planning-hotspots-baseline.json';
const snapshotPath=args.find(value=>value.startsWith('--snapshots='))?.slice(12)??'work/validation/performance-v2/g1-pipe592-snapshots.json';
const snapshotOnly=args.includes('--snapshots-only'),profileOnly=args.includes('--profile-only');
const itemCount=Number(args.find(value=>value.startsWith('--items='))?.slice(8)??4);
const hash=value=>createHash('sha256').update(typeof value==='string'||ArrayBuffer.isView(value)?value:JSON.stringify(value)).digest('hex');
const sourcePaths=['work/benchmark-planning-hotspots.mjs','dist/multi/robot-runtime.mjs','dist/multi/pipe-controller.mjs','dist/multi/planning.mjs',
  'dist/multi/planning-robot.mjs','dist/multi/planning-serialization.mjs','dist/multi/planning-worker-core.mjs',
  'dist/g1/grasp-controller.mjs','dist/g1/grasp-kinematics.mjs','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs',
  'dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/g1/models/g1-dynamic-scene.xml','dist/vendor/three.module.min.js',
  'dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm'];
const sourceHashes=Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,hash(await readFile(new URL(path,root)))])));
const startedAt=new Date().toISOString(),started=performance.now(),runtimes=[];
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const config=ROBOT_CONFIGS.g1,baseXml=await readFile(new URL(config.xml),'utf8'),urdf=await readFile(new URL(config.urdf),'utf8');
async function fixture(factory=createRobotRuntimeFromParts){
  const vfs=new api.MjVFS();let model,data;
  try{model=api.MjModel.from_xml_string(baseXml,vfs);data=new api.MjData(model);
    const runtime=factory({id:'g1',api,model,data,robot:parsePlanningRobot(urdf,{rootHeight:config.rootHeight}),sourceXml:baseXml,cooperativePlanning:true});
    runtimes.push(runtime);return runtime;
  }catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
}
function dispose(runtime){runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);}
const payload=controller=>capturePlanningResult(controller,controller.history.length-1);
const state=runtime=>({qpos:[...runtime.data.qpos],qvel:[...runtime.data.qvel],ctrl:[...runtime.data.ctrl],time:runtime.data.time});
const write=async(path,value)=>{await mkdir(new URL('.',new URL(path,root)),{recursive:true});await writeFile(new URL(path,root),JSON.stringify(value,null,2)+'\n');};

async function collect(){
  const runtime=await fixture(),rows=[];assert.equal(runtime.configureTask('pipes',{seed:521}).ok,true);runtime.reset({seed:592});
  for(let tick=0;tick<300;tick++)runtime.step(.001);runtime.syncVisuals();
  const controller=runtime.controller,prepare=controller.prepareItem.bind(controller);let pending=null;
  controller.prepareItem=(arm,id,continuing)=>{
    controller.arm=arm;controller.selected=id;
    pending={job:capturePlanningJob(runtime,{arm,id,continuing}),selected:id,index:rows.length,physicsTime:runtime.data.time};
    return prepare(arm,id,continuing);
  };
  assert.equal(runtime.start({task:'pipes'}),true);
  for(let tick=0;tick<180000&&rows.length<itemCount;tick++){
    if(runtime.planning){
      assert(pending);const before=state(runtime),began=performance.now();let result;
      do{result=runtime.advancePlanning();}while(result.pending);
      assert.equal(result.ok,true,result.reason??controller.message);assert.deepEqual(state(runtime),before);
      const accepted=payload(controller);rows.push({...pending,baseline:{wallMs:performance.now()-began,steps:result.steps,
        segmentElapsedMs:result.segmentElapsedMs,planHash:hash(accepted),payload:accepted}});
      console.error(JSON.stringify({captured:rows.length,id:pending.selected,steps:result.steps,ms:rows.at(-1).baseline.wallMs}));pending=null;
      if(rows.length===itemCount)break;
    }
    runtime.step(.001);runtime.dynamics.syncRobot();
    assert.notEqual(controller.state,'failed',controller.message);
  }
  assert.equal(rows.length,itemCount);dispose(runtime);
  const report={scope:'Genuine production G1 world, compiled pipe seed 521 then current layout seed 592, 300 idle 1 ms ticks; first four real automatic classification preparations. No physical gates or planner budgets changed.',startedAt,sourceHashes,rows};
  await write(snapshotPath,report);return report;
}

let active=null;
const hooks={solve(original,args){
  if(!active)return original(...args);
  const began=performance.now(),key=JSON.stringify([args[1],encodePlanningValue(args[2]),encodePlanningValue(args[3]),args[4],args[5],args[6]]);
  const existed=active.ikKeys.has(key);let result;
  try{return result=original(...args);}
  finally{const elapsed=performance.now()-began;active.ik.calls++;active.ik.ms+=elapsed;
    if(existed){active.ik.duplicateCalls++;active.ik.duplicateMs+=elapsed;}else active.ikKeys.set(key,true);
    active.ik.maxCallMs=Math.max(active.ik.maxCallMs,elapsed);
  }
}};
globalThis[Symbol.for('orbitPlanningBenchmarkHooks')]=hooks;
const asData=source=>`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`;
function absoluteImports(source,original,overrides={}){
  return source.replace(/\bfrom\s*(['"])([^'"]+)\1/g,(all,quote,path)=>`from ${quote}${overrides[path]??(path.startsWith('.')?new URL(path,original).href:path)}${quote}`)
    .replaceAll('import.meta.url',JSON.stringify(original.href));
}
async function profiledFactory(){
  const solver=new URL('dist/g1/grasp-kinematics.mjs',root),pipe=new URL('dist/multi/pipe-controller.mjs',root),runtime=new URL('dist/multi/robot-runtime.mjs',root);
  const shim=asData(`import * as original from ${JSON.stringify(solver.href)};export const naturalArmLimits=original.naturalArmLimits,naturalArmPose=original.naturalArmPose;export function solvePose(...args){return globalThis[Symbol.for('orbitPlanningBenchmarkHooks')].solve(original.solvePose,args);}`);
  const pipeModule=asData(absoluteImports(await readFile(pipe,'utf8'),pipe,{'../g1/grasp-kinematics.mjs':shim}));
  const runtimeModule=asData(absoluteImports(await readFile(runtime,'utf8'),runtime,{'./pipe-controller.mjs':pipeModule}));
  return(await import(runtimeModule)).createRobotRuntimeFromParts;
}
const emptyMetric=()=>({calls:0,ms:0,maxCallMs:0});
function instrument(runtime){
  const metrics={ik:{...emptyMetric(),duplicateCalls:0,duplicateMs:0},checkPose:{...emptyMetric(),duplicateCalls:0,duplicateMs:0},
    validateSegment:emptyMetric(),updateMatrixWorld:emptyMetric(),getObjectByName:emptyMetric(),jointSetCalls:0,unchangedJointSetCalls:0,ikKeys:new Map(),checkKeys:new Map()};
  const wrap=(object,name,metric,keyFor)=>{const original=object[name];object[name]=function(...args){
    if(!active)return original.apply(this,args);
    const began=performance.now(),key=keyFor?.(args),duplicate=key!==undefined&&metrics.checkKeys.has(key);
    try{return original.apply(this,args);}
    finally{const elapsed=performance.now()-began;metric.calls++;metric.ms+=elapsed;metric.maxCallMs=Math.max(metric.maxCallMs,elapsed);
      if(key!==undefined){if(duplicate){metric.duplicateCalls++;metric.duplicateMs+=elapsed;}else metrics.checkKeys.set(key,true);}}
  };};
  wrap(runtime.dynamics,'checkPose',metrics.checkPose,args=>JSON.stringify(encodePlanningValue(args)));
  wrap(runtime.controller,'validateSegment',metrics.validateSegment);
  wrap(runtime.robot,'updateMatrixWorld',metrics.updateMatrixWorld);
  wrap(runtime.robot,'getObjectByName',metrics.getObjectByName);
  for(const joint of Object.values(runtime.robot.joints)){
    const original=joint.setValue;joint.setValue=value=>{
      if(active){metrics.jointSetCalls++;const clamped=joint.jointType==='continuous'?value:Math.max(joint.limit.lower,Math.min(joint.limit.upper,value));if(clamped===joint.jointValue[0])metrics.unchangedJointSetCalls++;}
      return original(value);
    };
  }
  return metrics;
}
const session=new inspector.Session();session.connect();
const post=(method,params={})=>new Promise((resolve,reject)=>session.post(method,params,(error,value)=>error?reject(error):resolve(value)));
function aggregate(profile){
  const nodes=new Map(profile.nodes.map(node=>[node.id,node])),parents=new Map(),counts=new Map();
  for(const node of profile.nodes)for(const child of node.children??[])parents.set(child,node.id);
  for(let index=0;index<(profile.samples?.length??0);index++){
    let id=profile.samples[index],first=true;const time=(profile.timeDeltas?.[index]??1000)/1000;
    while(id!==undefined){const node=nodes.get(id);if(!node)break;let url=node.callFrame.url;
      if(url.startsWith('data:'))url='instrumented virtual production module';
      const name=`${node.callFrame.functionName||'(anonymous)'} @ ${url}:${node.callFrame.lineNumber+1}`;
      const row=counts.get(name)??{function:name,selfMs:0,inclusiveMs:0};row.inclusiveMs+=time;if(first)row.selfMs+=time;counts.set(name,row);first=false;id=parents.get(id);
    }
  }
  return{self:[...counts.values()].sort((a,b)=>b.selfMs-a.selfMs).slice(0,35),inclusive:[...counts.values()].sort((a,b)=>b.inclusiveMs-a.inclusiveMs).slice(0,35)};
}
try{
  const snapshots=profileOnly?JSON.parse(await readFile(new URL(snapshotPath,root),'utf8')):await collect();
  if(snapshotOnly){console.log(JSON.stringify({pass:true,snapshotPath,rows:snapshots.rows.length}));process.exitCode=0;}
  else{
    const factory=await profiledFactory(),rows=[];
    for(const input of snapshots.rows.slice(0,itemCount)){
      const runtime=await fixture(factory),job=decodePlanningValue(input.job);
      assert.equal(runtime.configureTask('pipes',{seed:job.modelPipeSeed}).ok,true);if(job.pipeSeed!==job.modelPipeSeed)runtime.reset({seed:job.pipeSeed});
      restorePlanningJob(runtime,input.job);const before=state(runtime),metrics=instrument(runtime),controller=runtime.controller;
      await post('Profiler.enable');await post('Profiler.setSamplingInterval',{interval:1000});await post('Profiler.start');
      active=metrics;const began=performance.now();assert.equal(controller.prepareItem(job.request.arm,job.request.id,job.request.continuing),true,controller.message);
      let result;do{result=controller.advancePlanning();}while(result.pending);const wallMs=performance.now()-began;active=null;
      const {profile}=await post('Profiler.stop');await post('Profiler.disable');
      assert.equal(result.ok,true,result.reason??controller.message);assert.deepEqual(state(runtime),before);
      const accepted=payload(controller);assert.deepEqual(accepted,input.baseline.payload,'Profiling changed the accepted plan or command state');assert.equal(result.steps,input.baseline.steps);
      const {ikKeys,checkKeys,...reported}=metrics;
      rows.push({index:input.index,selected:input.selected,steps:result.steps,baselineMs:input.baseline.wallMs,profiledWallMs:wallMs,planHash:hash(accepted),exactBaselinePlan:true,
        metrics:{...reported,uniqueIKInputs:ikKeys.size,uniqueCheckPoseInputs:checkKeys.size},cpu:aggregate(profile)});
      console.error(JSON.stringify({profiled:rows.length,selected:input.selected,steps:result.steps,wallMs,ikMs:metrics.ik.ms,collisionMs:metrics.checkPose.ms,duplicateIKMs:metrics.ik.duplicateMs,duplicateChecksMs:metrics.checkPose.duplicateMs}));dispose(runtime);
    }
    const after=Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,hash(await readFile(new URL(path,root)))]))),sourceFrozen=JSON.stringify(after)===JSON.stringify(sourceHashes);
    const report={pass:sourceFrozen,startedAt,wallSeconds:(performance.now()-started)/1000,sourceFrozen,sourceHashes,snapshotPath,
      scope:'Node mesh-free source URDF/production MuJoCo planning CPU benchmark, not browser frame rate or device performance. Instrumentation is inclusive/overlapping and adds overhead; duplicate-call time is an upper-bound opportunity, not a claimed speedup. Unchanged original planner budgets/search/gates. Full accepted plans and command state deep-equal baseline for every profiled input.',rows};
    await write(output,report);console.log(JSON.stringify({pass:report.pass,rows:rows.length,sourceFrozen,report:output,snapshotPath}));
  }
}finally{active=null;session.disconnect();for(const runtime of runtimes)runtime.dispose();delete globalThis[Symbol.for('orbitPlanningBenchmarkHooks')];}
