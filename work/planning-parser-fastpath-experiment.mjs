import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotRuntimeFromParts,ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {parsePlanningRobot} from '../dist/multi/planning-robot.mjs';
import {decodePlanningValue,restorePlanningJob,capturePlanningResult} from '../dist/multi/planning-serialization.mjs';

// A test-only wrapper experiments with the exact unchanged-value fastpath.
// Production modules and their defaults remain untouched. The helper module
// below reuses only the exported pure function declarations from the earlier
// frozen experiment, without executing its benchmark runner or changing it.
const root=new URL('../',import.meta.url),args=process.argv.slice(2),withCache=args.includes('--with-cache');
const outputPath=args.find(value=>value.startsWith('--out='))?.slice(6)??'work/validation/performance-v2/planning-parser-fastpath-experiment.json';
const inputPath='work/validation/performance-v2/g1-pipe592-snapshots.json';
const trials=Number(args.find(value=>value.startsWith('--trials='))?.slice(9)??2);
const itemCount=Number(args.find(value=>value.startsWith('--items='))?.slice(8)??4);
const startIndex=Number(args.find(value=>value.startsWith('--start='))?.slice(8)??0);
const hash=value=>createHash('sha256').update(typeof value==='string'||ArrayBuffer.isView(value)?value:JSON.stringify(value)).digest('hex');
const sourcePaths=['work/planning-parser-fastpath-experiment.mjs','work/planning-cache-experiment.mjs',
  'dist/multi/robot-runtime.mjs','dist/multi/pipe-controller.mjs','dist/multi/planning.mjs','dist/multi/planning-robot.mjs',
  'dist/multi/planning-serialization.mjs','dist/multi/planning-worker-core.mjs','dist/g1/grasp-controller.mjs',
  'dist/g1/grasp-kinematics.mjs','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs','dist/g1/assets/g1_29dof_with_dex1_1.urdf',
  'dist/g1/models/g1-dynamic-scene.xml','dist/vendor/three.module.min.js','dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm'];
const sourceHashes=Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,hash(await readFile(new URL(path,root)))])));
const helperSource=await readFile(new URL('work/planning-cache-experiment.mjs',root),'utf8');
const pureSource=helperSource.slice(helperSource.indexOf('export function exactKey'),helperSource.indexOf('function unitChecks'));
assert(pureSource.startsWith('export function exactKey')&&pureSource.includes('export function installJobMemo'));
const {exactKey,installJobMemo}=await import(`data:text/javascript;base64,${Buffer.from(`import assert from 'node:assert/strict';\n${pureSource}`).toString('base64')}`);

export function installJointFastpath(robot){
  const metrics={requests:0,skipped:0,computed:0},restore=[];
  for(const joint of Object.values(robot.joints)){
    const original=joint.setValue;let initialized=false;
    joint.setValue=function(value){
      metrics.requests++;
      // Delegate unusual/fixed inputs to the original validation unchanged.
      if(joint.jointType==='fixed'||!Number.isFinite(value)){metrics.computed++;return original.call(this,value);}
      const clamped=joint.jointType==='continuous'?value:Math.max(joint.limit.lower,Math.min(joint.limit.upper,value));
      if(initialized&&Object.is(clamped,joint.jointValue[0])){metrics.skipped++;return;}
      initialized=true;metrics.computed++;return original.call(this,value);
    };
    restore.push(()=>joint.setValue=original);
  }
  return{metrics,close(){for(const reset of restore)reset();}};
}
function unitChecks(){
  let calls=0;const joint={jointType:'continuous',jointValue:[0],setValue(value){assert(Number.isFinite(value));calls++;this.jointValue[0]=value;}};
  const original=joint.setValue,wrapper=installJointFastpath({joints:{joint}});
  try{
    joint.setValue(0);assert.equal(calls,1,'First zero assignment must still calculate its source quaternion');
    joint.setValue(0);assert.equal(calls,1);joint.setValue(-0);assert.equal(calls,2);assert(Object.is(joint.jointValue[0],-0));
    joint.setValue(-0);assert.equal(calls,2);assert.throws(()=>joint.setValue(Infinity));
  }finally{wrapper.close();}
  assert.equal(joint.setValue,original);
  let limitedCalls=0;const limited={jointType:'revolute',jointValue:[0],limit:{lower:-1,upper:1},setValue(value){limitedCalls++;this.jointValue[0]=Math.max(-1,Math.min(1,value));}};
  const limitedWrapper=installJointFastpath({joints:{limited}});
  try{limited.setValue(2);limited.setValue(3);assert.equal(limitedCalls,1,'Fastpath must compare after the original finite clamp');}
  finally{limitedWrapper.close();}
  return{firstAssignmentCalculated:true,signedZeroPreserved:true,nonfiniteValidationDelegated:true,clampBeforeComparison:true,originalSetterRestored:true};
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
async function run(input,optimized){
  const {runtime,job}=await fixture(input.job),trace=[],before=state(runtime),controller=runtime.controller;
  assert.equal(controller.planningBudgetMs,undefined,'Experiment must not override the original controller budget');
  const joints=optimized?installJointFastpath(runtime.robot):null;
  const memo=installJobMemo(runtime.dynamics,{capacity:512,trace,enabled:optimized&&withCache});let result;
  try{
    const began=performance.now();assert.equal(controller.prepareItem(job.request.arm,job.request.id,job.request.continuing),true,controller.message);
    do{result=controller.advancePlanning();}while(result.pending);const wallMs=performance.now()-began;
    assert.equal(result.ok,true,result.reason??controller.message);assert.equal(result.steps,input.baseline.steps);assert.equal(state(runtime),before);
    const payload=capturePlanningResult(controller,controller.history.length-1);assert.deepEqual(payload,input.baseline.payload);
    return{optimized,wallMs,steps:result.steps,planHash:hash(payload),exactPlan:true,unchangedPhysics:true,trace,
      jointMetrics:joints?.metrics??null,checkMetrics:memo.metrics};
  }finally{
    memo.close();joints?.close();assert.equal(memo.metrics.cleared,true);assert.equal(memo.metrics.restored,true);
    runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);
  }
}
try{
  const rows=[];
  for(const input of snapshots.rows.slice(startIndex,startIndex+itemCount))for(let trial=0;trial<trials;trial++){
    const order=trial%2?[true,false]:[false,true],runs=[];
    for(const optimized of order){runs.push(await run(input,optimized));console.error(JSON.stringify({item:input.index+1,trial:trial+1,optimized,wallMs:runs.at(-1).wallMs,skipped:runs.at(-1).jointMetrics?.skipped??0}));}
    const baseline=runs.find(row=>!row.optimized),optimized=runs.find(row=>row.optimized);
    assert.equal(optimized.trace.length,baseline.trace.length);
    for(let index=0;index<baseline.trace.length;index++)assert.deepEqual(optimized.trace[index],baseline.trace[index],`Safety request/result differed at ${index}`);
    const strip=({trace,...row})=>row;
    rows.push({index:input.index,selected:input.selected,trial:trial+1,order:order.map(value=>value?'optimized':'original'),baseline:strip(baseline),optimized:strip(optimized),
      safetyTraceHash:hash(baseline.trace),exactRequestOrderAndEveryResult:true,speedup:baseline.wallMs/optimized.wallMs});
  }
  const after=Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,hash(await readFile(new URL(path,root)))]))),sourceFrozen=JSON.stringify(after)===JSON.stringify(sourceHashes);
  const report={pass:sourceFrozen,startedAt,wallSeconds:(performance.now()-started)/1000,sourceFrozen,sourceHashes,inputPath,inputHash:hash(await readFile(new URL(inputPath,root))),checks,withCache,trials,
    scope:'Test-only exact-value setter wrapper, optional 512-entry job memo, fresh isolated real G1 planning worlds. Original budgets, candidate order, all safety results and final command state are bit/deep equal; no production source changed. Paired timing includes identical exact safety-trace encoding, not browser FPS.',rows};
  await mkdir(new URL('.',new URL(outputPath,root)),{recursive:true});await writeFile(new URL(outputPath,root),JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify({pass:report.pass,sourceFrozen,rows:rows.length,withCache,report:outputPath}));
}finally{for(const runtime of runtimes)runtime.dispose();}
