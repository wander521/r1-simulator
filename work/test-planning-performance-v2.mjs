import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotRuntimeFromParts,ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {parsePlanningRobot} from '../dist/multi/planning-robot.mjs';
import {createPlanningWorkerRobot,createPlanningWorkerService} from '../dist/multi/planning-worker-core.mjs';
import {createPlanningCheckCache} from '../dist/multi/planning-check-cache.mjs';
import {decodePlanningValue,restorePlanningJob,capturePlanningResult} from '../dist/multi/planning-serialization.mjs';

// Final source-bound experiment of the SHIPPED parser/cache functions, plus
// actual worker-service reuse across all four genuine planning snapshots.
const root=new URL('../',import.meta.url),args=process.argv.slice(2);
const output=args.find(value=>value.startsWith('--report='))?.slice(9)??'work/validation/performance-v2/planning-combined-final.json';
const inputPath='work/validation/performance-v2/g1-pipe592-snapshots.json';
const trials=Number(args.find(value=>value.startsWith('--trials='))?.slice(9)??2);
const hash=value=>createHash('sha256').update(typeof value==='string'||ArrayBuffer.isView(value)?value:JSON.stringify(value)).digest('hex');
const sourcePaths=['work/test-planning-performance-v2.mjs','work/planning-cache-experiment.mjs',
  'dist/multi/robot-runtime.mjs','dist/multi/pipe-controller.mjs','dist/multi/planning.mjs','dist/multi/planning-robot.mjs',
  'dist/multi/planning-check-cache.mjs','dist/multi/planning-serialization.mjs','dist/multi/planning-worker-core.mjs',
  'dist/g1/grasp-controller.mjs','dist/g1/grasp-kinematics.mjs','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs',
  'dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/g1/models/g1-dynamic-scene.xml','dist/vendor/three.module.min.js',
  'dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm'];
const sources=async()=>Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,hash(await readFile(new URL(path,root)))])));
const sourceHashes=await sources(),startedAt=new Date().toISOString(),started=performance.now();
const helper=await readFile(new URL('work/planning-cache-experiment.mjs',root),'utf8');
const helperSource=helper.slice(helper.indexOf('export function exactKey'),helper.indexOf('export function installJobMemo'));
const {exactKey}=await import(`data:text/javascript;base64,${Buffer.from(helperSource).toString('base64')}`);
const snapshots=JSON.parse(await readFile(new URL(inputPath,root),'utf8'));
for(const [path,expected]of Object.entries(snapshots.sourceHashes))if(path.startsWith('dist/g1/')||path.endsWith('.wasm')||path.endsWith('three.module.min.js'))assert.equal(sourceHashes[path],expected,`Original model/math changed: ${path}`);
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname});
const config=ROBOT_CONFIGS.g1,baseXml=await readFile(new URL(config.xml),'utf8'),urdf=await readFile(new URL(config.urdf),'utf8'),runtimes=[];
async function rawRuntime(optimized){
  const vfs=new api.MjVFS();let model,data;
  try{
    model=api.MjModel.from_xml_string(baseXml,vfs);data=new api.MjData(model);
    const robot=optimized?createPlanningWorkerRobot(urdf,{rootHeight:config.rootHeight}):parsePlanningRobot(urdf,{rootHeight:config.rootHeight});
    const runtime=createRobotRuntimeFromParts({id:'g1',api,model,data,robot,sourceXml:baseXml,cooperativePlanning:true});
    runtimes.push(runtime);return runtime;
  }catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
}
function dispose(runtime){runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);}
function assertPhysics(runtime,job){
  assert.equal(runtime.data.time,job.physics.time);
  for(const [name,values]of Object.entries(job.physics))if(name!=='time')assert.deepEqual(Array.from(runtime.data[name]),values,`Planning changed ${name}`);
  for(const [name,values]of Object.entries(job.derived))assert.deepEqual(Array.from(runtime.data[name]),values,`Planning changed ${name}`);
}
async function direct(input,optimized){
  const runtime=await rawRuntime(optimized),job=decodePlanningValue(input.job),trace=[];
  assert.equal(runtime.configureTask('pipes',{seed:job.modelPipeSeed}).ok,true);
  if(job.pipeSeed!==job.modelPipeSeed)runtime.reset({seed:job.pipeSeed});restorePlanningJob(runtime,input.job);
  const controller=runtime.controller,legacy=runtime.dynamics.checkPose,cache=optimized?createPlanningCheckCache(runtime.dynamics):null;
  const checked=runtime.dynamics.checkPose;
  runtime.dynamics.checkPose=function(...values){
    const value=checked.apply(this,values);trace.push([exactKey(values),exactKey(value)]);return value;
  };
  try{
    assert.equal(controller.planningBudgetMs,undefined);const began=performance.now();
    assert.equal(controller.prepareItem(job.request.arm,job.request.id,job.request.continuing),true,controller.message);
    let result;do{result=controller.advancePlanning();}while(result.pending);const wallMs=performance.now()-began;
    assert.equal(result.ok,true,result.reason??controller.message);assert.equal(result.steps,input.baseline.steps);assertPhysics(runtime,job);
    const payload=capturePlanningResult(controller,controller.history.length-1);assert.deepEqual(payload,input.baseline.payload);
    return{optimized,wallMs,steps:result.steps,trace,planHash:hash(payload),exactPlanAndCommand:true,unchangedAllPhysicsArrays:true,cache:cache?.stats??null};
  }finally{
    runtime.dynamics.checkPose=checked;cache?.dispose();assert.equal(runtime.dynamics.checkPose,legacy);
    if(cache){assert.equal(cache.stats.disposed,true);assert.equal(cache.stats.entries,0);assert.equal(cache.stats.bytes,0);}
    dispose(runtime);
  }
}
try{
  const rows=[];
  for(const input of snapshots.rows)for(let trial=0;trial<trials;trial++){
    const order=trial%2?[true,false]:[false,true],runs=[];
    for(const optimized of order){runs.push(await direct(input,optimized));console.error(JSON.stringify({item:input.index+1,trial:trial+1,optimized,wallMs:runs.at(-1).wallMs,hits:runs.at(-1).cache?.hits??0}));}
    const baseline=runs.find(row=>!row.optimized),optimized=runs.find(row=>row.optimized);
    assert.equal(optimized.trace.length,baseline.trace.length);
    for(let index=0;index<baseline.trace.length;index++)assert.deepEqual(optimized.trace[index],baseline.trace[index],`Safety request/result differed at ${index}`);
    const strip=({trace,...value})=>value;
    rows.push({index:input.index,selected:input.selected,trial:trial+1,order:order.map(value=>value?'worker-optimized':'original'),baseline:strip(baseline),optimized:strip(optimized),
      exactEverySafetyRequestAndResult:true,safetyTraceHash:hash(baseline.trace),speedup:baseline.wallMs/optimized.wallMs});
  }
  const serviceRuntimes=[],service=createPlanningWorkerService({createRuntime:async()=>{const runtime=await rawRuntime(true);serviceRuntimes.push(runtime);return runtime;}}),serviceRows=[];
  try{
    for(const input of snapshots.rows){
      const began=performance.now(),result=await service.run(input.job),runtime=serviceRuntimes.at(-1),job=decodePlanningValue(input.job);
      assert.equal(result.ok,true,result.reason);assert.equal(result.steps,input.baseline.steps);assert.deepEqual(result.payload,input.baseline.payload);assertPhysics(runtime,job);
      serviceRows.push({index:input.index,selected:input.selected,steps:result.steps,wallMs:performance.now()-began,exactPlanAndCommand:true,unchangedAllPhysicsArrays:true,planHash:hash(result.payload)});
    }
    assert.equal(serviceRuntimes.length,1,'Same compiled-world jobs should reuse one isolated worker world');
  }finally{service.dispose();for(const runtime of serviceRuntimes)runtimes.splice(runtimes.indexOf(runtime),1);}
  const endSourceHashes=await sources(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes);
  const report={pass:sourceFrozen,startedAt,wallSeconds:(performance.now()-started)/1000,sourceFrozen,sourceHashes,endSourceHashes,inputPath,inputHash:hash(await readFile(new URL(inputPath,root))),trials,
    scope:'Actual shipped Worker parser and 512-entry/2MiB encoded-text bounded job memo versus default original parser/no memo, on all four genuine G1 pipe592 snapshots. Original controllers, physics models, IK precision, collision gates, candidate order and planning budgets unchanged. Every safety input/result, final plan/command and all authoritative physics/derived arrays compare exactly. Additional actual worker-service run reuses one private world for all four jobs. Node paired timings include exact trace encoding and are not browser FPS.',rows,serviceRows};
  await mkdir(new URL('.',new URL(output,root)),{recursive:true});await writeFile(new URL(output,root),JSON.stringify(report,null,2)+'\n');
  console.log(JSON.stringify({pass:report.pass,sourceFrozen,pairedRows:rows.length,serviceRows:serviceRows.length,report:output}));
}finally{for(const runtime of runtimes)runtime.dispose();}
