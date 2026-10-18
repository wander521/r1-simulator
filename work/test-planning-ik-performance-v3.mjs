import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {gunzipSync} from 'node:zlib';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {createRobotRuntimeFromParts,ROBOT_CONFIGS} from '../dist/multi/robot-runtime.mjs';
import {parsePlanningRobot} from '../dist/multi/planning-robot.mjs';
import {enablePlanningWorkerIK} from '../dist/multi/planning-ik-context.mjs';
import {createPlanningCheckCache} from '../dist/multi/planning-check-cache.mjs';
import {decodePlanningValue,restorePlanningJob,capturePlanningResult} from '../dist/multi/planning-serialization.mjs';

const root=new URL('../',import.meta.url),args=process.argv.slice(2),arg=(name,fallback)=>args.find(value=>value.startsWith(`--${name}=`))?.slice(name.length+3)??fallback;
const output=arg('report','work/validation/performance-v4/planning-ik-paired.json'),trials=Number(arg('trials',2)),items=Number(arg('items',4)),inputPath='work/validation/performance-v2/g1-pipe592-snapshots.json.gz';
assert(Number.isInteger(trials)&&trials>0&&trials<=2);assert(Number.isInteger(items)&&items>0&&items<=4);
const hash=value=>createHash('sha256').update(typeof value==='string'||ArrayBuffer.isView(value)?value:JSON.stringify(value)).digest('hex');
const paths=['work/test-planning-ik-performance-v3.mjs','work/planning-cache-experiment.mjs','dist/multi/robot-runtime.mjs','dist/multi/planning.mjs','dist/multi/pipe-controller.mjs','dist/multi/planning-budget.mjs','dist/multi/planning-ik-context.mjs','dist/multi/planning-check-cache.mjs','dist/multi/planning-robot.mjs','dist/multi/planning-serialization.mjs',
  ...['planning','pipe-controller','robot-runtime','g1-kinematics'].map(name=>`work/fixtures/performance-v3/${name}-3949cdc.txt`),
  'dist/g1/grasp-kinematics.mjs','dist/g1/grasp-controller.mjs','dist/g1/g1-dynamics.mjs','dist/g1/g1-profile.mjs','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/g1/models/g1-dynamic-scene.xml','dist/vendor/three.module.min.js','dist/vendor/three.core.min.js','dist/r1/mujoco-runtime.mjs','dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm'];
const sources=async()=>Object.fromEntries(await Promise.all(paths.map(async path=>[path,hash(await readFile(new URL(path,root)))])));
const sourceHashes=await sources(),startedAt=new Date().toISOString(),started=performance.now();
const referenceHashes={planning:'9cfa8681783ec616ea24aa27b2991e30a1574cea74f84252540301fd96c39810','pipe-controller':'788ae5dc4dd5495816d5cb8b6934b382a11d4731e788cd53ba8577848b2e441e','robot-runtime':'80174f074646f9cff6494424e0b0dc26b98666010c8057ffd11a8781dd37d3ab'};
const asModule=source=>`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`;
const imports=(source,path,overrides={})=>source.replace(/\bfrom\s*(['"])([^'"]+)\1/g,(all,quote,relative)=>`from ${quote}${overrides[relative]??new URL(relative,new URL(path,root)).href}${quote}`).replaceAll('import.meta.url',JSON.stringify(new URL(path,root).href));
const frozen={};for(const name of Object.keys(referenceHashes)){const source=await readFile(new URL(`work/fixtures/performance-v3/${name}-3949cdc.txt`,root),'utf8');assert.equal(hash(source),referenceHashes[name]);frozen[name]=source;}
const planning=asModule(imports(frozen.planning,'dist/multi/planning.mjs')),pipe=asModule(imports(frozen['pipe-controller'],'dist/multi/pipe-controller.mjs'));
const originalFactory=(await import(asModule(imports(frozen['robot-runtime'],'dist/multi/robot-runtime.mjs',{'./planning.mjs?v=20261018-perf-2':planning,'./pipe-controller.mjs':pipe})))).createRobotRuntimeFromParts;
const helper=await readFile(new URL('work/planning-cache-experiment.mjs',root),'utf8');
const {exactKey}=await import(asModule(helper.slice(helper.indexOf('export function exactKey'),helper.indexOf('export function installJobMemo'))));
const compressed=await readFile(new URL(inputPath,root)),raw=gunzipSync(compressed),snapshots=JSON.parse(raw);
const api=await createMuJoCo({locateFile:()=>new URL('dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',root).pathname}),config=ROBOT_CONFIGS.g1;
const xml=await readFile(new URL(config.xml),'utf8'),urdf=await readFile(new URL(config.urdf),'utf8');
function physicsSame(runtime,job){assert.equal(runtime.data.time,job.physics.time);for(const [name,value]of Object.entries(job.physics))if(name!=='time')assert.deepEqual([...runtime.data[name]],value,`Changed physics ${name}`);for(const [name,value]of Object.entries(job.derived))assert.deepEqual([...runtime.data[name]],value,`Changed derived ${name}`);}
async function run(input,optimized){
  const vfs=new api.MjVFS();let model,data,runtime,cache;
  try{
    model=api.MjModel.from_xml_string(xml,vfs);data=new api.MjData(model);
    const robot=parsePlanningRobot(urdf,{rootHeight:config.rootHeight,reuseUnchangedJoints:true});if(optimized)enablePlanningWorkerIK(robot);
    runtime=(optimized?createRobotRuntimeFromParts:originalFactory)({id:'g1',api,model,data,robot,sourceXml:xml,cooperativePlanning:true});
    const job=decodePlanningValue(input.job);assert.equal(runtime.configureTask('pipes',{seed:job.modelPipeSeed}).ok,true);if(job.pipeSeed!==job.modelPipeSeed)runtime.reset({seed:job.pipeSeed});restorePlanningJob(runtime,input.job);
    cache=createPlanningCheckCache(runtime.dynamics);const checked=runtime.dynamics.checkPose,trace=createHash('sha256');let checks=0;
    runtime.dynamics.checkPose=function(...values){const result=checked.apply(this,values),request=exactKey(values),response=exactKey(result);trace.update(`${request.length}:`);trace.update(request);trace.update(`${response.length}:`);trace.update(response);checks++;return result;};
    const controller=runtime.controller;assert.equal(controller.planningBudgetMs,undefined);const began=performance.now();assert.equal(controller.prepareItem(job.request.arm,job.request.id,job.request.continuing),true,controller.message);
    let result;do{result=controller.advancePlanning();}while(result.pending);const wallMs=performance.now()-began;
    assert.equal(result.ok,true,result.reason??controller.message);assert.equal(result.steps,input.baseline.steps);physicsSame(runtime,job);
    const payload=capturePlanningResult(controller,controller.history.length-1);assert.deepEqual(payload,input.baseline.payload,'Changed accepted plan or command state');
    return{optimized,wallMs,steps:result.steps,safetyChecks:checks,safetyTraceHash:trace.digest('hex'),planHash:hash(payload),exactPlanAndCommand:true,unchangedAllPhysicsArrays:true,cache:{...cache.stats}};
  }finally{cache?.dispose();if(runtime)runtime.dispose();else{data?.delete();model?.delete();}vfs.delete();}
}
const rows=[];
for(const input of snapshots.rows.slice(0,items))for(let trial=0;trial<trials;trial++){
  const order=trial%2?[true,false]:[false,true],runs=[];
  for(const optimized of order){const row=await run(input,optimized);runs.push(row);console.error(JSON.stringify({item:input.index+1,trial:trial+1,optimized,wallMs:row.wallMs,checks:row.safetyChecks}));}
  const baseline=runs.find(row=>!row.optimized),optimized=runs.find(row=>row.optimized);assert.equal(baseline.safetyChecks,optimized.safetyChecks);assert.equal(baseline.safetyTraceHash,optimized.safetyTraceHash);assert.equal(baseline.planHash,optimized.planHash);
  rows.push({index:input.index,selected:input.selected,trial:trial+1,order:order.map(value=>value?'worker-v3':'worker-v2'),baseline,optimized,exactEverySafetyRequestAndResult:true,speedup:baseline.wallMs/optimized.wallMs});
}
const endSourceHashes=await sources(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes);assert(sourceFrozen);
const baselineMs=rows.reduce((sum,row)=>sum+row.baseline.wallMs,0),optimizedMs=rows.reduce((sum,row)=>sum+row.optimized.wallMs,0);
const report={pass:true,releaseCohort:'20261018-perf-4',startedAt,wallSeconds:(performance.now()-started)/1000,sourceFrozen,sourceHashes,endSourceHashes,inputPath,inputHash:hash(raw),compressedInputHash:hash(compressed),baselineCommit:'3949cdc3c156221f3db6268f6191b603ec546245',trials,items,
  summary:{baselineMs,optimizedMs,reductionFraction:1-optimizedMs/baselineMs,speedup:baselineMs/optimizedMs},rows,
  scope:'Four genuine pipe592 planning snapshots, reversed-order paired Worker-v2 (frozen 3949cdc factory/planning/pipe plus shipped unchanged-joint parser and exact collision cache) versus Worker-v3 node FK reuse and fine budget checks. Same original R1/G1/C5 solvers, full plan/command and complete ordered safety stream hash compare exactly; all authoritative/derived physics arrays unchanged. Original candidate order, 10/30s budgets and safety sampling retained. Node CPU timings include identical safety encoding/hash instrumentation, exclude fresh model initialization, are not native browser responsiveness/FPS or physical execution.'};
await mkdir(new URL('.',new URL(output,root)),{recursive:true});await writeFile(new URL(output,root),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({pass:true,sourceFrozen,rows:rows.length,...report.summary,report:output}));
