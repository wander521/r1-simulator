import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {performance} from 'node:perf_hooks';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {ROBOT_CONFIGS,createRobotRuntimeFromParts} from '../dist/multi/robot-runtime.mjs';
import {optimizeDynamics,restoreDynamics} from './validation/performance-v2/optimized-dynamics-own-getter.mjs';
import {loadTestRobot as loadR1} from './r1-test-robot.mjs';
import {loadG1TestRobot as loadG1} from './g1-test-robot.mjs';
import {loadC5TestRobot as loadC5} from './c5-test-robot.mjs';

const root=new URL('../',import.meta.url),args=process.argv.slice(2),contractsOnly=args.includes('--contracts-only');
const option=(name,fallback)=>args.find(arg=>arg.startsWith(`--${name}=`))?.slice(name.length+3)??fallback;
const ticks=Number(option('ticks',32000)),rounds=Number(option('rounds',3)),pipeTicks=Number(option('pipe-ticks',6000));
assert(Number.isSafeInteger(ticks)&&ticks>=1000);assert(Number.isSafeInteger(rounds)&&rounds>=1);assert(Number.isSafeInteger(pipeTicks)&&pipeTicks>=0);
const reportPath=option('report',`work/validation/performance-v2/rerun/dynamics-adapter-${contractsOnly?'contracts':'paired'}.json`);
const ids=['r1','g1','c5'],loaders={r1:loadR1,g1:loadG1,c5:loadC5};
const sources=[
  'work/test-dynamics-adapter-perf-v2.mjs','work/validation/performance-v2/optimized-dynamics-own-getter.mjs','dist/multi/robot-runtime.mjs',
  'dist/multi/r1-safe-dynamics.mjs','dist/multi/pipe-controller.mjs','dist/multi/pipe-layout.mjs','dist/multi/pipe-model.mjs',
  'dist/multi/planning.mjs','dist/multi/background-planning.mjs',
  ...ids.flatMap(id=>[`work/${id}-test-robot.mjs`,`dist/${id}/${id}-dynamics.mjs`,`dist/${id}/grasp-controller.mjs`,`dist/${id}/grasp-kinematics.mjs`,`dist/${id}/models/${id}-dynamic-scene.xml`]),
  'dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf',
  'dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs','dist/r1/mujoco-runtime.mjs','dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',
].sort();
const hashes=async()=>Object.fromEntries(await Promise.all(sources.map(async path=>[path,createHash('sha256').update(await readFile(new URL(path,root))).digest('hex')])));
const sourceHashes=await hashes(),started=performance.now(),startedAt=new Date().toISOString(),checks=[],measurements=[],getterCounts=[],runtimes=[];
let failure=null,api,exactTicks=0;
function check(name,fn){fn();checks.push({name,pass:true});}

function syntheticContracts(){
  check('same-call caching, no cross-call caching, forwarding and original receiver',()=>{
    let ctrlReads=0,biasReads=0,otherReads=0;
    const buffer=new ArrayBuffer(32),source={
      get ctrl(){assert.equal(this,source);ctrlReads++;return new Float64Array(buffer,0,2);},
      get qfrc_bias(){assert.equal(this,source);biasReads++;return new Float64Array(buffer,16,2);},
      get untouched(){assert.equal(this,source);otherReads++;return 17;},
      set untouched(value){assert.equal(this,source);this.written=value;},
    };
    const seen=[],dynamics={data:source,beforeStep(){seen.push(this.data.ctrl,this.data.ctrl,this.data.qfrc_bias,this.data.qfrc_bias);assert.equal(this.data.untouched,17);this.data.untouched=23;}};
    const original=dynamics.beforeStep;assert.equal(optimizeDynamics(dynamics),dynamics);const wrapper=dynamics.beforeStep;
    optimizeDynamics(dynamics);assert.equal(dynamics.beforeStep,wrapper);dynamics.beforeStep();assert.equal(dynamics.data,source);
    assert.equal(ctrlReads,1);assert.equal(biasReads,1);assert.equal(otherReads,1);assert.equal(source.written,23);
    assert.equal(seen[0],seen[1]);assert.equal(seen[2],seen[3]);dynamics.beforeStep();assert.equal(ctrlReads,2);assert.equal(biasReads,2);assert.notEqual(seen[0],seen[4]);
    assert.equal(restoreDynamics(dynamics),true);assert.equal(dynamics.beforeStep,original);assert.equal(restoreDynamics(dynamics),false);
  });
  check('detached ordinary memory refreshes inside the same command call',()=>{
    let memory=new ArrayBuffer(16),reads=0;
    new Float64Array(memory)[0]=1;
    const source={get ctrl(){reads++;return new Float64Array(memory);}};
    const dynamics={data:source,beforeStep(){
      const old=this.data.ctrl;assert.equal(old[0],1);
      structuredClone(memory,{transfer:[memory]});assert.equal(old.buffer.byteLength,0);
      memory=new ArrayBuffer(16);new Float64Array(memory)[0]=2;
      assert.equal(this.data.ctrl[0],2);assert.equal(this.data.ctrl[0],2);
    }};
    optimizeDynamics(dynamics);dynamics.beforeStep();assert.equal(reads,2);assert.equal(dynamics.data,source);
  });
  check('real WebAssembly.Memory.grow refreshes both cached views immediately',()=>{
    const memory=new WebAssembly.Memory({initial:1,maximum:2});let ctrlReads=0,biasReads=0;
    const source={get ctrl(){ctrlReads++;return new Float64Array(memory.buffer,0,2);},get qfrc_bias(){biasReads++;return new Float64Array(memory.buffer,16,2);}};
    const dynamics={data:source,beforeStep(){
      const ctrl=this.data.ctrl,bias=this.data.qfrc_bias;ctrl[0]=5;bias[0]=7;
      memory.grow(1);assert.equal(ctrl.buffer.byteLength,0);assert.equal(bias.buffer.byteLength,0);
      assert.equal(this.data.ctrl[0],5);assert.equal(this.data.qfrc_bias[0],7);
      assert.equal(this.data.ctrl.buffer,memory.buffer);assert.equal(this.data.qfrc_bias.buffer,memory.buffer);
    }};
    optimizeDynamics(dynamics);dynamics.beforeStep();assert.equal(ctrlReads,2);assert.equal(biasReads,2);assert.equal(dynamics.data,source);
  });
  check('shared memory never caches a potentially outdated non-detached view',()=>{
    if(typeof SharedArrayBuffer==='undefined')throw new Error('Node SharedArrayBuffer fixture unavailable');
    let memory=new SharedArrayBuffer(16),reads=0;
    const source={get ctrl(){reads++;return new Float64Array(memory);}};
    const dynamics={data:source,beforeStep(){assert.equal(this.data.ctrl.length,2);memory=new SharedArrayBuffer(32);assert.equal(this.data.ctrl.length,4);assert.equal(this.data.ctrl.length,4);}};
    optimizeDynamics(dynamics);dynamics.beforeStep();assert.equal(reads,3);
  });
  check('exceptions restore MjData identity and do not retain views',()=>{
    let reads=0;const source={get ctrl(){reads++;return new Float64Array(1);}},error=new Error('expected fixture exception');
    const dynamics={data:source,beforeStep(){this.data.ctrl[0]=3;throw error;}};
    optimizeDynamics(dynamics);assert.throws(()=>dynamics.beforeStep(),candidate=>candidate===error);assert.equal(dynamics.data,source);
    assert.throws(()=>dynamics.beforeStep());assert.equal(reads,2);
  });
  check('reentrant calls restore the enclosing facade then the original native data',()=>{
    const source={get ctrl(){return new Float64Array(1);}};let depth=0;
    const dynamics={data:source,beforeStep(){const entered=this.data;if(depth++===0)this.beforeStep();depth--;assert.equal(this.data,entered);this.data.ctrl[0]=1;}};
    optimizeDynamics(dynamics);dynamics.beforeStep();assert.equal(dynamics.data,source);
  });
  check('data/model replacement between calls uses the replacement memory',()=>{
    const first={ctrl:new Float64Array([1])},second={ctrl:new Float64Array([2])},seen=[];
    const dynamics={data:first,model:{id:1},beforeStep(){seen.push(this.data.ctrl[0]);}};
    optimizeDynamics(dynamics);dynamics.beforeStep();dynamics.data=second;dynamics.model={id:2};dynamics.beforeStep();assert.deepEqual(seen,[1,2]);assert.equal(dynamics.data,second);
  });
  check('a deliberate data replacement inside original code is preserved',()=>{
    const first={ctrl:new Float64Array([1])},second={ctrl:new Float64Array([2])};
    const dynamics={data:first,beforeStep(){this.data.ctrl[0]=3;this.data=second;}};
    optimizeDynamics(dynamics);dynamics.beforeStep();assert.equal(dynamics.data,second);
  });
  check('restore never removes a later third-party wrapper',()=>{
    const dynamics={data:{},beforeStep(){}};optimizeDynamics(dynamics);const later=()=>{};dynamics.beforeStep=later;
    assert.equal(restoreDynamics(dynamics),false);assert.equal(dynamics.beforeStep,later);
  });
}

async function create(id){
  const sourceXml=await readFile(new URL(ROBOT_CONFIGS[id].xml),'utf8'),vfs=new api.MjVFS();let model,data;
  try{model=api.MjModel.from_xml_string(sourceXml,vfs);data=new api.MjData(model);
    const runtime=createRobotRuntimeFromParts({id,api,model,data,sourceXml,robot:await loaders[id]()});runtimes.push(runtime);return runtime;
  }catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
}
function compare(pair,label){
  const {baseline:a,candidate:b}=pair;assert.equal(b.dynamics.data,b.data,`${a.id}/${label}: native data escaped beforeStep`);
  assert.equal(a.data.time,b.data.time);assert.equal(a.controller.state,b.controller.state);assert.equal(a.running,b.running);
  for(const key of ['qpos','qvel','ctrl','qfrc_actuator'])assert.deepEqual([...a.data[key]],[...b.data[key]],`${a.id}/${label}: ${key} changed`);
  for(const key of ['targets','previous','targetVelocity','fingerDesiredTargets','fingerRampedTargets','pipeFingerDesired','pipeFingerRamped'])assert.deepEqual(a.dynamics[key],b.dynamics[key],`${a.id}/${label}: ${key} changed`);
  exactTicks++;
}
function pairStep(pair,label,measurement=null,reverse=false){
  const entries=reverse?[[pair.candidate,'candidateStepMs'],[pair.baseline,'baselineStepMs']]:[[pair.baseline,'baselineStepMs'],[pair.candidate,'candidateStepMs']];
  for(const [runtime,key]of entries){const before=performance.now();runtime.step(.001);if(measurement)measurement[key]+=performance.now()-before;}
  compare(pair,label);
}
function commandTrace(pair,label,fn){fn(pair.baseline);fn(pair.candidate);compare(pair,label);}
function countViews(runtime){
  const counts={},restores=[];
  for(const name of ['ctrl','qfrc_bias']){
    let owner=runtime.data,descriptor;while(owner&&!(descriptor=Object.getOwnPropertyDescriptor(owner,name)))owner=Object.getPrototypeOf(owner);
    assert.equal(typeof descriptor?.get,'function');const previous=Object.getOwnPropertyDescriptor(runtime.data,name),get=descriptor.get;counts[name]=0;
    Object.defineProperty(runtime.data,name,{configurable:true,get(){counts[name]++;return get.call(runtime.data);}});
    restores.push(()=>previous?Object.defineProperty(runtime.data,name,previous):delete runtime.data[name]);
  }
  try{runtime.dynamics.beforeStep(.001);return counts;}finally{for(const restore of restores)restore();}
}

try{
  syntheticContracts();
  if(!contractsOnly){
    api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
    const pairs=[];
    for(const id of ids){const baseline=await create(id),candidate=await create(id);optimizeDynamics(candidate.dynamics);pairs.push({id,baseline,candidate});}
    for(let round=0;round<rounds;round++){
      const rows=pairs.map(pair=>({id:pair.id,round:round+1,ticks,baselineStepMs:0,candidateStepMs:0}));
      for(const pair of pairs){commandTrace(pair,'ordinary-reset',runtime=>runtime.reset());for(let i=0;i<300;i++)pairStep(pair,'ordinary-idle');
        if(round===0){const baseline=countViews(pair.baseline),candidate=countViews(pair.candidate);compare(pair,'view-getters');assert.equal(candidate.ctrl,1);assert.equal(candidate.qfrc_bias,1);getterCounts.push({id:pair.id,baseline,candidate});}
        commandTrace(pair,'start-blue',runtime=>assert.equal(runtime.start({objectId:'blue'}),true,runtime.controller.message));}
      for(let tick=0;tick<ticks;tick++)for(let index=0;index<pairs.length;index++)pairStep(pairs[index],`ordinary-${round+1}-${tick}`,rows[index],Boolean((tick+round)%2));
      for(let index=0;index<pairs.length;index++){
        const pair=pairs[index],row=rows[index];assert.deepEqual(pair.baseline.controller.history,pair.candidate.controller.history);assert.deepEqual(pair.baseline.controller.evidence,pair.candidate.controller.evidence);
        if(ticks>=32000){assert.equal(pair.baseline.controller.state,'complete');assert(pair.baseline.controller.evidence.dual);assert(pair.baseline.controller.evidence.lift>.035);}
        measurements.push({...row,reductionPercent:100*(1-row.candidateStepMs/row.baselineStepMs),state:pair.baseline.controller.state,evidence:{...pair.baseline.controller.evidence}});
      }
      console.log(JSON.stringify({round:round+1,complete:true,measurements:measurements.slice(-3)}));
    }
    for(const pair of pairs){
      commandTrace(pair,'reset-before-stop',runtime=>runtime.reset());for(let i=0;i<300;i++)pairStep(pair,'stop-idle');
      commandTrace(pair,'start-before-stop',runtime=>assert.equal(runtime.start({objectId:'blue'}),true));for(let i=0;i<2500;i++)pairStep(pair,'before-stop');
      commandTrace(pair,'stop',runtime=>runtime.stop());for(let i=0;i<100;i++)pairStep(pair,'stopped');
      commandTrace(pair,'reset-after-stop',runtime=>runtime.reset());for(let i=0;i<300;i++)pairStep(pair,'after-stop-reset');
      const previous=pair.candidate.dynamics,previousData=pair.candidate.data;
      for(const runtime of [pair.baseline,pair.candidate])assert.equal(runtime.configureTask('pipes',{seed:521}).ok,true);
      assert.notEqual(pair.candidate.dynamics,previous);assert.notEqual(pair.candidate.data,previousData);optimizeDynamics(pair.candidate.dynamics);compare(pair,'pipe-rebuild');
      for(let i=0;i<300;i++)pairStep(pair,'pipe-idle');
      if(pipeTicks){commandTrace(pair,'pipe-start',runtime=>assert.equal(runtime.start({task:'pipes'}),true,runtime.controller.message));
        for(let i=0;i<pipeTicks;i++)pairStep(pair,'pipe-running');commandTrace(pair,'pipe-stop',runtime=>runtime.stop());}
      for(const seed of [558,592]){
        const model=pair.candidate.model;commandTrace(pair,`pipe-reset-${seed}`,runtime=>runtime.reset({seed}));assert.equal(pair.candidate.model,model);
        for(let i=0;i<300;i++)pairStep(pair,`pipe-reset-${seed}-idle`);
        const fingerPose=Object.fromEntries([...pair.baseline.dynamics.map].filter(([,joint])=>joint.finger).map(([name,joint])=>[name,joint.lower]));
        commandTrace(pair,`pipe-${seed}-finger-command`,runtime=>runtime.dynamics.setTargets(fingerPose));for(let i=0;i<300;i++)pairStep(pair,`pipe-${seed}-finger-ramp`);
      }
      for(const runtime of [pair.baseline,pair.candidate])assert.equal(runtime.configureTask('sort').ok,true);
      optimizeDynamics(pair.candidate.dynamics);compare(pair,'legacy-rebuild');for(let i=0;i<300;i++)pairStep(pair,'legacy-rebuilt-idle');
      checks.push({name:`${pair.id}: full ordinary trajectory, mid-motion stop/reset, fresh pipe model, reused pipe seeds 521→558→592, finger ramps and legacy rebuild are bit-exact`,pass:true});
    }
  }
}catch(error){failure={message:error.message,stack:error.stack};}
finally{for(const runtime of runtimes)runtime.dispose();}
const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes);
const sums=measurements.reduce((sum,row)=>({baselineStepMs:sum.baselineStepMs+row.baselineStepMs,candidateStepMs:sum.candidateStepMs+row.candidateStepMs}),{baselineStepMs:0,candidateStepMs:0});
const report={pass:!failure&&sourceFrozen,startedAt,finishedAt:new Date().toISOString(),wallSeconds:(performance.now()-started)/1000,sourceFrozen,sourceHashes,endSourceHashes,
  mode:contractsOnly?'contracts-only':'paired-production-adapter',ticks,rounds,pipeTicks,exactTicks,checks,getterCounts,measurements,
  totals:{...sums,reductionPercent:sums.baselineStepMs?100*(1-sums.candidateStepMs/sums.baselineStepMs):null},failure,
  limitations:'Actual retained MuJoCo, original-source mesh-free URDF trees and original 1 ms motor formulas. Exact comparison covers every physical tick tested, not all possible trajectories. Node CPU timing is not browser FPS/GPU or rendered-mesh evidence. Stop/reset and partial pipe trajectory tests do not replace the established 72-pipe physical suite. No physical gate is weakened.'};
const destination=new URL(reportPath,root);await mkdir(new URL('.',destination),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));if(!report.pass)process.exitCode=1;
