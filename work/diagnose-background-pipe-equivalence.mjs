import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {performance} from 'node:perf_hooks';
import {Worker} from 'node:worker_threads';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {ROBOT_CONFIGS,createRobotRuntimeFromParts} from '../dist/multi/robot-runtime.mjs';
import {parsePlanningRobot} from '../dist/multi/planning-robot.mjs';
import {createPlanningWorkerTransport} from '../dist/multi/planning-worker-transport.mjs';
import {capturePlanningJob,encodePlanningValue} from '../dist/multi/planning-serialization.mjs';
import {loadTestRobot as loadR1} from './r1-test-robot.mjs';
import {loadG1TestRobot as loadG1} from './g1-test-robot.mjs';
const args=process.argv.slice(2),id=args.find(arg=>arg.startsWith('--robot='))?.slice(8)??'g1';
const seed=Number(args.find(arg=>arg.startsWith('--seed='))?.slice(7)??558);
const api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
const sourceXml=await readFile(new URL(ROBOT_CONFIGS[id].xml),'utf8'),urdf=await readFile(new URL(ROBOT_CONFIGS[id].urdf),'utf8'),loaders={r1:loadR1,g1:loadG1};
const runtimes=[],results=[];let encodedJob;
async function runtime(productionParser=false){
  const vfs=new api.MjVFS();let model,data;
  try{
    model=api.MjModel.from_xml_string(sourceXml,vfs);data=new api.MjData(model);
    const robot=productionParser?parsePlanningRobot(urdf,{rootHeight:ROBOT_CONFIGS[id].rootHeight}):await loaders[id]();
    const result=createRobotRuntimeFromParts({id,api,model,data,robot,sourceXml,cooperativePlanning:true});runtimes.push(result);
    assert.equal(result.configureTask('pipes',{seed}).ok,true);for(let tick=0;tick<300;tick++)result.step(.001);result.syncVisuals();
    result.controller.planningBudgetMs=120000;return result;
  }catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
}
function summarize(c){return{state:c.state,planError:c.planError,history:c.history,arm:c.arm,selected:c.selected,planData:encodePlanningValue(c.planData)};}
function drain(c){let result;const start=performance.now();while(c.planning)result=c.advancePlanning();return{...result,wallMs:performance.now()-start,...summarize(c)};}
function firstDifference(a,b,path=''){
  if(typeof a!==typeof b||a===null||b===null)return a===b?null:{path,a,b};
  if(typeof a!=='object')return Object.is(a,b)?null:{path,a,b};
  for(const key of new Set([...Object.keys(a),...Object.keys(b)])){const found=firstDifference(a[key],b[key],`${path}.${key}`);if(found)return found;}return null;
}
let worker,transport;
try{
  for(const productionParser of [false,true]){
    const r=await runtime(productionParser),c=r.controller,original=c.prepareItem.bind(c);
    c.prepareItem=(arm,objectId,continuing)=>{
      if(!encodedJob)encodedJob=capturePlanningJob(r,{arm,id:objectId,continuing});return original(arm,objectId,continuing);
    };
    assert.equal(r.start({task:'pipes'}),true);const result=drain(c);results.push({mode:productionParser?'production-parser-cooperative':'legacy-fixture-cooperative',...result});
    console.error(JSON.stringify({mode:results.at(-1).mode,ok:result.ok,steps:result.steps,wallMs:result.wallMs,reason:result.reason,selected:c.selected}));
  }
  worker=new Worker(new URL('./background-planning-node-worker.mjs',import.meta.url),{type:'module'});
  const adapter={postMessage:value=>worker.postMessage(value),terminate:()=>worker.terminate()};worker.on('message',data=>adapter.onmessage?.({data}));worker.on('error',error=>adapter.onerror?.({message:error.message,preventDefault(){}}));
  transport=createPlanningWorkerTransport({workerFactory:()=>adapter,timeoutMs:150000});transport.submit(encodedJob);let result;const start=performance.now();
  do{result=transport.poll();if(result.pending)await new Promise(resolve=>setTimeout(resolve,10));}while(result.pending);
  results.push({mode:'production-worker',...result,wallMs:performance.now()-start,planData:result.payload?.planData});
  const first=results[0].planData;for(const row of results.slice(1))row.firstDifferenceFromLegacy=firstDifference(first,row.planData);
  console.error(JSON.stringify({mode:'production-worker',ok:result.ok,steps:result.steps,wallMs:results.at(-1).wallMs,reason:result.reason,firstDifferenceFromLegacy:results.at(-1).firstDifferenceFromLegacy}));
}finally{transport?.dispose();if(worker)await worker.terminate();for(const r of runtimes)r.dispose();}
const report={robot:id,seed,idleTicks:300,productionBudgetOverrideMs:120000,results};
await mkdir(new URL('./validation/background-planning/',import.meta.url),{recursive:true});await writeFile(new URL(`./validation/background-planning/equivalence-${id}-${seed}-development.json`,import.meta.url),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({robot:id,seed,results:results.map(({planData,payload,...row})=>row)},null,2));
