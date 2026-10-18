import assert from 'node:assert/strict';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {performance} from 'node:perf_hooks';
import {Worker} from 'node:worker_threads';
import * as THREE from '../dist/vendor/three.module.min.js';
import createMuJoCo from '../dist/r1/mujoco-runtime.mjs';
import {ROBOT_CONFIGS,createRobotRuntimeFromParts} from '../dist/multi/robot-runtime.mjs';
import {createPlanningWorkerTransport} from '../dist/multi/planning-worker-transport.mjs';
import {encodePlanningValue,decodePlanningValue,capturePlanningJob,restorePlanningJob,applyPlanningResult} from '../dist/multi/planning-serialization.mjs';
import {parsePlanningRobot} from '../dist/multi/planning-robot.mjs';
import {createPlanningWorkerRobot} from '../dist/multi/planning-worker-core.mjs';
import {loadTestRobot as loadR1} from './r1-test-robot.mjs';
import {loadG1TestRobot as loadG1} from './g1-test-robot.mjs';
import {loadC5TestRobot as loadC5} from './c5-test-robot.mjs';

// Independent evidence for the actual background planner, not a timer mock
// presented as a frame-rate test. No retained acceptance report is overwritten.
const root=new URL('../',import.meta.url),args=process.argv.slice(2);
const contractsOnly=args.includes('--contracts-only');
const reportPath=args.find(arg=>arg.startsWith('--report='))?.slice(9)??'work/validation/background-planning/contracts-final.json';
const sourcePaths=[
  'work/test-background-planning.mjs','work/background-planning-node-worker.mjs',
  'dist/multi/planning-worker.mjs','dist/multi/planning-worker-core.mjs','dist/multi/planning-worker-transport.mjs',
  'dist/multi/planning-serialization.mjs','dist/multi/planning-robot.mjs','dist/multi/planning-check-cache.mjs','dist/multi/planning.mjs',
  'dist/multi/background-planning.mjs',
  'dist/multi/robot-runtime.mjs','dist/multi/pipe-controller.mjs','dist/multi/pipe-layout.mjs','dist/multi/pipe-model.mjs','dist/multi/r1-safe-dynamics.mjs',
  ...['r1','g1','c5'].flatMap(id=>[`work/${id}-test-robot.mjs`,`dist/${id}/grasp-controller.mjs`,`dist/${id}/grasp-kinematics.mjs`,
    `dist/${id}/${id}-dynamics.mjs`,`dist/${id}/models/${id}-dynamic-scene.xml`]),
  'dist/r1/assets/r1.urdf','dist/g1/assets/g1_29dof_with_dex1_1.urdf','dist/c5/assets/c5-dex1.urdf',
  'dist/g1/g1-profile.mjs','dist/c5/c5-profile.mjs','dist/vendor/three.module.min.js','dist/r1/mujoco-runtime.mjs',
  'dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',
].sort();
const sha=buffer=>createHash('sha256').update(buffer).digest('hex');
const hashes=async()=>Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,sha(await readFile(new URL(path,root)))])));
const sourceHashes=await hashes(),startedAt=new Date().toISOString(),started=performance.now();
const checks=[],measurements=[],runtimes=[],workers=new Set();let failure=null;
async function check(name,run){
  try{const details=await run();checks.push({name,pass:true,...(details?{details}:{})});}
  catch(error){checks.push({name,pass:false,error:error.message});throw error;}
}
class FakeWorker{
  constructor(){this.messages=[];this.terminated=0;}
  postMessage(message){this.messages.push(structuredClone(message));}
  terminate(){this.terminated++;}
  emit(message){this.onmessage?.({data:structuredClone(message)});}
  complete(result={ok:true,payload:{example:true}}){const request=this.messages.at(-1);this.emit({type:'result',jobId:request.jobId,generation:request.generation,result});}
}
function fakeTransport(options={}){
  const instances=[];let clock=0;
  const transport=createPlanningWorkerTransport({workerFactory:()=>{const worker=new FakeWorker();instances.push(worker);return worker;},now:()=>clock,...options});
  return{transport,instances,setClock:value=>{clock=value;}};
}
function physical(runtime){return{qpos:[...runtime.data.qpos],qvel:[...runtime.data.qvel],time:runtime.data.time};}
function planFields(controller){
  return encodePlanningValue(Object.fromEntries(['planData','pose','crouch','opening','openings','continuing','running','state','elapsed','from','phaseGoal','evidence','history','placeLowering','placePose','planError'].filter(key=>controller[key]!==undefined).map(key=>[key,controller[key]])));
}
const wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));
function createNodeWorker(){
  const worker=new Worker(new URL('./background-planning-node-worker.mjs',import.meta.url),{type:'module'});workers.add(worker);
  const adapter={postMessage:message=>worker.postMessage(message),terminate:()=>{workers.delete(worker);return worker.terminate();},onmessage:null,onerror:null,onmessageerror:null};
  worker.on('message',message=>adapter.onmessage?.({data:message}));
  worker.on('error',error=>adapter.onerror?.({message:error.message,preventDefault(){}}));
  worker.on('messageerror',()=>adapter.onmessageerror?.({}));
  return adapter;
}
async function awaitResult(transport,{runtime,timeoutMs=90000}={}){
  const before=runtime?physical(runtime):null,started=performance.now();let result;
  do{
    result=transport.poll();
    if(runtime)assert.deepEqual(physical(runtime),before,'A worker edited the authoritative physical state');
    if(!result.pending)break;
    assert(performance.now()-started<timeoutMs,'Worker fixture exceeded its finite deadline');
    await wait(10);
  }while(true);
  return result;
}
try{
  await check('Vector3 and Quaternion prototypes survive nested structured-clone payloads',()=>{
    const value={start:new THREE.Vector3(1,2,3),orientation:new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),.3),route:[{point:new THREE.Vector3(4,5,6)}]};
    const restored=decodePlanningValue(structuredClone(encodePlanningValue(value)));
    assert(restored.start instanceof THREE.Vector3);assert(restored.orientation instanceof THREE.Quaternion);assert(restored.route[0].point instanceof THREE.Vector3);
    assert.deepEqual(encodePlanningValue(restored),encodePlanningValue(value));assert.equal(restored.start.distanceTo(value.start),0);
  });
  await check('Malformed vector tags, functions and prototype injection are rejected',()=>{
    for(const input of [{__robotPlanningType:'Vector3',value:[1,2,NaN]},{__robotPlanningType:'Quaternion',value:[0,0,1]},{__robotPlanningType:'Function',value:[]},JSON.parse('{"__proto__":{"polluted":true}}')])assert.throws(()=>decodePlanningValue(input));
    assert.throws(()=>encodePlanningValue({callback(){}}));assert.equal({}.polluted,undefined);
  });
  await check('Cancel terminates synchronous worker work and invalidates a queued old result',()=>{
    const {transport,instances}=fakeTransport();transport.submit({request:'first'});const first=instances[0],old=first.messages[0];
    assert.equal(transport.poll().pending,true);const before=performance.now();assert.equal(transport.cancel(),true);assert(performance.now()-before<25);assert.equal(first.terminated,1);
    assert.equal(transport.poll().error.type,'cancelled');transport.submit({request:'second'});const second=instances[1];
    first.emit({type:'result',jobId:old.jobId,generation:old.generation,result:{ok:true,payload:{stale:true}}});assert.equal(transport.poll().pending,true);
    second.complete({ok:true,payload:{fresh:true}});assert.deepEqual(transport.poll().payload,{fresh:true});transport.dispose();
  });
  await check('A wrong request id or generation cannot finish the active request',()=>{
    const {transport,instances}=fakeTransport();transport.submit({});const worker=instances[0],request=worker.messages[0];
    for(const identity of [{jobId:request.jobId+1,generation:request.generation},{jobId:request.jobId,generation:request.generation+1}]){
      worker.emit({type:'result',...identity,result:{ok:true}});assert.equal(transport.poll().pending,true);
    }
    worker.complete();assert.equal(transport.poll().ok,true);transport.dispose();
  });
  await check('A cached idle worker that crashes is replaced before the next request',()=>{
    const {transport,instances}=fakeTransport();transport.submit({});instances[0].complete();assert.equal(transport.poll().ok,true);
    instances[0].onerror({preventDefault(){}});assert.equal(instances[0].terminated,1);transport.submit({retry:true});
    assert.equal(instances.length,2);instances[1].complete();assert.equal(transport.poll().ok,true);transport.dispose();
  });
  await check('Worker crash, message error and constructor failure fail without main-thread fallback',()=>{
    for(const kind of ['crash','message']){
      const {transport,instances}=fakeTransport();transport.submit({});const worker=instances[0];
      if(kind==='crash')worker.onerror({preventDefault(){}});else worker.onmessageerror({});
      assert.equal(transport.poll().ok,false);assert.equal(worker.terminated,1);transport.submit({retry:true});instances[1].complete();assert.equal(transport.poll().ok,true);transport.dispose();
    }
    const transport=createPlanningWorkerTransport({workerFactory(){throw new Error('Worker unsupported');}});transport.submit({});assert.equal(transport.poll().ok,false);assert.equal(transport.poll().error.type,'worker');transport.dispose();
  });
  await check('A finite watchdog terminates a stalled worker and the next request can succeed',()=>{
    const {transport,instances,setClock}=fakeTransport({timeoutMs:100});transport.submit({});setClock(101);const result=transport.poll();
    assert.equal(result.ok,false);assert.equal(result.error.type,'timeout');assert.equal(instances[0].terminated,1);
    transport.submit({});instances[1].complete();assert.equal(transport.poll().ok,true);transport.dispose();assert.throws(()=>transport.submit({}));
  });
  await check('Progress reports elapsed wall time and candidate counts without completing a request',()=>{
    const {transport,instances,setClock}=fakeTransport();transport.submit({});const worker=instances[0],request=worker.messages[0];setClock(50);
    worker.emit({type:'progress',jobId:request.jobId,generation:request.generation,progress:{stage:'checking',steps:7}});
    const state=transport.poll();assert.equal(state.pending,true);assert.equal(state.progress.steps,7);assert.equal(state.progress.elapsedMs,50);transport.dispose();
  });
  await check('Worker URDF parser retains all source joint transforms at legal sample poses',async()=>{
    const loaders={r1:loadR1,g1:loadG1,c5:loadC5};
    for(const id of ['r1','g1','c5']){
      const source=await readFile(new URL(ROBOT_CONFIGS[id].urdf),'utf8'),worker=createPlanningWorkerRobot(source,{rootHeight:ROBOT_CONFIGS[id].rootHeight}),reference=await loaders[id]();
      assert.deepEqual(Object.keys(worker.joints).sort(),Object.keys(reference.joints).sort());
      for(const factor of [.1,.5,.9]){
        const pose=Object.fromEntries(Object.entries(worker.joints).filter(([,joint])=>joint.jointType!=='fixed').map(([name,joint])=>[name,joint.limit.lower+(joint.limit.upper-joint.limit.lower)*factor]));
        worker.setJointValues(pose);reference.setJointValues(pose);worker.updateMatrixWorld(true);reference.updateMatrixWorld(true);
        for(const [name,joint]of Object.entries(worker.joints))for(let index=0;index<16;index++)assert(Math.abs(joint.matrixWorld.elements[index]-reference.joints[name].matrixWorld.elements[index])<1e-10,`${id}/${name} world transform differs`);
      }
    }
  });
  await check('Worker-only exact joint reuse preserves default parser behavior and every world matrix',async()=>{
    for(const id of ['r1','g1','c5']){
      const source=await readFile(new URL(ROBOT_CONFIGS[id].urdf),'utf8'),options={rootHeight:ROBOT_CONFIGS[id].rootHeight};
      const worker=createPlanningWorkerRobot(source,options),original=parsePlanningRobot(source,options);
      for(const factor of [.1,.5,.5,.9,.9,.1]){
        const pose=Object.fromEntries(Object.entries(worker.joints).filter(([,joint])=>joint.jointType!=='fixed').map(([name,joint])=>[name,joint.limit.lower+(joint.limit.upper-joint.limit.lower)*factor]));
        worker.setJointValues(pose);original.setJointValues(pose);worker.updateMatrixWorld(true);original.updateMatrixWorld(true);
        for(const [name,joint]of Object.entries(worker.joints)){
          assert.deepEqual(joint.jointValue,original.joints[name].jointValue);
          assert.deepEqual(joint.quaternion.toArray(),original.joints[name].quaternion.toArray());
          assert.deepEqual(joint.matrixWorld.elements,original.joints[name].matrixWorld.elements);
        }
      }
      const name=Object.keys(worker.joints).find(name=>worker.joints[name].jointType==='revolute'&&worker.joints[name].limit.lower<0&&worker.joints[name].limit.upper>0);
      assert(name);const optimized=worker.joints[name],legacy=original.joints[name];let workerCopies=0,legacyCopies=0;
      const wc=optimized.quaternion.copy,lc=legacy.quaternion.copy;
      optimized.quaternion.copy=function(...values){workerCopies++;return wc.apply(this,values);};
      legacy.quaternion.copy=function(...values){legacyCopies++;return lc.apply(this,values);};
      optimized.setValue(0);optimized.setValue(0);legacy.setValue(0);legacy.setValue(0);
      assert.equal(workerCopies,1);assert.equal(legacyCopies,2,'Default parser must retain its original repeated-assignment semantics');
      optimized.setValue(-0);legacy.setValue(-0);assert.equal(workerCopies,2);assert(Object.is(optimized.jointValue[0],-0));
      assert.deepEqual(optimized.quaternion.toArray(),legacy.quaternion.toArray());
      assert.throws(()=>optimized.setValue(Infinity));assert.throws(()=>legacy.setValue(Infinity));
    }
  });
  if(!contractsOnly){
    const api=await createMuJoCo({locateFile:()=>new URL('../dist/Rebot_Arm_AGV/assets/mujoco-Bp43jdDU.wasm',import.meta.url).pathname});
    const loaders={r1:loadR1,g1:loadG1,c5:loadC5};
    async function fixture(id,options={}){
      const sourceXml=await readFile(new URL(ROBOT_CONFIGS[id].xml),'utf8'),vfs=new api.MjVFS();let model,data;
      try{model=api.MjModel.from_xml_string(sourceXml,vfs);data=new api.MjData(model);const robot=await loaders[id](),runtime=createRobotRuntimeFromParts({id,api,model,data,robot,sourceXml,cooperativePlanning:true,...options});runtimes.push(runtime);return runtime;}
      catch(error){data?.delete();model?.delete();throw error;}finally{vfs.delete();}
    }
    const acceptedResults=new Map();
    for(const id of ['r1','g1','c5'])await check(`${id}: background plan and command state match the production route exactly`,async()=>{
      const reference=await fixture(id),target=reference.targets.find(item=>item.id==='blue'),controller=reference.controller;
      // A high comparison allowance avoids turning slow CI scheduling into an
      // altered search policy; actual production budgets are checked separately.
      controller.planningBudgetMs=90000;
      const job=capturePlanningJob(reference,{arm:reference.config.defaultArm,id:target.id,continuing:false});
      assert.equal(reference.start({objectId:'blue'}),true);while(controller.planning)controller.advancePlanning();assert(controller.planData);
      const expected=planFields(controller),other=await fixture(id),transport=createPlanningWorkerTransport({workerFactory:createNodeWorker,timeoutMs:90000});
      other.controller.planningBudgetMs=90000;transport.submit(job);const result=await awaitResult(transport,{runtime:other});assert.equal(result.ok,true,result.reason);
      acceptedResults.set(id,result);applyPlanningResult(other.controller,result.payload);assert.deepEqual(planFields(other.controller),expected);transport.dispose();
      for(const runtime of [reference,other]){runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);}
    });
    for(const id of ['r1','g1','c5'])await check(`${id}: a moving snapshot preserves sampled derived object poses and the exact accepted route`,async()=>{
      const reference=await fixture(id),isolated=await fixture(id),controller=reference.controller;
      for(let tick=0;tick<12;tick++)reference.step(.001);
      assert([...reference.data.qvel].some(value=>Math.abs(value)>1e-8),'Snapshot did not include non-zero actual velocity');
      controller.planningBudgetMs=90000;
      const job=capturePlanningJob(reference,{arm:reference.config.defaultArm,id:'blue',continuing:false});
      const sampled={xpos:[...reference.data.xpos],xquat:[...reference.data.xquat]},state=physical(reference);
      restorePlanningJob(isolated,job);assert.deepEqual(physical(isolated),state);
      for(const name of ['xpos','xquat'])assert.deepEqual([...isolated.data[name]],sampled[name],`${name} lost the source mj_step sample semantics`);
      for(const target of reference.targets)assert.deepEqual(encodePlanningValue(isolated.controller.objectPose(target.id)),encodePlanningValue(controller.objectPose(target.id)));
      // This must be a real lagged MuJoCo read-cache case, not a stationary
      // fixture that happens to pass with a fresh mj_forward recomputation.
      isolated.api.mj_forward(isolated.model,isolated.data);
      const freshDifference=Math.max(...sampled.xpos.map((value,index)=>Math.abs(value-isolated.data.xpos[index])));
      assert(freshDifference>1e-12,`${id}: fixture did not expose the integration/read-cache offset`);
      assert.equal(reference.start({objectId:'blue'}),true);while(controller.planning)controller.advancePlanning();assert(controller.planData);
      const expected=planFields(controller),transport=createPlanningWorkerTransport({workerFactory:createNodeWorker,timeoutMs:90000});
      transport.submit(job);const result=await awaitResult(transport);assert.equal(result.ok,true,result.reason);
      restorePlanningJob(isolated,job);applyPlanningResult(isolated.controller,result.payload);assert.deepEqual(planFields(isolated.controller),expected);
      assert.deepEqual(physical(reference),state,'Synchronous planning edited the input physical state');
      assert.deepEqual(physical(isolated),state,'Worker result application edited the mirrored physical state');
      measurements.push({scenario:`${id}-moving-snapshot-derived-and-plan-equivalence`,freshDerivedDifferenceM:freshDifference,candidateSlices:result.steps});transport.dispose();
      for(const runtime of [reference,isolated]){runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);}
    });
    for(const id of ['r1','g1'])await check(`${id}: reused compiled pipe worlds preserve exact plans across seeds and task rebuilds`,async()=>{
      const runtime=await fixture(id),transport=createPlanningWorkerTransport({workerFactory:createNodeWorker,timeoutMs:90000}),scenarios=[];
      assert.equal(runtime.configureTask('pipes',{seed:521}).ok,true);
      try{
        for(const scenario of [{seed:558,modelSeed:521,rebuild:false},{seed:592,modelSeed:521,rebuild:false},{seed:558,modelSeed:558,rebuild:true}]){
          runtime.stop();if(scenario.rebuild)assert.equal(runtime.configureTask('sort').ok,true);
          assert.equal(runtime.configureTask('pipes',{seed:scenario.seed}).ok,true);for(let tick=0;tick<300;tick++)runtime.step(.001);runtime.syncVisuals();
          const controller=runtime.controller;let job;const original=controller.prepareItem.bind(controller);
          controller.prepareItem=(arm,objectId,continuing)=>{
            controller.arm=arm;controller.selected=objectId;job=capturePlanningJob(runtime,{arm,id:objectId,continuing});return original(arm,objectId,continuing);
          };
          try{assert.equal(runtime.start({task:'pipes'}),true);}finally{controller.prepareItem=original;}
          const decoded=decodePlanningValue(job);assert.equal(decoded.pipeSeed,scenario.seed);assert.equal(decoded.modelPipeSeed,scenario.modelSeed);
          assert.equal(typeof decoded.modelXml,'string');assert(decoded.modelXml.includes('<worldbody>'));
          // Use the unchanged production controller budgets in these original
          // failure reproductions. A larger watchdog is not an IK allowance.
          assert.equal(controller.planningBudgetMs,undefined);
          const before=physical(runtime);while(controller.planning)controller.advancePlanning();assert(controller.planData,controller.message);
          const expected=planFields(controller);transport.submit(job);const result=await awaitResult(transport);assert.equal(result.ok,true,result.reason);
          const actual=decodePlanningValue(result.payload),expectedDecoded=decodePlanningValue(expected);
          for(const name of ['planData','crouch','opening','openings','continuing','running','state','elapsed','from','phaseGoal','evidence','placeLowering','placePose','planError'])
            if(actual[name]!==undefined||expectedDecoded[name]!==undefined)assert.deepEqual(encodePlanningValue(actual[name]),encodePlanningValue(expectedDecoded[name]),`${id}/${scenario.seed}/${name} differs across compiled model reuse`);
          assert.deepEqual(physical(runtime),before);scenarios.push({...scenario,selected:controller.selected,steps:result.steps});
        }
      }finally{transport.dispose();runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);}
      measurements.push({scenario:`${id}-cross-seed-compiled-order-equivalence`,productionBudgetOverride:false,cases:scenarios});
    });
    for(const id of ['r1','g1','c5'])await check(`${id}: actual background runtime only submits/polls and rejects changed state`,async()=>{
      const fake=fakeTransport(),runtime=await fixture(id,{backgroundPlanning:true,planningTransportFactory:()=>fake.transport}),controller=runtime.controller;
      controller.plan=()=>{throw new Error('The main thread called its IK route planner');};
      assert.equal(runtime.start({objectId:'blue'}),true);const state=physical(runtime),worker=fake.instances[0];
      assert.equal(worker.messages.length,1);assert.equal(runtime.advancePlanning().pending,true);assert.deepEqual(physical(runtime),state);
      // Deliberately mutate a test world through direct APIs: none of these
      // otherwise valid completed routes may be installed for an old snapshot.
      const destinationX=runtime.targets.find(target=>target.id==='blue').drop.x,edits=[
        {name:'qpos',run(){runtime.data.qpos[0]+=.00001;},restore(){}},
        {name:'destination',run(){runtime.targets.find(target=>target.id==='blue').drop.x+=.01;},restore(){runtime.targets.find(target=>target.id==='blue').drop.x=destinationX;}},
        {name:'opening',run(){controller.opening+=.001;},restore(){}},
        {name:'motor target',run(){const joint=Object.keys(runtime.dynamics.targets)[0];runtime.dynamics.targets[joint]+=.001;},restore(){}},
      ];
      for(const [index,edit]of edits.entries()){
        if(index){runtime.reset();assert.equal(runtime.start({objectId:'blue'}),true);}
        edit.run();fake.instances.at(-1).complete(acceptedResults.get(id));const result=runtime.advancePlanning();
        assert.equal(result.ok,false,`${edit.name}: old route was installed`);assert.equal(result.error.type,'stale');assert.equal(controller.planData,null);edit.restore();
      }
      runtime.reset();assert.equal(runtime.start({objectId:'blue'}),true);fake.instances.at(-1).complete(acceptedResults.get(id));
      const recovered=runtime.advancePlanning();assert.equal(recovered.ok,true);assert(controller.planData);assert.notEqual(controller.state,'planning');
      runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);
    });
    await check('Reset cancels an actual runtime request and an old queued result cannot revive it',async()=>{
      const fake=fakeTransport(),runtime=await fixture('g1',{backgroundPlanning:true,planningTransportFactory:()=>fake.transport});
      assert.equal(runtime.start({objectId:'blue'}),true);const first=fake.instances[0],message=first.messages[0];runtime.reset();
      assert.equal(first.terminated,1);assert.equal(runtime.planning,false);assert.equal(runtime.controller.state,'idle');
      assert.equal(runtime.start({objectId:'blue'}),true);first.emit({type:'result',jobId:message.jobId,generation:message.generation,result:acceptedResults.get('g1')});
      assert.equal(runtime.advancePlanning().pending,true);fake.instances[1].complete(acceptedResults.get('g1'));assert.equal(runtime.advancePlanning().ok,true);
      runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);
    });
    await check('A real hard G1 pipe route runs on a worker while the main event-loop heartbeat continues',async()=>{
      const runtime=await fixture('g1');assert.equal(runtime.configureTask('pipes',{seed:592}).ok,true);runtime.controller.planningBudgetMs=90000;
      const id=runtime.controller.pipeLayout.queue[0],job=capturePlanningJob(runtime,{arm:'right',id,continuing:false}),transport=createPlanningWorkerTransport({workerFactory:createNodeWorker,timeoutMs:90000});
      let heartbeats=0,maxGapMs=0,last=performance.now();const timer=setInterval(()=>{const now=performance.now();maxGapMs=Math.max(maxGapMs,now-last);last=now;heartbeats++;},10);
      const before=physical(runtime),start=performance.now();let result;
      try{transport.submit(job);result=await awaitResult(transport,{runtime});}
      finally{clearInterval(timer);transport.dispose();}
      const elapsedMs=performance.now()-start;assert.equal(result.ok,true,result.reason);assert.deepEqual(physical(runtime),before);
      assert(elapsedMs>200,'Fixture did not exercise substantial production route computation');assert(heartbeats>=Math.floor(elapsedMs/100),'Main event loop did not continue receiving timer callbacks');
      assert(maxGapMs<Math.max(250,elapsedMs*.2),`Main event loop was blocked for ${maxGapMs.toFixed(1)} ms`);
      measurements.push({scenario:'g1-pipes-592-first-route',elapsedMs,heartbeats,maxGapMs,candidateSlices:result.steps,computationSegmentMs:result.segmentElapsedMs,scope:'Node worker_threads CPU/event-loop isolation, not browser FPS'});
      runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);
    });
    await check('The real background runtime can cancel a hard pipe search and start a fresh request',async()=>{
      const runtime=await fixture('g1',{backgroundPlanning:true,planningTransportFactory:()=>createPlanningWorkerTransport({workerFactory:createNodeWorker,timeoutMs:90000})});
      assert.equal(runtime.configureTask('pipes',{seed:592}).ok,true);assert.equal(runtime.start({task:'pipes'}),true);
      await wait(300);assert.equal(runtime.advancePlanning().pending,true);const before=physical(runtime),started=performance.now();
      assert.equal(runtime.controller.cancelPlanning('取消困难路径测试'),true);const cancelMs=performance.now()-started;
      assert(cancelMs<50,`Cancel waited for the worker's current IK computation: ${cancelMs} ms`);assert.equal(runtime.planning,false);assert.equal(runtime.controller.state,'stopped');assert.deepEqual(physical(runtime),before);
      runtime.reset({seed:521});assert.equal(runtime.start({task:'pipes'}),true);
      const initial=physical(runtime),retryStarted=performance.now();while(runtime.planning){const result=runtime.advancePlanning();assert.deepEqual(physical(runtime),initial);if(result.done)assert.equal(result.ok,true,result.reason);assert(performance.now()-retryStarted<90000,'Retry exceeded its finite deadline');if(runtime.planning)await wait(10);}
      assert(runtime.controller.planData);measurements.push({scenario:'g1-real-runtime-cancel-and-pipe-retry',cancelMs});
      runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);
    });
    await check('A real unreachable target fails through the worker and reset permits a normal retry',async()=>{
      const runtime=await fixture('g1',{backgroundPlanning:true,planningTransportFactory:()=>createPlanningWorkerTransport({workerFactory:createNodeWorker,timeoutMs:90000})});
      runtime.controller.setObject('blue',new THREE.Vector3(2.5,runtime.config.tableHeight+.019,0));
      const before=physical(runtime),started=performance.now();assert.equal(runtime.start({objectId:'blue'}),true);let result;
      while(runtime.planning){result=runtime.advancePlanning();assert.deepEqual(physical(runtime),before);assert(performance.now()-started<15000,'An obviously unreachable target spent too long in candidate search');if(runtime.planning)await wait(10);}
      assert.equal(result.ok,false);assert.equal(result.error.type,'unreachable');assert.equal(runtime.controller.planData,null);
      const rejectionMs=performance.now()-started;runtime.reset();assert.equal(runtime.start({objectId:'blue'}),true);const resetState=physical(runtime),retryStarted=performance.now();
      while(runtime.planning){result=runtime.advancePlanning();assert.deepEqual(physical(runtime),resetState);assert(performance.now()-retryStarted<90000,'A normal retry did not finish');if(runtime.planning)await wait(10);}
      assert.equal(result.ok,true,result.reason);assert(runtime.controller.planData);measurements.push({scenario:'g1-real-runtime-unreachable-and-reset-retry',rejectionMs});
      runtime.dispose();runtimes.splice(runtimes.indexOf(runtime),1);
    });
  }
}catch(error){failure={message:error.message,stack:error.stack};}
finally{for(const worker of workers)await worker.terminate();for(const runtime of runtimes)runtime.dispose();}
const endSourceHashes=await hashes(),sourceFrozen=JSON.stringify(sourceHashes)===JSON.stringify(endSourceHashes);
const pass=!failure&&sourceFrozen&&checks.every(item=>item.pass);
const report={pass,startedAt,finishedAt:new Date().toISOString(),wallSeconds:(performance.now()-started)/1000,mode:contractsOnly?'contracts-only':'full',checks,checkCount:checks.length,measurements,failure,sourceFrozen,sourceHashes,endSourceHashes,
  scope:'Actual background planning with original production URDF, collision checks, MuJoCo and planner. Worker heartbeat validates event-loop isolation, not browser FPS. No physics gate is relaxed and no historical report is rebound.'};
const destination=new URL(reportPath,root);await mkdir(new URL('.',destination),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));if(!pass)process.exitCode=1;
