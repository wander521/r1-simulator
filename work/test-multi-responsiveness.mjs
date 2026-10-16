import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFile,mkdir,writeFile} from 'node:fs/promises';
import * as THREE from '../dist/vendor/three.module.min.js';
import {MultiRobotCoordinator} from '../dist/multi/coordinator.mjs';
import {createFrameScheduler,cameraFeedVisible,shouldRefitCamera} from '../dist/multi/visuals/frame-scheduler.mjs';

// These are declared station/clock fixtures. They execute the production
// coordinator and scheduling/view policies, not robot contacts, a browser,
// real GPU rendering or successful physical manipulation.
const root=new URL('../',import.meta.url),ids=['r1','g1','c5'];
const reportPath=process.argv.find(arg=>arg.startsWith('--report='))?.slice(9)
  ??'work/validation/multi-responsive/contracts-final.json';
const sourcePaths=['work/test-multi-responsiveness.mjs','dist/multi/coordinator.mjs','dist/multi/app.mjs','dist/multi/visuals/frame-scheduler.mjs',
  'dist/vendor/three.module.min.js','dist/vendor/three.core.min.js'];
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const hashes=async()=>Object.fromEntries(await Promise.all(sourcePaths.map(async path=>[path,sha(await readFile(new URL(path,root)))])));
const sourceHashes=await hashes(),checks=[],errors=[];
const appText=await readFile(new URL('dist/multi/app.mjs',root),'utf8');
function check(name,fn){try{const detail=fn();checks.push({name,pass:true,...(detail===undefined?{}:{detail})});}
  catch(error){checks.push({name,pass:false,error:error.message});errors.push({name,message:error.message});}}

function station(id,{slices=2,accept=true,reject=false,throwOnAdvance=false,omitReason=false,nextAt=null}={}){
  let job=null,generation=0;
  const s={id,ready:true,config:{id,label:id.toUpperCase(),defaultArm:'right'},group:{visible:true},
    controller:{running:false,state:'idle',message:'就绪'},data:{time:0},objectPosition:[.5,.75,0],
    calls:{start:[],advance:0,step:[],stop:0,reset:0,cancel:0,stale:0},
    get planning(){return Boolean(job);},
    get snapshot(){return {id,time:this.data.time,state:this.controller.state,running:this.controller.running,planning:this.planning,message:this.controller.message};},
    start(options){
      this.calls.start.push(options);
      if(!accept){this.controller.state='failed';this.controller.message='测试目标不可达';return false;}
      this.controller.running=true;
      if(slices>0)this.prepare(slices,{reject,throwOnAdvance,omitReason});else this.controller.state='approach';
      return true;
    },
    prepare(count,options={}){job={remaining:count,generation:++generation,...options};this.controller.state='planning';this.controller.running=true;return generation;},
    advancePlanning(){
      this.calls.advance++;
      if(!job)return {pending:false,done:true,ok:this.controller.state!=='failed'};
      if(job.throwOnAdvance)throw new Error('测试路径检查异常');
      if(--job.remaining>0)return {pending:true,done:false,ok:true};
      const failed=job.reject,noReason=job.omitReason;job=null;
      this.controller.state=failed?'failed':'approach';this.controller.running=!failed;
      if(failed){this.controller.message='测试分段目标不可达';return {pending:false,done:true,ok:false,...(noReason?{}:{reason:this.controller.message})};}
      return {pending:false,done:true,ok:true};
    },
    completeStale(ticket){this.calls.stale++;if(!job||job.generation!==ticket)return false;job=null;this.controller.running=true;this.controller.state='approach';return true;},
    step(dt){this.calls.step.push(dt);this.data.time+=dt;if(nextAt===this.calls.step.length){nextAt=null;this.prepare(2);}},
    stop(){this.calls.stop++;if(job)this.calls.cancel++;job=null;generation++;this.controller.running=false;this.controller.state='stopped';this.controller.message='测试已停止';},
    reset(){this.calls.reset++;job=null;generation++;this.data.time=0;this.controller.running=false;this.controller.state='idle';},
  };
  return s;
}
function fixture(options={}){
  const stations=new Map(ids.map(id=>[id,station(id,options[id])])),c=new MultiRobotCoordinator({stations});return {c,stations};
}
const advances=stations=>[...stations.values()].reduce((sum,s)=>sum+s.calls.advance,0);
const positions=stations=>[...stations.values()].map(s=>[...s.objectPosition]);
function drain(c,stations,{limit=100}={}){
  let result,slices=0;
  while(c.planning){
    const before=advances(stations),tick=c.tick,times=[...stations.values()].map(s=>s.data.time);
    result=c.advancePlanning();slices++;
    assert.equal(advances(stations)-before,1,'One coordinator planning call must advance at most one station candidate');
    assert.equal(c.tick,tick,'Candidate planning advanced the shared tick');
    assert.deepEqual([...stations.values()].map(s=>s.data.time),times,'Candidate planning advanced a physical clock');
    assert(slices<limit,'Pending preparation did not terminate');
  }
  return {result,slices};
}

check('Staged start freezes every world and commits one shared batch only after every selected plan accepts',()=>{
  const {c,stations}=fixture({r1:{slices:3},g1:{slices:1},c5:{slices:2}});
  for(let i=0;i<7;i++)c.step();const before=c.snapshot,result=c.start({task:'sort',objectId:'blue'});
  assert.equal(result.ok,true);assert.equal(result.preparing,true);assert(c.planning);assert.equal(c.state,'preparing');
  assert.equal(c.startTick,null);assert.deepEqual(c.startTimes,{});assert.equal(c.batchNumber,0);assert.equal(c.tick,before.tick);
  assert.throws(()=>c.step(),/规划|路径|planning/i);assert.equal(c.tick,before.tick);
  assert([...stations.values()].every(s=>s.calls.step.length===7));
  const completed=drain(c,stations);assert.equal(completed.slices,6);assert.equal(completed.result.ok,true);assert.equal(completed.result.done,true);
  assert.equal(c.planning,false);assert.equal(c.state,'running');assert.equal(c.startTick,before.tick);assert.equal(c.batchNumber,1);
  assert.deepEqual(c.startTimes,Object.fromEntries(ids.map(id=>[id,.007])));
  c.advancePlanning();assert.equal(c.batchNumber,1,'Completed plans must not commit a second batch');
  assert([...stations.values()].every(s=>s.calls.step.length===7));return {candidateSlices:completed.slices,startTick:c.startTick,physicsStepsDuringPreparation:0};
});
check('Selection, task changes and repeated Start reject without mutating an in-progress preparation',()=>{
  const {c,stations}=fixture();c.start();const selected=[...c.selectedIds],starts=[...stations.values()].map(s=>s.calls.start.length);
  assert.equal(c.select(['r1']).ok,false);assert.deepEqual(c.selectedIds,selected);
  assert.equal(c.select([]).ok,false);assert.deepEqual(c.selectedIds,selected);
  assert.equal(c.configureTask('stack').ok,false);assert.equal(c.start().ok,false);
  assert(c.planning);assert.deepEqual([...stations.values()].map(s=>s.calls.start.length),starts);
  drain(c,stations);assert.equal(c.batchNumber,1);
});
check('Stop cancels every pending participant and later drains cannot restart or commit the cancelled batch',()=>{
  const {c,stations}=fixture();c.start();c.advancePlanning();const before=positions(stations),count=advances(stations);
  c.stop();assert.equal(c.planning,false);assert.equal(c.state,'stopped');assert.equal(c.running,false);
  for(let i=0;i<3;i++)assert.equal(c.advancePlanning().pending,false);
  assert.equal(advances(stations),count);assert.equal(c.batchNumber,0);assert.equal(c.startTick,null);assert.deepEqual(c.startTimes,{});
  assert([...stations.values()].every(s=>s.calls.stop===1&&s.calls.cancel===1&&s.calls.reset===0));assert.deepEqual(positions(stations),before);
  assert(c.start().ok);drain(c,stations);assert.equal(c.batchNumber,1);assert.equal(c.state,'running');
});
check('Reset invalidates pending work before a new selection and preserves a clean next start',()=>{
  const {c,stations}=fixture();c.start();const tickets=[...stations.values()].map(s=>s.prepare(2));c.advancePlanning();c.reset();
  assert.equal(c.planning,false);assert.equal(c.state,'idle');assert.equal(c.tick,0);assert.deepEqual(c.runIds,[]);assert.equal(c.batchNumber,0);
  assert.deepEqual([...stations.values()].map((s,index)=>s.completeStale(tickets[index])),[false,false,false]);
  c.advancePlanning();assert.equal(c.batchNumber,0);assert.equal(c.state,'idle');
  assert(c.select(['c5']).ok);assert(c.start().ok);drain(c,stations);assert.deepEqual(c.runIds,['c5']);assert.deepEqual(c.startTimes,{c5:0});
  assert.equal(stations.get('r1').calls.start.length,1);assert.equal(stations.get('g1').calls.start.length,1);assert.equal(stations.get('c5').calls.start.length,2);
});
for(const omitReason of [false,true])check(`A staged rejection${omitReason?' without an explicit result reason':''} stops all attempted stations without committing or resetting objects`,()=>{
  const {c,stations}=fixture({r1:{slices:1},g1:{slices:2,reject:true,omitReason},c5:{slices:4}}),before=positions(stations);
  assert(c.start().ok);const {result}=drain(c,stations);assert.equal(result.ok,false);assert.equal(result.done,true);
  assert.match(result.reason,/测试分段目标不可达/);assert.equal(c.state,'failed');assert.equal(c.running,false);assert.equal(c.planning,false);
  assert.equal(c.batchNumber,0);assert.equal(c.startTick,null);assert.deepEqual(c.startTimes,{});assert.deepEqual(positions(stations),before);
  assert([...stations.values()].every(s=>s.calls.stop===1&&s.calls.reset===0));assert.equal(stations.get('c5').calls.advance,0);
});
check('A thrown candidate error follows the same cancellation path and retains the actionable reason',()=>{
  const {c,stations}=fixture({g1:{slices:1,throwOnAdvance:true}});assert(c.start().ok);const {result}=drain(c,stations);
  assert.equal(result.ok,false);assert.match(result.reason,/测试路径检查异常/);assert.equal(c.planning,false);assert.equal(c.running,false);assert.equal(c.batchNumber,0);
});
for(const task of ['stack','pipes'])check(`${task}: a later item begins planning after one complete global step, then pauses all worlds until ready`,()=>{
  const {c,stations}=fixture({r1:{slices:1,nextAt:1},g1:{slices:1},c5:{slices:1}});
  assert(c.start({task}).ok);drain(c,stations);const firstStart={...c.startTimes};
  c.step();assert.equal(c.tick,1);assert(c.planning);assert([...stations.values()].every(s=>s.calls.step.length===1&&s.data.time===.001));
  assert.throws(()=>c.step(),/规划|路径|planning/i);assert([...stations.values()].every(s=>s.calls.step.length===1));
  drain(c,stations);assert.equal(c.batchNumber,1);assert.deepEqual(c.startTimes,firstStart);assert.equal(c.startTick,0);assert.equal(c.tick,1);
  c.step();assert.equal(c.tick,2);assert([...stations.values()].every(s=>s.calls.step.length===2&&s.data.time===.002));
});
check('Unselected ready worlds retain the same 1 ms clock while only selected stations receive preparation',()=>{
  const {c,stations}=fixture();assert(c.select(['g1']).ok);assert(c.start().ok);drain(c,stations);
  for(let i=0;i<25;i++)c.step();assert.equal(c.tick,25);
  for(const [id,s]of stations){assert.equal(s.calls.start.length,id==='g1'?1:0);assert.equal(s.calls.step.length,25);assert(s.calls.step.every(dt=>dt===.001));}
});
check('The production scheduler caps wall backlog and advances only fixed 1 ms physical ticks',()=>{
  const values=[],scheduler=createFrameScheduler({now:()=>0});
  assert.equal(scheduler.advance(0,{step:dt=>values.push(dt)}).steps,0);
  const first=scheduler.advance(1000,{step:dt=>values.push(dt)});
  assert.equal(first.steps,24);assert(values.every(dt=>dt===.001));assert(Math.abs(first.backlogSeconds-.026)<1e-10);
  assert(Math.abs(first.droppedWallSeconds-.95)<1e-10);assert(first.backlogSeconds<=.05);
  const second=scheduler.advance(1016,{step:dt=>values.push(dt)});assert.equal(second.steps,24);
  assert(Math.abs(second.backlogSeconds-.018)<1e-10);assert.equal(values.length,48);assert.equal(second.droppedWallSeconds,first.droppedWallSeconds);
  return {maxStepsPerFrame:24,fixedPhysicalStep:.001,maximumBacklogSeconds:.05,droppedWallSeconds:first.droppedWallSeconds};
});
check('The 8 ms production work budget yields after the last complete physical tick',()=>{
  let cpu=0,steps=0;const scheduler=createFrameScheduler({now:()=>cpu});
  scheduler.advance(0,{step(){}});const result=scheduler.advance(40,{step(dt){assert.equal(dt,.001);steps++;cpu+=3;}});
  assert.equal(steps,3);assert.equal(result.steps,3);assert.equal(cpu,9);
  assert(result.backlogSeconds>.03&&result.backlogSeconds<.04);
  return {tickCostFixtureMs:3,completedTicks:3,cpuFixtureMs:cpu,scope:'A tick completes atomically; the budget is checked before each tick, not inside physics.'};
});
check('Hidden and preparing frames discard catch-up work, and reset resumes from the new frame time',()=>{
  let steps=0;const scheduler=createFrameScheduler({now:()=>0,maxSteps:2}),step=()=>steps++;
  scheduler.advance(0,{step});assert.equal(scheduler.advance(100,{step}).steps,2);assert(scheduler.backlogSeconds>0);
  assert.equal(scheduler.advance(10000,{paused:true,step}).steps,0);assert.equal(scheduler.backlogSeconds,0);
  assert.equal(scheduler.advance(10016,{paused:true,step}).steps,0);
  scheduler.reset(20000);assert.equal(scheduler.backlogSeconds,0);assert.equal(scheduler.advance(20001,{step}).steps,1);assert.equal(steps,3);
});
check('The exact production frame discards completed planning CPU time instead of catching it up as physics',()=>{
  const start=appText.indexOf('function frame(now)'),end=appText.indexOf('\nrequestAnimationFrame(frame);',start);
  assert(start>=0&&end>start);const source=appText.slice(start,end);let wall=1000,ticks=0,pending=true;
  const scheduler=createFrameScheduler({now:()=>wall});scheduler.reset(1000);
  const coordinator={get planning(){return pending;},advancePlanning(){wall=1600;pending=false;return {pending:false,done:true,ok:true};},step(dt){assert.equal(dt,.001);ticks++;}};
  const renderer={shadowMap:{needsUpdate:false},render(){}},scene={},camera={};
  const api=new Function('performance','coordinator','frameScheduler','renderer','scene','camera',
    `let lastFrame=1000,lastUI=1000,lastFeed=1000,physicsSteps=0,planningSlices=0,accumulator=0,preparing=false,notice='';
     const frameTimes=[],renderTimes=[],stations=new Map(),visions=new Map(),document={hidden:false},reducedMotion=false;
     const orbit={update(){}},updateUI=()=>{},updateLabels=()=>{},updateCameras=()=>{},requestAnimationFrame=()=>{};
     ${source};return {frame,get accumulator(){return accumulator;},get physicsSteps(){return physicsSteps;},get planningSlices(){return planningSlices;}};`)
    ({now:()=>wall},coordinator,scheduler,renderer,scene,camera);
  api.frame(1000);assert.equal(ticks,0);assert.equal(api.planningSlices,1);
  wall=1616;api.frame(1616);assert.equal(ticks,16);assert.equal(api.physicsSteps,16);
  assert(api.accumulator<1e-10);assert.equal(scheduler.droppedWallSeconds,0);
  return {planningStartedAtMs:1000,planningEndedAtMs:1600,nextFrameAtMs:1616,physicalTicks:16,droppedWallSeconds:0,
    scope:'Exact production frame function with deterministic wall/RAF clocks and a declared completing planner fixture.'};
});
check('The exact production frame refreshes feeds within the slow-render fallback while hidden and planning states still pause them',()=>{
  const start=appText.indexOf('function frame(now)'),end=appText.indexOf('\nrequestAnimationFrame(frame);',start);
  assert(start>=0&&end>start);let wall=1000,pending=false;const document={hidden:false},scheduler=createFrameScheduler({now:()=>wall});scheduler.reset(1000);
  const coordinator={get planning(){return pending;},advancePlanning(){return {pending:true,done:false,ok:true};},step(dt){assert.equal(dt,.001);}},
    renderer={shadowMap:{needsUpdate:false},render(){wall+=20;}};
  const api=new Function('performance','coordinator','frameScheduler','renderer','scene','camera','document',
    `let lastFrame=1000,lastUI=1000,lastFeed=1000,physicsSteps=0,planningSlices=0,accumulator=0,preparing=false,notice='',feedCalls=0;
     const frameTimes=[],renderTimes=[],stations=new Map(),visions=new Map(),reducedMotion=false;
     const orbit={update(){}},updateUI=()=>{},updateLabels=()=>{},updateCameras=()=>{feedCalls++;},requestAnimationFrame=()=>{};
     ${appText.slice(start,end)};return {frame,get feedCalls(){return feedCalls;}};`)
    ({now:()=>wall},coordinator,scheduler,renderer,{}, {},document);
  wall=1200;api.frame(1200);assert.equal(api.feedCalls,0,'A busy frame below the fallback interval should yield the feed pass');
  wall=2101;api.frame(2101);assert.equal(api.feedCalls,1,'Expensive main rendering must not starve camera feeds indefinitely');
  document.hidden=true;wall=4000;api.frame(4000);assert.equal(api.feedCalls,1);
  document.hidden=false;pending=true;wall=7000;api.frame(7000);assert.equal(api.feedCalls,1);
  return {mainRenderFixtureMs:20,refreshedAfterMs:1101,hiddenFeedPasses:0,planningFeedPasses:0,
    scope:'Exact production frame gate with deterministic render cost; visibility and one-feed behavior are separately exercised through the production feed function.'};
});
check('A new item planning request stops the scheduler after one complete global tick without a second-world advance',()=>{
  const {c,stations}=fixture({r1:{slices:1,nextAt:1},g1:{slices:1},c5:{slices:1}});c.start({task:'pipes'});drain(c,stations);
  const scheduler=createFrameScheduler({now:()=>0}),step=dt=>c.step(dt),planning=()=>c.planning;
  scheduler.advance(0,{step,planning});const started=scheduler.advance(30,{step,planning});
  assert.equal(started.steps,1);assert.equal(started.backlogSeconds,0);assert.equal(c.tick,1);
  assert([...stations.values()].every(s=>s.calls.step.length===1&&s.data.time===.001));
  assert.equal(scheduler.advance(2000,{step,planning}).steps,0);assert.equal(c.tick,1);
  drain(c,stations);scheduler.reset(2000);assert.equal(scheduler.advance(2001,{step,planning}).steps,1);assert.equal(c.tick,2);
});
check('Invalid frame times reject and backwards timestamps do not manufacture physical time',()=>{
  const scheduler=createFrameScheduler({now:()=>0});let steps=0;const step=()=>steps++;
  scheduler.advance(100,{step});assert.equal(scheduler.advance(90,{step}).steps,0);assert.equal(steps,0);
  for(const time of [NaN,Infinity,-Infinity])assert.throws(()=>scheduler.advance(time,{step}),TypeError);
});
check('Camera-feed eligibility skips zero-area and offscreen feeds while retaining partial intersections',()=>{
  const viewport={width:390,height:844},base={left:20,right:220,top:100,bottom:250,width:200,height:150};
  assert.equal(cameraFeedVisible(base,viewport),true);
  for(const change of [{width:0},{height:0},{left:390,right:590},{left:-201,right:0},{top:844,bottom:994},{top:-150,bottom:0}])
    assert.equal(cameraFeedVisible({...base,...change},viewport),false,JSON.stringify(change));
  for(const change of [{left:-100,right:100},{top:-50,bottom:100},{left:300,right:500},{top:800,bottom:950}])
    assert.equal(cameraFeedVisible({...base,...change},viewport),true,JSON.stringify(change));
  assert.equal(cameraFeedVisible(null,viewport),false);
});
check('The exact production camera-feed pass renders one visible feed at a time and restores renderer state on success or failure',()=>{
  const appSource=appText,start=appSource.indexOf('function updateCameras()'),end=appSource.indexOf('\nfunction resize(',start);
  assert(start>=0&&end>start);const source=appSource.slice(start,end),draws=[],resets=[],scene={},feedPixels=new Uint8Array(240*180*4);
  const focus={id:'r1',config:{tableCenter:.7,tableHeight:.75},localToWorld:vector=>vector},vision={overhead:new THREE.PerspectiveCamera(),camera:new THREE.PerspectiveCamera()};
  const renderer={shadowMap:{autoUpdate:true},setRenderTarget(value){resets.push(value);}},cameraFeedPass={render(_scene,camera,pixels){assert.strictEqual(_scene,scene);assert.strictEqual(pixels,feedPixels);draws.push(camera);}};
  const visible={left:10,right:250,top:10,bottom:190,width:240,height:180},offscreen={...visible,top:900,bottom:1080};
  const feeds=[0,1].map(index=>({rect:{...visible},image:{data:new Uint8ClampedArray(240*180*4)},writes:0,
    element:{getBoundingClientRect(){return feeds[index].rect;}},ctx:{putImageData(image,x,y){assert.strictEqual(image,feeds[index].image);assert.equal(x,0);assert.equal(y,0);feeds[index].writes++;}}}));
  const api=new Function('THREE','focused','visions','feeds','cameraFeedVisible','innerWidth','innerHeight','renderer','cameraFeedPass','scene','feedPixels',
    `let showCameras=true,nextFeed=0,feedRenderCount=0;${source};return {update:updateCameras,shown(value){showCameras=value;},get count(){return feedRenderCount;}};`)
    (THREE,()=>focus,new Map([['r1',vision]]),feeds,cameraFeedVisible,390,844,renderer,cameraFeedPass,scene,feedPixels);
  api.update();api.update();assert.deepEqual(draws,[vision.overhead,vision.camera]);assert.deepEqual(feeds.map(feed=>feed.writes),[1,1]);assert.equal(api.count,2);
  assert.equal(renderer.shadowMap.autoUpdate,true);assert.deepEqual(resets,[null,null]);
  feeds.forEach(feed=>feed.rect=offscreen);api.update();assert.equal(draws.length,2);
  feeds[1].rect=visible;api.update();api.update();assert.deepEqual(draws.slice(2),[vision.camera,vision.camera]);
  api.shown(false);api.update();assert.equal(draws.length,4);api.shown(true);
  cameraFeedPass.render=()=>{throw new Error('测试渲染失败');};assert.throws(()=>api.update(),/测试渲染失败/);
  assert.equal(renderer.shadowMap.autoUpdate,true);assert.equal(resets.at(-1),null);assert.equal(api.count,4);
  return {visibleFeedAlternation:true,offscreenRenders:0,hiddenRenders:0,rendererStateRestored:true,
    scope:'Exact production feed function with declared canvas/renderer fixtures and real Three cameras; no GPU or browser pixels.'};
});
check('The camera policy preserves an orbited view through Start, reset, task changes and resize until an explicit framing action',()=>{
  const camera={position:[2.7,1.3,-.6],target:[.7,.8,.1],distance:2.96},before=structuredClone(camera),applied=[];
  function request(reason){if(shouldRefitCamera(reason)){applied.push(reason);camera.position=[7,3,1];camera.target=[.4,.8,0];camera.distance=7.6;}}
  for(const reason of ['start','reset','task','resize','theme','planning','stop',undefined,'']){
    assert.equal(shouldRefitCamera(reason),false,`Unexpected camera owner: ${reason}`);request(reason);assert.deepEqual(camera,before);
  }
  for(const reason of ['initial','participants','panorama','focus']){assert.equal(shouldRefitCamera(reason),true);request(reason);}
  assert.deepEqual(applied,['initial','participants','panorama','focus']);assert.notDeepEqual(camera,before);
});

const sourceFrozen=JSON.stringify(await hashes())===JSON.stringify(sourceHashes);
if(!sourceFrozen)errors.push({name:'source freeze',message:'Responsiveness contract sources changed during verification'});
const report={pass:errors.length===0,total:checks.length,passed:checks.filter(item=>item.pass).length,checks,errors,sourceFrozen,sourceHashes,
  scope:'Production coordinator, frame scheduler, camera-feed visibility and camera ownership policies with explicit fake stations, rectangles and clocks. No real contacts, physical grasp, native browser interaction, rendered pixels or device responsiveness is certified.'};
const destination=new URL(reportPath,root);await mkdir(new URL('./',destination),{recursive:true});await writeFile(destination,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({pass:report.pass,total:report.total,passed:report.passed,sourceFrozen,report:reportPath,errors},null,2));
if(!report.pass)process.exitCode=1;
